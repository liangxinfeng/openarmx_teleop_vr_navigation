"""Explicit, two-confirmation arm resets for the active navigation headset."""
import asyncio
import json
import time
from std_msgs.msg import String
from aiohttp import web
from std_srvs.srv import Trigger


class ResetBridgeMixin:
    def _reset_menu_initialize(self):
        self._arm_reset_clients = {scope: self.create_client(Trigger,
            f'/openarmx_teleop_vr_306_v4/reset_arms_hold_{scope}')
            for scope in ('left', 'right', 'both')}
        self._reset_cancel_pub = self.create_publisher(
            String, '/openarmx_teleop_vr_306_v4/cancel_reset', 10)
        self._arm_reset_operations = {}
        self._arm_reset_busy = False

    async def _api_reset_arms(self, request):
        try:
            payload = await self._operator_payload(request)
            scope = payload.get('scope')
            operation = payload.get('operation_id')
            if not isinstance(scope, str) or scope not in self._arm_reset_clients or payload.get('confirmed') is not True or payload.get('preserve_grippers') is not True:
                raise ValueError('请选择复位范围，并明确确认保持两侧夹爪状态')
            if not isinstance(operation, str) or not 1 <= len(operation) <= 128:
                raise ValueError('复位请求缺少有效操作编号')
            signature = (payload.get('client_id'), scope)
            if operation in self._arm_reset_operations:
                previous = self._arm_reset_operations[operation]
                if previous['signature'] != signature:
                    raise ValueError('操作编号不可复用于不同复位命令')
                return web.json_response({'success': previous['success'], 'duplicate': True,
                    'message': previous['message']}, status=200 if previous['success'] else 409)
            if self._arm_reset_busy:
                raise RuntimeError('上一条复位请求仍在处理')
            status, _age, fresh = self._teleop_status_snapshot()
            backend = (status or {}).get('backend') or {}
            menu = (status or {}).get('reset_menu') or {}
            if not fresh or backend.get('hardware_ready') is not True or not backend.get('hardware_enabled'):
                raise RuntimeError('请先使能VR并等待新鲜的机器人反馈')
            if backend.get('quick_reset_active') or ((status or {}).get('quick_reset') or {}).get('active'):
                raise RuntimeError('机器人正在复位，请等待完成')
            if not (menu.get('enabled') and menu.get('active')):
                raise RuntimeError('请保持复位对话框打开后重新确认')
            if (status or {}).get('active_arms') or backend.get('head_follow_enabled'):
                raise RuntimeError('请松开双手腰键并锁定头部后重新确认')
            navigation_paused = False
            if self._navigation_is_active():
                pause_client = getattr(self, '_pause_navigation_client', None)
                if pause_client is None or not pause_client.service_is_ready():
                    raise RuntimeError('无法暂停当前导航，请稍候重试')
                paused, pause_message = await self._call_service(pause_client, Trigger.Request())
                if not paused:
                    raise RuntimeError(pause_message or '暂停当前导航失败，请稍候重试')
                navigation_paused = True
            with self._drive_lock:
                if self._drive_sample and self._drive_sample.get('deadman'):
                    raise RuntimeError('请先退出底盘接管')
            if self._task_height_target_m is not None:
                raise RuntimeError('请等待自动姿态调整完成，再确认复位')
            client = self._arm_reset_clients[scope]
            if not client.service_is_ready():
                raise RuntimeError('所选保持夹爪复位服务尚未就绪，请重新启动完整VR导航栈')
            # Record before awaiting: a timeout/repeated request must never replay motion.
            record = {'signature': signature, 'success': False, 'message': '复位请求已提交，请查看机器人状态'}
            self._arm_reset_operations[operation] = record
            while len(self._arm_reset_operations) > 256:
                del self._arm_reset_operations[next(iter(self._arm_reset_operations))]
            self._arm_reset_busy = True
            try:
                success, message = await self._call_service(client, Trigger.Request())
                record.update(success=success, message=message or '复位请求已处理')
                return web.json_response({'success': success, 'message': record['message'],
                    'scope': scope, 'preserve_grippers': True,
                    'navigation_paused': navigation_paused}, status=200 if success else 409)
            finally:
                self._arm_reset_busy = False
        except (RuntimeError, ValueError, TypeError) as error:
            return web.json_response({'success': False, 'message': str(error)}, status=409)

    async def _api_reset_stop(self, request):
        """Stop has no dependency on the start-request busy flag or menu state."""
        try:
            payload = await self._operator_payload(request)
            reset_id = payload.get('reset_id')
            if not isinstance(reset_id, str) or not 1 <= len(reset_id) <= 128:
                raise ValueError('缺少本次复位编号')
            message = String(data=json.dumps({'reset_id': reset_id}))
            deadline = time.monotonic() + 3.0
            next_send = 0.0
            while time.monotonic() < deadline:
                status, _age, fresh = self._teleop_status_snapshot()
                run = ((status or {}).get('backend') or {}).get('reset_run') or {}
                if fresh and run.get('id') == reset_id and run.get('state') in {
                        'stopped', 'completed', 'failed'}:
                    return web.json_response({'success': True, 'reset': run})
                if fresh and run.get('id') and run['id'] != reset_id:
                    raise RuntimeError('复位编号已变化，旧停止请求不会作用于新复位')
                if time.monotonic() >= next_send:
                    self._reset_cancel_pub.publish(message)
                    next_send = time.monotonic() + 0.20
                await asyncio.sleep(0.05)
            raise RuntimeError('停止尚未确认，请检查连接后重试 A 或 B')
        except (RuntimeError, ValueError, TypeError) as error:
            return web.json_response({'success': False, 'message': str(error)}, status=409)
