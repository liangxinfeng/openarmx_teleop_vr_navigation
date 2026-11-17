import asyncio,json
from types import SimpleNamespace
from unittest.mock import patch
from openarmx_teleop_vr_navigation_306.navigation_web_bridge import NavigationVrWebBridge

class Request:
    def __init__(self,body=None,origin=None):self.body=body;self.headers={'Origin':origin} if origin else {};self.host='robot.local:8445'
    async def json(self):return self.body

def bridge():return SimpleNamespace(get_parameter=lambda name:SimpleNamespace(value='0_345'))

def test_status_proxy_is_readonly_and_recorder_is_optional():
    with patch('openarmx_teleop_vr_navigation_306.teaching_ipc.request',return_value=dict(success=True,available=True,enabled=False)) as rpc:
        response=asyncio.run(NavigationVrWebBridge._api_teaching_status(bridge(),Request()))
        assert response.status==200 and json.loads(response.body)['available'];assert rpc.call_args.args[1]=='/status'
    with patch('openarmx_teleop_vr_navigation_306.teaching_ipc.request',side_effect=FileNotFoundError):
        response=asyncio.run(NavigationVrWebBridge._api_teaching_status(bridge(),Request()))
        assert response.status==200 and not json.loads(response.body)['available']

def test_browser_cannot_enable_recorder_or_issue_motor_commands():
    for op in ('enable','hardware-enable','execute','recover','delete_partial'):
        with patch('openarmx_teleop_vr_navigation_306.teaching_ipc.request') as rpc:
            response=asyncio.run(NavigationVrWebBridge._api_teaching_command(bridge(),Request(dict(op=op))))
            assert response.status==409;rpc.assert_not_called()

def test_record_command_has_no_desktop_privileges_and_rejects_cross_origin():
    body=dict(op='capture',session='example',request_id='same-id')
    with patch('openarmx_teleop_vr_navigation_306.teaching_ipc.request',return_value=dict(success=True,count=1)) as rpc:
        response=asyncio.run(NavigationVrWebBridge._api_teaching_command(bridge(),Request(body)))
        assert response.status==200 and rpc.call_args.args[2]==body and not rpc.call_args.kwargs.get('desktop')
        rpc.reset_mock()
        response=asyncio.run(NavigationVrWebBridge._api_teaching_command(bridge(),Request(body,'https://another.local')))
        assert response.status==403;rpc.assert_not_called()
