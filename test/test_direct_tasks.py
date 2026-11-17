import asyncio, json, math, threading, time
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import AsyncMock, Mock
import pytest
from openarmx_teleop_vr_navigation_306.navigation_web_bridge import NavigationVrWebBridge as B
from openarmx_teleop_vr_navigation_306.simple_task_catalog import parse_catalog,load_catalog
from openarmx_teleop_vr_navigation_306.task_workflow import TaskWorkflow,task_chain_templates

def request(payload):return NS(json=AsyncMock(return_value=payload))
def workflow(angle=0):
 c=parse_catalog(f'任务库\n配送区 | 放置衣物 | 0 | 否 | {angle}\n任务串：演示\n配送区 | 放置衣物 | 0 | 否 | {angle}')
 w=TaskWorkflow(c);w.submit('VR',task_chain_templates(c)[0]['tasks']);return w

@pytest.mark.parametrize('angle',[0,10,30,45])
def test_direct_workflow_has_no_acceptance_state(angle):
 w=workflow(angle);assert w.snapshot()['state']=='ready_to_start'
 t=w.confirm(['配送区']);assert t['head_pitch_deg']==angle
 w.navigation_started();w.navigation_arrived('配送区');w.height_adjusted()
 assert w.snapshot()['state']=='awaiting_action'
 assert w.complete_action()[0]=='done'

def test_localization_wait_has_explicit_retry_not_dispatch():
 w=workflow();w.manual_relocalization_required('等待定位')
 assert w.snapshot()['state']=='waiting_localization'
 assert w.confirm(['配送区'])['head_pitch_deg']==0

def test_external_dispatch_cannot_submit_or_move():
 s=NS(_task_workflow=Mock(),_call_service=AsyncMock())
 r=asyncio.run(B._api_tasks_submit(s,request({'tasks':[]})))
 assert r.status==410;s._task_workflow.submit.assert_not_called();s._call_service.assert_not_called()

def nav_subject():
 s=NS(_nav_lock=threading.Lock(),_client_lock=threading.Lock(),_waypoints=['配送区'],
      _control_session='owner',_connected_clients=1,_vr_navigation_rearm_client='rearm',
      _navigate_waypoint_client='navigate',_call_service=AsyncMock(return_value=(True,'ok')))
 return s

def test_both_entrypoints_use_rearm_before_goal():
 s=nav_subject();result=asyncio.run(B._request_waypoint_navigation(s,'配送区'))
 assert result==(True,'ok')
 calls=s._call_service.call_args_list
 assert [c.args[0] for c in calls]==['rearm','navigate']
 assert calls[1].args[1].name=='配送区'

@pytest.mark.parametrize('cause',['disconnected','unknown','rearm_failed','owner_changed'])
def test_shared_start_never_bypasses_failures(cause):
 s=nav_subject()
 if cause=='disconnected':s._connected_clients=0
 if cause=='rearm_failed':s._call_service.return_value=(False,'心跳未恢复')
 if cause=='owner_changed':
  async def call(*args):s._control_session='new';return True,'ok'
  s._call_service.side_effect=call
 ok,_=asyncio.run(B._request_waypoint_navigation(s,'未知' if cause=='unknown' else '配送区'))
 assert not ok
 assert all(c.args[0]!='navigate' for c in s._call_service.call_args_list)

@pytest.mark.parametrize('showroom',[False,True])
def test_single_point_preserves_neck_and_does_not_create_task(showroom):
 s=nav_subject();s._showroom_mode_enabled=showroom
 s._replace_active_task=AsyncMock();s._request_waypoint_navigation=AsyncMock(return_value=(True,'ok'))
 s._task_workflow=Mock();s._set_task_height=Mock()
 r=asyncio.run(B._api_navigation_waypoint(s,request({'name':'配送区'})))
 assert r.status==200;s._replace_active_task.assert_awaited_once()
 s._task_workflow.submit.assert_not_called();s._set_task_height.assert_not_called()

def test_cached_showroom_url_routes_to_plain_navigation():
 req=request({'name':'配送区'});s=NS(_api_navigation_waypoint=AsyncMock(return_value='plain'))
 assert asyncio.run(B._api_showroom_waypoint_start(s,req))=='plain'
 s._api_navigation_waypoint.assert_awaited_once_with(req)

@pytest.mark.parametrize('angle',[0,15,30])
def test_task_start_uses_file_angle_and_shared_navigation(angle):
 w=workflow(angle);w.confirm(['配送区'])
 s=NS(_task_workflow=w,_speak_task=Mock(),_showroom_single_task_active=False,
      _request_waypoint_navigation=AsyncMock(return_value=(True,'ok')),_exit_in_progress=False,
      _set_task_height=Mock())
 assert asyncio.run(B._start_current_task_navigation(s))=='ok'
 s._request_waypoint_navigation.assert_awaited_once_with('配送区')
 s._set_task_height.assert_called_once_with(0.,float(angle),reverse_squat=False)

def test_task_rearm_failure_is_retryable_without_acceptance():
 w=workflow();w.confirm(['配送区'])
 s=NS(_task_workflow=w,_speak_task=Mock(),_showroom_single_task_active=False,
      _request_waypoint_navigation=AsyncMock(return_value=(False,'心跳中断')),_exit_in_progress=False,
      _set_task_height=Mock(),_nav_lock=threading.Lock(),_pose_time=time.monotonic(),_relocalization_status={})
 with pytest.raises(RuntimeError,match='心跳'):asyncio.run(B._start_current_task_navigation(s))
 assert w.snapshot()['state']=='navigation_failed';s._set_task_height.assert_not_called()

@pytest.mark.parametrize('available',[True,False])
def test_start_gate_never_returns_to_acceptance(available):
 w=workflow();s=NS(_task_workflow=w,_nav_lock=threading.Lock(),_waypoints=['配送区'],
     _pose={} if available else None,_relocalization_status={},_start_current_task_navigation=AsyncMock(return_value='ok'))
 r=asyncio.run(B._api_tasks_confirm(s,None))
 if available:assert r.status==200;s._start_current_task_navigation.assert_awaited_once()
 else:assert r.status==409;assert w.snapshot()['state']=='waiting_localization';s._start_current_task_navigation.assert_not_called()

@pytest.mark.parametrize('age,valid',[(.01,True),(.5,True),(2.,False),(-2.,False)])
def test_tf_pose_freshness_matches_live_transform(age,valid):
 stamp=100-age;whole=int(stamp)
 tf=NS(header=NS(stamp=NS(sec=whole,nanosec=int((stamp-whole)*1e9))),
       transform=NS(translation=NS(x=1.,y=2.),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
 s=NS(_pose_tf_buffer=NS(lookup_transform=Mock(return_value=tf)),_nav_lock=threading.Lock(),
     get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_000_000_000)),_pose=None,_pose_time=0.)
 B._refresh_tf_pose(s)
 assert (s._pose is not None)==valid
 if valid:assert s._pose=={'x':1.,'y':2.,'yaw':0.}

def test_old_amcl_does_not_override_live_tf():
 s=NS(_pose={'x':1.,'y':2.,'yaw':0.},_last_tf_pose_time=time.monotonic())
 B._on_pose(s,None);assert s._pose['x']==1.

def test_actual_catalog_all_task_chain_angles_are_preserved():
 path=Path('/home/ubuntu/ros2_ws/src/openarmx_teleop_vr_navigation_306/config/task_actions.txt')
 catalog=load_catalog(path)
 for t in task_chain_templates(catalog):
  w=TaskWorkflow(catalog);snapshot=w.submit('test',t['tasks'])
  assert [x['head_pitch_deg'] for x in snapshot['tasks']]==[x['head_pitch_deg'] for x in t['tasks']]

def test_frontend_no_dispatch_acceptance_or_showroom_rerouting():
 root=Path(__file__).parents[1];js=(root/'web/nav_app.js').read_text()
 assert 'pendingTaskDecision' not in js and 'taskAcceptArmed' not in js
 assert '任务分发确认' not in js and 'A接受' not in js
 assert "navigationAction('/api/tasks/showroom-waypoint/start'" not in js
 assert "navigationAction('/api/tasks/start')" in js
 assert "navigationAction('/api/navigation/waypoint', { name })" in js
