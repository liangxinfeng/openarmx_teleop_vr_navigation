"""Actual bridge methods with fake transport/publishers; never constructs a node."""
import asyncio
import json
import threading
import time
from types import SimpleNamespace as NS, MethodType
from unittest.mock import Mock
import pytest
from openarmx_teleop_vr_navigation_306.navigation_web_bridge import NavigationVrWebBridge as Bridge

def subject():
    now=time.monotonic()
    status={'supported':True,'can_switch':True,'mode':'normal',
            'feedback_time_monotonic':now,'ack':{}}
    calls=[]
    c=NS(_exit_in_progress=False,_client_lock=threading.Lock(),_control_peer='headset',
         _control_client_id='one',_drive_lock=threading.Lock(),_drive_time=now,
         _drive_menu_paused_since=now-.5,_drive_sample={'deadman':True,'pause_for_menu':True},
         _task_height_lock=threading.Lock(),_task_height_target_m=None,
         _task_intervention_inflight=False,_publish_stop=Mock())
    c._teleop_status_snapshot=lambda:({'backend':{'manual_squat':status}},.01,True)
    def publish(message):
        p=json.loads(message.data);calls.append(p)
        if 'select_squat_mode' in p:
            status['ack']={'request_id':p['request_id'],'accepted':True,
                           'mode':p['select_squat_mode'],'message':'confirmed'}
            status['mode']=p['select_squat_mode']
    c._body_height_publisher=NS(publish=publish)
    for name in ('_manual_squat_status','_operator_payload'):
        setattr(c,name,MethodType(getattr(Bridge,name),c))
    async def data():return {'mode':'reverse','client_id':'one'}
    return c,NS(remote='headset',json=data),status,calls

def test_mode_api_waits_for_ack_and_sends_no_motion():
    c,request,status,calls=subject()
    response=asyncio.run(Bridge._api_manual_squat_mode(c,request))
    assert response.status==200 and json.loads(response.text)['manual_squat']['mode']=='reverse'
    assert len(calls)==1 and calls[0]['enabled'] is False and calls[0]['direction']==0
    assert not any('target' in k for k in calls[0])

@pytest.mark.parametrize('fault',['wrong_client','no_takeover','not_paused','stale_drive','crouched','stale_feedback','automatic_height'])
def test_rejected_requests_publish_nothing(fault):
    c,request,status,calls=subject()
    if fault=='wrong_client':request.remote='observer'
    elif fault=='no_takeover':c._drive_sample['deadman']=False
    elif fault=='not_paused':c._drive_sample['pause_for_menu']=False
    elif fault=='stale_drive':c._drive_time-=1
    elif fault=='crouched':status['can_switch']=False;status['message']='stand first'
    elif fault=='stale_feedback':status['feedback_time_monotonic']-=1
    elif fault=='automatic_height':c._task_height_target_m=.2
    response=asyncio.run(Bridge._api_manual_squat_mode(c,request))
    assert response.status==409 and not calls

def test_paused_packets_zero_axes_and_height_without_releasing_takeover(monkeypatch):
    from openarmx_teleop_vr_navigation_306 import navigation_web_bridge as module
    monkeypatch.setattr(module.rclpy,'ok',lambda:True)
    c,_,_,calls=subject()
    Bridge._set_drive_sample(c,dict(deadman=True,pause_for_menu=True,
        forward=1,lateral=-1,turn=.8,body_height_enabled=True,body_height_direction=1))
    assert c._drive_sample==dict(deadman=True,pause_for_menu=True,forward=0.,lateral=0.,turn=0.)
    assert calls==[{'enabled':False,'direction':0.}]
    c._publish_stop.assert_called_once()

def test_menu_heartbeat_does_not_overwrite_pending_mode_request(monkeypatch):
    from openarmx_teleop_vr_navigation_306 import navigation_web_bridge as module
    monkeypatch.setattr(module.rclpy,'ok',lambda:True)
    c,_,_,calls=subject();c._squat_mode_pending_until=time.monotonic()+1
    Bridge._set_drive_sample(c,dict(deadman=True,pause_for_menu=True))
    assert calls==[]
    c._publish_stop.assert_called_once()
