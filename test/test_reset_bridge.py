import asyncio
import json
import threading
from types import SimpleNamespace as NS

import pytest
from openarmx_teleop_vr_navigation_306.reset_bridge import ResetBridgeMixin


class Harness(ResetBridgeMixin):
    def __init__(self):
        self.calls=[]
        self._drive_lock=threading.Lock()
        self._drive_sample=None
        self._task_height_target_m=None
        self.nav_active=False
        self.status={'active_arms':[], 'backend':{'hardware_ready':True,'hardware_enabled':True,
            'head_follow_enabled':False},'reset_menu':{'enabled':True,'active':True,
                'grips_released':True,'input_fresh':True}}
        self._reset_menu_initialize()
    def create_client(self,_type,name):
        return NS(name=name,service_is_ready=lambda:True)
    async def _operator_payload(self,payload):
        if payload.get('client_id')!='vr':raise RuntimeError('wrong headset')
        return payload
    def _teleop_status_snapshot(self):return self.status,0,True
    def _navigation_is_active(self):return self.nav_active
    async def _call_service(self,client,request):
        self.calls.append(client.name)
        return True,'accepted'
    def payload(self,**kwargs):
        return dict(client_id='vr',scope='right',confirmed=True,preserve_grippers=True,operation_id='reset-1',**kwargs)


@pytest.mark.parametrize('scope',['right','left','both'])
def test_correct_scope_and_same_operation_is_not_replayed(scope):
    async def run():
        h=Harness();p=h.payload();p['scope']=scope
        assert (await h._api_reset_arms(p)).status==200
        assert json.loads((await h._api_reset_arms(p)).text)['duplicate']
        assert h.calls==['/openarmx_teleop_vr_306_v4/reset_arms_hold_'+scope]
    asyncio.run(run())


@pytest.mark.parametrize('field,value',[('active',False),('grips_released',False),('input_fresh',False),('enabled',False)])
def test_missing_live_menu_or_released_grips_rejects_reset(field,value):
    async def run():
        h=Harness();h.status['reset_menu'][field]=value
        assert (await h._api_reset_arms(h.payload())).status==409
        assert not h.calls
    asyncio.run(run())


def test_wrong_headset_navigation_or_reset_in_progress_never_calls_service():
    async def run():
        h=Harness();p=h.payload();p['client_id']='other'
        assert (await h._api_reset_arms(p)).status==409
        h.nav_active=True
        assert (await h._api_reset_arms(h.payload())).status==409
        h.nav_active=False;h.status['backend']['quick_reset_active']=True
        assert (await h._api_reset_arms(h.payload())).status==409
        assert not h.calls
    asyncio.run(run())
