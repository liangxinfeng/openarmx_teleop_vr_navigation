"""Cloud door requests are mocked; never touch a robot or production HTTP."""
import asyncio
import threading
import time
from types import SimpleNamespace as NS
from openarmx_teleop_vr_navigation_306.delivery_handoff import DeliveryHandoffMonitor
from openarmx_teleop_vr_navigation_306.navigation_web_bridge import NavigationVrWebBridge as B

def fixture(tmp_path,opened=True):
    m=DeliveryHandoffMonitor(tmp_path/'unused.json',NS(info=lambda *_:None,warning=lambda *_:None))
    data={'handoffs':[dict(order_id='test',task_id='test-PICKUP_FROM_ROOM',door_id='2',arrived=True,
                           opened=opened,closed=False,status='PICKUP_AT_LAUNDRY')]}
    m._apply_get_payload(data);m._last_success_monotonic=time.monotonic();m._url='https://unused.invalid/humanoid-handoff'
    e=dict(order_id='test',task_id='test-PICKUP_FROM_ROOM',door_id='2');calls=[]
    def post(*a):
        calls.append(a)
        return dict(action='close',result='applied',task_id=e['task_id'],door_id='2',idempotent=False)
    m._request_json=post
    return m,e,calls,data

def test_close_contract_idempotency_and_get_flags(tmp_path):
    m,e,c,d=fixture(tmp_path)
    assert m.snapshot()['state']=='opened' and m.snapshot()['can_close']
    assert m.request_close(e)[0] and len(c)==1
    assert c[0][0]=='POST' and c[0][1]=='https://unused.invalid/humanoid-handoff/close'
    assert c[0][3]==e
    m._apply_get_payload(d)  # older GET cannot undo close acknowledgement
    assert m.snapshot()['state']=='closed' and not m.snapshot()['can_close']
    assert m.request_close(e)[0] and len(c)==1
    assert not m.request_open(e)[0] and len(c)==1
    m,e,c,d=fixture(tmp_path);d['handoffs'][0]['closed']=True;m._apply_get_payload(d)
    assert m.snapshot()['state']=='closed'

def test_reject_not_open_not_arrived_stale_identity_and_offline(tmp_path):
    for kind in ('not_open','not_arrived','stale','unknown','task','order','door'):
        m,e,c,d=fixture(tmp_path,kind!='not_open')
        if kind=='not_arrived':m._arrived=False
        if kind=='stale':m._last_success_monotonic-=30
        if kind=='unknown':m._state='unknown'
        if kind in ('task','order','door'):e[kind+'_id']='old'
        assert not m.request_close(e)[0] and not c,kind

def test_close_does_not_overlap_open_or_second_close(tmp_path):
    m,e,c,d=fixture(tmp_path);entered=threading.Event();release=threading.Event();post=m._request_json
    def blocking(*a):
        entered.set();assert release.wait(2);return post(*a)
    m._request_json=blocking
    thread=threading.Thread(target=lambda:m.request_close(e));thread.start()
    try:
        assert entered.wait(1)
        m._apply_transport_error('test interrupted poll')
        m._apply_get_payload(d)
        assert m.snapshot()['state']=='closing'
        assert not m.request_close(e)[0] and not m.request_open(e)[0]
    finally:release.set();thread.join(3)
    assert len(c)==1 and m.snapshot()['state']=='closed'

def test_timeout_or_bad_response_never_claims_closed(tmp_path):
    for kind in ('timeout','bad_response','changed_order'):
        m,e,c,d=fixture(tmp_path);post=m._request_json
        def reply(*a):
            if kind=='timeout':raise TimeoutError('test timeout')
            if kind=='bad_response':return dict(result='applied',action='open')
            d['handoffs'][0]['task_id']='new';m._apply_get_payload(d);return post(*a)
        m._request_json=reply
        assert not m.request_close(e)[0]
        assert m.snapshot()['state']!='closed' and m._door_inflight is None

def test_close_api_requires_confirmation_and_headset(tmp_path):
    m,e,c,d=fixture(tmp_path)
    async def owner(p):return p
    r=NS(_operator_payload=owner,_delivery_handoff=m,
         _delivery_offer_lock=threading.Lock(),_delivery_close_required=None,
         _delivery_offer=None,_delivery_chain_note=None,
         _task_workflow=NS(snapshot=lambda:dict(state='idle',source_text='',current=None)))
    r._delivery_state_payload=lambda:B._delivery_state_payload(r)
    r._api_delivery_handoff_open=lambda req,action='open':B._api_delivery_handoff_open(r,req,action)
    assert asyncio.run(B._api_delivery_handoff_close(r,e)).status==409 and not c
    assert asyncio.run(B._api_delivery_handoff_close(r,{**e,'confirmed':True})).status==200
    assert len(c)==1
    async def denied(p):raise RuntimeError('wrong headset')
    r._operator_payload=denied
    assert asyncio.run(B._api_delivery_handoff_close(r,{**e,'confirmed':True})).status==409


def test_return_load_requires_open_then_close_releases_charge_navigation(tmp_path):
    del tmp_path
    delivery=dict(return_task_chain='洗衣房净衣装舱',last_success_age=0,
                  state='opened',can_close=True,order_id='return-order',
                  task_id='return-order-RETURN_TO_ROOM',door_id='2',
                  task_type='RETURN_TO_ROOM',orders=[])
    workflow_state=dict(state='awaiting_action',source_text='配送机器人联动：洗衣房净衣装舱',
                        current=dict(location='取衣处',task='放衣服进配送机器人'))
    class Workflow:
        def snapshot(self):return dict(workflow_state)
        def complete_action(self):
            workflow_state.update(state='preparing',current=dict(location='充电桩2',task='充电'))
            return 'next',dict(location='取衣处',task='放衣服进配送机器人'),workflow_state['current']
    calls=[]
    monitor=NS(snapshot=lambda:dict(delivery),request_close=lambda expected:(calls.append(expected) or (True,'关门成功')))
    async def owner(payload):return payload
    async def start():calls.append('charge');return '正在前往充电桩2'
    r=NS(_task_workflow=Workflow(),_delivery_handoff=monitor,
         _delivery_offer_lock=threading.Lock(),_delivery_close_required=None,
         _delivery_offer=None,_delivery_chain_note=None,_showroom_single_task_active=False,
         _speak_task=lambda *_:None,_operator_payload=owner,_start_current_task_navigation=start,
         get_logger=lambda:NS(warning=lambda *_:None))
    r._delivery_state_payload=lambda:B._delivery_state_payload(r)
    r._api_delivery_handoff_open=lambda req,action='open':B._api_delivery_handoff_open(r,req,action)
    response=asyncio.run(B._api_tasks_complete(r,None))
    assert response.status==200 and r._delivery_close_required
    assert calls==[] and workflow_state['current']['location']=='充电桩2'
    expected={key:delivery[key] for key in ('order_id','task_id','door_id')}
    response=asyncio.run(B._api_delivery_handoff_close(r,{**expected,'confirmed':True}))
    assert response.status==200 and calls==[expected,'charge']
    assert r._delivery_close_required is None
