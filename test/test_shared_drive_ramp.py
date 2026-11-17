"""Production bridge callbacks, fake publishers: no node/robot instantiated."""
from types import SimpleNamespace as NS,MethodType
import threading
import pytest
from control_center_chassis_guard import VelocityRamp
from openarmx_teleop_vr_navigation_306 import navigation_web_bridge as mod
def subject(monkeypatch):
 clock=[100.];monkeypatch.setattr(mod.time,'monotonic',lambda:clock[0]);published=[]
 c=NS(_exit_in_progress=False,_control_peer='headset',_drive_was_active=False,
      _drive_current=[0.,0.,0.],_last_drive_tick=100.,_drive_time=100.,_drive_settle_until=0.,
      _drive_lock=threading.Lock(),_drive_sample=dict(deadman=True,pause_for_menu=False,forward=1.,lateral=0.,turn=0.),
      _drive_ramp=VelocityRamp(clock=lambda:clock[0]),_navigation_paused_for_drive=False)
 c.get_parameter=lambda n:NS(value={'drive_watchdog_seconds':.35,'maximum_forward_speed':.4,'maximum_lateral_speed':.3,'maximum_yaw_speed':.7}[n])
 c._publish_manual_override=lambda v:None;c._pause_for_drive=lambda:None;c._navigation_is_active=lambda:False
 c._drive_publisher=NS(publish=lambda msg:published.append((msg.linear.x,msg.linear.y,msg.angular.z)))
 c._publish_stop=MethodType(mod.NavigationVrWebBridge._publish_stop,c)
 def tick(fresh=True):
  clock[0]+=.02
  if fresh:c._drive_time=clock[0]
  mod.NavigationVrWebBridge._drive_tick(c)
 return c,published,tick,clock
def test_start_release_and_reverse_match_control_center(monkeypatch):
 c,out,tick,clock=subject(monkeypatch);reference=VelocityRamp(clock=lambda:clock[0])
 for target in [1.]*60+[0.]*60+[-1.]*60+[1.]*90:
  c._drive_sample['forward']=target;tick()
  reference.set_target(target*.4,0,0)
  assert out[-1]==pytest.approx(reference.step(clock[0]))
 assert out[0][0] < .02
 assert 0<out[60][0]<out[59][0]  # release starts deceleration, not a hard stop
 assert out[119][0]==0
@pytest.mark.parametrize('fault',['stale','disconnect','menu','release','exit'])
def test_faults_and_explicit_release_stop_without_a_ramp_tail(monkeypatch,fault):
 c,out,tick,clock=subject(monkeypatch)
 for _ in range(40):tick()
 if fault=='stale':clock[0]+=.4
 elif fault=='disconnect':c._control_peer=None
 elif fault=='menu':c._drive_sample['pause_for_menu']=True
 elif fault=='release':c._drive_sample['deadman']=False
 elif fault=='exit':c._exit_in_progress=True
 tick(fresh=fault!='stale')
 assert out[-1]==(0.,0.,0.) and c._drive_ramp.idle
