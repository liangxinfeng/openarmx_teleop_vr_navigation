import pytest
from test_task_head_pose import controller, command
from test_task_height_feedback import controller_status
from openarmx_teleop_vr_navigation_306.navigation_web_bridge import NavigationVrWebBridge as Bridge

def test_s3_task_30_degrees_commands_downward_positive_pitch():
    c=controller()
    command(c,30)
    assert c._task_head_target[1] == pytest.approx(30)
    assert list(c._task_head_target[[0,2]]) == [3,4]

def test_s3_arrival_accepts_down_and_rejects_up():
    data=controller_status(.15)
    data['backend']['task_head_goal']={
        'request_id':'s3','requested_deg':30.,'effective_deg':30.,'safe_max_deg':43.}
    data['backend']['measured_neck_deg']=[0,30,0]
    assert Bridge._task_head_ready(data,True,30,'s3')
    data['backend']['measured_neck_deg'][1]=-30
    assert not Bridge._task_head_ready(data,True,30,'s3')

@pytest.mark.parametrize('angle,expected', [(0,0),(10,10),(30,30),(45,42.5)])
def test_real_s3_limits_and_feedback_direction(angle,expected):
    import numpy as np
    from ament_index_python.packages import get_package_share_directory
    from openarmx_teleop_vr_306_v5.kinematics import IndependentArmKinematics
    share=get_package_share_directory('openarmx_teleop_vr_306_v5')
    c=controller()
    c._kinematics=IndependentArmKinematics(share+'/urdf/robot_v3_1_simplified.urdf',
        collision_check=False,limit_margin_rad=np.deg2rad(2.0))
    command(c,angle,'model')
    actual=float(c._task_head_target[1])
    assert actual==pytest.approx(expected,abs=.001)
    assert c._task_head_goal['effective_deg']==pytest.approx(actual)
    assert c._task_head_goal['safe_max_deg']==pytest.approx(42.5,abs=.001)
    data=controller_status(.15)
    data['backend']['task_head_goal']=dict(c._task_head_goal)
    data['backend']['measured_neck_deg']=[3,actual,4]
    assert Bridge._task_head_ready(data,True,angle,'model')
    assert not Bridge._task_head_ready(data,False,angle,'model')
    data['backend']['measured_neck_deg'][1]=-30 if angle==30 else actual+2
    assert not Bridge._task_head_ready(data,True,angle,'model')
