"""S3 head mapping and neutral-reset geometry, no hardware control."""
import math
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory
from openarmx_teleop_vr_306_v5.teleop_core import HeadMapper, HeadSample
from openarmx_teleop_vr_306_v5.kinematics import IndependentArmKinematics

def mapper():
    return HeadMapper(filter_alpha=1.,maximum_step_deg=[100,100,100],
        axis_gain=[0.,-1.25,1.25],maximum_offset_deg=[18,40,55],
        joint_minimum_deg=[-18,-12,-55],joint_maximum_deg=[18,12,55],
        tracking_mode='relative_neutral_quaternion',neutral_neck_deg=[0,0,0])

def sample(x_deg):
    angle=math.radians(x_deg)/2
    return HeadSample(rotation=np.array([x_deg,0,0],float),
                      quaternion=np.array([math.sin(angle),0,0,math.cos(angle)]))

def test_looking_down_commands_robot_down_and_pitch_is_bounded():
    m=mapper();m.latch(sample(0),[0,0,0])
    assert m.map(sample(-4))[1]==pytest.approx(5)
    assert m.map(sample(-30))[1]==pytest.approx(12)
    m.map(sample(0))  # Respect the 35-degree reference-jump protection.
    assert m.map(sample(30))[1]==pytest.approx(-12)
    assert m.map(sample(0))[1]==pytest.approx(0)
    # URDF neck axis is +Y: rotating robot forward (+X) about +Y turns it
    # toward negative Z, i.e. the same downward direction as WebXR -X.
    pitch=math.radians(5);assert -math.sin(pitch)<0

def test_s3_model_head_return_is_collision_free_over_sampled_yaw_grid():
    share=get_package_share_directory('openarmx_teleop_vr_306_v5')
    ik=IndependentArmKinematics(share+'/urdf/robot_v3_1_simplified.urdf',
                                 share+'/urdf/robot_v3_1.srdf',collision_check=True)
    assert ik.collision_available
    groups=dict(leg_waist=[0,0,0,0],left_arm=[20,0,0,-110,0,0,0],
                right_arm=[20,0,0,-110,0,0,0],neck=[0,0,0])
    for yaw in range(-55,56,5):
        for pitch in range(-12,13,3):
            groups['neck']=[0,pitch,yaw]
            pairs=ik.collision_pairs(ik.q_from_feedback(groups,clip=False))
            assert not pairs,(pitch,yaw,pairs)
