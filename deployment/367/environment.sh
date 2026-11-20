# 仅本迁移包加载；不修改登录环境或原厂服务。
export AUTOLIFE_ROBOT_MODEL=autolife_s1
export AUTOLIFE_ROBOT_VERSION=robot_v3_1
export AUTOLIFE_CONFIG='{
  "sdk": "/home/ubuntu/autolife_config_v3",
  "arm": "/home/ubuntu/autolife_config_v3",
  "gv": "/home/ubuntu/autolife_config_v3",
  "media": "/home/ubuntu/autolife_config_v3",
  "network": "/home/ubuntu/autolife_config_v3",
  "kiosk": "/home/ubuntu/autolife_config_v3",
  "ai-chat": "/home/ubuntu/autolife_config_v3",
  "service-center": "/home/ubuntu/autolife_config_v3",
  "face-detection": "/home/ubuntu/autolife_config_v3",
  "chat-motion": "/home/ubuntu/autolife_config_v3",
  "diagnostics": "/home/ubuntu/autolife_config_v3",
  "toolbox": "/home/ubuntu/autolife_config_v3",
  "flow": "/home/ubuntu/autolife_config_v3"
}'
export AUTOLIFE_CONFIG_DIR=/home/ubuntu/autolife_config_v3
export AUTOLIFE_SDK_CONFIG_DIR=/home/ubuntu/autolife_config_v3
export AUTOLIFE_NATIVE_LIB_DIR=/home/ubuntu/autolife-v3/lib
export AUTOLIFE_INTERFACE_TYPE=real
export AUTOLIFE_SDK_ENVIRONMENT=/opt/autolife/sdk-environment/ubuntu24-x86_64-f8e0e56ebe7a994e/manifest.json
export AUTOLIFE_REALSENSE_RUNTIME_DIR=/opt/autolife/sdk-environment/ubuntu24-x86_64-f8e0e56ebe7a994e/realsense_cpu/lib
export AUTOLIFE_GSTREAMER_ROOT=/opt/autolife/gstreamer/1.28.8-r5
export LD_LIBRARY_PATH=/home/ubuntu/autolife-v3/lib/ros/lib:/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib:/opt/autolife/gstreamer/1.28.8-r5/lib:/opt/autolife/gstreamer/1.28.8-r5/deps/lib:/opt/autolife/sdk-environment/ubuntu24-x86_64-f8e0e56ebe7a994e/realsense_cpu/lib:/opt/autolife/sdk-environment/ubuntu24-x86_64-f8e0e56ebe7a994e/orbbec/lib
export PYTHONPATH=/home/ubuntu/autolife-v3/lib/ros/lib/python3.12/site-packages:/opt/ros/jazzy/opt/gz_sim_vendor/lib/python:/opt/ros/jazzy/opt/sdformat_vendor/lib/python:/opt/ros/jazzy/opt/gz_transport_vendor/lib/python:/opt/ros/jazzy/opt/gz_msgs_vendor/lib/python:/opt/ros/jazzy/opt/gz_math_vendor/lib/python:/opt/ros/jazzy/lib/python3.12/site-packages:/opt/autolife/gstreamer/1.28.8-r5/lib/python3.12/site-packages
export ROBOT_ID=367
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI=file:///home/ubuntu/autolife-v3/config/cyclonedds.xml
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/autolife-v3/lib/ros/local_setup.bash
source /home/ubuntu/migration367_20261118/install/local_setup.bash
export PYTHONPATH=/home/ubuntu/migration367_20261118:$PYTHONPATH
export PATH=/home/ubuntu/migration367_20261118/runtime/bin:$PATH
export AUTOLIFE_CAMERA_PROTOCOL=ALSHM001
export AUTOLIFE_TTS_CONFIG=/home/ubuntu/migration367_20261118/tts_matcha.yaml

# 原厂Python链接OpenSSL 3.3+；避免rclpy先加载系统3.0造成导入顺序相关故障。
export LD_LIBRARY_PATH=/home/ubuntu/migration367_20261118/runtime_native_libs:$LD_LIBRARY_PATH

if [[ -r "/home/ubuntu/migration367_20261118/releases/control-center-current/overlay.sh" ]]; then source "/home/ubuntu/migration367_20261118/releases/control-center-current/overlay.sh"; fi
