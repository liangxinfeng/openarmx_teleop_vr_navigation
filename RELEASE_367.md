# 367 VR 遥操导航

VR 导航启动 367 的 V5 手臂遥操、导航桥和 2D 导航栈。生效的桌面与栈启动器在 deployment/367 中。

这是原机环境的源码快照。SOURCE_367.json 列出每个机器人文件的来源、SHA-256 和权限。根目录采用生效文件，存在不同底稿时保存在 deployment/367/base_sources。deployment/367 保存启动所需的原机文件布局；Desktop、systemd/user、home/.ros 分别对应用户目录的桌面、用户服务和 ROS 配置。启动器中的绝对路径对应原机工作区 /home/ubuntu/migration367_20261118。系统 ROS Jazzy、原厂 SDK、服务、驱动以及已编译工作区由目标机器提供。

同批配套分支

- [liangxinfeng/autolife_robot_control_center](https://github.com/liangxinfeng/autolife_robot_control_center/tree/367)
- [liangxinfeng/control_center_navigation](https://github.com/liangxinfeng/control_center_navigation/tree/367)
- [liangxinfeng/control_center_slam](https://github.com/liangxinfeng/control_center_slam/tree/367)
- [liangxinfeng/openarmx_teleop_vr](https://github.com/liangxinfeng/openarmx_teleop_vr/tree/367)
- [liangxinfeng/openarmx_teleop_vr_navigation](https://github.com/liangxinfeng/openarmx_teleop_vr_navigation/tree/367)

原机 ROS 索引和启动入口的附加清单见 SOURCE_367_OVERLAY.json。符号链接按原机 Linux 目标保存，其中 SHA-256 校验链接目标文本。五个仓库的 deployment/367 按路径合并后对应同一套发布布局。
