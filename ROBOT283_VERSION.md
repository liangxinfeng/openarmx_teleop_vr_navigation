# 283 当前版本 · 2026-11-18

本分支 `robot283-latest-20261118` 从283实机当前源目录导出，包含本机配置和未提交修改。

本包沿用283当前配置；VR导航依赖 openarmx_teleop_vr_306_v4 和 control_center_navigation。

## 配套版本
- [openarmx_teleop_vr_306_v4](https://github.com/liangxinfeng/openarmx_teleop_vr_v5/tree/robot283-latest-20261118)
- [openarmx_teleop_vr_navigation_306](https://github.com/liangxinfeng/openarmx_teleop_vr_navigation_306/tree/robot283-latest-20261118)
- [autolife_robot_control_center](https://github.com/liangxinfeng/autolife_robot_control_center/tree/robot283-latest-20261118)

## 校验与部署范围
- 源文件内容按 SHA256 与实机快照核对；详见 `ROBOT283_SOURCE_MANIFEST.json`。
- 本次是源码归档上传，未修改283运行配置、默认分支或其他机器人分支。
- 遥操及VR导航已在回退时完成原版一致性、编译及隔离无动作启动检查；上传不新增真机运动验收结论。
- 排除缓存、编译输出、地图历史备份、证书私钥。控制中心另排除文件名含反斜杠的旧残留副本，正常路径文件保留。
- 虚拟环境、ROS系统依赖、原厂SDK/驱动、个人证书与Home目录启动器不在此源码快照内；本分支不是完整离线安装器。
- 部署到283时应停止对应程序后再编译加载；迁移到其他编号需核对话题后缀、模型及复位配置。
