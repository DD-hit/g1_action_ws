# 当前工作区布局与验证

各项目位于 `/workspace/src`；公共编译产物位于 `/workspace/build`、`/workspace/install`、`/workspace/log`。旧 `ele_ws` 已删除。

`g1_action_ws` 保持完整，内部 `src` 只有 `g1_actions`、`g1_action_interfaces`。唯一最新版驱动位于 `brsd_manipulation/src/brsd_upbody_control`。

双臂驱动构建及启动脚本与成功 fetch 的 `origin/fix/g1-gravity-compensation` 完全一致；使用原脚本自动构建依赖。基础库原脚本也已恢复到缓存的远端版本，基础库和消息库的最新 fetch 因缺少 Gitee 凭据未成功。

动作库 `scripts/build.sh` 只编译自己的两个包，加载公共安装环境，输出至公共目录。

## 配置

每个动作一个 YAML：`elevator.yaml`、`handshake.yaml`。`actions.yaml` 仅为公共动作注册表。
公共双臂驱动 pipelines 位于 `robot.yaml`，电梯业务位于 `elevator.yaml`，握手业务位于 `handshake.yaml`。从原电梯配置抽取公共驱动参数时逐项保留原值。

## 验证结果

- 基础依赖、双臂驱动、动作接口和动作业务全部编译通过。
- 5 组 CTest、18 项统一接口测试、4 项驱动来源测试、4 项进程生命周期测试全部通过。
- 合并配置后，真实 ROS 接口、C++ 规划/执行器、假 SDK 的上下行完整回放通过。
- JSON、取消、异常返回、互斥、释放后返回最终结果均由接口测试覆盖。
- 测试使用隔离 localhost ROS 域，未发送真机运动指令；测试程序退出后不保留服务。

完整日志位于 `data/validation/workspace_layout`。未进行 Git 提交或推送。
