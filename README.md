# G1 动作库

电梯按压与握手共用一个后台、一个调度器和一套驱动。同一时刻只执行一个动作，命令默认等待收尾后的最终结果。

## 编译

首次构建或驱动更新后，先用驱动原脚本自动编译驱动与依赖：

```bash
bash /workspace/src/brsd_manipulation/src/brsd_upbody_control/script/build.sh \
  unitree_g1 -DCMAKE_BUILD_TYPE=Release
```

再编译动作库；仅修改业务代码时执行这一步即可：

```bash
cd /workspace/src/g1_action_ws
bash scripts/build.sh
```

动作库脚本只编译 `g1_actions` 和 `g1_action_interfaces`。产物统一位于 `/workspace/build`、`/workspace/install`、`/workspace/log`。

## 脚本测试

终端 1 启动服务，保持运行；`Ctrl+C` 退出并清理本次启动的组件：

```bash
cd /workspace/src/g1_action_ws
bash scripts/g1.sh serve
```

终端 2 调用动作；以下 `run` 命令会驱动真机运动：

```bash
cd /workspace/src/g1_action_ws
bash scripts/g1.sh list                 # 查看动作和就绪状态
bash scripts/g1.sh run elevator --direction up
bash scripts/g1.sh run elevator --direction down
bash scripts/g1.sh run handshake --arm auto  # 左人手→左臂，右人手→右臂
bash scripts/g1.sh run handshake --arm right --hold-s 4
bash scripts/g1.sh status               # 查看当前状态和最近结果
bash scripts/g1.sh stop                 # 停止当前动作，等待最终收尾结果
bash scripts/g1.sh shutdown             # 请求收尾并关闭本后台所属组件
```

需要阶段反馈、JSON 结果或查询历史任务时：

```bash
bash scripts/g1.sh run elevator --direction up --arm right --feedback
bash scripts/g1.sh up --json
bash scripts/g1.sh get <请求ID> --json
```

命令默认等待最终结果；成功表示运动、收尾完成且控制权已归还，不代表已验证按钮点亮。
退出码：`0` 成功，`1` 任务失败或取消，`2` 客户端调用异常。`--json` 返回一份最终结果，`--feedback` 显示阶段进度。

不连接真机的接口自测：

```bash
cd /workspace/src/g1_action_ws
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
python3 src/g1_actions/test/test_unified_actions.py -v
```

## 握手规则与配置

默认 `--arm auto` 使用稳定的人手左右识别标签，不按图像左右或目标所在侧猜测。显式 `--arm left/right` 可以覆盖选择。`--subject-handedness left/right` 仅筛选识别目标，不代表机器人手臂。

首次获取一只手的新鲜稳定 RGB-D 后锁定目标；后续不再依赖相机。位置为真实 3D 手腕沿四指根方向偏移 `0.06 m`，橡胶手中心对齐目标，四指朝前下方 `45°`，到位保持 `4 s`，返回初始位姿后归还控制权。

目标的可达性、限位或碰撞预检失败时，尝试同侧固定备用握手位姿；不换臂、不使用随机规划。备用位姿仍要经过完整预检。未识别出稳定左右手、驱动故障、急停、取消或运动中异常不会触发备用运动。

- `src/g1_actions/config/robot.yaml`：公共双臂驱动参数，保留已验证数值。
- `src/g1_actions/config/elevator.yaml`：电梯检测、按压与速度参数。
- `src/g1_actions/config/handshake.yaml`：握手感知、工具中心、固定姿态和备用位置。
- `src/g1_actions/config/actions.yaml`：动作注册信息。

备用位置位于 `execution.fixed_fallback`，以 `torso_link` 为参考，单位为 m，表示橡胶手中心：左 `[0.30, 0.20, 0.10]`，右 `[0.30, -0.20, 0.10]`。这是临时候选值，尚未经过真机验证，后续现场确定。

## 入口与日志

`serve` 同时补齐驱动、共享相机、电梯识别和握手识别；复用已存在且来源正确的组件。`stop` 只停止动作，`shutdown` 请求停止动作后再关闭本服务启动的组件；外部启动的组件不由本服务关闭。`shutdown` 的响应表示已接受关闭请求，实际退出看后台终端。

`up/down`、`handshake`、`handshake-serve` 仍为兼容别名。旧 `handshake-approach --execute` 转交统一动作接口，诊断 `--output` 不再决定生产执行日志位置。

统一日志位于 `data/runtime/server_<时间>/`，`action_events.jsonl` 使用请求 ID 关联阶段及最终结果；握手子进程原始日志和预检报告在 `executions/handshake_*/`。

调试工具统一使用 `debug`，不与生产启动命令混排：

```bash
bash scripts/g1.sh debug handshake live --duration 20 --record-rgbd
bash scripts/g1.sh debug handshake video /absolute/path/video.MOV
bash scripts/g1.sh debug handshake replay <录制目录>
bash scripts/g1.sh debug handshake preflight <RGB-D目录> --arm right --output <新结果目录>
```

握手依赖的虚拟环境属于本项目 `.venv`。首次运行 `bash scripts/setup_handshake_video.sh`，并按 [握手坐标说明](docs/HANDSHAKE_TARGET_GEOMETRY.md) 构建原生预检工具。录像和只读感知详见 [RGB-D 说明](docs/HANDSHAKE_RGBD.md)。

## 接口调用

先启动上述服务。在调用 ROS 2 接口的终端加载环境：

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

执行上行按压，每次新任务生成新的请求 ID：

```bash
request_id=$(python3 -c 'import uuid; print(uuid.uuid4())')
ros2 action send_goal /g1_actions/execute g1_action_interfaces/action/Execute \
  "{request: {api_version: 1, request_id: '${request_id}', action_name: elevator, arm: 0, timeout_s: 60.0, parameters_json: '{\"direction\":\"up\"}'}}" \
  --feedback
```

`direction` 可改为 `down`；`arm` 为 `0` 自动、`1` 左手、`2` 右手。

```bash
# 查询动作列表
ros2 service call /g1_actions/list_actions g1_action_interfaces/srv/ListActions '{}'

# 请求停止当前任务；accepted 仅表示接受停止请求
ros2 service call /g1_actions/stop_active g1_action_interfaces/srv/StopActive '{}'

# 订阅状态及最近结果，确认任务终态和控制权
ros2 topic echo /g1_actions/status
```

按请求 ID 查询的接口为 `/g1_actions/get_execution`，类型为 `g1_action_interfaces/srv/GetExecution`。
完整字段、错误码及取消语义见 [接口规范](docs/API.md)。

握手测试先在一个终端运行 `bash scripts/g1.sh handshake-serve`，保持驱动、相机和识别后台运行；再在另一个终端调用 `handshake-approach`。系统重启后需重新启动后台，详见 [握手目标与测试](docs/HANDSHAKE_TARGET_GEOMETRY.md)。
