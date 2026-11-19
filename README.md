# G1 动作库

面向 Unitree G1 的电梯按压和握手动作库。两个动作同级，共用一个后台、一个调度器和 `brsd_manipulation` 提供的双臂驱动，同一时刻执行一个动作。

## 构建和测试

在机器人 ROS 2 Humble 容器内执行。首次构建或更新驱动后，先用驱动脚本构建驱动及依赖：

```bash
bash /workspace/src/brsd_manipulation/src/brsd_upbody_control/script/build.sh \
  unitree_g1 -DCMAKE_BUILD_TYPE=Release
```

再构建业务。只更新业务代码时，从这一步开始：

```bash
cd /workspace/src/g1_action_ws
bash scripts/build.sh
```

业务脚本只构建 `g1_actions` 和 `g1_action_interfaces`，共用 `/workspace` 下的 `build`、`install` 和 `log`。

终端 1 启动统一后台，保持运行：

```bash
cd /workspace/src/g1_action_ws
bash scripts/g1.sh serve
```

终端 2 查询就绪状态并选择一个动作执行。`run` 会驱动真机运动：

```bash
cd /workspace/src/g1_action_ws
bash scripts/g1.sh list
bash scripts/g1.sh run elevator --direction up
bash scripts/g1.sh run elevator --direction down
bash scripts/g1.sh run handshake --arm auto
```

电梯也可简写为 `bash scripts/g1.sh up` 或 `bash scripts/g1.sh down`。默认 ROS 2 通信限于本机，Unitree SDK 通过配置的网口通信。首次部署和跨机访问见 [部署与 RGB-D 通信](docs/DEPLOYMENT.md)。

不连接真机的接口自测：

```bash
cd /workspace/src/g1_action_ws
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
python3 src/g1_actions/test/test_unified_actions.py -v
```

## 动作与配置

- **电梯按压**用 `--direction up|down` 指定方向。识别到稳定按钮目标后，依次到达预备点、直线按压、沿记录的轨迹撤回并归还控制权。动作完成表示流程收尾成功，按钮是否点亮需另外确认。
- **握手**默认 `--arm auto`，按识别到的人手选择同侧机器人手臂，也可用 `--arm left|right` 指定。等待稳定目标的配置预算为 `0.9 s`，随后锁定目标，默认到位停留 `2 s`。若真实目标的几何或规划预检被拒绝，最多尝试三种姿态，保持掌心目标位置，再尝试经过预检的同侧固定备用位姿。未取得稳定目标时，本次失败且不接管控制权。

| 配置文件 | 用途 |
|---|---|
| [`elevator.yaml`](src/g1_actions/config/elevator.yaml) | 按钮识别、目标补偿、按压和撤回参数 |
| [`handshake.yaml`](src/g1_actions/config/handshake.yaml) | 手部识别、掌心目标、停留和备用位姿 |
| [`robot.yaml`](src/g1_actions/config/robot.yaml) | 公共双臂驱动与规划参数 |
| [`actions.yaml`](src/g1_actions/config/actions.yaml) | 动作注册信息 |

## 状态、停止和日志

| 命令（前缀 `bash scripts/g1.sh`） | 用途 |
|---|---|
| `list` | 查询动作及各自的就绪状态 |
| `status` | 查询服务状态、当前任务和上次任务结果。上次结果是历史记录 |
| `stop` | 停止当前任务并等待最终收尾结果 |
| `get <请求ID> --json` | 按请求 ID 查询执行记录 |
| `shutdown` | 请求收尾并关闭本后台启动的组件，以后台终端退出为准 |

`run` 默认等待最终结果。加 `--feedback` 可看阶段进度，加 `--json` 可取结构化结果。退出码 `0` 表示成功，`1` 表示任务失败或取消，`2` 表示客户端调用异常。

例如日志显示「失败阶段：直线按压」「原因：`PILZ_PATH_COLLISION`」「控制权：已归还」，表示按压直线规划失败，本次已完成控制权归还。用请求 ID 执行 `get`，或在后台本次运行目录 `data/runtime/server_<时间>/action_events.jsonl` 中查询阶段记录和耗时。

## ROS 2 接口与调试

统一执行接口为 `/g1_actions/execute`。在 `/workspace/src/g1_action_ws` 下启动后台后，调用示例：

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
export ROS_DOMAIN_ID=${G1_ROS_DOMAIN_ID:-${ELE_ROS_DOMAIN_ID:-1}}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source scripts/ros_transport.sh

request_id=$(python3 -c 'import uuid; print(uuid.uuid4())')
ros2 action send_goal /g1_actions/execute g1_action_interfaces/action/Execute \
  "{request: {api_version: 1, request_id: '${request_id}', action_name: elevator, arm: 0, timeout_s: 60.0, parameters_json: '{\"direction\":\"up\"}'}}" \
  --feedback
```

- [接口规范](docs/API.md)包含参数、错误码、取消语义和管理接口。
- [握手 RGB-D 调试](docs/HANDSHAKE_RGBD.md)和[握手目标几何](docs/HANDSHAKE_TARGET_GEOMETRY.md)包含录像、回放及离线预检。可用 `bash scripts/g1.sh help` 查看调试入口。
