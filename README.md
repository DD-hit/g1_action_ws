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
bash scripts/g1.sh run handshake --arm right --hold-s 2
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

首次在 `1 s` 预算内获取一只手的新鲜稳定 RGB-D 后锁定目标（超时则失败，不接管控制权）；后续不再依赖相机。位置为真实 3D 手腕沿四指根方向偏移 `0.06 m`，橡胶手中心对齐目标，四指朝前上方 `20°`，到位保持 `2 s`，返回初始位姿后归还控制权。接近与返回请求比例均为 `speed=0.45`、`acceleration=0.35`，并复用电梯第一阶段的关节计时预算。

握手使用驱动公共规划：固定候选 IK + PTP 优先，必要时允许有限 OMPL 绕行，并限制路径长度。识别目标规划仍失败时，尝试同侧固定备用握手位姿；不换臂，备用也经过同样的规划和检查。未识别出稳定左右手、驱动故障、急停、取消或运动中异常不会触发备用运动。

- `src/g1_actions/config/robot.yaml`：公共双臂驱动参数，保留已验证数值。
- `src/g1_actions/config/elevator.yaml`：电梯检测、按压与速度参数。
- `src/g1_actions/config/handshake.yaml`：握手感知、工具中心、固定姿态和备用位置。
- `src/g1_actions/config/actions.yaml`：动作注册信息。

备用位置位于 `execution.fixed_fallback`，以 `torso_link` 为参考，单位为 m，表示橡胶手中心：左 `[0.30, 0.20, 0.10]`，右 `[0.30, -0.20, 0.10]`。这是临时候选值，尚未经过真机验证，后续现场确定。

驱动先检查并缓存完整轨迹，再接管控制权执行原缓存，不重新选关节姿态。返回阶段也先规划再执行。缓存有效期为 `15 s`；请求、起点或场景变化，以及取消，都会使旧缓存失效。实测起点偏移上限为 `0.035 rad`，覆盖双臂、腰部及腿部碰撞上下文。

公共策略位于 `robot.yaml` 的 `pipelines[0].moveit`：`arm_transit_planner: ptp_first` 表示允许有限绕行，`ptp` 表示只用固定 PTP。当前 `arm_transit_require_preplan: true` 要求握手实际执行必须匹配已缓存的规划。电梯的面板、完整按压检查及原有速度策略继续保留。

## RGB-D 传输配置

所有生产入口默认加载 `src/g1_actions/config/cyclonedds.xml`，增大图像传输 socket 缓冲；保留 `RELIABLE`（可靠传输）及原来的图像时间戳、新鲜度阈值。已有 `CYCLONEDDS_URI` 时尊重外部配置。

默认设置 `ROS_LOCALHOST_ONLY=1`，使 ROS 2 相机、识别和动作通信仅在本机进行，隔离 RGB-D 流量与机器人 SDK 控制网络。Unitree SDK 仍使用配置中的 `domain_id: 0` 和 `eth0`，不受此 ROS 设置影响。需要跨机器访问 ROS 2 时，须在动作库后台与客户端启动前设置 `export G1_ROS_LOCALHOST_ONLY=0`（外部普通 ROS 节点设置 `ROS_LOCALHOST_ONLY=0`）；这会取消上述通信隔离，需要重新验证运动平滑性。项目使用独立开关，避免 ROS 环境初始化预设的 `ROS_LOCALHOST_ONLY=0` 覆盖默认隔离设置。修改后需重启已有后台。

机器人宿主机需设置并持久化缓冲上限（不是在每次启动时修改）：

```bash
printf 'net.core.rmem_max=16777216\nnet.core.wmem_max=16777216\n' | sudo tee /etc/sysctl.d/90-g1-rgbd-buffers.conf
sudo sysctl -p /etc/sysctl.d/90-g1-rgbd-buffers.conf
```

小缓冲可能造成大图像分片丢失、可靠传输重传和发布阻塞，最终触发 RGB-D 过期；无需放宽 `max_frame_age: 0.6`。

## 入口与日志

`serve` 同时补齐驱动、共享相机、电梯识别和握手识别；复用已存在且来源正确的组件。`stop` 只停止动作，`shutdown` 请求停止动作后再关闭本服务启动的组件；外部启动的组件不由本服务关闭。`shutdown` 的响应表示已接受关闭请求，实际退出看后台终端。

`up/down`、`handshake`、`handshake-serve` 仍为兼容别名。旧 `handshake-approach --execute` 转交统一动作接口，诊断 `--output` 不再决定生产执行日志位置。

统一日志位于 `data/runtime/server_<时间>/`。控制台按“时间、动作、请求 ID、阶段”输出，并显示阶段耗时、累计耗时和最终控制权状态；`action_events.jsonl` 保留完整结构化记录，握手每次执行记录在 `executions/handshake_*/trial/`。`status` 中的“上次任务结果”是历史记录，不代表又执行了一次。

握手运动在常驻服务工作线程内执行，复用已有 ROS 2 连接；真实 29 关节起点、目标和完整轨迹由驱动统一预检，不再每次启动 SDK 读取及模型检查子进程。RGB-D GPU 识别仍独立运行。生产感知不逐帧写图片和压缩深度文件，无图像订阅者时也不绘制骨骼；诊断 `debug handshake live --record-rgbd` 保留录制能力。

调试工具统一使用 `debug`，不与生产启动命令混排：

```bash
bash scripts/g1.sh debug handshake live --duration 20 --record-rgbd
bash scripts/g1.sh debug handshake video /absolute/path/video.MOV
bash scripts/g1.sh debug handshake replay <录制目录>
bash scripts/g1.sh debug handshake preflight <RGB-D目录> --arm right --output <新结果目录>
```

握手依赖的虚拟环境属于本项目 `.venv`。首次运行 `bash scripts/setup_handshake_video.sh`。原生预检工具仅用于离线诊断，构建方法见 [握手坐标说明](docs/HANDSHAKE_TARGET_GEOMETRY.md)。录像和只读感知详见 [RGB-D 说明](docs/HANDSHAKE_RGBD.md)。

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
