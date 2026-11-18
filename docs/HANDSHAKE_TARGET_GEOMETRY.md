> 当前统一入口：`bash scripts/g1.sh serve` 启动公共后台，`bash scripts/g1.sh run handshake --arm auto` 执行。旧 `handshake-serve` 是别名，旧 `handshake-approach --execute` 会转交统一调度。自动选臂按识别左右手同侧匹配，预检失败可尝试同侧固定备用位姿。生产日志位于 `data/runtime`，下述诊断命令用于只读预览和离线检查。

# 握手目标坐标转换

目标预览与离线预检查不接管控制权，也不调用运动接口。一次性诊断另有显式 `--execute` 入口。

## 工具中心候选点

电梯动作的工具点是中指尖，握手不能直接复用。握手使用橡胶手掌面中心候选点；它不是网格质心，也不是整只手包围盒中心。

在原橡胶手网格的 `x=0.04 m, z=0` 处沿 Y 轴与三角形求交，左手取 -Y 掌面、右手取 +Y 掌面。这个 X/Z 锚点是手掌主体中央的初始选点，尚未证明是实际握手接触中心；需要核对实物。配置保留 `source: mesh_palm_surface_candidate` 和 `verified: false`。

| 坐标系 | 左手 XYZ / m | 右手 XYZ / m |
|---|---|---|
| rubber_hand | [0.04, -0.00880175, 0] | [0.04, 0.00880514, 0] |
| wrist_yaw_link | [0.0815, -0.00580175, 0] | [0.0815, 0.00580514, 0] |

腕部到手掌的安装平移来自原 URDF：左 [0.0415, 0.003, 0]、右 [0.0415, -0.003, 0]；固定旋转均为零。没有修改机器人模型或电梯配置。

## 相机变换来源

`T_torso_optical = T_torso_mount × T_mount_camera_link × T_camera_link_optical`。

- 第一项来自原 URDF 的固定 `d435_joint`。
- 第二项使用单位变换，沿用电梯动作已使用的机身到安装坐标系约定。
- 第三项读取当前相机的 `/tf_static`，保留彩色成像器的内部平移和旋转；不是只套光学坐标轴旋转。原始采集保存在机器人项目 `data/handshake/calibration/camera_tf_20261117.json`。

实测发布的 `camera_link → camera_color_optical_frame` 平移约为 [-0.000298, 0.014808, 0.000140] m。用户已确认相机外参在电梯按压业务中验证过，因此本动作复用同一条 URDF 安装变换与相机内部 TF 链路，设置 `verified: true`、`validation_source: user_confirmed_elevator_operation`；这不是本阶段新增的外参标定。电梯业务的目标补偿不作为相机外参复制到握手。

## 从视觉位置到腕部目标

先将估计人手掌心从相机坐标系转换到 `torso_link`。按用户要求，`execution.precontact_distance_m` 当前为 `0.0`，取消原先 10 cm 的退让，直接对准人手估计掌心。非零时才沿躯干 -X 退让，不改变高度。

`目标掌心 = 人手掌心 + [-precontact_distance_m, 0, 0]`

`腕部目标 = 目标掌心 - 固定姿态旋转 × 腕部到橡胶手掌心偏移`

因此规划器最终使用的是腕部目标，不能把人手掌心 XYZ 直接当作腕部 XYZ。前方间距只是几何偏移，不等于已验证完整橡胶手或机械臂与人的安全距离。

## 离线预览

从项目根目录运行：

```bash
bash scripts/g1.sh handshake-target-preview data/handshake/rgbd/handshake_record_20261117_01 --arm right --output data/handshake/target_previews/right_01
```

输出 `targets.jsonl` 和 `summary.json`，含人手躯干坐标、预备掌心坐标、腕部位置和固定四元数。使用原录制的关键点和真实深度，不需要重新开相机或启动机械臂驱动。输入也可以是单个 RGB-D `.npz` 快照。

预览处理每个可估计位置，没有替调用方确认手的选择或连续稳定性，也没有检查 IK、关节限位、碰撞或工作空间。输出始终 `executable: false`。`require_verified=True` 会检查相机安装、工具中心和固定姿态三项验证标记；该参数只用于几何函数，不提供运动能力。

实际运动前仍需核对工具中心和固定姿态，并检查目标是否可达及无碰撞；相机外参按用户确认复用电梯已验证链路。

## MoveIt 离线预检查

以下是只读 PTP 抽样诊断工具，不是生产握手的目标规划器。生产仅用它检查真实 29 关节起点（内部采样数为 `0`），目标求解、有限绕行及缓存执行统一由驱动负责。

```bash
bash scripts/g1.sh handshake-preflight data/handshake/rgbd/handshake_record_20261117_01 --arm right --samples 8 --output data/handshake/preflight/right_01
```

脚本将本项目的独立检查程序编译到 `/workspace/build/handshake_preflight`。使用现有 MoveIt、KDL、Pilz 和驱动 IK 种子头文件；不修改或重新编译驱动，不链接 Unitree SDK，不创建控制权、运动或轨迹执行接口。ROS 域固定为 `211`，仅限本机。

默认从有效目标中均匀抽样 8 个，初始姿态采用记录过的中立姿态，并明确标记为非真机当前姿态。按固定顺序尝试 IK 种子，检查限位、正运动学误差、自碰撞；找到固定关节目标后使用 Pilz PTP，按最多 0.02 rad 关节间隔插值复核路径。不使用 OMPL 绕行。KDL 的数值求解不代表穷尽了所有 IK 解；搜索未找到不能证明目标在数学上绝对不可达。

输出 `preflight.json`、`preflight.yaml` 和 `planning.log`，含通过数量、失败原因、碰撞连杆与关节目标。场景只包含机器人模型，没有建模人、面板和周围障碍；不应将通过结果作为真机运动授权。

## 读取真机当前关节状态

```bash
bash scripts/g1.sh handshake-read-state --output data/handshake/state_captures/read_01
bash scripts/g1.sh handshake-preflight data/handshake/rgbd/handshake_record_20261117_01 --arm right --samples 8 --state data/handshake/state_captures/read_01/state.json --output data/handshake/preflight/right_actual_01
```

最终读状态程序直接使用仅含 `ChannelSubscriber` 的独立 SDK 辅助程序订阅 `rt/lowstate`，获取完整 29 个关节后退出。不启动机械臂驱动，不创建 `ChannelPublisher`，不调用控制权或运动接口。电机索引采用官方 G1_29 顺序，并与模型关节集合、已有驱动的上半身 SDK 映射交叉校验。辅助程序及日志保存在共享 build 和本次数据目录，不覆盖电梯配置。开发期间曾用禁用运动的临时驱动采集上半身快照，该快照仅用于诊断。

`--state` 使用保存的真实关节值，`full_body_state: true` 时要求模型全部关节都有测量数据；早期只有上半身的快照仍兼容，但明确标记腿部为默认值。状态快照及录制目标仅用于离线检查，不能作为日后执行时的实时状态。若起点超限或自碰撞，检查立即失败并报告具体关节或连杆，不自动放宽限制或移动手臂。

## 2026-11-17 检查记录

原录制目标共 84 个，使用记录的中立起点均匀抽样 8 个：右手 4 个、左手 2 个通过固定关节目标的 Pilz PTP 及路径复核。未通过的目标包含身体中线附近的自碰撞分支，以及固定种子搜索未找到 IK 的较远目标；这些抽样不能推广为全工作空间结论。

随后通过订阅器采集完整 29 关节。真实快照在模型中被起点碰撞检查拒绝，涉及左右腕部与髋部；没有继续生成或执行真机运动。用户明确确认当时机器人尚未正常站立／仍在吊装，因此该快照仅作吊装状态诊断，不作为正常站立起点，不据此修改限位、碰撞矩阵或固定 20° 姿态。

正常站立后需要重新采集状态和新鲜手部目标。此前的状态、目标及离线通过结果都不能直接用于执行。

零偏移时，显式执行可能使橡胶手与人手接近或接触，不能再称为非接触测试。保留工具中心换算、固定 20° 姿态、真实深度和 IK／碰撞检查；模型候选工具点尚不等于精确标定的接触点。

## 启动握手测试后台

系统重启后，需要先恢复驱动、相机和手部识别。在终端 1 执行并保持运行：

```bash
cd /workspace/src/g1_action_ws
bash scripts/g1.sh handshake-serve
```

显示“后台就绪”后，在终端 2 放入一只手并调用 `handshake-approach`。后台仅启动依赖，不接管控制权、不发起运动。每次任务首次锁定仍需要相机；锁定后不再依赖视觉。

后台复用当前工作区、同一 ROS 域的现有进程，避免重复占用相机；遇到不同来源或重复的驱动会拒绝启动。统一后台日志保存在 `data/runtime/server_<时间>/`。子进程退出时会显示具体日志；关闭时只清理本次启动的进程，且等待活动动作结束和控制权归还后才关闭自己启动的驱动。

任务等待目标时会分别报告驱动和视觉原因，包括未收到驱动反馈、未收到识别状态、图像过期、未检测到手、深度无效或等待稳定，不再只显示笼统超时。

## 一次锁定目标，再执行固定动作

当前 `handshake-approach` 流程：

1. 等待新鲜 RGB-D、有效深度及稳定的唯一人手目标，图像年龄仍须在 0.6 s 内。
2. 保存不可修改的位置快照和原始时间戳，取消本任务对视觉状态的订阅，打印 `TARGET_LOCKED`。
3. 用该快照计算前上方 20° 的腕部目标，读取实时全身关节验证起点；再由驱动公共流程做固定候选 IK、PTP 优先／有限绕行及路径检查，缓存通过的轨迹。
4. 接管控制权，核对实测双臂／腰部起点和场景未改变后，执行原缓存轨迹，不重新规划。
5. 到位后停留 `execution.hold_s: 2.0` 秒，记录 `HOLD` 和 `HOLD_COMPLETED`。
6. 向驱动预规划返回起始位置的轨迹，检查通过后执行原缓存，再释放控制权。

锁定后不重新识别人手、不刷新目标，也不依据断流、图像年龄或人手位置变化中止动作。人手离开时机器人仍会完成已锁定目标的运动，因此执行期间人手需要保持在锁定位置。驱动反馈的新鲜度、SDK 连接、急停、驱动错误、控制权及限位／碰撞检查仍保留；失败时仍取消运动并尝试释放控制权。

此修改移除了早期持续视觉监控的 3 cm 位移判定和图像超时等待规则，不再输出 `VISION_UPDATE_PENDING`／`VISION_UPDATE_RECOVERED`。取消的是本任务的视觉订阅，不主动关闭其他动作可能共享的相机或识别进程。下一次任务仍需重新获取新鲜目标。

测试命令保持不变：

```bash
bash scripts/g1.sh handshake-approach --arm right --execute --output "data/handshake/approach_trials/right_$(date +%Y%m%d_%H%M%S)"
```

当前额外距离偏移为 0，实际执行可能接触人手。本修改仅经过隔离测试验证，完整真机流程需实际运行后确认。
