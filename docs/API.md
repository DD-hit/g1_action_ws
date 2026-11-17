# G1 动作库接口规范 v1

状态：电梯与握手均接入统一执行服务端和 CLI，共用互斥、取消、超时和最终结果。
运行 `bash scripts/g1.sh serve` 后提供本文的统一接口。电梯兼容入口共用同一调度器。
编译、脚本测试和接口调用示例见根目录 `README.md`。

## 1. 包边界

- `g1_actions`：唯一业务包，`actions/elevator` 与 `actions/handshake` 同级。
- `g1_action_interfaces`：仅存放消息、服务、Action 定义，不是第三个业务动作。
- 两个动作共用机器人驱动和执行调度器，同时最多一个任务拥有控制权。
- 双臂驱动由 `brsd_manipulation` 提供，动作库不重复维护驱动源码。

## 2. 对外接口

| 名称 | 类型 | 用途 |
|---|---|---|
| `/g1_actions/execute` | `g1_action_interfaces/action/Execute` | 执行动作，持续反馈，返回最终结果，支持标准取消 |
| `/g1_actions/list_actions` | `g1_action_interfaces/srv/ListActions` | 查询动作、实现状态、就绪状态和参数 Schema |
| `/g1_actions/get_execution` | `g1_action_interfaces/srv/GetExecution` | 使用业务请求 ID 查询执行状态或最终结果 |
| `/g1_actions/stop_active` | `g1_action_interfaces/srv/StopActive` | 客户端丢失 GoalHandle 后请求停止，优先传明确请求 ID |
| `/g1_actions/shutdown` | `std_srvs/srv/Trigger` | 请求收尾并关闭统一后台；响应仅表示已接受，实际退出看后台终端 |
| `/g1_actions/status` | `g1_action_interfaces/msg/ServerStatus` | 面向所有观察者的服务状态及最近结果 |

ROS 2 Action 是长任务接口，不意味着业务命令立即返回成功。
CLI 默认等待最终结果；必须完成退出、收尾、控制权归还后才打印成功。
不另外提供容易混淆的“已接收即成功”的同步 Trigger 入口。

## 3. 执行请求

Action goal 的 `request` 类型为 `ExecutionGoal`。

| 字段 | 约定 |
|---|---|
| `api_version` | 固定为 `1`；不支持的版本返回 `UNSUPPORTED_VERSION` |
| `request_id` | 客户端生成的小写 UUID v4，与 ROS Action 自身的 goal UUID 分开 |
| `action_name` | `elevator` 或 `handshake`，区分大小写，不使用任意模块路径 |
| `arm` | `0=auto`、`1=left`、`2=right`；支持范围由 `ListActions` 声明 |
| `timeout_s` | 有限数，`1～120 s`，默认 `60 s`；主动工作阶段的总预算 |
| `parameters_json` | UTF-8 JSON 对象，最大 4096 字节；只接受对应 Schema 声明的字段 |

外层和内层参数不得重复定义 `arm`、超时或请求 ID。
参数 JSON 禁止重复键、NaN、Infinity、未知字段。Schema 的默认值由服务端显式补齐后再执行。
速度、关节限制、按压深度、相机外参和工具标定属于服务端配置，不允许调用方通过任意 JSON 覆盖。

电梯参数：`{"direction":"up"}` 或 `{"direction":"down"}`。
握手参数：`{"subject_handedness":"any","hold_s":4.0}`。
`subject_handedness` 是被识别的人手筛选条件，`arm` 是 G1 使用的手臂。握手 `arm=0` 时左人手对应左臂、右人手对应右臂，不按图像位置选臂；显式 `arm=1/2` 覆盖自动选择。
目标预检失败时，仅尝试同侧 `handshake.yaml` 中的固定备用位姿，备用也需通过 IK、限位、碰撞和 PTP 路径检查；不改用另一侧，不使用 OMPL。故障、急停、取消或运动中异常不触发备用动作。
握手保持时间从到达规定位置并确认停稳开始计时，范围 `0～10 s`，默认 `4 s`。
握手执行只使用新鲜实时 RGB-D，不接受录制视频或外部未经验证的位姿作为真机运动目标。

示例（先启动统一服务端）：

```bash
ros2 action send_goal /g1_actions/execute g1_action_interfaces/action/Execute \
  '{request: {api_version: 1, request_id: "de305d54-75b4-431b-adb2-eb6b9e546014", action_name: "elevator", arm: 2, timeout_s: 60.0, parameters_json: "{\"direction\":\"down\"}"}}' --feedback
```

## 4. 接收、互斥和重复请求

ROS 层接受 goal 只说明已建立结果通道，不等同于业务验证通过。
可反序列化的请求都通过结果通道返回结构化结果，包括 `BUSY`、参数错误和未就绪；
这些拒绝任务不接管控制权，不影响当前正在执行的任务。
消息本身无法反序列化等传输错误才表现为协议层拒绝或无响应。

不排队，不自动抢占。运行中收到另一个动作，立即返回 `BUSY`。
同一个 `request_id` 不得触发第二次执行；重复提交返回 `DUPLICATE_REQUEST`，客户端改用 `GetExecution` 查询。
同 ID 不同参数也拒绝，不静默覆盖；重复请求不取消原任务。
保存最近 100 条或 30 分钟内的终态记录，以先达到的限制为准；活跃记录不淘汰。
去重仅在本次服务进程生命周期和缓存有效期内保证；服务重启后禁止自动重试不确定是否已执行的动作。
未找到 ID 时 `found=false`，不能据此断言历史上从未执行。
`GetExecution.found=false` 时忽略 feedback/result；`terminal=false` 时只有 feedback 有意义，
`terminal=true` 时 result 才是最终结果，不能读取活跃任务的默认 result 当作失败。

未加载的动作提供者返回 `NOT_IMPLEMENTED`；已加载但相机或驱动未就绪返回 `NOT_READY`。
`ListActions.ready` 是动作级就绪状态，不以“某个包能导入”代替硬件就绪。

## 5. 生命周期与反馈

公共阶段：`VALIDATING` → `WAITING_TARGET` → `PLANNING` → `ACQUIRING_AUTHORITY` →
`APPROACHING` → 动作特有阶段 → `WITHDRAWING` → `RELEASING_AUTHORITY` → `FINALIZING`。
后端若必须先接管再规划，可以交换这两个阶段；反馈必须反映实际执行顺序，不虚报已完成阶段。
取消和异常可进入 `CANCELLING`、`RECOVERING`，最终为 `COMPLETED`、`CANCELLED` 或 `FAILED`。
电梯动作特有阶段：`PRESSING`、`CONTACT_HOLD`；握手动作：`CONTACT_HOLD`、`RETURNING`。

`ExecutionFeedback` 至少在阶段变化时立即发送，执行期间目标频率为 5 Hz。
`progress` 在 `[0,1]` 内，无法可靠估计则为 `-1`，不得用计时器伪造运动进度。
只有最终成功才为 `1`。`elapsed_s` 使用单调时钟，包含任务处理与收尾时间。
`target_pose_valid=false` 时忽略位姿字段；为 true 时 Header 表示参考坐标系，
`controlled_frame` 表示被控制工具中心，不能把腕部、指尖和橡胶手中心混用。
目标锁定后不因视觉抖动或重新检测而跳变；握手第一版也不在接近途中追踪移动人手。
握手锁定后不再读取相机，不跟随人手、不因之后断流或人手移动取消；驱动反馈、急停与控制权监测仍持续。到位保持后返回，再归还控制权。

## 6. 最终结果与错误码

Action result 的 `result` 类型为 `ExecutionResult`。
`success` 必须等价于 `code == SUCCEEDED`，不能由“命令已接受”推导。
本版两个运动动作的成功必要条件：`motion_completed=true`、`cleanup_completed=true`、
`authority_state=AUTHORITY_RELEASED`。任何一个不满足都不能返回成功。
`selected_arm=0` 表示尚未选定，不能假报右手。
`failed_stage` 保存实际失败阶段，成功时为空；`message` 面向人，逻辑只依赖数值错误码。
`details_json` 是诊断扩展，最多 16 KiB，不重复改变标准字段含义。

| 错误码 | 含义 |
|---|---|
| `0 SUCCEEDED` | 运动及收尾完成，控制权归还 |
| `10～16` | 参数、动作名、未就绪、忙、重复 ID、未实现、版本错误 |
| `20 TARGET_TIMEOUT` | 预算内没有获取稳定目标 |
| `21 TARGET_INVALID` | 坐标、深度、置信度、外参或工作范围不满足要求 |
| `22 TARGET_LOST` | 需要监视的目标已失效或移动超限 |
| `30 PLANNING_FAILED` | 规划失败，无法进一步归类 |
| `31 COLLISION` | 完整路径碰撞验证失败 |
| `32 JOINT_LIMIT` | 位置、速度或加速度约束不满足 |
| `33 TRACKING_ERROR` | 执行跟踪偏差超限 |
| `34 EFFECT_VERIFICATION_FAILED` | 已启用的效果验证明确失败 |
| `40 CANCELLED` | 用户取消，收尾及控制权归还已确认 |
| `41 DEADLINE_EXCEEDED` | 主动工作时间超限，收尾已完成 |
| `50 DRIVER_ERROR` | 驱动故障或通信异常 |
| `51 AUTHORITY_ERROR` | 接管失败或执行中丢失控制权 |
| `52 CLEANUP_FAILED` | 取消、撤回、停稳或控制权归还未确认 |
| `99 INTERNAL_ERROR` | 未预期的内部错误 |

`effect_verification` 独立描述效果：`UNVERIFIED=0`、`VERIFIED=1`、`FAILED=2`。
例如电梯按压流程完成但没有灯态反馈，返回运动成功、效果 `UNVERIFIED`；
若动作启用了明确的效果验证并得到失败反馈，返回 `EFFECT_VERIFICATION_FAILED`，使用诊断信息说明。
握手到达几何目标不代表已与人手接触或握紧；没有传感器反馈时也只能标记 `UNVERIFIED`。

ROS Action 终态：成功用 `SUCCEEDED`；用户取消且安全收尾完成用 `CANCELED`；其他错误用 `ABORTED`。
收尾失败优先返回 `CLEANUP_FAILED/ABORTED`，原始失败码和取消原因放入 `details_json`。
不把请求取消、已停止运动和仍保存的历史规划错误混成一个状态。

## 7. 超时、停止和控制权

标准取消接口和 `StopActive` 都只是请求取消，响应 `accepted=true` 不表示已停稳或归还控制权。
CLI 的 stop 默认继续等待原请求的最终结果，再显示实际停止和归还状态。
对不存在或已结束的明确 ID，`StopActive.accepted=false`，不误取消另一个任务。
服务端确认接受取消后不得再下发新的正常阶段轨迹，仅允许有界的退出/收尾动作。

`timeout_s` 超时触发取消，最多另外使用配置中的 `cleanup_timeout_s=10 s` 完成收尾。
这是工作预算加收尾预算，不能宣称在 `timeout_s` 时刻一定已完成物理停止。
收尾预算耗尽则返回 `CLEANUP_FAILED`，发布 `FAULT` 状态并拒绝新动作。
控制权无法确认时保持 `UNKNOWN`；仅有明确持有证据时才能写 `RETAINED`。
状态不明时不自动强制归还，不把原厂归位当成无条件安全路径；后续由独立人工恢复流程处理。
已确认未接触、未开始运动的规划失败可直接走既有归还流程，不能被历史规划错误阻塞。

客户端等待超时/断网不等于任务已取消。客户端使用原请求 ID 查询或请求停止，不新建 ID 重发。
第一版采用目标锁定后自主完成的任务语义，不因反馈订阅者消失而隐式改变运动；
机器人急停和驱动自身保护独立于此接口，客户端断网策略如需改变必须版本化声明。

## 8. 视觉接口与坐标约定

| Topic | 消息 | 含义 |
|---|---|---|
| `/g1_actions/perception/hands` | `HandObservation` | 每只手一条 21 点检测，图像归一化坐标 |
| `/g1_actions/perception/palms` | `PalmObservation` | 每只手一条米制掌心位姿及深度质量信息 |

Header 使用传感器采集时间，不能改成推理完成或发布时间；坐标系必须明确。
RGB、对齐深度、内参必须对应同一个已标定数据流。MediaPipe 的相对 Z 不作为米制深度。
HandObservation 的 21 点采用 MediaPipe 标准索引；`x/y` 为图像归一化坐标，`z` 仅是模型相对深度。
`handedness` 应纠正图像镜像关系后表达真实左右手，不能直接混用自拍预览标签。
`tracking_id` 不因临时排序改变，换人/换手必须分配新 ID。
完整姿态接口中的无效姿态不得用作运动姿态；深度不足时禁止用全零位置伪装有效目标。
无手时不重复发布旧检测；消费者以时间戳过期判断目标丢失。
置信度范围 `[0,1]`，无协方差估计时 `covariance_valid=false`，不得把零矩阵解释为完美精度。

初始目标最大年龄默认 `0.6 s`，RGB-D 时间差上限和深度有效点阈值放入标定后的服务端配置。
图像光学系遵循 REP-103：`+X` 向右、`+Y` 向下、`+Z` 向前；机器人工作系为 `torso_link`。
所有位置单位为 m，时间为 s，角度为 rad，姿态四元数顺序为 `x,y,z,w`，必须归一化。
握手当前采用位置识别：真实深度恢复的手腕点，沿手腕指向四指根部的 3D 单位方向偏移 `0.06 m`；不使用二维像素偏移代替三维位移。
姿态固定为 `torso_link` 中四指前下方 `45°`，不跟随人手旋转。橡胶手中心偏移通过 URDF 固定链换算为腕部目标，左右手模型已镜像，不能再把坐标整体取反。
相机外参复用已由电梯动作验证的安装关系；橡胶手中心和临时固定备用位置仍为模型候选值，不宣称已完成物理标定。
电梯控制点为既有按压指尖，握手为橡胶手中心，两个动作不共用业务偏移量。
位置模式的 `PalmObservation.valid=false` 表示完整姿态无效，不能据此否定位置结果；运动位置从 `/handshake/perception_status` 的新鲜 `geometry_valid`、`stable` 目标读取，并且只锁定一次。

## 9. QoS 与状态

执行与服务传输使用 ROS 2 Action/Service 默认可靠 QoS，定制客户端必须匹配。
`/g1_actions/status`：可靠、瞬态本地、深度 1；阶段变化立即发布，空闲时 2 Hz 心跳。
视觉观测：best effort（尽力传输）、volatile（易失）、keep last 深度 1；发布与订阅两端保持匹配。
这些 Topic 不承载原始图像，不把图像队列堆积当成目标稳定。
状态心跳超过 2 s 未收到时，监控端标记连接未知，不依据缓存状态再次下发运动。
`accepting_goals` 仅表达全局调度器可接收新任务，具体动作就绪仍查询 `ListActions`。

## 10. 动作插件和日志约定

当前动作提供者由统一调度器启动，提供就绪检查、启动、停止和异步最终结果。
握手使用项目 `.venv` 内的执行子进程隔离 Python 依赖，阶段事件回流到统一反馈和日志；不启动第二套调度器。
调度器统一管理互斥、请求 ID、超时、反馈和最终结果；动作不得创建第二套并行控制权管理器。
电梯先复用现有 service/task 的控制权生命周期，适配层只映射结果，不能双重接管和归还。
取消入口必须线程安全且幂等，执行线程负责退出/收尾；cleanup 必须幂等，不能运行两次归位。
握手的 MediaPipe、RGB-D 处理和坐标变换不得直接发布机械臂指令。

标准日志字段：`timestamp`、`request_id`、`action_name`、`stage`、`level`、`message`；
最终结果附完整 ExecutionResult。相同请求 ID 贯穿识别、规划、执行、退出和结果。
CLI 默认中文摘要并返回退出码 `0=成功`、`1=任务失败/取消`、`2=客户端调用异常`。
`--json` 只输出一份最终结果 JSON；持续反馈通过显式 `--feedback` 输出到 stderr，避免破坏 JSON。
规划器原始输出留在驱动日志，最终结果必须说明失败阶段、原因、控制权和耗时。

## 11. 接入顺序与兼容

统一入口、校验、状态、取消、结果缓存、电梯和握手适配已实现。旧握手执行命令转交统一入口。
电梯旧接口 `/elevator/start`、`/elevator/stop`、`/elevator/service_status` 仅作兼容入口，
接入后必须经过同一个调度器/互斥锁，不能同时存在两套互不知情的任务管理器。
当前电梯兼容入口已通过 guard 接入同一调度器；既有直接控制机械臂的低层调试脚本不属于公共执行 API，不得与调度任务混用。
v1 中不改变字段类型、含义、枚举数值；不兼容变更升级 API 版本和接口包主版本。
