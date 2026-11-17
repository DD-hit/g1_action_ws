# 握手 RGB-D 掌心识别

本阶段仅识别，不接管控制权、不调用机械臂运动接口。握手动作执行仍未启用。

## 运行

在机器人容器内，从 `/workspace/src/g1_action_ws` 执行：

```bash
bash scripts/g1.sh handshake-live --duration 30
```

依赖项目内 `.venv`、已运行的相机，以及 `/workspace/install` 中的 ROS 2 接口。默认使用 GPU。结果保存到项目内 `data/handshake/rgbd/时间戳`。

## 位姿含义

位置单位为米，参考系为彩色相机光学坐标系：相机右方为 +X、下方为 +Y、前方为 +Z。掌心自身坐标系的 +Z 朝掌外、+Y 朝四指根部、+X 由右手定则确定，因此左右手的 +X 不保证都朝大拇指。

默认采用 `wrist_offset`：从手腕和四个指根附近的小块深度取中位数，结合内参恢复三维点；沿“手腕 → 有效指根平均位置”的三维方向偏移 `0.06 m`，得到估计掌心位置。至少需要三个可靠指根，拒绝缺失、深度不连续及明显落到背景的样本。模型预测的 Z 值不参与计算；不再要求手别确认、掌心平面或模型朝向有效。

本模式只估计位置，不能验证解剖学掌心中心的绝对误差。偏移距离在 `perception.rgbd.wrist_offset.distance_m` 调整。旧平面方法保留供离线对比，设置 `position_method: plane` 可选择。

默认模式连续三帧位置一致后发布 `/handshake/palm_positions`。缺失深度、图像不同步或数据过期时，状态报告无效且不继续发布位置；订阅方必须检查时间戳，不能复用历史目标。该判定表示观测稳定，不代表已完成位置精度标定。

视觉位置输出仍使用相机光学坐标系，不能直接作为腕部目标。目标预览沿用用户确认在电梯业务中验证过的相机到躯干变换；工具中心与固定姿态仍待验证。协方差尚未标定，`covariance_valid=false`。

## 输出

- `/handshake/hand_observations`：图像关键点和手别。
- `/handshake/palm_positions`：`PointStamped`，稳定的相机坐标系位置，单位米。
- `/handshake/palm_observations`：兼容原位姿接口；位置模式下 `valid=false`，明确没有完整姿态。
- `/handshake/palm_poses`：位置模式保持空列表，避免用虚构旋转冒充完整位姿。
- `/handshake/perception_status`：JSON 状态和无效原因。
- `/handshake/image_annotated`：绘有掌心坐标轴的图像。

结果目录包含逐帧 `observations.jsonl`、`summary.json`、最新图像 `latest.jpg` 和 RGB-D 快照 `rgbd_latest.npz`；检测到稳定掌心后额外保存 `valid.jpg`、`rgbd_valid.npz`。后两者是历史有效样本，不能当成实时目标。

## 一次采集，反复回放

普通 MOV 只有彩色画面，能复测 MediaPipe 关键点，但不能还原真实深度。需要复测三维位姿时，保存同步 RGB-D 序列：

```bash
bash scripts/g1.sh handshake-live --duration 20 --record-rgbd --output data/handshake/rgbd/sample_01
bash scripts/g1.sh handshake-replay data/handshake/rgbd/sample_01 --output data/handshake/rgbd/replay_01
```

第一条命令仅在相机运行时采集一次；第二条不需要相机、ROS 2 或机器人，可反复修改几何参数后回放。输出目录必须不存在，避免覆盖实验结果。每帧存储彩色图、米制对齐深度、内参、时间戳和关键点，存储量约为每帧数百 KB，应使用有限时长采集。

回放复用缓存关键点并重新计算三维几何，不会重新执行 MediaPipe 推理，也不会将单帧结果当作已通过连续稳定性检查。对旧数据可直接传入 `rgbd_valid.npz` 或 `rgbd_latest.npz`；之前只保存快照的采集不能还原成完整 RGB-D 视频。

旧 `plane` 方法在掌心接近侧对相机时可能拟合到手侧面，因此默认改为手腕偏移方法。新方法仍依赖可靠的手腕和指根深度，不能以位置输出率代替绝对精度验证。

## 机器人固定握手姿态

`execution.orientation_mode: fixed`，参考系为 `torso_link`。`finger_downward_deg: 45.0` 表示机器人四指基准轴相对躯干水平面向前下倾 45°；不是相机坐标系的俯仰角。

模型依据：驱动末端为左右 `wrist_yaw_link`，到对应 `rubber_hand` 的固定关节旋转均为零。手指基准为局部 +X、拇指侧为局部 +Z，左掌法向为 -Y、右掌为 +Y。拇指侧通过模型网格核对，掌法向沿用现有电梯工具约定。橡胶手指有弯曲，上述为整体参考轴，不表示所有指节都平行。

换算为绕 +Y 旋转 45°，`quaternion_xyzw = [0, 0.3826834323650898, 0, 0.9238795325112867]`。变换后手指基准方向为 `[0.7071, 0, -0.7071]`，拇指侧为 `[0.7071, 0, 0.7071]`。左右手模型已镜像，所以两个腕部旋转相同，但掌法向互为镜像。手指下倾后拇指侧也随刚性手转动，保持向上分量，并非仍严格竖直。

`actions.handshake.geometry.fixed_orientation()` 检查语义角度与四元数是否一致，返回指定腕部在躯干系下的朝向。配置仍为 `source: urdf_nominal`、`verified: false`；尚未验证实际安装、IK（逆运动学）可达性或碰撞，不能声称已在真机达到该姿态。`require_verified=True` 会拒绝未验证配置。

机器人固定姿态属于目标，不写入视觉的位置观测。已增加左右橡胶手掌面中心的模型候选点，以及相机内部 TF；实际工具中心尚未现场验证；相机安装外参按用户确认复用电梯已验证链路。握手执行仍关闭，没有发送运动指令。详见 [目标坐标转换](HANDSHAKE_TARGET_GEOMETRY.md)。
