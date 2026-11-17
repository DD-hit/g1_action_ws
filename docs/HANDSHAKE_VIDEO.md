# 握手开发：录制视频识别

当前阶段只检测手部关键点，不接 ROS、不估计相机坐标下的米制掌心位姿，也不执行机械臂动作。统一动作接口中的 `handshake` 仍返回未实现。

## 环境

在 `g1_action_ws` 根目录执行：

```bash
bash scripts/setup_handshake_video.sh
```

脚本创建本项目的 `.venv`，安装 `requirements-handshake-video.txt` 中的依赖，并下载固定版本、校验 SHA-256 的官方 Hand Landmarker 模型。环境不复制到其他机器，每台机器分别在项目目录内创建。Jetson 的依赖兼容性和性能需要单独验证。

已验证的平台版本：Jetson/Linux 使用 MediaPipe `1.0.1`，macOS 使用 `1.2.3`。依赖文件按平台选择，避免把 Jetson 的 GPU 兼容版本用于 macOS CPU 路径。

`.venv`、模型下载文件和 `data` 均不进入 Git。依赖文件和下载脚本进入 Git。

## 使用

```bash
bash scripts/g1.sh handshake-video /absolute/path/video.MOV
# 或指定一个空的结果目录
bash scripts/handshake_video.sh /absolute/path/video.MOV --output /absolute/path/review
# 本次运行明确选择后端
bash scripts/g1.sh handshake-video /absolute/path/video.MOV --delegate gpu
bash scripts/g1.sh handshake-video /absolute/path/video.MOV --delegate cpu
```

默认结果位于 `data/handshake/reviews`；配置统一位于 `src/g1_actions/config/handshake.yaml`。解码器按视频方向元数据处理手机竖屏视频，再缩放至指定长边。推理按视频时间顺序进行，降低处理帧率时保留源帧索引和时间戳。

`perception.delegate` 支持 `auto`、`cpu`、`gpu`。`auto` 在可见 Jetson GPU 设备的 Linux ARM64 主机选择 GPU，其他主机选择 CPU；明确选择 GPU 后，运行失败会报错，不自动降级。输出的 `inference_delegate` 记录实际选择。

## 结果

- `annotated.mp4`：H.264 标注视频，包含 21 个关键点、骨架、模型左右手标签和推理耗时。
- `landmarks.jsonl`：每个处理帧的时间戳、图像尺寸、左右手分类置信度、21 个图像归一化关键点、21 个模型局部空间关键点。
- `summary.json`：检测覆盖率、连续漏检时长、分类标签变化次数、推理耗时、版本和模型校验值。
- `preview.jpg`：均匀抽取的标注画面。

模型左右手标签保持原值，尚未用已知左右手样本验证解剖学约定；分类分数不是每个关节点的置信度。模型局部空间坐标及归一化 z 都不能替代 RGB-D 深度或机器人坐标。标签变化统计包含主动换手，不等于误分类次数；未检出帧可能没有手，检测覆盖率不等于识别准确率。

## IMG_2044 在 Jetson 上的首轮结果

使用 `/workspace/src/g1_action_ws/.venv`，MediaPipe 1.2.3，CPU 后端，输入长边缩至 960，约 30 FPS 抽帧。视频 10.64 s，源帧 638，处理并编码 319 帧。

- 261 帧有检测，Right 标签 114 帧、Left 标签 147 帧，没有双手同时检出的帧。
- 3.635～4.868 s 是最长未检出区间，预览中可见换手时的空画面；其他未检出区间不能仅凭统计判为漏检。
- 单手标签在 6.770～6.903 s、9.037～9.070 s 短暂变为 Right 后又恢复 Left，需要在后续阶段检查侧掌时的分类稳定性。
- 单帧推理中位数 66.8 ms，P95 76.5 ms；整段解码、识别和编码约 111 s。这是离线结果，不代表达到实时处理速率。

MediaPipe `1.2.3` 的 GPU 独立探测能初始化 EGL 并识别 NVIDIA Tegra Orin；其 LiteRT GPU WebGPU/Vulkan 后端首帧推理失败，错误为 `Failed to create custom tensor buffer`。这是原始失败记录；后续修复见下节。

结果位于 `data/handshake/reviews/IMG_2044_jetson`。当前只完成手部关键点识别；米制掌心位姿和机械臂运动仍未接入。

## Jetson GPU 修复与验证

固定 Linux MediaPipe `1.0.1` 后，GPU 在同一系统、同一模型上完成实际推理。未更新 JetPack、NVIDIA 驱动或电梯配置。

同版本、同一批 107 个预解码视频帧，排除最初 5 帧预热后的结果：

| 后端 | 中位耗时 | P95 |
|---|---:|---:|
| CPU | 65.4 ms | 87.4 ms |
| GPU | 16.8 ms | 25.7 ms |

推理中位耗时改善约 3.9 倍。这个对比不包含解码和编码。

完整原始视频的 GPU 回放处理、编码均为 319 帧，输出包含左右手。带解码负载时，推理中位耗时 26.4 ms、P95 34.7 ms，整段约 62.4 s，其中解码约 49.7 s。视频跳帧路径也改为仅对选中帧进行 BGR 转换，所以端到端时间改善不能全部归因于 GPU。

GPU sysfs 采样在空闲阶段为 0，回放期间出现负载，峰值约 86.1%。输出源帧索引和时间戳与 CPU 回放一致，关键点均为有限数值。日志仍包含上游 Tensor 多写诊断，不能据此宣称任意持续负载都已验证稳定；当前验证范围是完整测试视频与重复启动测试。

GPU 视频位于 `data/handshake/reviews/IMG_2044_jetson_gpu`。同帧性能基准和设备负载记录位于 `data/handshake/gpu_probe`。当前仍是离线视频功能，不能把推理帧率当作真实相机端到端帧率。
