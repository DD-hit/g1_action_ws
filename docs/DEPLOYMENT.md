# 部署与 RGB-D 通信

所有生产入口默认加载 `src/g1_actions/config/cyclonedds.xml`，增大图像传输 socket 缓冲；保留 `RELIABLE`（可靠传输）及原来的图像时间戳、新鲜度阈值。已有 `CYCLONEDDS_URI` 时尊重外部配置。

默认设置 `ROS_LOCALHOST_ONLY=1`，使 ROS 2 相机、识别和动作通信仅在本机进行，隔离 RGB-D 流量与机器人 SDK 控制网络。Unitree SDK 仍使用配置中的 `domain_id: 0` 和 `eth0`，不受此 ROS 设置影响。需要跨机器访问 ROS 2 时，须在动作库后台与客户端启动前设置 `export G1_ROS_LOCALHOST_ONLY=0`（外部普通 ROS 节点设置 `ROS_LOCALHOST_ONLY=0`）；这会取消上述通信隔离，需要重新验证运动平滑性。项目使用独立开关，避免 ROS 环境初始化预设的 `ROS_LOCALHOST_ONLY=0` 覆盖默认隔离设置。修改后需重启已有后台。

机器人宿主机需设置并持久化缓冲上限（不是在每次启动时修改）：

```bash
printf 'net.core.rmem_max=16777216\nnet.core.wmem_max=16777216\n' | sudo tee /etc/sysctl.d/90-g1-rgbd-buffers.conf
sudo sysctl -p /etc/sysctl.d/90-g1-rgbd-buffers.conf
```

小缓冲可能造成大图像分片丢失、可靠传输重传和发布阻塞，最终触发 RGB-D 过期；无需放宽 `max_frame_age: 0.6`。


项目内的 `.venv` 由 `bash scripts/setup_handshake_video.sh` 创建，供握手视频与调试脚本使用。离线工具和数据格式见 [握手 RGB-D 调试](HANDSHAKE_RGBD.md)。
