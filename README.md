# G1 动作库

当前支持电梯上行、下行按钮按压；握手动作尚未实现。
统一入口负责执行、进度反馈、停止和结果查询，同时只执行一个动作。

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

终端 2 调用动作；`up`、`down` 会驱动真机运动：

```bash
cd /workspace/src/g1_action_ws
bash scripts/g1.sh list                 # 查看动作和就绪状态
bash scripts/g1.sh up                   # 上行，自动选手
bash scripts/g1.sh down                 # 下行，自动选手
bash scripts/g1.sh up --arm right       # 指定右手，可改为 left
bash scripts/g1.sh status               # 查看当前状态和最近结果
bash scripts/g1.sh stop                 # 请求停止，等待收尾结果
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
