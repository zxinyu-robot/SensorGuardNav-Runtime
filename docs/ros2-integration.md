# ROS 2 集成边界

核心库刻意不导入 `rclpy`，以便在CI、离线rosbag分析和不同中间件中复用。
`sensor_guard_nav.adapters.base` 定义输入与决策输出协议。

建议实际工程拆成三个 lifecycle node：

1. `sensor_health_node`：订阅点云、IMU、轮速和定位诊断，完成同步与契约转换；
2. `sensor_guard_node`：调用纯Python运行时，发布健康度、模式和速度限制；
3. `recovery_coordinator`：调用Nav2、AMCL、SLAM Toolbox或LIO提供的重定位服务。

建议话题契约：

```text
/sensor_guard/health
/sensor_guard/mode
/sensor_guard/speed_limit
/sensor_guard/recovery_request
/diagnostics
```

## 接入要求

- 使用传感器时间戳完成同步，不能以回调到达时间代替；
- 明确 `base_link`、`odom`、`map` 坐标契约；
- 速度限制必须由本地控制器再次执行，不能只依赖上层规划器；
- 急停与硬件安全链路保持独立；
- 记录阈值配置版本、软件提交和标定版本；
- 原始大点云由rosbag按需记录，核心JSONL只保存决策证据。

当前仓库没有宣称完成上述节点；本文件是实现边界，不是部署证明。
