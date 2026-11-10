# SensorGuard-NAV Runtime

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

> 本仓库已并入 [legged-navigation-runtime](https://github.com/zxinyu-robot/legged-navigation-runtime) 的 `python/sensor_guard_nav/`，此处只读归档。

## 项目状态

| 项目 | 状态 |
| --- | --- |
| 维护状态 | Archived |
| 后续维护 | [legged-navigation-runtime](https://github.com/zxinyu-robot/legged-navigation-runtime) |
| 实现类型 | 与中间件无关的 Python 参考实现 |
| 数据范围 | 确定性合成场景；不含真实机器人数据或实机性能结论 |

## 项目简介

SensorGuard-NAV 面向传感器污染与观测残缺场景的定位自救。首版聚焦 LiDAR 污染与 IMU / 轮速冗余，将故障注入、异常检测、可信度隔离、安全降级、恢复门控和诊断回放串成可运行、可测试的闭环。

系统把“定位是否可信”和“当前还能以多快的速度移动”分开计算：前者决定数据源隔离与恢复路径，后者由视场、不确定性、制动距离和故障持续时间共同约束。

## 核心闭环

```text
LiDAR quality + IMU/wheel consistency
        → temporal trust arbiter
        → self-rescue state machine
        → safety envelope + recovery gate
        → runtime decision + JSONL telemetry
```

核心状态：`HEALTHY → SUSPECT → DEGRADED_ODOMETRY → SAFE_STOP`，并支持局部 / 全局恢复和人工接管。

## 已实现

- LiDAR 点数、有限值、角覆盖、距离分布和强度质量检测。
- scan matching 与 IMU / 轮速增量一致性检测。
- EMA、滑动窗口、滞回阈值与连续帧确认。
- 七态定位自救状态机、超时保护和安全速度包络。
- 局部对齐、全局重定位和人工接管恢复门控。
- dropout、扇区遮挡、偏置、噪声、NaN、冻结和延迟注入。
- 合成污染场景、JSONL 时间线及摘要报告。

## 快速开始

```bash
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -e ".[dev]"
sensor-guard-nav demo --config configs/lidar_mud.json --output artifacts/lidar-mud
pytest
ruff check .
ruff format --check .
```

演示生成 `timeline.jsonl` 和 `summary.json`；这些数据只验证软件行为，不代表真实检测率或安全认证。

## 评价指标

污染检测延迟、假降级率、最大定位漂移、不安全运动持续时间、停车距离、重定位耗时、恢复成功率、错误传感器重新接管次数，以及每次状态转换的可解释性。

详细文档：[架构与状态机](docs/architecture.md) · [故障处理](docs/fault-handling.md) · [ROS 2 集成边界](docs/ros2-integration.md) · [评价口径](docs/evaluation.md)。

## 归档边界

当前仅实现 LiDAR 主定位源和合成参考闭环；未提供 AMCL、SLAM Toolbox、FAST-LIO 等二进制适配节点。默认阈值必须通过传感器标定、系统辨识和实车回放重新确定；`SAFE_STOP` 只产生决策，最终制动由机器人安全控制器执行。

## 许可证

[MIT](LICENSE)。第三方依赖和未来接入的数据、资产、ROS 软件包保留各自许可证，详见 [THIRD_PARTY.md](THIRD_PARTY.md)。
