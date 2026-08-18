# 域间接口定义

> 版本：v1.0（对齐五域接口规范 v3.1）
> 适用范围：`robot_system` 仓库全部域实现
> 状态：接口冻结基线。本文是 monorepo 内域间通信的唯一权威清单。

## 1. 本文管什么

本文只定义**跨域边界**：谁产生、谁消费、ROS 名称、类型、方向、QoS、频率、时效和成功语义。域内节点划分、算法、参数文件和实现方式不属于本文范围，由 `domains/<domain>/` 自行记录。

任何不在本文表格中的跨域 endpoint 都视为未授权。新增、删除或改变本文任一条目的语义都是破坏性公共变更，按 `AGENTS.md` 与 `docs/collaboration-and-commit-standards.md` 的接口变更流程处理。

## 2. 域与 Navigation 的归属裁决

系统只有**四个业务域**：RT-Control、Perception、Motion、Autonomy。Gateway 是对外通信与部署边界，不计入业务域，也不在本文定义其外部协议。

原 Navigation 域已退役，能力按职责拆给两个域：

| 原 Navigation 能力 | 新归属 |
| --- | --- |
| 雷达/IMU/三维数据处理、建图、定位、融合里程计、`map → odom`、占据栅格与语义地图 | **Perception** |
| Nav2、导航任务 Action、底盘软件门、导航取消收敛、发往 RT-Control 的安全速度 | **Motion** |

`/navigation/*` 保留为“导航能力”命名空间，**不表示存在 Navigation 域或 Navigation 容器**。ROS graph 中不得出现独立 Navigation 域服务端，也不得出现对独立 Navigation 容器的必需依赖。

语义地图增删改查、语义工作站到导航位姿的解析都属于 Perception 域内能力。P-01 生成计划时已为每个 `PickSequence` 写入可直接执行的 `station_nav_pose`；Autonomy 只转发，Motion 只调用 Nav2，双方都不解析语义地图。

## 3. 调用方向与依赖规则

```text
本地 CLI/HMI ─G-01→ Autonomy ─P-01/P-02→ Perception
                       │
                       ├─N-01/N-02, M-01/M-02/M-03→ Motion ─N-04, M-04/M-05→ RT-Control
                       │
                       └─（只读状态）← Perception / Motion / RT-Control
```

底线规则：

- Autonomy 是唯一任务编排、顺序、取消、恢复和任务持久化责任域，但**不直接控制 RT-Control**。
- Motion 是完整 14 轴 FJT 和 VacuumGrip 的唯一业务客户端，也是 `/cmd_vel_safe` 的唯一生产者。
- Perception 不直接调用 Motion 或 Nav2。
- Motion 不获取或释放底盘锁的**业务所有权**；N-02 服务端在 Motion 域内，但 acquire/release 由 Autonomy 发起。
- Gateway 是对外出口；外部任务只能进 Autonomy，不得绕过其直达 Motion/RT-Control。
- `/diagnostics` 只作诊断，任何域都不得用它替代 Action Result、readiness、SafetyState 或 Gate 证据。

## 4. QoS 剖面

跨域通信只使用以下命名剖面，不得逐条自定义：

| 剖面 | 定义 | 用途 |
| --- | --- | --- |
| `Q_CONTROL` | reliable / volatile / keep_last 1；deadline 100 ms；lifespan 200 ms | 周期控制命令（`/cmd_vel_safe`） |
| `Q_FAST_STATE` | reliable / volatile / keep_last 小深度 | 高频状态（`/joint_states`、`/odom`、`/wheel/odom`） |
| `Q_STATE` | reliable / volatile / keep_last 小深度 | 中频状态（`/vacuum/state`、`/control/safety_state`） |
| `Q_LATCHED` | reliable / transient_local / keep_last 1 | 版本与就绪基线（readiness、`/robot_model/info`、`/calibration/info`、`/map`、N-03） |
| `Q_DIAGNOSTIC` | reliable / volatile / keep_last 10 | `/diagnostics` |
| `Q_ROLLING_COMMAND` | **原型占位，非生产冻结**；reliability/history/deadline/lifespan 待 ELECTRI-102 压测 | rolling 权威未来批次；双方必须使用同一已批准 profile 版本 |
| `Q_ROLLING_STATE` | **原型占位，非生产冻结**；reliability/history/deadline/lifespan/发布频率待 ELECTRI-102 压测 | rolling ack、buffer、stop 与模式状态 |
| 标准 Action / Service QoS | ROS 2 默认 | 全部 Action 与 Service |

## 5. 接口总表

ID 前缀含义：`G` = Gateway/本地入口，`P` = Perception 提供，`N` = 导航能力（Motion 或 Perception 提供），`M` = Motion 提供，`R-IN` = RT-Control 输入，`R-OUT` = RT-Control 输出。

### 5.1 任务入口（Autonomy 提供）

| ID | ROS 名称 | 形式 / 类型 | 方向 | 关键约束 |
| --- | --- | --- | --- | --- |
| G-01 | `/autonomy/execute_demo_task` | Action / `alfa_task_interfaces/action/ExecuteDemoTask` | 本地 CLI/HMI ⇄ Autonomy | 同时只接受一个活动 Goal，第二个在 Goal 响应阶段 reject；终态三选一 |

### 5.2 Perception 提供

| ID | ROS 名称 | 形式 / 类型 | 方向 | 关键约束 |
| --- | --- | --- | --- | --- |
| P-01 | `/perception/build_wall_task_plan` | Action / `alfa_task_interfaces/action/BuildWallTaskPlan` | Autonomy ⇄ Perception | 真实单次同步采集；25 箱；13～15 序列；每个 G-01 只调用一次；严格成功 |
| P-02 | `/perception/refine_sequence_poses` | Action / `alfa_task_interfaces/action/RefineSequencePoses` | Autonomy ⇄ Perception | 同箱同序；`base_link`；stamp=真实曝光时刻；底盘静止、TF、标定不变 |
| P-04 | `/perception/readiness` | Topic / `alfa_system_interfaces/msg/DomainReadiness` | Perception → Autonomy | `Q_LATCHED`；故障立即 `ready=false`；变化立即发；稳定 1 Hz |
| P-03 | `/perception/obstacle_cloud` | Topic / `alfa_perception_interfaces/msg/ObstacleCloud` | Perception → Motion | **产品预留**；Demo 不部署、不订阅、不依赖 |
| N-05 | `/navigation/localization/status` | Topic / `alfa_navigation_interfaces/msg/LocalizationStatus` | Perception → Motion、Autonomy | 发布者唯一；10～20 Hz；最大年龄 200 ms |
| N-07 | `/map` | Topic / `nav_msgs/msg/OccupancyGrid` | Perception → Motion、观察工具 | `map` frame；`Q_LATCHED` |
| N-09 | `/navigation/scan` | Topic / `sensor_msgs/msg/LaserScan` | Perception → Motion | Nav2 local costmap 输入；frame/stamp 新鲜；过期时 Motion 不输出非零速度 |
| P-NAV-01 | `/odom` | Topic / `nav_msgs/msg/Odometry` | Perception → Motion、Autonomy | 融合里程计；50 Hz；最大年龄 200 ms；**最终停稳证据** |
| P-NAV-02 | `map → odom` TF | Topic / `tf2_msgs/msg/TFMessage` | Perception → 全系统 | 唯一发布者 |

### 5.3 Motion 提供

| ID | ROS 名称 | 形式 / 类型 | 方向 | 关键约束 |
| --- | --- | --- | --- | --- |
| N-01 | `/navigation/navigate_to_pose` | Action / `alfa_navigation_interfaces/action/NavigateToPoseTask` | Autonomy ⇄ Motion | `station_nav_pose(map)` 来自 P-01；真实调用 Nav2；最多总调用三次 |
| N-02 | `/navigation/set_base_motion_inhibit` | Service / `alfa_navigation_interfaces/srv/SetBaseMotionInhibit` | Autonomy → Motion | 单次请求；CAS token；Response 不代替 N-03 |
| N-03 | `/navigation/base_motion_gate_state` | Topic / `alfa_navigation_interfaces/msg/BaseMotionGateState` | Motion → Autonomy、Perception | `Q_LATCHED`；10 Hz；最大年龄 200 ms；owner/token 匹配 |
| N-06 | `/navigation/readiness` | Topic / `alfa_system_interfaces/msg/DomainReadiness` | Motion → Autonomy | 导航执行能力准入；变化立即发；稳定 1 Hz |
| M-01 | `/motion/move_to_camera_view_pose` | Action / `alfa_task_interfaces/action/MoveToCameraViewPose` | Autonomy ⇄ Motion | 1～2 目标；同集同序；每序列只调用一次 |
| M-02 | `/motion/plan_and_execute_pick` | Action / `alfa_task_interfaces/action/PlanAndExecutePick` | Autonomy ⇄ Motion | 精位姿必须新鲜；Gate 匹配；每序列一次；Result 载荷等级 `UNVERIFIED` |
| M-03 | `/motion/plan_and_execute_place` | Action / `alfa_task_interfaces/action/PlanAndExecutePlace` | Autonomy ⇄ Motion | 固定放置配置；每序列一次；**不可重放**；Result 载荷等级 `UNVERIFIED` |
| M-06 | `/motion/readiness` | Topic / `alfa_system_interfaces/msg/DomainReadiness` | Motion → Autonomy | 机械能力准入；变化立即发；稳定 1 Hz |
| M-07 | `/motion/base_travel_readiness` | Topic / `alfa_motion_interfaces/msg/BaseTravelReadiness` | Motion → Autonomy | 固定四项公式；owner/token 匹配；严格时效 |
| N-04 | `/cmd_vel_safe` | Topic / `geometry_msgs/msg/TwistStamped` | Motion → RT-Control | 见 R-IN-01；Motion 是唯一生产者 |

### 5.4 RT-Control 输入

| ID | ROS 名称 | 形式 / 类型 | 方向 | 关键约束 |
| --- | --- | --- | --- | --- |
| R-IN-01 | `/cmd_vel_safe` | Topic / `geometry_msgs/msg/TwistStamped` | Motion → RT-Control | `base_link`；仅 `linear.x`、`angular.z` 非零；`Q_CONTROL`；20～50 Hz；**200 ms 本地看门狗** |
| R-IN-02 | `/whole_body_jtc/follow_joint_trajectory` | Action / `control_msgs/action/FollowJointTrajectory` | Motion ⇄ RT-Control | 完整 14 轴；`allow_partial_joints_goal=false`；整组取消 |
| R-IN-03 | `/control/set_enabled` | Service / `alfa_control_interfaces/srv/SetControlEnabled` | 维护/生命周期工具 → RT-Control | 不复位急停、安全继电器或 STO；不属于箱级任务流程 |
| R-IN-04 | `/vacuum/pump/set_enabled` | Service / `alfa_control_interfaces/srv/SetPumpEnabled` | 维护工具或 RT 内部管理器 → RT-Control | 活动真空命令或可能持箱时拒绝普通停泵 |
| R-IN-05 | `/vacuum/grip` | Action / `alfa_control_interfaces/action/VacuumGrip` | Motion ⇄ RT-Control | 通道同数量/同集/同序；GRIP 每通道新鲜表压 `<= -50 kPa`；RELEASE 仍 `UNVERIFIED` |
| R-IN-06 | `/rt/joint_control/set_mode` | Service / `robot_interfaces/srv/SetJointControlMode` | Motion → RT-Control | protocol `1.0`；expected-mode CAS；仅稳态接管；结果只描述 controller switch，不代替 FJT Action 终态 |
| R-IN-07 | `/rt/rolling_joint_control/open` | Service / `robot_interfaces/srv/OpenRollingJointSession` | Motion → RT-Control | UUID boot/client/request；固定 axis SHA-256；单 session；返回 test-only 标志、能力、容量和初始 horizon |
| R-IN-08 | `/rt/rolling_joint_control/update` | Topic / `robot_interfaces/msg/RollingJointTargetBatch` | Motion → RT-Control | `Q_ROLLING_COMMAND`；固定 14 轴 q/qdot；session 相对时间；authoritative suffix；schema 上限 256，运行容量由 open 返回 |
| R-IN-09 | `/rt/rolling_joint_control/close` | Service / `robot_interfaces/srv/CloseRollingJointSession` | Motion → RT-Control | graceful 两阶段 close；Stopping 中幂等且不重建 stop；Holding 中首次新 request ID 才销毁 session |

### 5.5 RT-Control 输出

| ID | ROS 名称 | 形式 / 类型 | 方向 | 关键约束 |
| --- | --- | --- | --- | --- |
| R-OUT-01 | `/tf`、`/tf_static` | Topic / `tf2_msgs/msg/TFMessage` | RT-Control 或状态发布器 → 全系统 | 本体坐标边唯一；静态/动态发布责任不重复；`map→odom` 不由本域发布 |
| R-OUT-02 | `/wheel/odom` | Topic / `nav_msgs/msg/Odometry` | RT-Control → Perception | `Q_FAST_STATE`；50 Hz；最大年龄 200 ms；**不作为到站或停稳最终证据** |
| R-OUT-03 | `/joint_states` | Topic / `sensor_msgs/msg/JointState` | RT-Control → Motion、Perception、Autonomy | 完整 14 轴；`Q_FAST_STATE`；**50 Hz（BQ-068）**；最大年龄 200 ms；不作为 250 Hz rolling 内环反馈 |
| R-OUT-04 | `/battery_state` | Topic / `sensor_msgs/msg/BatteryState` | RT-Control → Autonomy、观测工具 | 1 Hz；只读，不作为业务控制入口 |
| R-OUT-05 | `/vacuum/state` | Topic / `alfa_control_interfaces/msg/VacuumState` | RT-Control → Motion、Autonomy、观测工具 | `Q_STATE`；20～50 Hz；只 RT-Control 用于 GRIP 判定，其他域不自行计算成功 |
| R-OUT-06 | `/control/safety_state` | Topic / `alfa_control_interfaces/msg/SafetyState` | RT-Control → Perception、Motion、Autonomy | `Q_STATE`；10～50 Hz；最大年龄 200 ms；deny 或过期时禁止新动作 |
| R-OUT-07 | `/robot_model/info` | Topic / `alfa_system_interfaces/msg/RobotModelInfo` | RT-Control → 各域 | `Q_LATCHED`；14 轴集合与校验值一致；任务期间版本不变 |
| R-OUT-08 | `/calibration/info` | Topic / `alfa_system_interfaces/msg/CalibrationInfo` | RT-Control → 各域 | `Q_LATCHED`；曝光前与返回前版本一致 |
| R-OUT-09 | `/rt_control/readiness` | Topic / `alfa_system_interfaces/msg/DomainReadiness` | RT-Control → Autonomy、观测工具 | 故障立即 `ready=false`；变化立即发；稳定 1 Hz |
| R-OUT-10 | `/diagnostics` | Topic / `diagnostic_msgs/msg/DiagnosticArray` | 各域 → 诊断聚合器 | `Q_DIAGNOSTIC`；不替代 Action Result、SafetyState、Gate 或硬安全链 |
| R-OUT-11 | `/rt/rolling_joint_control/state` | Topic / `robot_interfaces/msg/RollingJointControlState` | RT-Control → Motion、观测工具 | `Q_ROLLING_STATE`；boot/session/generation/sequence、buffer/age、q/qdot、RejectCode 与 StopReason 分离；test-only limits 显式 |

## 6. 关键运行规则

### 6.1 成功语义由服务端保证

Action 返回 ROS `SUCCEEDED` 时，服务端必须已实际完成本次功能目标，不得返回伪成功：

- **N-01**：底盘实际到达本次 `station_nav_pose`、位置/朝向误差满足约定、形成稳定到位证据。目标 pose 非法、地图/定位不可用或 Nav2 无法到达时必须明确失败。
- **P-01**：真实完成一次同步采集，并输出完整 25 箱、13～15 序列的计划；任一序列缺失即失败。
- **P-02**：结果是当时、当地、当前箱的精位姿，stamp 为真实曝光时刻。
- **M-02**：全部真空通道均为 `ATTACHED_VERIFIED` 才 `SUCCEEDED`；双箱任一侧未验证则整个 M-02 失败。
- **M-05 GRIP**：每通道新鲜表压 `<= -50 kPa`；全部达到阈值才 `ATTACHED_VERIFIED`。
- **M-05 RELEASE / M-03**：物理释放结果**永远记为 `UNVERIFIED`**，不以压力判定释放成功。

调用方只做轻量一致性检查（box_id、序号、owner/token、verification_level、时效），不重新计算服务端的判定。

### 6.2 完整 14 轴是一条不可拆分的轨迹

R-IN-02 的 Goal 必须包含完整且唯一的 14 个 `joint_names`，每个轨迹点含 14 项 `positions`；`velocities`/`accelerations` 可空，非空时同样各含 14 项；`time_from_start` 严格递增。禁止部分关节 Goal，取消按整组生效。

#### Rolling joint mode

- Motion 先取消并等待自己持有的 FJT 终态，再按 R-IN-06 的 expected-mode CAS 请求稳态切换；不得直接调用 controller-manager，也没有运动中 force switch。
- 进入 `ROLLING_READY` 后，Motion 按 R-IN-07 open、R-IN-08 prime/update、R-IN-09 stop/finalize close 的顺序使用唯一 session。`STOPPING`/`HOLDING` 不接受 update 恢复；重新运行必须 fresh open。
- Service 调用方本地超时不等于服务端失败，也不能发新 request ID 重放；只用相同 client/request ID 查询幂等结果。switch timeout/部分结果不确定时进入 `RESTART_REQUIRED`，不自动回滚或重试。
- update 的错误 batch 只更新 `RejectCode`，不等于 session 停止；session 停止原因只看独立 `StopReason`。错误 boot/session/client 的数据不得取得当前 session 所有权。
- `/joint_states` 继续为 50 Hz 域外观测。rolling controller 在同进程 250 Hz update 中直接读 ros2_control state interfaces；Motion 不得用提高 `/joint_states` 频率替代未来缓冲。
- `test_only_limits=true` 的 open/state 只授权 Mock/算法验证。生产动态限制、QoS、horizon、guard、timeout 和 tolerance 未经证据冻结时，不得激活生产 rolling 模式。

### 6.3 双箱序列是一个整体

M-01、M-02、M-03 每个序列各只调用一次，双箱共用同一个 Goal。任一失败则当前序列不 Commit，当前 G-01 进入失败收敛。

### 6.4 重试预算只有一套，且由 Autonomy 统一计数

| 接口 | 预算 |
| --- | --- |
| P-01 | 每个 G-01 只调用一次 |
| N-01 | 当前到站步骤最多三次总尝试（含第一次） |
| P-02 | 当前序列最多三次总尝试（含第一次） |
| M-01 / M-02 / M-03 | 每序列各只调用一次，失败不重放 |

只有同时满足以下条件才允许下一次调用：前次已有确定终态、`ErrorInfo.retryable=true`、总调用次数 < 3、G-01 未收到 cancel、且对应的停止/Gate/导航目标/精定位不变量仍成立。

子域**不得叠加第二套业务重试**。只有 N-01/P-02 的 ControlledRetry 读取 `retryable`；不得解析 `ErrorInfo.code`、`message` 或最后一条 feedback 决定重试。`COMMUNICATION_UNKNOWN`、Goal response timeout、取消后无终态、Goal rejected、`retryable=false` 均禁止重发。

### 6.5 物理动作禁止自动重放

M-02/M-03 失败、超时或通信未知时不重发，也不为再次 Pick 重跑 M-01/P-02。`GRIP_NOT_SENT`、`RELEASE_NOT_SENT` 和 command disposition 只用于诊断，不授权物理重试。`*_command_completed=false` **不等于**“命令一定未发送”。吸取命令可能已执行时保持真空和 Gate；释放命令可能已执行时不发送重复释放，现场转人工处置。

### 6.6 readiness 心跳与三种本地状态

各域必须在本地故障时立即发布 `ready=false`，状态变化立即发布，稳定时 1 Hz。Autonomy 按 steady clock 的本地接收年龄派生：

| 本地状态 | 判定 | 行为 |
| --- | --- | --- |
| `HEALTHY` | 消息合法、`ready=true`、版本匹配、age ≤ 2 s | 可按行为树派发 |
| `DEGRADED` | 上一条原本合法且 `ready=true`，2 s < age ≤ 3 s | 禁止派发新业务 Action；不能仅因静默取消正在执行的 Action |
| `UNAVAILABLE` | age > 3 s，或新鲜消息明确 `ready=false`/内容非法 | 按阶段依赖处理 |

额外 1 s 宽限只容忍未收到新心跳或 DDS 到达延迟。新鲜 `ready=false`、fatal reason、版本/hash 不匹配、`producer_instance_id` 改变或 SafetyState deny **不享受宽限**。1 s 内由同一 `producer_instance_id` 恢复视为 DDS 抖动；实例 ID 改变即进程重启，新实例不得加入旧 G-01。

N-03、M-07 和 SafetyState 的 200 ms 严格 freshness **不使用** readiness 的 1 s 宽限。

### 6.7 Gate 每个请求只发送一次并用请求后状态对账

每次 acquire/release 前，Autonomy 先取得新鲜 N-03，记录本地接收序号和 baseline token，并写入 N-02 `expected_token`。请求发出后记录新的接收边界；**每个请求只发送一次**，服务响应超时后不得重发，也不得用旧 N-03 判断成功。

Motion 的 N-02 服务端实际完成一次状态变更时，token generation 保持不变，epoch 必须严格大于 baseline；Response 与对应 N-03 使用同一新 token。无状态变化的幂等请求可保留原 token，但 response 丢失时不能把它冒充成已确认的新变化。

对账规则：

- **acquire 收到 response**：只接受请求后新鲜 N-03；token 等于 response `current_token`，owner 匹配，且为 `INHIBITED + base_stationary_verified=true`。
- **acquire response 丢失**：只有请求后 N-03 接收序号更大、owner 完全匹配、`INHIBITED` 且已停稳、generation 等于 baseline、epoch 大于 baseline 时才能原子采纳其 token；否则为 UNKNOWN 并保持 fail-closed。
- **release**：只接受请求后新鲜 `OPEN`，接收序号更大、generation 等于 baseline、epoch 大于 baseline。Gate 打开后不得继续要求消息回显已释放的 owner。
- owner、token 或代际无法对账时，不派发后续动作并保持 fail-closed。

### 6.8 M-07 固定公式

N-02 解锁服务在 compare-and-set 线性化点必须再次读取最新、owner/token 匹配且新鲜的 M-07；不满足时继续保持锁住。

```text
ready_for_supervised_demo_travel ==
    whole_body_in_travel_envelope
    && no_active_motion_goal
    && hardware_fault_free
    && (!release_command_required || release_command_completed)
```

M-07 还必须新鲜、owner/token 匹配，且在本序列正常 M-03 后报告 `load_verification_level=UNVERIFIED`。`ready=true` 时 `blocker.code` 必须为 `SUCCESS`；任何字段矛盾都按 not-ready 处理。

### 6.9 底盘必须能在本地停车

Gate 锁住、N-01 不活动、定位/地图/安全条件无效或消息链路异常时，Motion 不得继续输出非零速度。RT-Control 在 `/cmd_vel_safe` 过期（>200 ms）或非法时**独立停车**，不依赖上游。

### 6.10 250 Hz 控制环不得跨容器

RT-Control 的 250 Hz 实时控制循环必须在单一进程/容器内闭环。总线故障和硬安全链（急停、安全继电器、STO、驱动器保护、机械限位）不得由软件诊断代替。

### 6.11 feedback 静默只告警

任何 Action 的 feedback 静默超过 1 s 只产生告警，不单独触发取消、失败或重试。分支由总执行时限和最终 Action 状态决定。

### 6.12 TF 与坐标系所有权

| 坐标关系 | 唯一发布者 |
| --- | --- |
| `map → odom` | Perception |
| 本体关节与固定传感器边 | RT-Control / 机器人状态发布器（按权威模型） |

`/tf` 中每一条坐标关系只允许一个权威发布者。

## 7. ErrorInfo 与错误码分段

`ErrorInfo` 字段：`code`、`retryable`、`message`、`origin`。

- 重试判定：仅 N-01/P-02 的 ControlledRetry 读取 `retryable`。
- 诊断字段：`code`、`message`、`origin` 只用于诊断和日志，不参与控制分支。
- 错误码按域分段（Perception 使用 2xxx 段），具体分段表随统一 interfaces package 落地。

## 8. G-01 TaskPhase 冻结常量

`ExecuteDemoTask.Feedback` 的 `task_phase` 冻结为 0～14，顺序与命名不得改动：

```text
UNKNOWN=0
STARTING=1
GATING_OBSERVATION=2
BUILDING_PLAN=3
RELEASING_OBSERVATION_GATE=4
NAVIGATING=5
ACQUIRING_SEQUENCE_GATE=6
POSITIONING_CAMERA=7
LOCALIZING=8
PICKING=9
PLACING=10
COMMITTING=11
RELEASING_SEQUENCE_GATE=12
FINALIZING=13
CANCELING=14
```

G-01 `final_state` 三选一：`COMPLETED_UNVERIFIED` / `FAILED` / `CANCELED`。成功时 `verification_level=PICK_VERIFIED_RELEASE_UNVERIFIED`。

## 9. 待统一 interfaces package 落地的 wire 变更

以下变更**不新增公共 endpoint**，必须在 `src/interfaces` 的统一 package major/schema 升级时由四域同批生成、编译和部署。本文修改不等于 IDL 已经改变；新旧消息类型不得混跑。

| 类型 | 变更 |
| --- | --- |
| `DomainReadiness` | 新增 `unique_identifier_msgs/UUID producer_instance_id`；producer 每次进程启动生成随机 UUID，运行期不变 |
| `WallTaskPlan` | 删除 `valid_until`；保留 `source_stamp`；计划生命周期改由当前 G-01 管理 |
| `PlanAndExecutePick.Goal` | 删除 `plan_valid_until`；保留 `max_pose_age` |
| `VacuumGrip.Goal` | 删除 `timeout`；物理命令预算只取版本化 `grip_profile`，父 Action deadline 独立且不得更短 |
| `ExecuteDemoTask.Feedback` | 在同一 `.action` 冻结第 8 节 TaskPhase 0～14；删除旧 `BASE_GATED`、`RELEASING_GATE`、`NEXT_OR_FINISH` |
| `VacuumChannelResult` | 仅保留 `channel`、`command_accepted`、`valve_actuation_completed`、`verification_level`、`error` |

## 10. 边界验收清单

联合联调前必须逐条核对。任一项为否即接口边界不通过。

### 10.1 禁止的通信边

- [ ] ROS graph 中不存在 Autonomy → `/cmd_vel_safe`。
- [ ] ROS graph 中不存在 Autonomy → `/whole_body_jtc/follow_joint_trajectory`。
- [ ] ROS graph 中不存在 Autonomy → `/control/set_enabled`。
- [ ] ROS graph 中不存在 Autonomy → `/vacuum/pump/set_enabled`。
- [ ] ROS graph 中不存在 Autonomy → `/vacuum/grip`。
- [ ] 整个活动 G-01 中行为树调用 `/vacuum/pump/set_enabled=false` 的次数为零。
- [ ] Perception 不直接调用 Motion 或 Nav2。
- [ ] 不存在独立 Navigation 域服务端，也不存在对独立 Navigation 容器的必需依赖。
- [ ] 不存在 restart Topic、readiness epoch Service、command-disposition Service、G-01 到 RT-Control 的新增控制边或其他未授权 endpoint。

### 10.2 所有权唯一性

- [ ] `/cmd_vel_safe` 的唯一发布者是 Motion。
- [ ] Motion 是完整 14 轴 FJT 和 VacuumGrip 的唯一业务客户端。
- [ ] `map → odom` 只由 Perception 发布。
- [ ] `/tf` 中每一条坐标关系只有一个权威发布者。
- [ ] N-05、N-07、N-09、`/odom` 提供方是 Perception；N-01、N-02、N-03、N-06 服务端/发布者是 Motion。

### 10.3 时序与频率

- [ ] RT-Control 250 Hz 实时控制循环满足本机时序要求，且未跨容器。
- [ ] `/odom` 50 Hz，`/joint_states` 50 Hz；超过 200 ms 后不再作为新鲜状态证据。
- [ ] `/cmd_vel_safe` 停发或过期后 RT-Control 在 200 ms 内本地停车。
- [ ] N-03、M-07、SafetyState 使用 200 ms 严格时效，未套用 readiness 的 1 s 宽限。

### 10.4 语义与证据

- [ ] P-01 每个 `PickSequence` 提供非空、`map` 坐标系、可执行的 `station_nav_pose`。
- [ ] Autonomy 不查询、订阅或修改语义地图，也不自行生成导航位姿。
- [ ] `/diagnostics` 未被用来替代 Action Result、readiness、SafetyState 或 Gate 证据。
- [ ] G-01 Feedback 只使用冻结的 TaskPhase 0～14，且 `RELEASING_SEQUENCE_GATE=12`、`FINALIZING=13`、`CANCELING=14`。
- [ ] Action feedback 静默超过 1 s 只告警。
- [ ] 全部在部署节点使用同一版接口 schema，无新旧消息类型混跑。
- [ ] RELEASE 与 M-03 的载荷结果记为 `UNVERIFIED`，未被写成已验证释放。

### 10.5 Demo 可裁剪性

- [ ] Demo 在完全没有 P-03、Gateway、暂停恢复和持久化任务组件时可以正常启动和验收。
- [ ] Nav2 在 GPU 三维节点故障或 CUDA 不可用时仍能独立启动并进入可诊断状态。
- [ ] Perception 与 Motion 同时使用 GPU 的压力测试中，定位、底盘速度时限和显存使用满足运行要求。

## 11. 接口变更流程

1. 判定变更类型：新增字段/topic/状态码为**非破坏性**；删除或重命名字段、改变类型/单位/坐标系/终态语义/错误语义为**破坏性**。
2. 非破坏性变更：更新本文与 `src/interfaces`，PR 中列出全部生产者与消费者，通知受影响域负责人。
3. 破坏性变更：先开 design issue 说明原因、影响域、迁移方案与发布顺序，经接口所有者与全部消费域评审后，随统一 package 原子发布。**没有消费者迁移和原子发布方案不得合并。**
4. 两类变更都必须同批更新本文的接口总表、关键运行规则和边界验收清单。

详细协作与提交要求见 [collaboration-and-commit-standards.md](collaboration-and-commit-standards.md)。
