# ELECTRI-102 滚动关节控制执行计划

> 状态：**Gate 0 已批准；执行范围截止 Mock Checkpoint F1**
> 分支：`feature/electri-102-rolling-joint-control`
> 基线：`c4e8f90bea47a15890da45da653b109f875beedf`
> 执行者：Codex
> 独立评审：Claude Fable 5，`xhigh`，只读 plan mode
> 日期：2026-08-18
> 需求：[ELECTRI-102](https://linear.app/sevenova/issue/ELECTRI-102/rt-control-支持运控滚动目标队列实时更新与连续执行)
> 背景与交接：[electri-102-rolling-horizon-background-handoff.md](electri-102-rolling-horizon-background-handoff.md)

## 1. 结论先行

推荐为 ELECTRI-102 新增一个独立的 `rolling_trajectory_controller` ros2_control 插件，与现有 `dual_arm_jtc` 互斥占用完整 14 轴 position command interfaces；不要把高频滚动队列协议继续塞进 JTC，也不要让上层持续发送 `TwistStamped` 或反复替换长 FJT 来冒充该能力。

`enable_manager` 继续作为唯一控制器切换所有者，负责 JTC/rolling 模式切换以及 disable、group fault、意外掉使能时的抢占。rolling controller 自己拥有实时会话、固定容量未来缓冲、插值、连续性/限值校验、输入新鲜度、低水位、有界减速和状态发布。Motion 负责 IK、碰撞、奇异/跳变检查以及生成短时域完整 14 轴 `q/qdot` 目标。

本计划保留三道不可跨越的权限门：

1. **Gate 0（已通过）**：用户已批准本计划和默认设计裁决，允许在功能分支离线实现到 Mock Checkpoint F1。
2. **目标机非运动门**：构建、通信与压力测试需要单独授权，不使能、不运动。
3. **实机运动门**：只有动态限制、停止包络、急停/监护和低速步骤全部批准后，才能单独申请台架/实机运动授权。

## 2. 未明说但真正关键的问题

真正的问题不是“能否更快发送单点轨迹”，而是：

> 当 Motion 的更新经 DDS 异步到达、可能抖动/丢失/乱序/中断时，rt-control 如何在不重启控制器、不跳位置/速度、不破坏现有 FJT 与使能/故障语义的前提下，把最新的未来关节目标确定性地转成 250 Hz position command，并且在任何输入中断点都仍有可证明的有界停车路径？

因果链如下：

- 只提高 `/joint_states` 频率，只改变域外观测，不会形成 250 Hz 命令闭环；
- 只降低老化阈值，只会更早发现“数据旧”，不会自动产生连续、可行的停车轨迹；
- 只连续替换 JTC topic 轨迹，无法同时证明 session 隔离、authoritative suffix、`qdot` 拼接连续、固定容量 RT 交换和 timeout 后不追赶历史；
- 只有把会话、缓冲、替换、连续性、可停车性、生命周期和状态合同一起定义，才能满足 ELECTRI-102。

## 3. 已核实的仓库事实与约束

| 主题 | 当前事实 | 对计划的约束 |
|---|---|---|
| Controller Manager | 250 Hz，FIFO priority 80 | rolling 的采样和 command write 必须留在本进程 `update()`，不是 DDS 回环 |
| 运动接口 | 完整 14 轴；硬件 command/state 均只有 position | V1 仍接收 `qdot` 作为插值边界和连续性证据，但只写 position command |
| 普通轨迹 | `dual_arm_jtc`，`allow_partial_joints_goal=false`，`open_loop_control=false` | 必须保留普通 FJT 回归；rolling 与 JTC 不能同时 active |
| JTC 补丁 | 只接入 FJT Action goal admission | BQ-019 的窄补丁边界不得借 102 扩大成通用 rolling 实现 |
| 生命周期 | JTC 初始 INACTIVE；enable 后激活；disable/fault 路径当前硬编码 JTC | `enable_manager` 必须先有 characterization tests，再泛化为 active motion controller |
| Group fault | 任一轴稳定 Enabled 后失去 Operation Enabled，会 quick stop 并停用 JTC | 新模式必须把“停用 JTC”正式扩展为“停用当前 active motion controller” |
| `/joint_states` | BQ-068 已冻结为 50 Hz；250 Hz 官方整数分频无法精确得到 100 Hz，配置 100 会成为 125 Hz | ELECTRI-102 不修改 `/joint_states`；若要提高需另行取代 BQ-068 |
| 反馈老化 | FJT enter admission 500 ms；运行期 process age/WC 仍是 WARN-only | rolling enter 可申请复用 admission；运行中不得静默新增基于反馈 age 的自动停车 |
| 动态限制 | `joint_limits.yaml`、URDF 与 PLC 提取值存在来源/数值冲突；当前 launch 未加载生产 limits 文件 | 纯算法可用显式 test-only limits；生产/实机在唯一权威限制批准前保持 blocked |
| Mock | GenericSystem 的 `status_word=0x0040` 静态，不能模拟完整 CiA402 使能迁移 | `/rt/enable`、fault、mode switch 全链路需要专用 test fake/seam |
| 接口 | Phase 1 基线时 `robot_interfaces` 只有 `PlcIoState.msg`、`RtEnable.srv`；T1-02/T1-03 已新增 rolling V1 IDL | 新 IDL 必须与跨域合同保持原子一致，后续实现不得私建 wire 类型 |
| 跨域文档 | [cross-domain-interfaces.md](../../../docs/cross-domain-interfaces.md) 有用户未提交修改；其中原有 `/joint_states` 100 Hz 冲突已在 T1-04 合并纠正为 BQ-068 的 50 Hz | 保留用户其他修改；后续只在 rolling 合同范围做增量合并 |
| Watchdog | BQ-006 禁止独立 `rt_watchdog` 和 motion/autonomy heartbeat | 只允许 rolling controller 内部的输入新鲜度/低水位状态机 |

## 4. 范围与非范围

### 4.1 本需求负责

- 完整 14 轴 rolling mode 的 enter、update、state、exit、故障终止和 fresh-session recovery；
- session/boot/generation/sequence 隔离；
- 固定容量未来缓冲、不可修改边界、authoritative suffix 原子替换；
- `q/qdot` 三次 Hermite 采样、逐轴位置/速度/加速度/减速度与可停车性校验；
- 250 Hz RT 消费和 position command 输出；
- update timeout、低水位、队列耗尽前的受控停止与 terminal hold；
- 与 JTC 的严格互斥、disable/group-fault/切换歧义处理；
- Motion 可独立使用的公开 IDL、命名 QoS、Mock producer 和证据。

### 4.2 本需求不负责

- 视觉特征、相机标定、图像 Jacobian、IBVS/PBVS 算法；
- IK、碰撞检测、奇异规避、关节跳变规划和 6D 笛卡尔控制；
- 新增 velocity/effort 硬件 command interface；
- 修改硬急停、STO、安全继电器或 CiA402 硬安全语义；
- 把 `/joint_states` 从 50 Hz 提高；
- 恢复独立 watchdog 包或 motion/autonomy heartbeat；
- 未经单独授权的目标机部署、使能或运动。

## 5. 推荐架构与所有权

```text
Motion
  ├─ control plane ──> enable_manager
  │                     └─ 唯一调用 controller_manager strict switch
  │                        dual_arm_jtc <──互斥──> rolling_trajectory_controller
  │
  ├─ session/update ─> rolling_trajectory_controller 非 RT callbacks
  │                     └─ 校验后写入固定容量 generation 交换区
  │
  └─ state/reject <── rolling controller RT-safe snapshot/publisher

250 Hz controller_manager update()
  └─ rolling controller：读取完整 generation → 采样 q/qdot → 写 14×position
                                              └─ timeout/低水位 → 有界减速 → hold

enable_manager 始终高优先级处理 /rt/disable、group fault、掉使能和 switch ambiguity。
```

### 5.1 组件职责

| 组件 | 唯一职责 | 禁止承担 |
|---|---|---|
| `enable_manager` | 唯一 mode switch owner；记录 active motion controller；disable/fault 抢占 | 轨迹插值、滚动缓冲、视觉算法 |
| `rolling_trajectory_controller` | session、buffer、validation、250 Hz sampling、input freshness、stop、state | IK、碰撞、直接控制 CiA402 control word |
| `dual_arm_jtc` | 既有完整 14 轴 FJT | rolling session/queue 协议 |
| `robot_interfaces` | 公共 schema 与版本 | 运行时策略和参数默认值 |
| `rt_control_bringup` | 配置、spawn INACTIVE、依赖安装 | 跨域算法语义 |
| Motion Mock | 只按公开合同生产短时域队列和故障注入 | 读取控制器私有内存或绕过 mode owner |

### 5.2 推荐状态机

```text
FJT_READY
  └─ cancel/wait FJT + source-quiescent gate + set_mode(ROLLING)
       └─> ROLLING_READY (active + validated last-command hold)
       └─ open_session ─> PRIMING
            └─ 首批满足 horizon/连续性/可停车性 ─> RUNNING
                 ├─ 正常 close / timeout / low-water ─> STOPPING ─> HOLDING
                 └─ disable / group fault / internal fatal ─> TERMINATED + active controller deactivation
       HOLDING ── close_session ─> ROLLING_READY
  ROLLING_READY ── set_mode(FJT) ─> FJT_READY
```

V1 不允许 `HOLDING → PRIMING` 复用旧 session；必须 close 后重新 open。任何 timeout、queue exhaustion、producer restart、controller restart 或 boot ID 变化都要求 fresh session。

补充边界：

- `PRIMING/RUNNING/STOPPING/HOLDING` 中请求切回 FJT，一律拒绝并要求先完成 close；
- 重复请求当前 mode 是幂等查询，不重启 controller，也不改变现有 session；
- disable/group fault 与在途 mode switch 竞争时，disable/fault 优先；switch 结果不确定仍按 BQ-058/BQ-080 进入 `restart_required`；
- 普通 mode switch 不提供“运动中强切”能力；硬件安全抢占继续走既有 disable/fault/quick-stop 路径。

## 6. Gate 0 默认裁决包

用户批准本计划，即表示批准下表“默认建议”进入 Phase 0 的正式 BQ/协议文本；若希望修改，只需指出 ID。handoff D-006（即本计划 R-008）的生产数值不因批准计划而自动批准。

| ID | 默认建议 | 理由/代价 | 状态 |
|---|---|---|---|
| R-001 架构 | 新建 `rolling_trajectory_controller`，不扩大 JTC fork | 保留 BQ-019 与普通 FJT；代价是新增插件和模式切换测试 | 已批准（BQ-130） |
| R-002 mode owner | `enable_manager` 是唯一 strict switch owner；rolling/JTC 同时至多一个 active；它读取共享 position state 做 source-quiescent gate | 延续 BQ-080；需先建立 enable_manager 测试基线并泛化 fault/disable 路径 | 已批准（BQ-130） |
| R-003 active FJT / 切换 | 正常切换只允许源运动已稳态：Motion 先 cancel 自己的 FJT 并等待 Action result；rt-control 再以 N 周期位置有限差分和接管误差门控。成功停用 JTC 时，若仍有在途 goal，JTC 原生 `on_deactivate()` 会 abort，Action result 才是 goal 终态权威；mode response 只回报源 controller 已停用，不声称检测到 goal 是否存在 | 避免运动中切到 hold 造成无界减速度；不依赖有竞态的 action-status 快照，也不扩大 JTC 补丁。V1 不向 Motion 暴露 force-preempt mode；disable/fault 另走既有硬优先路径 | 已批准（BQ-130） |
| R-004 目标/连续性 | 固定 14 轴 position + velocity 必填；分段 cubic Hermite；要求 C0/C1；V1 不要求 C2 | position-only 无法证明替换时 `qdot` 连续；硬件仍只写 position | 已批准（BQ-130） |
| R-005 时间 | 消息时间是 session-relative；RT execution time 累加经范围校验的 `update()` period；update arrival age 使用 monotonic steady clock，且 arrival timestamp 与 generation 原子交接；墙钟只作日志 | 避免跨机时钟和墙钟回拨；若 period 非正/异常大则锁定 `CLOCK_ANOMALY` stop，并按名义 4 ms 小步推进已生成停车段，不用异常大 period 跳采样 | 已批准（BQ-130） |
| R-006 替换/序号 | `replace_from` 必须等于新批首点；committed prefix 不可改；suffix 整批原子替换；高 sequence 自包含时可跳号；重复/低序拒绝；结构失败也消耗 sequence | 最新完整未来优先；避免补追历史和隐式锚点 | 已批准（BQ-130） |
| R-007 安全停止 | timeout 或 `H_available <= T_stop(qdot)+guard` 时锁定 STOPPING；在 RT 中用固定存储一次生成同步 C1 减速段，随后 hold；STOPPING 后所有 update 拒绝；V1 用保守解析上界验证整段 stopping viability | 比要求 Motion 每批携带停车尾更符合 rt-control 的安全职责；保守上界可能在软限位附近误拒，但证明边界清晰 | 已批准（BQ-130） |
| R-008 动态限制 | 建立 14 轴唯一 position/velocity/方向相关 acceleration/deceleration、mode-switch stability/tracking-error threshold authority；未关闭项为 production/hardware blocker；测试值必须显式标记且不能进入生产配置 | 当前 URDF/PLC/limits 冲突，不能猜值 | **数值保持 TBD** |
| R-009 反馈 age | rolling enter 申请复用 500 ms admission check；runtime 只观察反馈 age/WC，不新增自动停车；输入 timeout/low-water 属 rolling 内部 | 不暗改 BQ-045；硬件安全仍由既有路径负责 | 已批准（BQ-130） |
| R-010 session recovery | timeout/耗尽/fault 后不自动恢复；terminal hold 后 close + fresh open | 状态空间更小，旧数据不会复活 | 已批准（BQ-130） |
| R-011 public contract | V1 IDL 落 `robot_interfaces`；update/state 使用新增命名 QoS profile；schema 与跨域文档同一变更冻结 | 满足跨域接口规则；QoS 数值需压测后生产冻结 | 已批准（BQ-130；数值仍 TBD） |
| R-012 `/joint_states` | 保持 BQ-068 的 50 Hz；rolling 直接读 state interfaces | 它不是命令环；若视觉闭环另需更高速观测，另立需求/裁决 | 已冻结，不在 102 修改 |

### 6.1 R-007 必须维持的数学不变量

对每个轴 `i`，定义 `v_i^+=max(v_i,0)`、`v_i^-=max(-v_i,0)`，并使用经过批准的方向相关减速度 `a_stop_i^+ > 0`、`a_stop_i^- > 0`。任意可执行样本 `(q_i, v_i)` 的同步停车时间为

```text
T_stop = max_i(v_i^+ / a_stop_i^+, v_i^- / a_stop_i^-)
q_stop_i = q_i + 0.5 * v_i * T_stop
```

当全轴 `v=0` 时，`T_stop=0` 并立即进入 hold，不执行除零。其他情况下，同步段每轴实际减速度 `|v_i|/T_stop` 不超过对应限制。`q_stop` 必须落在扣除了批准 tracking-error、离散采样和软限位 margin 的安全 position bounds 内，速度、加速度和 250 Hz 离散 command 也不得越界。

V1 不声称 14 轴耦合 stopping envelope 本身可直接做精确解析极值。对每个 cubic segment，先解析求每轴 `q_i,min/q_i,max`、最大正速度 `v_i,max^+` 和最大负速度幅值 `v_i,max^-`，再构造保守上界：

```text
T_stop,seg^max = max_j(v_j,max^+ / a_stop_j^+, v_j,max^- / a_stop_j^-)
upper_stop_i = q_i,max + 0.5 * v_i,max^+ * T_stop,seg^max
lower_stop_i = q_i,min - 0.5 * v_i,max^- * T_stop,seg^max
```

只有 `upper_stop_i/lower_stop_i` 都在收缩后的安全 bounds 内，整段才可接受。这个上界把不同时刻的 q/v 极值组合起来，可能在软限位附近保守误拒；V1 接受这个代价，换取不依赖未知 RT 采样相位的可证明性。若后续要改成网格+margin，必须另行证明 period 抖动/相位覆盖，不能在实现中静默替换。

同步匀减速只保证 q/qdot 连续；进入/离开 STOPPING 时加速度可跳变，jerk 在理想模型中无界。该 C1-only 限制必须在 P0-03 被显式批准，不能被描述成 C2/jerk-limited stop。

生产运行还必须维持：

```text
H_available > T_stop + scheduling_guard
```

当不等式即将失效时必须先进入 STOPPING，不能等队列已经耗尽。P0-03 先冻结 `scheduling_guard` 的组成，至少包括一个周期的检测延迟、该周期内 `T_stop` 的最大增长、non-RT→RT generation 交接抖动和 period/采样量化 margin；Phase 5/6 再用测量填数。update timeout、最大 horizon、拼接/接管容差也不得凭经验写死。

## 7. 公共合同草案边界

以下名称只用于拆解文件和职责，Phase 0 可在不改变语义的前提下统一命名：

- mode control：`SetJointControlMode.srv`，由 `enable_manager` 提供；
- session control：`OpenRollingJointSession.srv`、`CloseRollingJointSession.srv`，由 rolling controller 提供；
- data plane：`RollingJointPoint.msg`、`RollingJointTargetBatch.msg`；
- state plane：`RollingJointControlState.msg`；
- update topic 使用命名 profile `Q_ROLLING_COMMAND`，state topic 使用 `Q_ROLLING_STATE`；可靠性、depth、deadline、lifespan 数值在压测前标记为 prototype，不得宣称 production frozen。

IDL 必须覆盖：schema major/minor、controller boot ID、session ID、generation、sequence、session-relative time、固定 14 轴顺序/单位、replace boundary、accept/reject、stop reason、buffer horizon、update age、mode、progress、取消/终止、重启兼容。`RejectCode` 与 `StopReason` 分开，避免把“本批拒绝但 session 继续”和“session 正在终止”混为一类。

## 8. 通用执行规则与验证命令

### 8.1 工作区规则

- 不修改或清理用户已有的未提交文件。当前已知 dirty 清单为：
  - `docs/cross-domain-interfaces.md`；
  - `tools/repository_gate.py`、`tools/tests/test_repository_gate.py`、`tools/tests/test_rt_io_integration.py`；
  - `domains/rt_control/docs/deployment-operations-runbook.md`；
  - `domains/rt_control/docs/docker-deployment-performance-summary.md`；
  - `domains/rt_control/docs/host-setup-record.md`；
  - `domains/rt_control/docs/integration-readiness-summary.md`；
  - `domains/rt_control/docs/onboarding-knowledge-map.md`。
- T1-04、T4-07 或任何其他任务触及上述文件前，必须由用户先提交/处置，或明确授权我在现有修改上合并；否则命中 stop condition。
- 每项任务开始前执行 `git status --short --branch` 并核对预期文件。
- 文件编辑只落在本分支，不 commit、不 push，除非用户另行明确要求。
- build/install/log/vendor 保持在仓库外：`/home/kkozia/rt-control-dev-local`。
- 不运行真实 hardware launch、`/rt/enable`、SDO write、controller activation 或运动命令。

### 8.2 复用验证命令

文档/仓库门禁：

```bash
git diff --check
tools/quality_gate.sh
python3 tools/repository_gate.py
```

原生构建（按任务替换 package 列表）：

```bash
RT_CONTROL_NATIVE_WS=/home/kkozia/rt-control-dev-local tools/bootstrap_native_dev.sh doctor
RT_CONTROL_NATIVE_WS=/home/kkozia/rt-control-dev-local tools/bootstrap_native_dev.sh build --packages-select <packages>
```

定向测试：

```bash
cd /home/kkozia/rt-control-dev-local
colcon test --packages-select <packages>
colcon test-result --verbose --test-result-base build
```

每个 C++ RT 任务还必须补充人工检查：`update()` 内无 heap allocation、mutex、阻塞 wait、文件/网络 I/O、日志风暴或共享对象析构；所有数组边界、NaN/Inf、单位、溢出、生命周期和错误传播有对应测试。

当前计划文档验证中，`git diff --check` 和相对链接检查已通过。`repository_gate.py`/`quality_gate.sh` 当前只报告 `docker/compose.yaml` 的 middleware/domain/volume 规则失败；`docker/compose.yaml` 不是本任务修改，而 gate 本身处于用户 dirty 状态。因此该结果记为“基于用户未提交 gate 的基线失败”，不能由 ELECTRI-102 顺手修复，也不能伪报全绿；用户处置相关修改后需重新建立门禁基线。

## 9. 详细执行任务

任务大小：XS < 0.5 天，S ≈ 1 天，M ≈ 2～3 天。估算用于拆分，不是交付承诺。每项最多约 5 个文件；若实现发现需要扩大，先拆任务再继续。

### Phase 0 — 正式冻结设计（仅文档，无功能代码）

#### P0-01（S）冻结架构、切换和故障关系

- **目标**：把 R-001、R-002、R-003、R-009、R-010 写成一条新的 BQ/ADR，并给出 enter、运行、exit、disable、group fault 五条时序；冻结“先 cancel/wait，再稳态/接管门控，最后 strict switch”的正常切换协议。
- **可能文件（≤3）**：`domains/rt_control/BLOCKED-questions.md`、本计划、`electri-102-rolling-horizon-background-handoff.md`。
- **依赖**：用户批准 Gate 0。
- **验收**：任意状态至多一个 controller claim 14 个 position interfaces；disable/fault 一定指向当前 active controller；普通 mode switch 不会在非零运动中直接切成 hold，且 BQ-019/JTC 补丁边界保持原义。
- **验证**：逐项对照 BQ-006/019/041/042/043/044/045/059/080/118；`git diff --check`。
- **停止条件**：正式文字需要推翻而非扩展任一冻结 BQ，或团队要求 V1 向 Motion 暴露运动中 force-preempt；后者需单独批准物理停车后果和新设计。
- **执行级别**：离线文档。

#### P0-02（M）冻结 session、时间、替换和状态机

- **目标**：形成唯一可判定的 open/update/close、boot/session/generation/sequence、committed boundary 和 fresh-session 规范。
- **可能文件（≤2）**：新建 `domains/rt_control/docs/electri-102-protocol-spec.md`、本计划。
- **依赖**：P0-01。
- **验收**：至少 20 个到达顺序/边界例子均有唯一 accept/reject、buffer 和 state 结果；明确 period/steady-clock 异常以及 STOPPING 中异常 period 的名义步进；覆盖重复 mode、活动 session 中切 FJT、disable 与 switch 竞争；删除同 session `HOLDING→PRIMING`。
- **验证**：表驱动例子双人独立判定；文档链接检查；`git diff --check`。
- **停止条件**：同一例子仍可由两种解释得到不同结果。
- **执行级别**：离线文档。

#### P0-03（M）冻结连续性、可停车性和限制证据矩阵

- **目标**：把 R-004、R-007、R-008 的 cubic 极值、C0/C1、保守 stopping-viability 上界、guard 组成、mode-switch 稳态/接管门槛与 14 轴限制来源写成可测试规则。
- **可能文件（≤2）**：新建 `domains/rt_control/docs/electri-102-dynamic-envelope.md`、`electri-102-protocol-spec.md`。
- **依赖**：P0-01。
- **验收**：每轴 position/velocity/方向相关 acceleration/deceleration、稳态与 tracking-error margin 都有唯一权威来源或明确 `BLOCKING_TBD`；测试值与生产值物理隔离；保守 envelope/guard 公式、全轴零速路径和 C1-only/jerk 不连续限制明确；不把 FJT 的 1°/0.05 m admission tolerance 当拼接/接管 tolerance。
- **验证**：对照 `joint_limits.yaml`、URDF、PLC 提取记录与 BQ-067/BQ-118；离线手算/脚本复核停车公式。
- **停止条件**：必须猜一个生产数值才能继续；只阻塞 production/hardware，不阻塞显式 test limits 的纯算法。
- **执行级别**：离线文档/数学验证。

**Checkpoint A（3 项后）**：Codex 自审 + Claude 只读复审决策文本。若与 Gate 0 默认裁决一致则继续；任何语义偏离都回到用户，不自行决定。

### Phase 1 — 回归基线和公共接口

#### T1-01（M）建立 enable_manager characterization test 基线

- **目标**：在不改变生产行为的前提下，覆盖现有 enable/disable/preempt/fault/reset/switch ambiguity 语义。
- **可能文件（≤5）**：`enable_manager/CMakeLists.txt`、`enable_manager/package.xml`、`enable_manager/test/test_enable_manager_characterization.cpp`、至多两个 test fake/helper。
- **依赖**：Checkpoint A。
- **验收**：覆盖 IDLE/ENABLING/ENABLED、disable 抢占、EmergencyQuickStop、downward、reset、switch ambiguous→restart_required；生产 source 行为无变更；失败路径有断言。
- **验证**：原生 build；`colcon test --packages-select enable_manager`；`colcon test-result --verbose --test-result-base build`。
- **停止条件**：不改生产代码无法注入 status_word 或 controller switch；若需要最小 test seam，先单独报告 diff 和行为零变化证据。
- **执行级别**：单元/fake，无硬件。

#### T1-02（S）定义 control-plane IDL

- **目标**：定义 mode/open/close 三个 service 的版本、ID、幂等、source-quiescent rejection、timeout、取消、重启和返回语义。
- **可能文件（5）**：`robot_interfaces/srv/SetJointControlMode.srv`、`OpenRollingJointSession.srv`、`CloseRollingJointSession.srv`、`CMakeLists.txt`、`package.xml`。
- **依赖**：P0-01、P0-02。
- **验收**：重复请求可判定；mode response 只回报 source controller 是否成功停用/target 是否激活，不声称知道 FJT goal 是否存在；boot/session/version 不匹配有稳定错误码。
- **验证**：原生 build `robot_interfaces`；rosidl 生成类型检查；schema 静态测试。
- **停止条件**：服务归属或 mode/session 两阶段语义仍未冻结。
- **执行级别**：接口构建，无运行时。

**Checkpoint B1（2 项后）**：检查 characterization coverage 与 control-plane schema；Claude 只读 review，不运行实现代理。

#### T1-03（S）定义 data/state-plane IDL

- **目标**：定义固定 14 轴 point/batch/state，拆分 `RejectCode` 与 `StopReason`。
- **可能文件（5）**：`robot_interfaces/msg/RollingJointPoint.msg`、`RollingJointTargetBatch.msg`、`RollingJointControlState.msg`、`CMakeLists.txt`、`package.xml`。
- **依赖**：P0-02、P0-03、T1-02。
- **验收**：point 固定数组由 Humble rosidl 生成 `std::array`；batch 外层 points 有协议最大长度；state 足以让 Motion 不读日志判断进度/健康/终止。
- **验证**：原生 build；编译期 `static_assert`/schema tests；越界长度与 major-version mismatch 测试向量。
- **停止条件**：字段需要未批准的生产限值或 QoS 数值才能编译。
- **执行级别**：接口构建，无运行时。

#### T1-04（S）原子冻结跨域 endpoint 与命名 QoS

- **目标**：把 mode/open/update/state/close 条目和 `Q_ROLLING_COMMAND`/`Q_ROLLING_STATE` 纳入跨域合同，并同步纠正 BQ-068 的 50 Hz 条目。
- **可能文件（≤3）**：`docs/cross-domain-interfaces.md`、`electri-102-protocol-spec.md`、必要的接口合同测试文件。
- **依赖**：T1-02、T1-03；用户先处置当前 dirty 的 cross-domain 文档，或明确授权我在其修改上合并。
- **验收**：IDL 与文档名称/方向/单位/时效/拒绝/重启/版本完全一致；没有逐 endpoint 私有 QoS；未测数值明确标成 prototype blocker。
- **验证**：`rg` 双向核对全部 endpoint/type/profile；repository gate；原生构建所有接口消费者。
- **停止条件**：无法在不覆盖用户修改的情况下合并；QoS 被要求以无证据数值直接标成 production。
- **执行级别**：文档/接口。

**Checkpoint B2（2 项后，接口冻结）**：用户仅在 endpoint 命名或语义偏离 Gate 0 时介入；否则保存构建/合同证据并继续。

### Phase 2 — 纯 C++ 确定性核心（无 ROS graph、无硬件）

#### T2-01（M）创建 package 与固定容量 buffer 骨架

- **目标**：建立无动态增长的内部 point/segment/generation 类型、容量边界和 GoogleTest 基础设施。
- **可能文件（5）**：新包 `CMakeLists.txt`、`package.xml`、`include/.../rolling_buffer.hpp`、`src/rolling_buffer.cpp`、`test/test_rolling_buffer.cpp`。
- **依赖**：T1-03。
- **验收**：容量在 configure 时固定；NaN/Inf、空批、非递增时间、错误轴数全部拒绝；reject 后 active generation 字节级不变。
- **验证**：原生 build/test；ASan/UBSan 纯库构建；边界容量 `0/1/max/max+1`。
- **停止条件**：实现需要在 RT 路径创建/扩容 STL 容器。
- **执行级别**：纯单元测试。

#### T2-02（M）实现 session/sequence/authoritative suffix

- **目标**：实现 boot/session/generation/sequence 校验、committed boundary、suffix 全有或全无替换和 sampling cursor。
- **可能文件（≤4）**：`rolling_buffer.hpp/.cpp`、`test/test_rolling_buffer.cpp`、新 `test/test_protocol_vectors.cpp`。
- **依赖**：T2-01、P0-02 的 20+ 向量。
- **验收**：duplicate/old/late/history rewrite 拒绝；self-contained gap 可接受；无任何 partial mutation。
- **验证**：协议向量逐项自动化；属性测试随机排列 update 到达顺序；ASan/UBSan。
- **停止条件**：测试暴露 P0-02 文本未覆盖的语义分叉。
- **执行级别**：纯单元测试。

**Checkpoint C1（2 项后）**：只看 deterministic buffer 结果，不接 ROS；保存随机种子和 sanitizer 结果。

#### T2-03（M）实现 cubic Hermite 与解析限值检查

- **目标**：采样 `q/v/a`，解析检查段内 position/velocity/acceleration 极值和 splice C0/C1。
- **可能文件（5）**：`cubic_hermite.hpp/.cpp`、`limit_checker.hpp/.cpp`、`test/test_interpolation_and_limits.cpp`。
- **依赖**：T2-01、P0-03。
- **验收**：端点精确命中；解析值与高精度参考/有限差分一致；rotary rad 与 prismatic m 用例不混单位。
- **验证**：GoogleTest 参数矩阵；随机曲线参考对照；ASan/UBSan。
- **停止条件**：生产数值未知不阻塞算法，但任何 test limit 未显式标记时停止。
- **执行级别**：纯单元测试。

#### T2-04（M）实现 stopping viability 与一次性减速规划

- **目标**：实现 `T_stop`、`q_stop`、软限位可停车域、低水位判定和固定存储 STOPPING trajectory。
- **可能文件（≤5）**：`session_core.hpp/.cpp`、`limit_checker.hpp/.cpp`、`test/test_stopping_envelope.cpp`。
- **依赖**：T2-03、P0-03。
- **验收**：多初速度/方向/软限位/14 轴同步矩阵通过；保守 segment envelope 无漏接收（可有已记录的保守拒绝）；全轴零速立即 hold；STOPPING 后 update 不可取消且最终 `v=0` 后只 hold。
- **验证**：公式对照；临界值 ±1 ns/±1 cycle；不等式属性测试；ASan/UBSan。
- **停止条件**：任一可接受 normal sample 没有可行 stop，或停止算法需要未界定的隐式 clamp。
- **执行级别**：纯单元测试。

#### T2-05（M）实现非 RT→RT 固定存储 generation 交换

- **目标**：用预分配多槽/所有权协议让 RT 只看到完整 immutable generation，并把 generation 与 steady arrival timestamp 原子配对，避免 shared_ptr 最后析构落在 RT。
- **可能文件（≤4）**：`rolling_snapshot.hpp/.cpp`、`test/test_rolling_snapshot.cpp`、必要的 package/CMake 更新。
- **依赖**：T2-02、T2-04。
- **验收**：RT 无锁读取完整 generation+arrival timestamp；producer 不覆盖正在读的槽；高并发 publish/read 下无 torn snapshot、年龄错配或 ABA。
- **验证**：TSan 仅跑纯核心；百万次 producer/consumer stress；RT allocation trap。
- **停止条件**：需要 mutex、等待或可能在 RT 中触发对象析构/分配。
- **执行级别**：纯并发测试。

**Checkpoint C2（3 项后）**：Codex 做数学/RT 自审，Claude 只读 review 核心 diff 和测试；发现不变量缺口先修，不接 controller_manager。

### Phase 3 — rolling controller 插件（fake controller_manager）

#### T3-01（M）测试先行实现 controller lifecycle 与接管校验

- **目标**：controller 可 configure/activate/deactivate，声明完整 14 轴 position command/state 和共享 `ethercat_domain/process_data_age_ms` state；先写 lifecycle/接管测试，再实现最小类。
- **可能文件（5）**：controller `hpp/cpp`、`test/test_controller_lifecycle.cpp`、`CMakeLists.txt`、`package.xml`。
- **依赖**：T1-03、T2-01。
- **验收**：激活只接受 finite 且与 actual 在批准接管误差内的既有 position command；第一周期保持该 command，保证 command C0；NaN/stale/out-of-tolerance command 确定性失败，不会跳到 actual。
- **验证**：controller_interface 单元测试；command/actual residual 边界；process-age state interface 的声明/读取/缺失测试；无真实 launch。
- **停止条件**：同一 enable session 内无法在 activation 读取持久 command 值，或必须用 actual-position 阶跃替换 command 才能激活。
- **执行级别**：fake ros2_control。

#### T3-02（S）注册插件并验证资源互斥

- **目标**：完成 pluginlib 注册和 fake resource-manager strict switch tests，证明 JTC/rolling 不能双占 14 轴。
- **可能文件（≤5）**：plugin XML、`CMakeLists.txt`、`package.xml`、`test/test_controller_switch.cpp`、一个 fake helper。
- **依赖**：T3-01。
- **验收**：插件可加载；JTC 与 rolling 双 active 失败；合法 source-quiescent switch 后第一条 rolling command 与切换前 command 相等。
- **验证**：pluginlib load test；fake controller_manager activate/deactivate/switch success/failure；资源 claim 列表断言。
- **停止条件**：测试只能靠真实 hardware 或默认 launch 才能构造接口。
- **执行级别**：fake controller_manager。

**Checkpoint D1（2 项后）**：检查接管误差、command 连续、process-age admission 和资源互斥证据。

#### T3-03（M）接入 open/update/close 非 RT callbacks

- **目标**：IDL → validation/core → generation publish；callback 只提交全量合法 generation，session 控制幂等。
- **可能文件（≤5）**：controller `hpp/cpp`、`session_core.hpp/.cpp`、`test/test_controller_callbacks.cpp`。
- **依赖**：T1-02、T2-02、T2-05、T3-02。
- **验收**：错误 version/ID/sequence/batch 不改变 active generation；open 对缺失/过旧 process age 按新 BQ 拒绝；close 锁定 stop；deactivate 清除 session/boot epoch。
- **有界幂等**：每个活动 session 固定缓存 8 个 non-finalizing close 结果；额外新 ID 返回 `WrongRequest`；最近一次 finalize 结果使用独立单槽缓存，boot 变化时失效。
- **验证**：executor callback tests；重复/并发 open-close-update；ROS message↔core 映射测试。
- **停止条件**：callback 并发模型没有单一 writer 或无法证明 generation/timestamp 所有权。
- **执行级别**：ROS 单元/fake，无硬件。

#### T3-04（M）接入 250 Hz RT sampling/write

- **目标**：`update()` 推进 session time、读取完整 generation、采样/停止并写 14 个有限 position commands。
- **可能文件（≤4）**：controller `hpp/cpp`、`test/test_rt_update.cpp`、必要的 core 文件。
- **依赖**：T2-03～T2-05、T3-03。
- **验收**：运动中 batch replacement 不触发生命周期重启；每周期恰写 14 项 finite 值；异常 period 锁定 `CLOCK_ANOMALY` 并按名义周期小步推进停车，不用异常 delta 跳采样。
- **验证**：250 Hz fake loop；allocation trap；长稳与 jitter/period anomaly 注入；人工逐行 RT audit。
- **停止条件**：`update()` 出现 heap、锁、阻塞、ROS service call、动态日志或非确定遍历。
- **执行级别**：fake 250 Hz。

#### T3-05（S）实现 RT-safe state/reject/stop publisher

- **目标**：从 RT snapshot 使用 `realtime_tools::RealtimePublisher`/仓库既有 seqlock 模式发布 public state。
- **可能文件（≤4）**：controller `hpp/cpp`、`test/test_state_publisher.cpp`、必要的 QoS config test。
- **依赖**：T1-03、T3-04。
- **验收**：Motion 可判定最后 accept/reject、progress、horizon、update age、STOPPING/HOLDING、stop reason；state 发布失败不阻塞 update；session 终止变化可观测。
- **验证**：内部模型与 topic state 逐场景断言；慢订阅者/无订阅者测试；allocation audit。
- **停止条件**：发布路径会让 RT 等待 DDS 或复用批次 reject 作为 stop reason。
- **执行级别**：ROS/fake。

**Checkpoint D2（3 项后）**：rolling 插件独立功能评审；仍不触碰 enable_manager/bringup 实机路径。

### Phase 4 — enable_manager 与 bringup 集成（全部 fake/mock）

#### T4-01（S）无行为重构 motion-controller registry

- **目标**：把 `jtc_name_`/`switchJtc()` 泛化为配置的 motion controller registry + active controller 记录，但保持现有外部行为完全不变。
- **可能文件（≤4）**：`enable_manager_controller.hpp/.cpp`、`controllers.yaml`、T1-01 test。
- **依赖**：T1-01、R-002 已冻结。
- **验收**：T1-01 原断言不修改即全过；默认 active 仍是 JTC；diff 不引入新 mode service/分支。
- **验证**：enable_manager build/test；旧 FJT enable/disable characterization；逐行 diff review。
- **停止条件**：无法做到无行为重构，必须引入新语义。
- **执行级别**：单元/fake。

#### T4-02（M）实现 source-quiescent mode switch

- **目标**：实现 `SetJointControlMode`；仅在总体 Enabled、14 轴 position state 连续 N 周期稳态且 source→actual 接管条件满足时，用单次 STRICT request 原子 deactivate current + activate target。
- **可能文件（≤5）**：`enable_manager_controller.hpp/.cpp`、`controllers.yaml`、两个 mode/stationarity tests。
- **依赖**：T1-02、T3-02、T4-01。
- **验收**：运动中请求确定性拒绝；成功后恰一个 active 且 command 不阶跃；mode response 只回报 controller switch，任何 FJT goal 终态由 Action result 回报；失败/超时/歧义沿用 BQ-080 `restart_required`。
- **验证**：mock controller_manager success/reject/timeout/ambiguous；有限差分窗口/阈值边界；cancel-wait 后切换；重复 mode request 幂等；在途 FJT 到达/终止竞争测试。
- **停止条件**：需要订阅 action status 才能保证物理稳态、需要扩大 JTC 补丁，或团队要求无稳态门直接 force switch。
- **执行级别**：mock controller_manager/state interfaces。

**Checkpoint E1（2 项后）**：Claude 只读 review `enable_manager` 重构与 switch diff；普通 FJT 回归证据必须保持绿。

#### T4-03（M）泛化 disable/fault/掉使能抢占

- **目标**：所有已有“deactivate JTC”路径都改为 deactivate 当前 active motion controller，并在 rolling deactivate 时销毁 session。
- **可能文件（≤5）**：`enable_manager_controller.hpp/.cpp`、characterization test、fault test、rolling lifecycle test。
- **依赖**：T4-02、T3-03。
- **验收**：每个 rolling 状态下注入 `/rt/disable`、group fault、unexpected Operation Enabled loss，当前 controller 均被停用；硬件 quick-stop 顺序不变；旧 session/FJT 不复活。
- **验证**：状态×事件矩阵；现有 BQ-041/042/044/059 回归；switch ambiguity 仍 restart-only。
- **停止条件**：任何测试要求弱化既有 group fault/disable 语义。
- **执行级别**：fake status/controller_manager。

#### T4-04（S）测试先行搭建 CiA402 fake system

- **目标**：补足 GenericSystem 静态 status_word 的缺口，建立只在 `BUILD_TESTING` 下使用的可迁移 CiA402 fake plugin。
- **可能文件（5）**：fake `hpp/cpp`、test plugin XML、对应 `CMakeLists.txt`、`package.xml`。
- **依赖**：T1-01、T4-03。
- **验收**：可脚本化 expected status transitions、fault、掉使能；不链接真实 EtherCAT；production install/launch 不可发现该插件。
- **验证**：plugin unit tests；install manifest 审查；mock-only/`BUILD_TESTING` 静态检查。
- **停止条件**：fake 会被默认安装/launch 到生产路径，或需要真实 EtherCAT 才能运行。
- **执行级别**：专用 fake system。

#### T4-05（M）完成 CiA402/controller-manager 全链路矩阵

- **目标**：用 T4-04 fake 自动跑 enable→mode switch→disable/fault/recovery，不改生产 xacro。
- **可能文件（≤5）**：test URDF/xacro、integration test、scenario table、expected results、必要的 test CMake 更新。
- **依赖**：T4-04。
- **验收**：每个 rolling 状态可注入 status/fault；无真实 command 输出；source-quiescent、switch ambiguity、旧 session/FJT 不复活均有断言。
- **验证**：controller_manager integration tests；完整状态×事件矩阵；普通 FJT 对照场景。
- **停止条件**：任何场景需要真实 hardware、默认 launch 或弱化既有 BQ 才能通过。
- **执行级别**：专用 fake system/controller_manager。

**Checkpoint E2（3 项后）**：执行完整 fault/disable/mode matrix；这是更改既有安全语义载体后的强制证据点。若失败不进入 bringup。

#### T4-06（S）bringup 配置 rolling controller 为 INACTIVE

- **目标**：安装/加载插件、配置参数并 spawn rolling INACTIVE；enable 后默认仍只激活 JTC。
- **可能文件（≤5）**：`controllers.yaml`、`rt_control.launch.py`、`rt_control_bringup/package.xml`、`CMakeLists.txt`、launch test。
- **依赖**：T3-05、T4-05。
- **验收**：启动后 enable_manager ACTIVE、JTC/rolling 初始状态符合既有合同；未显式 mode switch 不会 claim rolling interfaces；普通 FJT 路径不变。
- **验证**：mock launch test；`ros2 control list_controllers` 状态断言；全包 build/test。
- **停止条件**：launch 会自动 enable、activate rolling 或触及真实 hardware。
- **执行级别**：`use_mock_hardware:=true` only。

#### T4-07（S）更新原生依赖/安装清单与仓库门禁

- **目标**：让新 package/interface 在 BQ-128 原生工作区可复现构建，不改变冻结 vendor commits。
- **可能文件（≤4）**：`deps.repos`（仅确需新 vendor 时；默认不改）、`tools/bootstrap_native_dev.sh`、相关 repository gate test、package manifests。
- **依赖**：T4-06；触及当前 dirty gate 文件前必须先满足 §8.1 的用户处置条件。
- **验收**：全新外部 workspace 可发现新包；frozen vendor tree 验证仍通过；没有仓库内 build/install/log。
- **验证**：bootstrap doctor/build；repository gate；`git status --short` 检查无生成物。
- **停止条件**：需要新增/升级 vendor 版本，或与用户 dirty 的 gate/tests 无法安全合并；先单独评审/处置。
- **执行级别**：构建/静态门禁。

**Checkpoint E3（2 项后）**：mock bringup + 普通 FJT + rolling lifecycle 回归；用户只有在需改冻结依赖、dirty gate 文件或默认启动语义时介入。

### Phase 5 — Mock producer、异常矩阵与参数证据

#### T5-01（M）实现只依赖 public IDL 的 Motion Mock producer

- **目标**：支持 FJT cancel/wait、稳态后 mode switch、open/prime/update/close/return-to-FJT，并注入 jitter/drop/duplicate/reorder/pause/discontinuity/version/session 错误。
- **可能文件（≤5）**：独立 test/tool package 的 `CMakeLists.txt`、`package.xml`、producer source、scenario config、test。
- **依赖**：T1-04、T4-06。
- **验收**：不读取 controller 私有 state/file；频率/horizon/扰动可配置；输出 machine-readable scenario result。
- **验证**：mock launch 一键场景；接口依赖图检查；重复运行结果一致。
- **停止条件**：Mock 必须调用 controller_manager switch service 才能工作（应只调用 public mode service）。
- **执行级别**：mock/fake。

#### T5-02（M）执行功能/异常/恢复全矩阵

- **目标**：覆盖正常滚动、运动中替换、边界拒绝、运动中 mode 请求、活动 session 中切 FJT、producer restart、rt-control restart、timeout、low-water、exit、disable、fault 和 switch ambiguity。
- **可能文件（≤4）**：scenario tests、expected results、report generator、CI/quality gate hook。
- **依赖**：T5-01、T4-03。
- **验收**：所有场景有 command continuity、state、reject/stop reason、controller lifecycle 断言；旧数据不复活；普通 FJT 回归通过。
- **验证**：自动矩阵；至少 10 分钟 fake 250 Hz 长稳；失败保存 seed/trace/report。
- **停止条件**：任何异常只能靠人工看日志判断，或 command 出现 NaN/Inf/未解释跳变。
- **执行级别**：mock/fake。

**Checkpoint F1（2 项后）**：用户检查 mock 证据摘要；这是申请任何目标机测试前的强制门。

#### T5-03（M）本机 DDS/QoS/调度原型测量

- **目标**：比较命名 QoS 候选，测 update/state 的 p50/p95/p99.9/max、drop/reorder、CPU 与序列化负载，缩小目标机参数范围。
- **可能文件（≤5）**：benchmark tool、QoS profiles、scenario config、report、protocol spec。
- **依赖**：T5-02。
- **验收**：可靠/尽力而为、depth、message size、频率/horizon 有对比证据；所有数值注明“本机原型，非生产冻结”；结果可复现。
- **验证**：固定 seed/负载矩阵；报告命令和环境摘要；state/command loss 行为自动比对。
- **停止条件**：结果被要求直接当生产阈值，或测量环境/时钟不可解释。
- **执行级别**：本机非运动 benchmark。

#### T5-04（S）整理 Motion 交接与已知限制

- **目标**：发布最小 producer 流程、错误恢复、状态判读、版本兼容、仍 blocked 的生产参数和验证证据索引。
- **可能文件（≤3）**：protocol spec、背景交接、Mock README/report index。
- **依赖**：T5-03。
- **验收**：Motion 只看 public docs/IDL 即可实现；明确先 cancel/wait FJT 再切 mode、Action result 是 goal 终态权威；明确 `TwistStamped` 不用于机械臂 rolling；明确 `/joint_states` 仍 50 Hz。
- **验证**：从全新 workspace 构建 IDL+Mock；按文档反向演示 mode/open/update/reject/close。
- **停止条件**：文档依赖私有 topic/参数或仍存在新旧 schema 混跑。
- **执行级别**：文档/Mock。

**Checkpoint F2（2 项后）**：Codex + Claude 最终代码/证据 review；准备目标机非运动授权请求。到此仍不使能、不运动。

### Phase 6 — 单独授权的目标机与实机验证

#### T6-01（M，需单独授权）目标机非运动压力验证

- **目标**：在 BQ-128/BQ-129 生产原生环境、CPU affinity、Fast DDS Domain 0 以及 Motion/Perception/GPU 压力下复测通信和 RT loop。
- **可能文件（≤3）**：benchmark report、protocol spec 参数表、deployment evidence index。
- **依赖**：Checkpoint F2；用户明确授权目标机构建/运行；不得 enable/activate motion。
- **验收**：QoS、update rate、horizon、guard、timeout 均有目标机上界依据或明确 blocking reason；无未解释 deadline miss；普通系统负载可复现。
- **验证**：p50/p95/p99.9/max 矩阵、deadline/overrun/CPU trace、配置摘要。
- **停止条件**：任何测量需要硬件使能、SDO write 或运动；转为 T6-02 前重新申请。
- **执行级别**：目标机，非运动。

#### T6-02（M，高风险，需单独现场授权）低速台架/实机

- **目标**：验证实际 q/qdot 跟踪、滚动替换、正常 exit、断更有界停止和既有 group fault 优先级。
- **可能文件（≤3）**：批准的 runbook、evidence report、参数冻结记录。
- **依赖**：R-008 全部生产限制关闭；T6-01 通过；急停/隔离/监护/回退/FJT 实机回归齐备；用户明确运动授权。
- **验收**：先单轴低速、再 14 轴小幅；每场景保存 desired/actual q-v、buffer/state、总线诊断、停止时间；无未解释跳变/故障。
- **验证**：逐级现场 checklist；每一级人工复核后才扩大；timeout 试验不能替代硬安全链验收。
- **停止条件**：任何未解释跳变、limit margin 不足、诊断异常或急停条件不完整，立即停止扩大范围。
- **执行级别**：受控实机运动。

#### T6-03（S）冻结生产合同并交接 MOTION-124

- **目标**：把目标机/实机证据批准的参数、QoS、schema、样例和已知限制原子发布给 Motion。
- **可能文件（≤5）**：接口包、`cross-domain-interfaces.md`、protocol spec、evidence index、release note。
- **依赖**：T6-01；需要运动参数时还依赖 T6-02。
- **验收**：生产不存在 prototype/TBD 数值；Motion 可反向演示完整流程；旧 major schema 不会被误接收。
- **验证**：clean workspace 全构建/全测试；Motion 合同验收；repository/quality gate。
- **停止条件**：任一 production safety 参数仍只来自本机原型或 test limits。
- **执行级别**：发布/交接。

**Checkpoint G（3 项后，最终发布）**：用户/安全/RT/Motion 联合签收证据与合同；未签收不关闭 ELECTRI-102，也不解除 MOTION-124 的依赖门。

## 10. 最低验证矩阵

| 类别 | 必测场景 | 核心断言 |
|---|---|---|
| 协议 | duplicate、old、gap、late、history rewrite、wrong boot/session/version | accept/reject 唯一；reject 不改变 active buffer |
| 连续性 | 批次内、replace boundary、临界 tolerance、rad/m 混合 | q/qdot 连续；段内 q/v/a 不越界 |
| 可停车性 | 正/负速度、软限位附近、14 轴不同减速度、低水位 ±1 cycle | 每个 accepted sample 都有可行 stop；最终 v=0 hold |
| RT 交换 | producer/consumer 并发、满容量、连续替换 | 无 torn generation、ABA、RT allocation/lock |
| lifecycle | 初始、稳态门控、activate hold、JTC↔rolling、重复 mode、switch reject/timeout/ambiguous | 至多一个 motion controller active；首周期 command=经 actual 残差校验的切换前 command；运动中 mode 请求拒绝 |
| 异常 | update timeout、queue low-water、producer restart、controller restart | 不外推、不追历史、不自动恢复、旧数据不复活 |
| 安全抢占 | 每个 rolling state 下 disable、fault、掉 Operation Enabled | 既有 quick-stop/disable 优先；当前 controller 被停用 |
| 回归 | 普通 enable→FJT→disable→re-enable | BQ-019/BQ-044 等既有行为不变 |
| 通信 | QoS、最大 batch、jitter/drop/reorder、慢订阅者 | 最新未来优先；state 足以解释结果；参数有测量依据 |

## 11. Definition of Done

ELECTRI-102 只有同时满足下列条件才可标记完成：

- Gate 0 决策、协议、动态限制来源和跨域合同已批准；
- 普通 FJT 与全部 enable/disable/fault 冻结语义回归通过；
- 纯核心、controller fake、CiA402 fake、Mock producer、异常矩阵和长稳测试通过；
- RT 路径通过无分配/无锁/无阻塞/无 I/O 审查；
- QoS/horizon/timeout/guard/tolerance/limits 没有无证据生产默认值；
- `/joint_states` 仍遵守 BQ-068 50 Hz，除非另有正式取代裁决；
- 不存在独立 `rt_watchdog`、motion/autonomy heartbeat 或绕过 enable_manager 的 switch；
- 目标机/实机证据按授权级别完成，或明确标注未授权而不能宣称 production-ready；
- Motion 可仅凭公开 IDL/docs/Mock 完成 enter/update/reject/stop/close/return-to-FJT；
- `git diff --check`、定向 build/test、repository gate、quality gate 全绿，且未覆盖用户已有修改。

## 12. Claude 独立评审处置

Claude Fable 5 `xhigh` 完成三轮只读审查：第一轮停在其“是否开始编码”确认框，未获授权；第二轮核对计划并给出“无 BLOCKER、修正 M-1～M-3 后可提交 Gate 0”的有条件 GO；第三轮在用户批准后复核 BQ-130、协议和动态包络，确认 Hermite、同步 C1 停车及 14 轴保守上界成立，并对离线 TDD/Mock 给出 GO。三轮均未执行代码修改、构建或实机动作。以下是 Codex 的处置：

| 评审项 | 处置 | 落点 |
|---|---|---|
| P0-1 enable_manager 无测试基线 | 采纳 | T1-01 必须先于任何 enable_manager source 修改 |
| P0-2 fault/disable 硬编码 JTC | 采纳 | T4-01 无行为重构，T4-03 泛化安全路径 |
| P0-3 QoS 必须是命名剖面 | 采纳 | R-011、T1-04、T5-03/T6-01 |
| P0-4 mode switch 接管是唯一软件防线 | 采纳风险、修正初始方案 | 不从 actual 阶跃播种；T3-01 以 actual 校验切换前 command 残差，首周期保持该 command，并由 R-003 稳态门控 |
| P0-5 active FJT 缺无竞态检测机制 | 采纳问题、调整方案 | R-003 不做 active-goal 猜测：Motion cancel/wait + 物理稳态门控；JTC 若仍有 goal 则原生 abort，Action result 权威；不扩大补丁 |
| P1-1 时钟角色未冻结 | 采纳问题、修改建议 | R-005 使用受校验 period 累加 session time，steady clock 只算 arrival age；异常 period 锁定 stop 并用名义步长推进 |
| P1-2 enter/exit endpoint 归属不清 | 采纳并拆 control/session plane | §5、§7、T1-02；mode owner 与 session owner 分离 |
| P1-3 HOLDING→PRIMING 与 fresh-session 冲突 | 采纳 | 删除该边，close 后 fresh open |
| P1-4 仓库无 C++ test 脚手架先例 | 采纳 | T1-01、T2-01 显式计入 |
| P1-5 `float64[14]` 支持无需再猜 | 采纳 | T1-03 直接使用固定数组并做生成代码检查 |
| P1-6 目标机压测授权需单列 | 采纳 | T6-01 与实机运动 T6-02 分开授权 |
| P2-1 RejectCode/StopReason 应拆分 | 采纳 | §7、T1-03 |
| P2-2 复用 RealtimePublisher/seqlock | 采纳 | T3-05 |
| P2-3 sequence 溢出价值低 | 采纳优先级判断 | 合同规定不回绕；只做廉价边界单测，不投入专项性能工作 |
| P2-4 空 `rt_watchdog` 目录无需动作 | 采纳 | 不创建、不扩展该包 |
| Codex 补充：BQ-068 冻结 50 Hz | 已纠正文档 | 背景交接 §5.2/§6.1、R-012；不纳入 102 调频 |
| Codex 补充：timeout 时需维持 stopping viability envelope | 加入核心不变量 | §6.1、T2-04，避免靠近软限位时才发现无法停车 |
| 第二轮 M-1：运动中切换会产生 position-hold 阶跃停车 | 采纳并改变 Gate 0 默认 | R-003 改为 cancel/wait + source-quiescent + command/actual 接管校验；V1 无 force mode |
| 第二轮 M-2：14 轴耦合 viability 不能笼统声称精确解析 | 采纳 | §6.1 选择保守 segment 解析上界；明确可能误拒和替代算法需另行证明 |
| 第二轮 M-3：dirty 清单和 gate 重叠不完整 | 采纳 | §8.1 列出全部已知 dirty 文件；T1-04/T4-07 增加用户处置 stop condition |
| 第二轮 N-1/N-2：mode response 与状态机缺边 | 采纳 | R-003 不声称检测 goal；§5.2/P0-02/T5-02 补重复 mode、活动 session 切换和 disable 竞争 |
| 第二轮 N-3：rolling enter 需要 process-age state interface | 采纳 | T3-01 声明/读取，T3-03 open admission 测试 |
| 第二轮 N-4/N-5：异常 period、arrival 原子性、guard 组成 | 采纳 | R-005、§6.1、P0-02/P0-03、T2-05/T3-04 |
| 第二轮 N-6：测试与 5 文件预算冲突 | 采纳 | controller 拆 T3-01/T3-02；CiA402 fake 拆 T4-04/T4-05 |
| 第二轮 N-7/N-8：决策编号、零速与 jerk 边界 | 采纳 | 统一引用 R-008；§6.1/P0-03/T2-04 显式覆盖 |
| 第三轮 M-1：close 重试与销毁语义冲突 | 采纳 | 协议 §4.2/§5.3、V-28/V-29 改为原 request 幂等 + 新 request finalize 的两阶段 close |
| 第三轮 M-2：空 Priming 无旧轨迹可拼接 | 采纳 | 协议 §7、V-06/V-42 冻结 `replace_from=0`、首点 `t=0`，首批不采样旧 candidate |
| 第三轮 M-3：update 未携带 producer identity | 采纳 | update 绑定 `client_instance_id`，新增 `WrongClient` 与 V-37，错误身份不消耗 sequence |
| 第三轮 M-4：`terminal representation` 未定义 | 采纳 | 删除隐式终点概念；末点只是普通 q/qdot 边界，low-water/stop 状态机权威，新增 V-43 |
| 第三轮 M-5：pending 时拼接基线不唯一 | 采纳 | 协议 §9 冻结 latest accepted validation head、完整前缀继承、发布前 generation/late-boundary 复核，新增 V-23/V-44/V-45 |
| 第三轮 MINOR：prime age、错误优先级、非接收态 sequence、安全 hold、计划状态 | 采纳 | 新增 `PrimeTimeout`/V-41、payload 验证顺序、V-47、`UnsafeHold`/V-46，并更新 Gate 0 状态 |
| 第三轮 follow-up：Stopping 中新 close request ID | 采纳 | 协议 §5.3/V-48 冻结为接受并缓存、不重建 stop；只有首次在 Holding 出现的新 ID 才 finalize |

## 13. 当前执行边界

用户已批准：

> 批准 ELECTRI-102 Gate 0 默认方案；R-008 生产动态限制继续保持 TBD；先执行到 mock Checkpoint F1，不做目标机或实机动作。

Codex 从 P0-01 开始执行；Claude 在 Checkpoint A、B1/B2、C2、E1、F2 做只读 review。任何决策偏离本计划、重叠用户修改无法安全合并、生产参数缺证据或需要新权限时，立即停在对应 stop condition，不自行扩大范围。
