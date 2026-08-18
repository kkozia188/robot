# rt-control blocked questions

Only tasks listed under each question are blocked. Unrelated tasks continue in unattended mode.

## BQ-001 — T-005 sanctioned legacy deltas [RESOLVED 2026-07-20]

- Evidence: `/home/kkozia/robot_driver@6bc94cd` has Ti5 RxPDO/TxPDO `0x1600/0x1A00`, extra velocity/effort/mode/gap channels, and no `0x60C2` startup SDOs. Frozen REQ-ECAT-002/004 and the hardware mapping require position-only `0x1601/0x1A01` plus `0x60C2:01=4`, `0x60C2:02=-3`.
- Conflict: REQ-MIG-001 requires `diff_legacy.py` to report 100% against that named legacy baseline, but faithfully implementing the higher-priority frozen mapping necessarily produces semantic differences.
- Decision: the frozen requirements are correct and authoritative; Ti5 uses CSP position control only. T-005 shall apply a narrowly scoped frozen-requirement overlay to `6bc94cd`: change Ti5 PDO assignments to `0x1601/0x1A01`, retain only `0x6040/0x607A` and `0x6041/0x6064`, add `0x60C2:01=4` and `0x60C2:02=-3`, and allow `auto_state_transitions: false` on slave profiles. File restructuring is compared by logical axis. Every other semantic difference remains a gate failure.
- Unblocked scope: BQ-001 no longer blocks T-005. T-005 still depends on T-004/BQ-009.

## BQ-002 — ZeroErr preload and trajectory admission [RESOLVED 2026-07-20]

- Evidence: local upstream `EcCiA402Drive` writes the target-position default only when `mode_of_operation_display_ != MODE_NO_MODE` and does not gate `0x000F` on a completed write. Frozen ZeroErr TPDO has no `0x6061`; the authoritative fork solved this with a source patch, while AMB-007 requires a zero-patch target and an `enable_manager` that claims only `control_word`/`status_word`.
- Superseding decision: implement two independent safeguards. First, actuator initialization automatically reads raw `0x6064`, writes the same raw value to `0x607A` before permitting `0x000F`, and holds that position while no trajectory command exists; the reviewed minimal preload gate from the authoritative ICube fork is authorized for this purpose. Motion does not send a preparatory hold FJT. Second, the existing `dual_arm_jtc` performs a `trajectory_start_consistency_check` directly in its FJT goal-validation path; this is not an autonomy or hardware-driver responsibility.
- Frozen proximity tolerance: for each of the 13 EtherCAT joints, `abs(first_point - actual_position) <= 0.017453292519943295 rad` (1 degree). For updown, `abs(target - actual_position) <= 0.05 m`.
- Frozen feedback freshness: EtherCAT joint position feedback must be no older than `500 ms` at JTC admission time; stale or missing feedback rejects the goal. BQ-014 later withdraws the separate updown `500 ms` feedback-age admission gate because the user accepts the continuously updating PDO behavior. The updown `0.05 m` position-consistency requirement remains in force pending BQ-015's exact single-point semantics.
- Frozen responsibility/boundary: modify the existing JTC minimally; do not add an admission package, node, proxy, internal controller, or Action rename. The public endpoint remains `/dual_arm_jtc/follow_joint_trajectory`. Goal validation rejects missing-joint, stale-feedback, or first-point-proximity failures and reports the offending joint, error, and feedback age. This follows system architecture §8.6 (`FollowJointTrajectory: motion -> rt-control`) and §8.2 (controller plugins reside in the rt-control container). Updown remains on its independent command path; its equivalent check uses the frozen `0.05 m`/`500 ms` values and is finalized with that path.
- Unblocked scope: BQ-002 no longer blocks T-010. The preload gate provides exact pre-enable acknowledgement and ZeroErr command pass-through without mapped `0x6061`.
- Blocked scope: T-010 safety completion and T-013 enable authorization remain blocked only on those exact upper-layer contract details. Configuration and non-enable tasks continue.

## BQ-003 — CANopen operation-mode command ownership [RESOLVED 2026-07-20]

- Evidence: the exact design skeleton requires `operation_mode: 1/3/3` in `bus.yml`, but pinned ros2_canopen Humble `Cia402System` does not consume that key. It changes mode only through edge-triggered `position_mode_cmd`/`velocity_mode_cmd` hardware command interfaces.
- Conflict: REQ-CAN-002 freezes PP/PV modes, but no specified controller owns the three mode-command edges or confirms them before target commands.
- Decision: operation-mode selection belongs to the CANopen hardware activation flow, not the updown or diff-drive controllers. Node 1 uses Profiled Position (`0x6060=1`) and Nodes 2/3 use Profiled Velocity (`0x6060=3`), with `0x6061` confirmation. Upper controllers do not pulse mode command interfaces themselves. BQ-005 subsequently supersedes the earlier hardware-specific ordering: mode/state transitions follow the unmodified upstream ros2_canopen standard implementation, and the drive configuration is adapted to it.
- Unblocked scope: BQ-003 no longer blocks T-007 mode wiring. T-014 activation still depends on BQ-005's implementation-vehicle decision.

## BQ-004 — Updown 0x6081 command ownership and ordering [RESOLVED 2026-07-20]

- Evidence: the EDS maps 0x6081 in RPDO2, and ros2_canopen exposes it only through generic `tpdo/index`, `tpdo/subindex`, `tpdo/data`, `tpdo/ons` one-shot interfaces. Its typed velocity interface means a velocity-mode target, not PP profile velocity.
- Conflict: AMB-015 requires each updown move to carry target position plus 0x6081 maximum velocity, but the specification does not assign ownership or ordering of the generic PDO write relative to the position command.
- Decision: every updown move supplies a target position and that move's maximum velocity. The updown controller owns those two motion-intent values; it does not misuse the normal velocity command interface (`0x60FF`). The CANopen execution path writes `0x6081` through the already mapped PDO before submitting `0x607A`. BQ-010 subsequently removes the former SDO readback/confirmation requirement; no readback failure gate remains. BQ-005 supersedes the fixed `0x003F`/next-cycle `0x000F` rule: set-point triggering and clearing follow the unmodified upstream Profiled Position bit-12 acknowledgement handshake.
- PP clarification (2026-07-20): statusword bit 12 is set-point acknowledgement, not a velocity field. Per-move profile velocity is the separate `0x6081` object; profile acceleration/deceleration use standard `0x6083/0x6084`. Use the drive's standard PP profile generator for acceleration/deceleration and do not implement a custom ramp. The approved numeric `0x6083/0x6084` values remain commissioning `TBD` and must be read back/verified rather than guessed.
- Unblocked scope: BQ-004 no longer blocks the ownership/order semantics. T-007/T-014 concrete wiring follows the implementation vehicle selected in BQ-005.

## BQ-005 — CANopen frozen activation/PP semantics versus stock Motor402 [RESOLVED 2026-07-20]

- Evidence: stock `Motor402::handleInit()` enables before PP/PV preload, uses an unconditional fault-reset arm, and does not implement the frozen retry/deadline sequence. Stock `ProfiledPositionMode` waits on statusword bit 12 and does not guarantee the required one-cycle `0x003F` then `0x000F` pulse.
- Conflict: these behaviors are not equivalent to REQ-CAN-004/005, and bus configuration alone cannot change them.
- Superseding decision: use the official pinned ros2_canopen Humble implementation unchanged and adapt the LD2 drive configuration toward its standard CiA402 behavior. Standard PP set-point acknowledgement through statusword bit 12 replaces the former fixed one-cycle pulse rule. Upstream mode/state initialization replaces the former hardware-specific activation ordering and timing where they differ. Do not create a private ros2_canopen patch set or replacement SystemInterface for these behaviors.
- Narrow exception: BQ-006 later authorizes an interface-exposure-only dependency patch for the existing EMCY callback and master broadcast NMT Stop operation. That exception does not authorize any change to CiA402 activation, mode/state transitions, Profiled Position behavior, or protocol semantics.
- Required deliverable: before drive reconfiguration/commissioning, produce a drive-adaptation mapping table covering every required OD/PDO/status behavior, current value/evidence, target standard behavior, proposed drive-side change, persistence method, and verification. It must explicitly cover `0x6040/0x6041` PP handshake bits, `0x6060/0x6061`, `0x607A/0x6064`, `0x6081`, `0x6502` Homing advertisement/automatic-init consequence, safe no-target hold behavior, and relevant PDO/heartbeat/EMCY exposure. Unknown vendor settings remain TBD rather than guessed.
- Unblocked scope: no code-design blocker remains for choosing the CANopen implementation vehicle. T-014 production activation remains gated by the approved drive-side mapping, readback, and hardware verification.

## BQ-006 — CANopen communication/fault handling after watchdog removal [RESOLVED 2026-07-20]

- Superseding decision: delete the standalone `rt_watchdog` package and do not implement any of its former four centralized inputs: `/heartbeat/motion`, `/heartbeat/autonomy`, a separate 4000 ms CANopen-heartbeat age, or a separate 3000 ms updown-PDO age. These former AMB-009/REQ-SAFE-001 timing requirements are explicitly superseded.
- Approved normal no-command behavior: both track commands remain continuously zero; updown automatically holds its current position.
- Approved CANopen boundary: the unmodified upstream ros2_canopen/Lely implementation owns CANopen heartbeat consumption, NMT error control, EMCY receipt, and their diagnostics. Do not add a raw-SocketCAN observer or reconstruct heartbeat/EMCY protocol handling elsewhere.
- Approved track coupling: a heartbeat timeout or EMCY from either track must stop both tracks.
- Approved heartbeat mechanism (2026-07-20): use upstream Lely `master.heartbeat_consumer: true` and `master.stop_all_nodes: true`, with left/right track nodes marked `mandatory: true`. A heartbeat error-control event on either track broadcasts NMT Stop to every CANopen node; the user explicitly approved that updown also enters NMT Stopped. Updown is not itself marked mandatory, so this decision does not make an updown heartbeat fault a group-stop trigger.
- Approved EMCY behavior (2026-07-20): on a track EMCY, use ros2_canopen's EMCY event and request the same all-node NMT Stop; no raw CAN parsing, new watchdog, or reconstructed EMCY protocol is allowed.
- Integration audit/correction: `NodeCanopenBaseDriver::register_emcy_cb()` is public at the lower node-interface layer, but the pinned `Cia402Driver` facade used by `Cia402System` does not forward that method. `Cia402System` registers only NMT/RPDO callbacks, its initialization hook is private, and the pinned master implementation does not instantiate the `set_nmt` service claimed by its documentation. Consequently the previously described no-patch callback hookup is not available through the actual public `Cia402System` surface.
- Authorized implementation exception (2026-07-20): a narrowly scoped patch to the pinned ros2_canopen dependency may expose the existing EMCY callback through the `Cia402Driver`/`Cia402System` integration and expose the existing master broadcast NMT Stop operation. The callback applies the broadcast only to EMCY events from left/right track node IDs. The patch must not change the CiA402 state machine, heartbeat/EMCY parsing, activation behavior, Profiled Position implementation, or any protocol semantics; it must not add a new package/node or raw-CAN observer.
- Unblocked scope: BQ-006 no longer blocks T-011/T-012 implementation. T-014 remains gated by the dependency patch tests, generated-DCF inspection, drive-adaptation mapping/readback, and hardware validation.

## BQ-007 — Docker Compose frozen fields versus build/version wiring [RESOLVED 2026-07-20]

- Corrected evidence: `docker/compose.yaml` has not been created; only `docker/rt-control/.gitkeep` exists. The frozen design fragment specifies `build.context: .`, Dockerfile `docker/rt-control/Dockerfile`, and volume `./docker/cyclonedds.xml`. If that fragment is created under `docker/compose.yaml` and invoked normally, those relative paths resolve from `docker/` and are wrong unless Compose is explicitly given the repository root as project directory. The same spec requires `versions.env` to be the single IgH version source and the image tag to equal the git SHA; normal Compose wiring needs `build.args` and `image` fields absent from the frozen field list.
- Conflict: either editing the Compose field set or relying on an external wrapper/build command is a deployment-contract choice not covered by the specification. The unresolved T-009 CPU set also cannot be committed as a literal.
- Decision: retain every frozen runtime security/transport field, and authorize only the additional reproducible-build wiring: `build.args` for `IGH_VERSION`, `image` with a git-SHA tag supplied by the wrapper, and environment-substituted `cpuset` so no CPU number is invented. Invoke Compose from the repository root with `--project-directory . --env-file versions.env -f docker/compose.yaml`; the wrapper derives the image tag from the current commit. No `privileged` field or broader device/capability permission is authorized.
- Unblocked scope: BQ-007 no longer blocks T-008. Container-vs-host IgH version comparison and the concrete CPU set remain T-009/T-014 environment validations.

## BQ-008 — T-003 named URDF feature source is unavailable [RESOLVED 2026-07-20]

- Corrected evidence: the unauthenticated GitHub page returns 404 because the repository/ref requires authentication. Using the workstation's authenticated GitHub account, `feature/joint-naming-unify-underscore-20260718` exists at commit `8297e386b3fa8e8184820f7de5c91226c726bd94`, and `ros2_ws/src/alfa_robot_description` contains its package manifest, CMake, config, launch, meshes, tests, and URDF directories.
- Decision: freeze commit `8297e386b3fa8e8184820f7de5c91226c726bd94` and its `ros2_ws/src/alfa_robot_description` subtree as the T-003 source explicitly supplied by the user. Do not substitute the older local package or infer renames.
- Unblocked scope: BQ-008 no longer blocks T-003. The source must be fetched/read-only by exact commit during implementation; no remote write or push is authorized.

## BQ-009 — T-004 slave-profile count and filename mapping conflict [RESOLVED 2026-07-20]

- Evidence: T-004 requires exactly 10 `slaves/*.yaml`, while spec §4 names `{zeroerr_j1..j5,left_j6,right_j6,ti5_j2,ti5_j3,turn}.yaml`. Expanding that set labels J2/J3 as both ZeroErr and Ti5, contradicting the hardware authority (J2/J3 are Ti5 only). The authoritative xacro actually uses eight profiles: ZeroErr J1/J4/J5/left-J6/right-J6/turn and Ti5 J2/J3; its ninth source file is an unused generic `joint6.yaml`. Its Ti5 J2 and J3 files are distinct but each is shared unchanged by left and right axes.
- Conflict: retaining shared profiles yields eight active files; splitting both Ti5 profiles per physical axis yields ten, but that changes the specified filename set and duplicates identical stored configuration. Retaining or repurposing the unused generic J6 profile would create an unreferenced or hardware-inapplicable artifact. The specification does not choose among these structures.
- Decision: authorize the eight active profiles proven by the authoritative xacro and supersede the erroneous "exactly 10 files" constraint. The frozen mapping is: left/right J1 -> `zeroerr_j1.yaml`; left/right J2 -> `ti5_j2.yaml`; left/right J3 -> `ti5_j3.yaml`; left/right J4 -> `zeroerr_j4.yaml`; left/right J5 -> `zeroerr_j5.yaml`; left J6 -> `left_j6.yaml`; right J6 -> `right_j6.yaml`; turn -> `turn.yaml`. Do not migrate the unused generic `joint6.yaml`, and do not create `zeroerr_j2.yaml` or `zeroerr_j3.yaml` aliases.
- Unblocked scope: BQ-009 no longer blocks T-004 or its dependent tasks. Numeric content and logical-axis comparison remain governed by the frozen hardware export and the narrowly sanctioned BQ-001 overlay.

## BQ-010 — Updown 0x6081 “write confirmation” has no stock PDO acknowledgement [RESOLVED 2026-07-20]

- Evidence: the frozen mapping sends `0x6081` to Node 1 in mapped RPDO2 (`COB-ID 0x301`), but neither approved updown TPDO carries `0x6081`. Pinned `Cia402System` exposes the generic `tpdo/index`, `tpdo/subindex`, `tpdo/data`, and `tpdo/ons` one-shot command interfaces, so the controller can transmit `0x6081` without remapping. However, that path exports no transmit-result/acknowledgement interface and stock `Cia402System::write()` discards the boolean return from `tpdo_transmit()`.
- Conflict: BQ-004 requires the execution path to write **and confirm** `0x6081` before submitting `0x607A`, while BQ-005 permits no CANopen dependency change except BQ-006's EMCY/NMT-Stop interface exposure. A PDO readback confirmation is impossible with the approved live mapping because `0x6081` is not returned in a TPDO.
- Decision: retain the mapped PDO write of `0x6081` for every new updown move, but remove SDO readback and all device-readback gating. In the same `Cia402System::write()` pass, the generic PDO one-shot is processed before the Profiled Position `set_target()` call, so the required software ordering is `0x6081` transmit request then `0x607A` target request. No extra delay, retry, or acknowledgement mechanism is added.
- Unblocked scope: BQ-010 no longer blocks the updown portion of T-007. T-014 still verifies the live PDO mapping and motion behavior during commissioning, without making runtime motion conditional on `0x6081` readback.

## BQ-011 — No stock controller accepts the approved composite updown command [RESOLVED 2026-07-20]

- Evidence: the approved command contains both target position and per-move `0x6081` maximum velocity and must apply admission checks before commanding motion. Pinned `Cia402DeviceController` does not implement its target service (the code is commented out) and does not claim the `position` command interface. `CanopenProxyController` can send the generic PDO only. The standard position/forward controllers cannot combine the low-level PDO one-shot with the PP position target or enforce their ordering/admission as one operation.
- Conflict: T-007 names an `updown_position` controller but neither the specification nor the package skeleton assigns an implementation vehicle. Running two stock controllers or adding a bridge node would split one approved motion command across independently scheduled paths. Modifying upstream `Cia402DeviceController` would broaden the BQ-005 dependency patch exception.
- Decision: implement a minimal `updown_position_controller` plugin inside the existing `robot_hw_canopen` package. It is a controller-manager-loaded C++ class/library, not a new package, process, standalone node, or proxy. It claims only the updown position command/state plus the existing generic PDO one-shot interfaces and performs the approved admission/order logic in one controller. The external message type/topic and feedback-age source are frozen separately.
- Unblocked scope: the implementation vehicle no longer blocks the updown portion of T-007.

## BQ-012 — External updown command message type is not frozen [RESOLVED 2026-07-20]

- Evidence: the contract requires one event-level command containing target position and per-move `0x6081` maximum velocity, but the specification leaves its carrier to T-007. No standard ROS 2 message directly expresses exactly those two updown motion-intent fields. `Float64MultiArray` encodes them only by positional convention, while `JointTrajectory` introduces trajectory/time semantics that this single PP move does not implement.
- Additional unit evidence: the hardware authority freezes updown position conversion (`m <-> counts`) but does not state the LD2 interpretation or SI conversion of `0x6081`; the EDS declares only an unsigned-32 profile velocity with no unit. Therefore `m/s -> 0x6081 raw` must not be inferred.
- Proposed decision: add `robot_interfaces/msg/UpdownCommand.msg` with `float64 position` (m) and `uint32 profile_velocity_raw` (the exact `0x6081` value), and make it the input type of `updown_position_controller`. The topic name is frozen separately.
- Benefit: field meaning and position unit are explicit; the controller can range-check the raw unsigned value and does not guess an unverified velocity conversion. The contract still hides PDO indices, controlword, and one-shot mechanics.
- Drawback: raw profile velocity exposes one drive-specific quantity to motion/autonomy and is less portable when the updown actuator changes. The consumer must also depend on and rebuild against the existing `robot_interfaces` package; this is a domain-interface change requiring coordination.
- Alternative cost: `Float64MultiArray` avoids a custom message but leaves order and units implicit; `JointTrajectory` reuses a standard type but falsely suggests interpolation/timing support and carries unused fields.
- Decision: add `robot_interfaces/msg/UpdownCommand.msg`; BQ-015 later expands its final field set to `float64 expected_start_position`, `float64 position` (both metres), and `uint32 profile_velocity_raw` carrying the exact unsigned value written to `0x6081`. Do not infer or advertise an SI velocity conversion until verified drive evidence exists.
- Unblocked scope: the message carrier no longer blocks the updown portion of T-007. Its topic name and feedback-age source are frozen separately.

## BQ-013 — External updown command topic name is not frozen [RESOLVED 2026-07-20]

- Evidence: spec §13 explicitly leaves the updown command topic name to T-007. The approved controller instance is `updown_position`, while the public contract should describe the mechanism-independent updown function rather than the controller implementation.
- Proposed decision: freeze the public topic as `/updown/command`, carrying `robot_interfaces/msg/UpdownCommand`. The controller uses its private `~/command` subscription remapped by bringup to that public topic.
- Benefit: the external contract stays stable if the controller class or instance name changes; the functional name is concise and does not expose CANopen/PP details. A private controller subscription still follows normal controller composition and is easy to remap under a robot namespace.
- Drawback: bringup must maintain one explicit remapping, and an absolute topic requires launch-time namespace handling for future multi-robot deployments. Using `/updown_position/command` would remove that remap but couple the domain contract to the controller instance name.
- Decision: freeze `/updown/command` as the public topic carrying `robot_interfaces/msg/UpdownCommand`; bringup remaps it to the `updown_position` controller's private `~/command` subscription. Future multi-robot bringup must apply the robot namespace explicitly rather than silently changing this single-robot contract.
- Unblocked scope: the topic contract no longer blocks the updown portion of T-007.

## BQ-014 — Stock Cia402System cannot prove the approved 500 ms updown-position freshness [RESOLVED 2026-07-20]

- Evidence: BQ-002 requires the updown `0x6064` position feedback used for command admission to be no older than 500 ms. Pinned `Cia402System` exports the cached position value but no PDO receive timestamp, sequence, or age. `/joint_states` republishes that cache periodically even when CAN PDO updates have stopped, and CANopen heartbeat proves node liveness rather than `0x6064` freshness.
- Conflict: the existing interfaces cannot distinguish a stationary but freshly reported position from a stale cached position. Inferring freshness from value changes would falsely reject a stationary axis and falsely accept some repeated traffic. BQ-005 currently permits dependency patches only for BQ-006's EMCY/NMT-Stop exposure.
- Proposed decision: extend the already authorized interface-only ros2_canopen patch to timestamp actual `0x6064` RPDO receipt with a monotonic clock and export a read-only `position_feedback_age_ms` state interface. `updown_position_controller` rejects a command when this age exceeds 500 ms or has never been initialized. Do not alter PDO parsing, mapping, CiA402 state transitions, or PP behavior.
- Benefit: enforces the approved safety gate using the exact transport event; no extra node, polling, SDO, or guessed heartbeat equivalence is introduced. It also distinguishes a valid stationary axis from stale cached data.
- Drawback: this adds another small downstream ros2_canopen patch and state-interface ABI to maintain/rebase and test. Live commissioning must verify that the configured updown TPDO updates the timestamp as expected.
- Alternative cost: dropping the freshness check preserves a more upstream-pure dependency, but a command could be admitted using an arbitrarily old cached position while heartbeat remains healthy.
- Decision: withdraw only the updown `500 ms` feedback-freshness admission gate and do not add a `position_feedback_age_ms` state interface or related ros2_canopen patch. The user accepts that the deployed PDO updates normally. This does not withdraw the EtherCAT JTC's separate 500 ms feedback-freshness check, the updown `0.05 m` position-consistency requirement, live PDO mapping verification, or ros2_canopen heartbeat/EMCY handling.
- Unblocked scope: no feedback-age interface blocks the updown portion of T-007/T-014.

## BQ-015 — The approved two-field updown command has no trajectory initial point for the 0.05 m check [RESOLVED 2026-07-20]

- Evidence: the original 0.05 m rule compares a commanded trajectory's initial point with actual updown position. The approved `UpdownCommand` currently contains only the final PP target and `0x6081` velocity. A single PP target has no separate initial point.
- Conflict: comparing the final target with actual position changes the rule into a maximum move distance of 0.05 m per command, which is not the same as rejecting a stale planned trajectory. Skipping the check would silently remove the approved 0.05 m safeguard.
- Proposed decision: extend `UpdownCommand` with `float64 expected_start_position` (m). On receipt, compare it with actual `0x6064`; reject when the absolute difference exceeds 0.05 m, then execute the separate final `position` target. The message becomes `expected_start_position`, `position`, and `profile_velocity_raw`.
- Benefit: preserves the intended stale-plan/start-state check without limiting a legitimate move to 0.05 m; the rejection can report the expected start, actual position, and error directly.
- Drawback: the motion/autonomy producer must provide a trustworthy expected start and the just-approved message gains a third field. If the producer simply copies current feedback immediately before publishing, the check adds little independent protection.
- Alternative cost: keeping two fields and comparing final target to actual imposes 5 cm incremental moves; removing the check is simpler but abandons the approved updown start-consistency protection.
- Decision: add `float64 expected_start_position` in metres to `UpdownCommand`. Admission requires `abs(expected_start_position - actual_position) <= 0.05 m`; passing that check releases the separate final `position` target. Do not compare the final target to actual position as a 5 cm step-size limit.
- Unblocked scope: the updown start-consistency semantics no longer block T-007.

## BQ-016 — A one-way updown topic has no business-level rejection result [RESOLVED 2026-07-20]

- Evidence: `/updown/command` is a topic, so DDS delivery does not tell the producer whether `updown_position_controller` accepted or rejected the 0.05 m start-consistency check. FJT can return a rejected action goal, but this separate PP path has no action/service response. A log or 1 Hz diagnostic is not correlated to a particular command.
- Upstream capability audit: ros2_canopen provides `canopen_interfaces/srv/COTargetDouble` on a standalone/in-container CiA402 driver's `~/target` endpoint and returns only `bool success` from `Motor402::setTarget()`. It carries neither `0x6081` nor expected start, performs no 0.05 m admission check, supplies no reason/completion result, and would bypass the approved composite controller path. The controller-layer `Cia402DeviceController` declares a target service member but its implementation is commented out. The standard forward-position controller is topic-only and position-only.
- Conflict: the approved behavior says an inconsistent command is rejected, but the external contract does not specify how motion/autonomy learns that outcome. Silently dropping the command could leave the state machine waiting for motion that never started.
- Proposed decision: add a producer-supplied `uint64 command_id` to `UpdownCommand` and publish a numeric, allocation-free `robot_interfaces/msg/UpdownCommandStatus` on `/updown/command_status`. The result echoes `command_id`, reports accepted/rejected plus a fixed reason code, and includes expected start, actual position, and absolute error. `accepted` means only software admission and submission, not physical move completion.
- Benefit: every asynchronous command has a deterministic correlated admission result; rejection is machine-readable without polling logs, diagnostics, or controller internals. Numeric reason codes keep the RT publication bounded and allocation-free.
- Drawback: adds one request field, one custom status message, and one public topic; the producer must allocate unique IDs and handle asynchronous results. It still does not report target-reached completion, which would require a separate contract.
- Alternative cost: diagnostics-only reporting has fewer interfaces but is delayed and cannot reliably correlate a rejection with a particular command; switching to a service/action would give a native response but overturn the already approved topic contract.
- Decision: retain only the `/updown/command` topic. Do not add `command_id`, a command-status message/topic, or a command service. A rejected command is not forwarded to the drive, and motion/autonomy receives no per-command admission result. This explicitly accepts the one-way contract and supersedes the proposed correlated-result mechanism.
- Unblocked scope: no admission-result interface blocks T-007.

## BQ-017 — Overlapping updown PP commands have no frozen policy [RESOLVED 2026-07-20]

- Evidence: `/updown/command` is event-level but the contract does not say what happens when a new command arrives before the prior PP move reaches its target. Upstream `ProfiledPositionMode` asserts the Immediate bit and can accept a different target after the normal bit-12 acknowledgement cycle. The approved controller has no completion/status contract or command queue.
- Conflict: silently choosing overwrite, reject-while-busy, or queue changes observable motion behavior. Reject-while-busy additionally needs a reliable target-reached signal and would be invisible to the producer under BQ-016; queueing needs depth, overflow, cancellation, and stale-start policies not covered by the specification.
- Proposed decision: use latest-valid-command replacement. Every new command independently passes the 0.05 m expected-start check; if it passes, its `0x6081` and final position replace the active target through the upstream Immediate PP behavior. Do not queue. A failed admission leaves the currently active target unchanged.
- Benefit: matches the open-source PP implementation, adds no target-reached patch/queue/state machine, and allows a correctly replanned command to redirect an in-progress move. The existing expected-start check prevents a command planned from a stale position from replacing the target.
- Drawback: a valid new command can change direction or destination before the prior move completes, and the producer receives no explicit notice that replacement occurred. It is unsuitable if commands are intended as an ordered sequence that must all execute.
- Alternative cost: reject-while-busy is simpler conceptually but needs target-reached exposure and silently drops commands; queueing preserves order but introduces unapproved buffering/cancellation semantics and can execute stale motion later.
- Decision: use latest-valid-command replacement with no queue. Every new command independently passes the 0.05 m expected-start check; an accepted command replaces the active `0x6081` and final target through upstream Immediate PP behavior. A rejected command leaves the currently active target unchanged.
- Unblocked scope: overlapping-command semantics no longer block T-007/T-014.

## BQ-018 — `/updown/command` QoS is not frozen [RESOLVED 2026-07-20]

- Evidence: the interface is an event-level command topic with latest-valid-command replacement and no acknowledgement. QoS therefore determines whether a command may be lost, accumulated, or replayed after controller restart. The specification does not define reliability, history depth, or durability.
- Proposed decision: use `RELIABLE`, `KEEP_LAST(1)`, `VOLATILE` QoS for both producer and controller subscription. Do not use transient-local durability or a queue depth greater than one.
- Benefit: reliable delivery reduces silent command loss; depth one matches latest-command replacement and prevents a backlog of stale moves; volatile durability prevents a retained command from replaying when the controller restarts or reconnects.
- Drawback: reliable DDS can deliver a command later after transient network congestion rather than dropping it immediately. The 0.05 m expected-start gate limits but does not eliminate that delayed-command risk, and reliable delivery still does not provide business-level acceptance.
- Alternative cost: best-effort minimizes retransmission/delay but can silently lose the only event command; transient-local helps late joiners but can replay an old motion command, which is unsafe for this interface.
- Decision: use matching `RELIABLE + KEEP_LAST(1) + VOLATILE` QoS on the `/updown/command` publisher and controller subscription. Do not retain/replay commands or accumulate more than the latest sample.
- Unblocked scope: the updown input QoS no longer blocks T-007.

## BQ-019 — Humble JTC has no configurable/subclass hook for the approved FJT admission check [RESOLVED 2026-07-20]

- Evidence: BQ-002 requires the existing `/dual_arm_jtc/follow_joint_trajectory` goal path itself to enforce the 13-joint first-point check, with no wrapper/proxy/package or action rename. In Humble `JointTrajectoryController`, `goal_received_callback()` and `validate_trajectory_msg()` are protected but non-virtual, and `on_configure()` binds the action server directly to the base-class callback. No parameter/plugin hook can inject an additional goal validator. The installed binary is 2.52.1; the supplied local Humble upstream reference is `ros2_controllers@cbcf66218ff43353f9fb5fe7a2c33f458d578d73` (package version 2.53.2).
- Clarification (2026-07-20): the user was referring to the admission check agreed in this conversation, not a controller found in a repository. The frozen behavior is: when `dual_arm_jtc` receives an FJT goal, compare all 13 actual joint positions with the trajectory's first point and reject the whole goal if any absolute error exceeds exactly `0.017453292519943295 rad` (1 degree). This logic belongs inside the existing JTC goal-admission path and is not a separate controller/package/proxy.
- Historical-controller audit: `/home/kkozia/robot_driver_review@07c87e5` separately configures `dual_arm_command_permit_controller`, but it is a standard `ForwardCommandController` publishing alternating lease tokens to 13 custom `command_permit` hardware interfaces. It never reads a `FollowJointTrajectory` goal, never compares its first point, and cannot reject the action goal. It also exists only in the later reference checkout, not the authoritative `robot_driver@6bc94cd`, and depends on extensive non-authoritative EtherCAT-plugin gate changes. The authoritative plugin's separate `position_command_sync_tolerance_=0.0175` check only controls the first hardware-command takeover after preload; a JTC trajectory can first interpolate through the current position and then continue to a distant point, so that check is not an FJT start-consistency rejection either.
- Conflict: the approved check cannot be implemented by controllers.yaml or a subclass while retaining the original action server. It requires a small patch to the upstream JTC source, but the current dependency manifest does not source-build ros2_controllers and no exact patched version is frozen.
- Proposed decision: pin the supplied Humble upstream commit `cbcf66218ff43353f9fb5fe7a2c33f458d578d73`, carry a narrowly scoped patch inside the existing `joint_trajectory_controller` goal-validation callback, and source-build that package as an overlay. Preserve the standard controller class, action endpoint, interpolation, execution, cancellation, and result behavior; add only parameterized first-point/freshness validation and focused tests.
- Benefit: implements the check at the actual FJT admission point, preserves all normal open-source JTC behavior and the public action name, and avoids the separate package/node/proxy previously rejected by the user. The dependency and patch are reproducibly pinned.
- Drawback: replaces the installed 2.52.1 binary with source-built 2.53.2 plus a downstream patch; the patch must be maintained/rebased and the upstream version difference must be regression-tested. Checking out ros2_controllers also increases dependency/build footprint even if only JTC is selected for build.
- Alternative cost: a wrapper/proxy avoids patching upstream but violates BQ-002; subclassing does not intercept the bound callback; omitting the check violates the approved requirement.
- Decision: use the supplied official Humble `ros2_controllers` source at exact commit `cbcf66218ff43353f9fb5fe7a2c33f458d578d73` (`joint_trajectory_controller` 2.53.2) and apply the minimal source-overlay patch described above. Do not modify `/opt/ros/humble`, the read-only legacy baseline, the controller/action name, or unrelated JTC execution behavior.
- Unblocked scope: the JTC source/version vehicle no longer blocks the 13-axis FJT admission implementation. The exact EtherCAT feedback-freshness source is frozen separately.

## BQ-020 — Existing EtherCAT `read()` cannot prove the approved 500 ms JTC feedback freshness [RESOLVED 2026-07-20]

- Evidence: the pinned ICube `EthercatDriver::read()` returns `OK` whenever `EthercatBusManager::read()` does not return `kError`, while the bus manager returns `kCompleted` immediately after `EcMaster::readData()`. `readData()` calls `processData()` even when the EtherCAT domain Working Counter is `ZERO` or `INCOMPLETE`; `checkDomainState()` only logs and caches that state. Therefore a successful controller-manager read cycle can repeatedly expose an old process image. The driver exports joint position values but no receive timestamp, sequence, Working Counter validity, or age interface.
- Conflict: BQ-002 requires the 13-axis FJT admission path to reject feedback older than 500 ms. Neither the JTC callback time nor the last cached-position access time proves that a new valid EtherCAT frame arrived. Treating an unchanged position as stale would also reject a stationary but healthy robot.
- Proposed decision: make the already required ICube source overlay export one read-only domain-level state interface named `ethercat_domain/process_data_age_ms`. Update its monotonic timestamp only after a receive cycle whose domain state is `EC_WC_COMPLETE`; initialize it as non-finite/stale. The patched JTC 2.53.2 claims this one extra state interface, snapshots it with the 13 actual positions in its normal update path, and rejects a new FJT goal when the snapshot is missing/non-finite or older than 500 ms. Do not create a node/watchdog, infer freshness from position changes, or turn an incomplete Working Counter directly into a hardware-component error.
- Benefit: the gate measures an actual complete EtherCAT process-data event, accepts healthy stationary feedback, adds only one scalar for the whole synchronized domain, and keeps a transient bus issue scoped to rejecting new trajectories rather than forcing an unapproved hardware lifecycle transition.
- Drawback: this adds a small downstream ICube interface patch in addition to the JTC patch, and the new state-interface name/configuration must be maintained. Because freshness is domain-level, any configured slave that makes the domain Working Counter incomplete causes admission of the whole 13-axis trajectory to fail; it does not identify one stale joint.
- Alternative cost: withdrawing the 500 ms gate avoids the extra interface/patch but permits a new FJT goal to be admitted against an arbitrarily old cached EtherCAT position. Returning hardware `ERROR` after 500 ms avoids the JTC interface but changes controller-manager lifecycle and active-motion fault behavior beyond the approved admission-only requirement.
- Decision: add the single read-only domain-level state interface `ethercat_domain/process_data_age_ms` exactly as proposed. This interface is used only by the FJT admission check; it does not create a watchdog or change hardware-component lifecycle/fault behavior.
- Unblocked scope: the freshness portion of the JTC overlay may proceed with the 500 ms threshold.

## BQ-021 — A rejected ROS 2 action goal cannot return the approved detailed FJT failure result [RESOLVED 2026-07-20]

- Evidence: `rclcpp_action::GoalResponse::REJECT` produces a SendGoal response containing only `accepted=false` (plus the protocol timestamp); the client receives a null goal handle and no action result. Although `control_msgs/action/FollowJointTrajectory` defines `INVALID_GOAL`, `INVALID_JOINTS`, and `error_string`, those fields are available only after the server first accepts the goal and later terminates it. Stock JTC already rejects invalid goal messages in `goal_received_callback()` and reports details only in its server log.
- Conflict: BQ-002 says a stale or first-point-inconsistent goal is rejected and also reports the offending joint, position error, and feedback age. Strict goal rejection and returning those details through the existing FJT action result are mutually exclusive under the ROS 2 action protocol.
- Proposed decision: preserve strict admission rejection. Return `GoalResponse::REJECT` before any trajectory becomes active, and emit one structured `ERROR` log containing the rejection reason, offending joint, absolute position error in radians, 1-degree limit, and domain feedback age in milliseconds. The FJT client sees only that the goal was rejected. Do not add another public status topic/service.
- Benefit: an invalid trajectory is never represented as accepted or briefly active; this matches stock JTC validation semantics, preserves the existing endpoint, and gives local commissioning/diagnostics a precise reason without another interface.
- Drawback: motion/MoveIt cannot obtain the detailed reason programmatically from the action response and receives only generic goal rejection; correlation relies on the contemporaneous server log.
- Alternative cost: accepting then immediately aborting with `INVALID_GOAL` and a detailed `error_string` gives the client a machine-readable result, but changes the contract from rejection to accepted-then-failed and enters the accepted-goal callback path. A separate result topic/service preserves strict rejection plus machine-readable details but adds a new public interface and correlation contract.
- Decision: preserve strict admission rejection exactly as proposed. Emit one structured JTC `ERROR` log with the rejection reason and available offending-joint/error/limit/feedback-age fields; do not accept-then-abort and do not add a public result interface. Motion/MoveIt receives only the standard rejected-goal indication.
- Unblocked scope: FJT rejection reporting no longer blocks the JTC patch.

## BQ-022 — Stock ros2_canopen diagnostics do not provide the legacy CANopen age/counter fields [RESOLVED 2026-07-20]

- Evidence: the pinned upstream `NodeCanopen402Driver` uses `diagnostic_updater` and publishes the native keys `device_state`, `nmt_state`, `emcy_state`, `cia402_mode`, and `cia402_state`. The EMCY text contains the received emergency error code/register/manufacturer bytes. The implementation does not timestamp heartbeat receipt, export `heartbeat_age_ms`, or count SDO timeouts, and its CiA402 diagnostic callback has no `sdo_timeout_count` field. Lely's heartbeat error-control and the approved all-node stop still operate independently of these missing observability fields.
- Conflict: the original T-012 diagnostics table requires `heartbeat_age_ms`, `last_emcy_code`, and `sdo_timeout_count` for every CANopen node, but BQ-006 supersedes the separate heartbeat-age watchdog and directs communication-fault handling to the upstream ros2_canopen/Lely implementation. Recreating exact ages/counters requires more downstream instrumentation than the already approved callback/broadcast exposure patch.
- Proposed decision: use ros2_canopen's native per-node diagnostics as the CANopen diagnostic authority. Preserve its device/NMT/EMCY/CiA402 fields and the EMCY error data, but remove the requirements for custom `heartbeat_age_ms` and `sdo_timeout_count`; do not reconstruct them in `rt_diagnostics`. Normalize/aggregate the native statuses under the rt-control diagnostic tree without changing their underlying safety decisions.
- Benefit: keeps communication monitoring and diagnostics aligned with the selected open-source implementation, avoids another protocol-timing/counter patch, and prevents duplicated fault state from disagreeing with Lely's actual decisions.
- Drawback: operators cannot see a continuously increasing heartbeat age or an accumulated SDO-timeout counter in rqt; they see the resulting native NMT/device/EMCY state instead. The diagnostic key set differs from the original frozen table.
- Alternative cost: instrumenting ros2_canopen to expose exact heartbeat timestamps and SDO-timeout counters preserves the old table but expands the maintained dependency patch and duplicates information not used by the approved safety path.
- Decision: use ros2_canopen's native per-node diagnostics as proposed. Preserve and aggregate its device/NMT/EMCY/CiA402 fields and EMCY error data; remove the custom `heartbeat_age_ms` and `sdo_timeout_count` requirements and do not reconstruct them in `rt_diagnostics`.
- Unblocked scope: the CANopen diagnostics row of T-012 may proceed using the upstream-native fields; heartbeat/EMCY stop behavior remains governed by BQ-006.

## BQ-023 — Per-slave EtherCAT diagnostics cannot cover unconfigured ring positions 0 and 13 [RESOLVED 2026-07-20]

- Evidence: the frozen topology has 15 responding ring positions `0..14`, but only positions `1..12` and `14` are configured motion slaves. ICube obtains per-slave `ec_slave_config_state_t` only from the `ec_slave_config_t` handles created by `ecrt_master_slave_config()` for configured modules. Positions 0 and 13 have no module/config handle, and REQ-ECAT-010 explicitly forbids guessing their identities. The master-level API still reports the aggregate `slaves_responding` count.
- Conflict: the original T-012 table says `ethercat/slave_<pos>` diagnostics for every slave, while the approved configuration can truthfully publish per-slave AL/online/operational state only for the 13 known motion positions before T-013 commissioning identifies positions 0 and 13.
- Proposed decision: publish per-slave runtime diagnostics for the 13 configured motion positions only, and use `ethercat/master.slaves_responding == 15` plus the T-013 preflight `ethercat slaves -v` archive to cover the two unconfigured topology members. Do not create guessed/passive configurations for positions 0 and 13 and do not poll the privileged `ethercat` CLI continuously from `rt_diagnostics`. Add per-position diagnostics for 0 and 13 only in a later frozen hardware revision if their identities and supported monitoring vehicle are explicitly approved.
- Benefit: reports every state ICube actually owns, preserves the no-guess hardware rule, avoids changing the EtherCAT domain/configuration for non-motion members, and still detects a missing member through the aggregate responding count.
- Drawback: during runtime, a drop from 15 responders cannot be attributed specifically to position 0 versus 13 from the diagnostic tree; detailed attribution requires the commissioning CLI/archive. The emitted per-slave set is 13 entries rather than the original table's implied 15.
- Alternative cost: configuring diagnostic-only slaves after guessing or prematurely freezing vendor/product IDs violates REQ-ECAT-010; invoking and parsing `ethercat slaves` at 1 Hz adds privilege, process creation, and a second state source outside ICube.
- Decision: publish per-slave runtime diagnostics only for configured positions `1..12` and `14`, and cover the full ring with `ethercat/master.slaves_responding == 15` plus the T-013 `ethercat slaves -v` preflight archive. Do not guess/configure positions 0 and 13 or poll the privileged CLI from `rt_diagnostics`.
- Unblocked scope: the EtherCAT per-slave coverage of T-012 may proceed for the 13 configured motion positions and the aggregate master count.

## BQ-024 — EtherCAT diagnostic snapshot transport and WC-error counting are not frozen [RESOLVED 2026-07-20]

- Evidence: ICube caches the current EtherCAT master state, domain Working Counter state, and 13 configured slave states internally, but exports none of them as ROS diagnostics or ros2_control interfaces. The existing Joint State Broadcaster claims all available read-only state interfaces and publishes arbitrary non-motion interfaces on `/dynamic_joint_states`; only prefixes with position/velocity/effort enter `/joint_states`. BQ-020 already authorizes the domain process-data age interface.
- Conflict: `rt_diagnostics` is a separate node and cannot access ICube's private C++ objects. Publishing directly from the hardware plugin, exporting a stable read-only state-interface ABI, or polling a CLI are materially different designs. The frozen `wc_error_count` also does not define whether to count incomplete cycles or only state transitions.
- Proposed decision: extend the ICube overlay with these numeric read-only state interfaces: `ethercat_domain/process_data_age_ms`; `ethercat_master/link_up`; `ethercat_master/slaves_responding`; `ethercat_master/wc_error_count`; and `ethercat_slave_<pos>/al_state` for positions `1..12` and `14`. Keep each joint's existing `status_word` as the CiA402 source. JSB publishes the synthetic interfaces on `/dynamic_joint_states`, and `rt_diagnostics` converts the snapshots to the approved 1 Hz tree. Define `wc_error_count` as the cumulative number of activated read cycles whose domain state is not `EC_WC_COMPLETE`, initialized to zero on process start; diagnostics warns while the counter is increasing rather than forever after a historical error.
- Benefit: keeps all bus reads in ICube, gives both JTC and diagnostics a single transport-validity source, uses the standard ros2_control/JSB path, leaves `/joint_states` unchanged, and makes a one-second WC outage correspond to roughly 250 bad process-data cycles rather than one ambiguous transition.
- Drawback: adds a maintained state-interface ABI and extra fields to `/dynamic_joint_states`; `rt_diagnostics` depends on JSB being active. Counting bad cycles makes the number increase quickly during an outage and does not equal the number of distinct outage episodes.
- Alternative cost: counting only COMPLETE-to-incomplete transitions hides outage duration and repeated bad frames; publishing diagnostics directly from the hardware plugin adds ROS presentation/timers to the bus component; CLI polling adds privilege and a second state source.
- Decision: export the proposed read-only state-interface set through ICube and JSB `/dynamic_joint_states`, with `rt_diagnostics` producing the 1 Hz diagnostic tree. Define `wc_error_count` as one increment for every activated read cycle whose domain state is not `EC_WC_COMPLETE`, reset on process start; WARN only while the counter increases.
- Unblocked scope: the EtherCAT snapshot transport and WC counter semantics no longer block T-012 or the ICube overlay.

## BQ-025 — Humble controller_manager does not expose the frozen runtime overrun counter [RESOLVED 2026-07-20]

- Evidence: the pinned/local Humble ros2_control source (`controller_manager` 2.53.1) calculates the measured loop period and runs `read() -> update() -> write() -> sleep_until()`, but does not count deadline misses or publish control-loop statistics. Its changelog records that controller-manager diagnostics were backported and then reverted in 2.26.0 because the change was ABI-breaking. No service, topic, state interface, or public getter supplies the frozen `overrun_count` to `rt_diagnostics`.
- Conflict: the original T-012 table requires a runtime `cyclic/overrun_count`, while retaining it requires source-building and patching core ros2_control/controller_manager in addition to the already approved JTC and ICube overlays. Inferring 250 Hz loop overruns from JSB's 100 Hz topic is invalid, and no overrun threshold/tolerance is frozen.
- Proposed decision: remove the runtime `cyclic/overrun_count` diagnostic row for this Humble implementation and keep the already required host `cyclictest`, 250 Hz configuration check, realtime scheduling/affinity startup logs, and commissioning stop criterion (`cyclictest > 100 us`). Do not infer a counter from `/joint_states` or `/dynamic_joint_states`, and do not patch controller_manager solely for this field.
- Benefit: keeps the core ros2_control runtime upstream-pure, avoids a high-blast-radius dependency overlay and an invented threshold, and retains the explicit host realtime acceptance test that is already the hardware gate.
- Drawback: after commissioning there is no persistent ROS diagnostic showing application-loop deadline misses; operators must use logs and rerun the host timing test when investigating runtime scheduling problems.
- Alternative cost: a controller_manager runner patch can count exact scheduled-deadline misses and publish them, but requires freezing a ros2_control version, counter/reset/threshold semantics, an RT-safe export path, and regression testing the core control loop.
- Decision: remove the runtime `cyclic/overrun_count` diagnostic row for the Humble implementation. Retain the 250 Hz configuration check, realtime scheduling/affinity startup evidence, host `cyclictest`, and the `>100 us` commissioning stop criterion; do not patch controller_manager or infer a counter from published state topics.
- Unblocked scope: the absence of a runtime cyclic counter no longer blocks T-012; host timing verification remains part of T-009/T-015.

## BQ-026 — Recovery after the approved CANopen all-node NMT Stop is not defined [RESOLVED 2026-07-20]

- Evidence: BQ-006 configures Lely to broadcast NMT Stop when a mandatory track heartbeat fails and adds the same broadcast for a track EMCY. The pinned master does not instantiate the documented `set_nmt` service. `Cia402System` exposes per-node `nmt/start` command interfaces, but no approved controller or service claims them. The deleted watchdog's former five-second automatic recovery no longer exists, and BQ-006 does not define a replacement recovery transition.
- Conflict: silently auto-starting nodes after communication returns can resume a system whose original EMCY/heartbeat cause is not cleared, while leaving all nodes stopped forever without a documented recovery path makes the approved group stop operationally incomplete.
- Proposed decision: make the group stop fail-latched for this phase. After correcting and acknowledging the physical/communication fault, recover by restarting the rt-control container (or equivalently performing the full controller-manager hardware cleanup/configure/activate sequence), which reruns the standard ros2_canopen boot, mode selection, initialization, and zero/hold preload. Do not auto-restart nodes and do not add a partial NMT-Start service/controller in the current scope.
- Benefit: recovery always passes through the same verified cold-start initialization and safe zero/hold preload, cannot silently resume old commands, and adds no new safety-critical service or partial re-enable state machine.
- Drawback: recovery is manual and disruptive: all rt-control controllers and all three CANopen nodes restart, active goals/commands are lost, and downtime is longer than a targeted NMT recovery.
- Alternative cost: a dedicated recovery service could validate all three node states, broadcast/start nodes, rerun CiA402 initialization, and report failures without restarting the container, but it requires a new public interface, authorization rules, concurrency semantics, and another tested safety FSM. Automatic restart is simpler operationally but can re-enable after an uncleared fault.
- Decision: make the CANopen group stop fail-latched. After correcting and acknowledging the fault, recover only through an rt-control container restart or the equivalent full hardware cleanup/configure/activate sequence; rerun standard boot/mode/init and zero/hold preload, never replay old commands, and add no automatic or partial NMT recovery interface in this phase.
- Unblocked scope: the post-fault operating procedure is frozen; T-011/T-014 must verify the full-restart recovery path.

## BQ-027 — NMT Stop alone does not guarantee mechanical track stop after communication loss [RESOLVED 2026-07-21]

- Evidence: CANopen NMT Stop changes the node communication state and disables normal PDO processing; it does not by itself standardize the drive's mechanical deceleration or clear a previously latched nonzero `0x60FF`. If a track has actually lost CAN communication, the master cannot deliver a later zero-speed or `0x0002` command to that node. The current bus configuration has no master heartbeat producer, so each slave's configured `heartbeat_consumer: true` currently creates no effective drive-side supervision. The LD2 EDS exposes heartbeat consumer object `0x1016` and quick-stop option `0x605A`, but not the vendor `0x5010` watchdog candidate; the exact loss-reaction configuration is not frozen.
- Conflict: BQ-006's master-side heartbeat/EMCY detection and all-node NMT Stop can coordinate healthy nodes, but cannot alone prove that an already disconnected track stops. The requirement that either track fault stops both therefore needs an independent drive-side communication-loss reaction and commissioning proof.
- Proposed decision: make the safety design two-layered. Keep Lely master-side heartbeat/EMCY detection and all-node NMT Stop. In the drive-adaptation mapping, require both track drives to monitor the rt-control master heartbeat and enter a vendor-confirmed controlled stop/Quick Stop on master-heartbeat loss or NMT Stop, with no automatic motion resume; require the updown drive to enter its vendor-confirmed safe stop/hold/brake behavior on the same all-node stop. Do not authorize any OD value by guess: the exact `0x1016`, master producer period, timeout, `0x605A`, and vendor reaction settings remain blocked until the LD2 manual/live readback and low-speed supported commissioning test prove them. If the LD2 cannot provide the required reaction, track motion remains blocked and an independent hardware safety mechanism is required.
- Benefit: covers both halves of the failure: the master stops the healthy peer, while the disconnected drive stops itself. It avoids pretending that an undeliverable software command can stop a lost node and turns the requirement into an explicit drive configuration/readback/test gate.
- Drawback: requires drive-side configuration changes, a master heartbeat producer, vendor documentation, and destructive-to-motion commissioning tests. Final stopping latency depends on the later approved heartbeat/drive reaction values, and the solution remains non-safety-rated unless implemented by rated hardware.
- Alternative cost: relying only on broadcast NMT Stop is simpler but cannot guarantee the disconnected motor stops; trying to send zero/Quick Stop only from ros2_canopen has the same reachability limitation on the failed link.
- Decision: adopt the proposed two-layer design. Keep Lely master-side heartbeat/EMCY detection and group NMT Stop, and require independently verified drive-side master-heartbeat/NMT-stop reactions for both tracks and a safe stop/hold/brake reaction for updown. Freeze no OD value without vendor evidence/readback; if the LD2 cannot provide the reaction, require an independent hardware safety mechanism.
- Blocked scope: all powered track motion in T-014/T-015 remains blocked until the drive-side reaction is documented, configured, read back, and tested; software configuration work may continue.

## BQ-028 — Bidirectional CANopen heartbeat timing is not frozen [RESOLVED 2026-07-21]

- Evidence: BQ-027 requires both the Lely master to monitor nodes and each LD2 drive to monitor the rt-control master. The current live record says drive `0x1017=1000 ms`, the supplied EDS default is 2000 ms, and the master heartbeat producer is disabled (`0 ms`). The former separate 4000 ms watchdog was deleted, so it is not a valid fallback. CANopen heartbeat supervision needs explicit producer periods and consumer deadlines in both directions.
- Proposed decision: use symmetric heartbeat timing: the master and nodes 1/2/3 each produce heartbeat every `100 ms`, and both master-side and drive-side consumers expire after `500 ms` (multiplier 5). Treat 500 ms as the communication-fault detection bound before the configured stop reaction begins; physical stopping time additionally includes the drive's approved deceleration/Quick Stop behavior. Apply the values to all three production nodes so the approved all-node behavior has one timing contract.
- Benefit: detects a lost link within half a second while tolerating approximately five consecutive missed heartbeat periods. Four heartbeat producers at 10 Hz add negligible load on a 500 kbit/s bus, and a symmetric contract is straightforward to archive and test.
- Drawback: changes each drive's current recorded `0x1017=1000 ms`, produces more heartbeat traffic, and a host or CAN stall longer than 500 ms causes a latched group stop requiring full restart. It does not itself guarantee the mechanical stopping distance.
- Alternative cost: retaining 1000 ms producers requires a consumer deadline longer than one second and therefore slower fault detection; using only one or two missed periods reacts faster but raises nuisance-trip risk under transient scheduling/bus delays.
- Decision: instead use the user-approved symmetric values: the master and nodes 1/2/3 each produce heartbeat every `1000 ms`, and both master-side and drive-side consumers expire after `5000 ms` (multiplier 5). The 5000 ms bound covers communication-fault detection only; configured drive stopping time follows afterward.
- Benefit of the approved values: matches the recorded live drive producer period, tolerates five consecutive missed heartbeats, minimizes nuisance trips and CAN heartbeat traffic, and keeps one symmetric timing contract.
- Drawback of the approved values: a communication loss can leave detection pending for up to five seconds before any configured stop reaction begins, producing a substantially longer worst-case stopping distance than the proposed 100/500 ms values.
- Unblocked scope: heartbeat timing is frozen; configuration ownership is resolved by BQ-029.

## BQ-029 — Ownership of heartbeat OD writes [RESOLVED 2026-07-21]

- Evidence: enabling drive-side supervision requires node `0x1016` consumer entries, and making Lely's master-side supervision deterministic requires the configured node `0x1017` producer period to match the generated DCF. The earlier T-006/TBD-011 decision omitted `heartbeat_producer` and prohibited automatic `0x1017` writes because the old external watchdog was the authority. BQ-006/BQ-027 supersede that architecture.
- Decision: ros2_canopen/dcfgen owns the heartbeat configuration at every startup. Configure the master and nodes 1/2/3 for `heartbeat_producer: 1000 ms`, enable consumers in both directions, and use multiplier 5 to produce `5000 ms` consumer deadlines. Authorize generated startup writes for drive `0x1016` and `0x1017` only; this does not authorize PDO remapping or writes to other unresolved OD objects. Inspect the generated DCF/bin, archive the pre-change values, and read back the effective values before powered motion.
- Benefit: the running network cannot silently depend on stale manually persisted heartbeat values; every boot deterministically reapplies the approved contract through the standard ros2_canopen configuration path.
- Drawback: startup now modifies each drive's communication OD, superseding the former no-write rule; a failed/mismatched SDO configuration must block activation. Repeated boot configuration also depends on the drives accepting these writes reliably.
- Unblocked scope: bus.yml and the drive-adaptation table may include the exact heartbeat producer/consumer configuration; live activation remains gated by generated-DCF inspection and SDO readback.

## BQ-030 — Partial-batch EtherCAT enable failure has no rollback action [RESOLVED 2026-07-21]

- Evidence: the five enable batches are sequential, so a timeout/fault in a later batch can leave earlier batches in Operation Enabled. The frozen service response identifies the failed batch/joint/statusword, and normal `/rt/disable` has an approved three-stage sequence, but no source defines what the enable FSM itself does with already enabled axes after a batch failure. “Atomic service” defines one request/result boundary but not the hardware rollback command.
- Conflict: leaving earlier batches enabled after returning `ok=false` violates the expected all-or-nothing operating state. Blindly starting the same control word on all 13 axes is also invalid because a partial enable leaves axes in different CiA402 states. Quick Stop was reserved for emergency motion/fault paths and is unnecessary for axes that have not accepted a motion command.
- Corrected proposed decision (2026-07-21): on any enable-stage timeout, invalid state, preload refusal, or fault, stop advancing immediately and preserve the original failure fields. Enter a per-axis state-aware standard disable rollback in the RT update loop: Operation Enabled axes receive `0x0007` until Switched On (`0x006F == 0x0023`); Switched On axes receive `0x0006` until Ready to Switch On (`0x006F == 0x0021`); Ready axes receive `0x0000` until Switch On Disabled (`0x004F == 0x0040`); axes already farther down join at the matching step. Use the approved `disable_stage_timeout: 4.0s` for each downward stage. Do not use Quick Stop, automatically fault-reset, or resume enable. An axis already in Fault/Fault Reaction receives the non-enabling `0x0000`, remains reported as Fault, and requires explicit fault resolution/full restart; it is not falsely reported as having reached `0x0040`.
- Benefit: rolls every successfully enabled axis down through the approved standard CiA402 sequence, works with the mixed states created by a partial batch, avoids the Ti5 direct-shutdown defect, and preserves the original enable failure diagnosis.
- Drawback: the rollback FSM is per-axis rather than one broadcast phase and can take up to the configured stage timeouts. Faulted axes cannot complete the normal statusword confirmations without a later explicit fault resolution, so the group remains latched FAILED even after healthy axes are disabled.
- Alternative cost: leaving earlier batches enabled is operationally ambiguous and unsafe; forcing every axis to begin at `0x0007` can stall axes that never reached Operation Enabled; immediately forcing `0x0000` on enabled healthy axes skips the approved standard sequence; Quick Stop is not the normal disable path.
- Decision: use the corrected per-axis, state-aware standard disable rollback exactly as proposed. Preserve the original enable failure, use the 4.0-second stage timeout, do not Quick Stop or auto-reset, and leave Fault axes latched for the explicit `/rt/reset_fault` service.
- Unblocked scope: enable-manager failure rollback and retry gating are fully specified for T-010.

## BQ-031 — Explicit EtherCAT Fault Reset service scope is not frozen [RESOLVED 2026-07-21]

- User requirement (2026-07-21): retain an explicit service interface for Fault Reset. It belongs to the existing `enable_manager`, which already exclusively owns the 13 EtherCAT `control_word` command interfaces; do not create another package/node or allow rollback to invoke it automatically.
- Evidence: an axis in CiA402 Fault (`statusword & 0x004F == 0x0008`) does not reach Switch On Disabled merely by receiving `0x0000`. It requires an explicit controlword bit-7 rising edge and confirmation that Fault cleared. The existing `/rt/enable` and `/rt/disable` services have no request fields and do not provide a target-selection contract.
- Proposed decision: add `/rt/reset_fault` as one group-level service implemented by `enable_manager`. One call inspects all 13 axes, applies a bounded standard Fault Reset pulse only to axes currently in Fault while holding every other axis non-enabled, and succeeds only when every targeted Fault axis leaves Fault and reaches Switch On Disabled; otherwise it reports the first failed joint, raw statusword, and reset stage. Reuse the existing structured `robot_interfaces/srv/RtEnable` response shape for this empty-request group operation. Reject the call while enable/disable/reset is already running; never continue automatically into enable.
- Benefit: gives operations one explicit, auditable recovery action for the same 13-axis atomic group, preserves structured failure reporting, and cannot accidentally reset an arbitrary joint name supplied by a caller. It also keeps controlword ownership in one controller.
- Drawback: every currently faulted EtherCAT axis is reset together; selective single-axis maintenance is unavailable. The service type name `RtEnable` is semantically broader than this endpoint even though its response fields fit.
- Alternative cost: a target-joint request allows selective maintenance but requires a new service schema, rules for empty/duplicate/unknown joints, and can leave the group in a partially fault-cleared state; using `std_srvs/Trigger` loses the approved structured joint/statusword failure result.
- Deferred separate decision: the exact low/high pulse duration and timeout are intentionally left to the next question after the service scope is approved.
- Decision: add `/rt/reset_fault` as the proposed 13-axis group-level service in `enable_manager`. Reset only axes currently in Fault, keep every other axis non-enabled, require every targeted axis to reach Switch On Disabled, return structured first-failure information, reject concurrent FSM operations, and never continue automatically into enable. Reuse the existing empty-request `robot_interfaces/srv/RtEnable` response shape for the endpoint.
- Unblocked scope: the service ownership and group scope are frozen; its pulse/timeout behavior remains BQ-032.

## BQ-032 — EtherCAT group Fault Reset pulse and timeout are not frozen [RESOLVED 2026-07-21]

- Evidence: CiA402 requires a controlword bit-7 rising edge to leave Fault, but does not require the application to hold a fixed 200 ms pulse. The control loop is 250 Hz (4 ms). The authoritative legacy fork contains configurable 50-cycle low/high holds for a different automatic-reset path, while the new service is explicit and must not retry or continue into enable. No approved service-specific pulse duration exists.
- Proposed decision: for each group-service call, hold `0x0000` for one full 4 ms RT cycle on all currently faulted axes to guarantee bit 7 is low; then write `0x0080` to those axes and hold it only until each axis leaves Fault or a single `fault_reset_timeout: 4.0s` expires. As soon as an axis leaves Fault, return that axis to `0x0000` and require `statusword & 0x004F == 0x0040`. Make exactly one rising-edge attempt per service call, with no automatic retry. On timeout, keep all controlwords at `0x0000`, latch FAILED, and return the first still-faulted joint/raw statusword/stage.
- Benefit: follows the standard edge semantics, is state-confirmed rather than delay-confirmed, avoids an arbitrary fixed high pulse, and reuses the already approved 4-second safety-stage budget. A failed reset cannot chatter or flow into enable.
- Drawback: some vendor firmware may require a longer minimum low/high pulse despite the standard edge model; that must be detected during commissioning. Holding `0x0080` until state change can last up to four seconds on a non-clearing fault, although no enable command is issued.
- Alternative cost: copying the legacy fixed 50-cycle low plus 50-cycle high sequence adds 400 ms of unneeded fixed delay and was designed for automatic plugin reset rather than an explicit state-confirmed service; allowing repeated edges within one call can mask a persistent fault and create reset chatter.
- Decision: implement the proposed single-attempt, state-confirmed reset sequence: one 4 ms `0x0000` cycle, then `0x0080` until each targeted axis exits Fault or the 4.0-second timeout, followed by `0x0000` and confirmation of `0x0040`. Do not retry or enable automatically; timeout latches FAILED and reports the first remaining fault.
- Unblocked scope: the Fault Reset execution path of T-010 is fully specified, subject to commissioning confirmation of drive pulse acceptance.

## BQ-033 — EtherCAT output-process watchdog timeout is not frozen [RESOLVED 2026-07-21]

- Evidence: the pinned ICube `GenericEcSlave::setup_syncs()` already configures SyncManager 2 (master-to-drive RPDO) with `EC_WD_ENABLE` whenever the slave YAML omits an explicit `sm` section. Therefore the output-process watchdog is enabled by the open-source implementation for all current motion profiles; no additional software watchdog node is needed for this mechanism.
- Remaining gap: ICube never calls IgH `ecrt_slave_config_watchdog()`. IgH consequently leaves registers `0x0400` (watchdog divider) and `0x0420` (watchdog intervals) at the slave/SII defaults. Neither the five authority documents nor a matching ESI/live register archive records those defaults for the ZeroErr and Ti5Robot drives. The effective timeout and each drive's resulting CiA402/mechanical reaction are therefore unproven. A broken EtherCAT link cannot be made safe by sending a later host-side hold/disable command, so this drive-local behavior must be verified before powered motion.
- Proposed decision A (recommended, closest to upstream): retain ICube's existing `EC_WD_ENABLE` behavior and do not patch its watchdog-timing API. Before powered motion, read/archive `0x0400` and `0x0420` for both drive families and perform a supported low-risk link/process-data interruption test that records the measured trip latency, AL/CiA402 state, brake behavior, and mechanical stop/hold result. Reject commissioning if the observed timeout/reaction is not acceptable; any later timing change then requires vendor/ESI evidence and a new approval.
- Benefit of A: preserves the tested open-source path, adds no dependency patch or guessed register write, and verifies the behavior that the hardware actually executes rather than assuming it from configuration. It also respects the hardware mapping's prohibition on inventing watchdog values.
- Drawback of A: the timeout may differ by drive family or firmware and is not deterministically reprogrammed at startup; an unacceptable factory/persisted default will block powered motion and require a follow-up design decision.
- Alternative B: extend the ICube overlay with explicit `watchdog_divider`/`watchdog_intervals` configuration and call `ecrt_slave_config_watchdog()` before master activation, after an exact timeout and per-drive reaction have been justified by vendor/ESI evidence.
- Benefit of B: makes the watchdog timing deterministic and reproducible on every startup.
- Drawback of B: creates another safety-critical downstream ICube patch and writes ESC watchdog registers; no authoritative numeric value exists today, so choosing B immediately creates a second blocked decision and carries nuisance-trip risk at the 4 ms process-data period.
- Question: approve A—keep the upstream-enabled watchdog with slave/SII timing, but block powered EtherCAT motion until the two drive families' effective registers and physical reaction are archived and tested—or choose B and defer implementation until an explicit timeout is separately approved?
- Decision: approve A. Retain the pinned ICube implementation's existing `EC_WD_ENABLE` behavior and the slave/SII watchdog timing; do not add an `ecrt_slave_config_watchdog()` timing patch or write guessed divider/interval values. Before any powered EtherCAT motion, read and archive `0x0400`/`0x0420` for both ZeroErr and Ti5Robot families and complete a supported low-risk interruption test recording the effective trip latency, AL/CiA402 state, brake behavior, and mechanical stop/hold result. An unacceptable result blocks motion and requires a separate evidence-backed timing/reaction decision.
- Unblocked scope: software may retain upstream SyncManager watchdog configuration without another ICube timing patch; powered T-014/T-015 EtherCAT motion remains gated by the register archive and physical-reaction test.

## BQ-034 — The frozen 13-DoF planning axis conflicts with the adjudicated JTC joint set [RESOLVED 2026-07-21]

- Evidence: frozen `REQ-IF-007` says MoveIt plans 13 DoF as the 12 arm joints plus updown, executes the 12 arm axes through JTC and extracts updown as a PP endpoint; waist/turn is outside that planning group. System architecture v1.2 says the same: left arm 6 + right arm 6 + lift/updown form the 13-DoF group, while waist/turn is discrete and workflow-separated. The supplied URDF confirms that `turn` is a continuous rotary joint and `updown` is a distinct prismatic joint mounted after it. Hardware mapping confirms `turn` is the thirteenth EtherCAT CSP drive while `updown` is CANopen Node 1 PP.
- Conflict: the later AMB-002/BQ-002 implementation record instead configures `dual_arm_jtc` as 12 arms plus `turn`, applies the one-degree JTC admission check to 13 EtherCAT axes, and places updown entirely on a separate command path. That changes which physical axis participates in coordinated planning and directly contradicts the frozen requirement that the user previously declared correct.
- Proposed decision A (recommended, follows frozen requirements): restore the planning/execution split to 12 arm joints in `dual_arm_jtc`; motion supplies the arm FJT plus the separately extracted updown PP command from the same 13-DoF plan. Keep both admission checks inside rt-control: the patched JTC checks the 12 arm first-point errors against `0.017453292519943295 rad`, while `updown_position_controller` checks `expected_start_position` against actual updown position within `0.05 m`. Keep `turn` outside this JTC/planning group; freeze its separate command controller/interface in the next question. This supersedes only the conflicting joint-set portions of AMB-002/BQ-002/BQ-019, not the approved preload, freshness, JTC-patch, or updown-controller behavior.
- Benefit of A: matches the frozen REQ and system architecture, preserves updown's real PP semantics instead of streaming it as CSP, and keeps the already approved checks in rt-control. It also avoids falsely treating turn as the coordinated lift axis.
- Drawback of A: motion must split the 13-DoF planned result into a 12-arm FJT and one `/updown/command`; their start coordination is not atomic at the ROS interface, and the separate turn command contract still needs one decision. The JTC admission check covers 12 radian joints rather than the currently recorded 13.
- Alternative B: retain the current adjudication—12 arms plus turn in one 13-axis EtherCAT JTC, with updown independent.
- Benefit of B: all EtherCAT position axes share one stock JTC execution timeline and it needs no new turn command path.
- Drawback of B: it contradicts frozen `REQ-IF-007` and the architecture's kinematic planning model; MoveIt would coordinate waist rotation instead of vertical lift, while the planned lift dimension would not follow the frozen execution split.
- Rejected hybrid: placing CANopen PP updown directly in a streaming JTC with the 12 CSP arm axes would send interpolated position samples to a point-to-point drive and overturn the approved PP endpoint semantics.
- Question: approve A and restore the frozen `12 arms + updown` planning model, or explicitly choose B and authorize a frozen-requirement change?
- Decision: choose B and explicitly supersede the conflicting joint-membership portion of frozen `REQ-IF-007` and system architecture v1.2. `dual_arm_jtc` controls exactly the 12 arm joints plus EtherCAT `turn` as one 13-axis FJT group; CANopen `updown` remains completely independent on its approved PP command path. The JTC first-point consistency check consequently covers those 13 EtherCAT radian joints at the exact one-degree tolerance, while the separate updown controller retains its `0.05 m` expected-start check. Do not later reinterpret the thirteenth JTC axis as updown.
- Benefit of the selected design: all thirteen EtherCAT CSP axes execute on one JTC timeline, and updown retains its hardware-native point-to-point command semantics.
- Drawback of the selected design: this is an explicit frozen-requirement/architecture change—vertical lift is no longer part of the 13-DoF coordinated FJT group, while turn is; any motion/MoveIt configuration derived from the older `12 arms + updown` model must be updated consistently.
- Unblocked scope: retain the existing BQ-002/BQ-019 13-EtherCAT-axis JTC design and the independent BQ-011 through BQ-018 updown path. T-003/T-007 and motion-facing configuration must use the selected membership consistently.

## BQ-035 — Service results when `/rt/disable` preempts an in-progress `/rt/enable` are not frozen [RESOLVED 2026-07-21]

- Evidence: spec section 8 requires an enable request received in `ENABLED` to be idempotently successful and requires a disable received during enable to finish the current batch and then transition to `DISABLING`. `RtEnable.srv` is also required to return the final structured outcome. However, neither the enable response after this preemption nor the callback execution mechanism is specified.
- Runtime constraint: the pinned `ros2_control_node` uses a `MultiThreadedExecutor` and runs read/update/write in a separate RT thread, but services created in a controller's default mutually-exclusive callback group execute serially. If `/rt/enable` synchronously waits there for its final result, `/rt/disable` cannot enter until enable has already returned. Returning immediately would allow preemption but would turn `ok` into mere request acceptance and lose the required final hardware failure fields.
- Proposed decision A (recommended): keep all three services synchronous to the terminal hardware result. Put `/rt/enable` and `/rt/disable` in separate mutually-exclusive callback groups so they may overlap in the non-RT executor. Each callback transfers only an atomic/request record to the RT update FSM and polls a sequence-tagged atomic result outside RT; the RT path takes no lock and creates no string. When disable arrives during enable, complete the currently active enable batch as specified, do not start another batch, then run the approved state-aware group disable. The original enable call returns `ok=false`, `failed_batch=-1`, empty `failed_joint`, `status_word=0`, `stage="preempted_by_disable"`; the disable call waits for the downward sequence and returns its actual final structured success/failure. A preemption is not falsely attributed to a drive or batch.
- Benefit of A: preserves final-result service semantics, makes the already frozen preemption behavior executable, keeps RT free of locks/services/allocations, and lets the caller that requested disable know whether all axes actually reached the disabled terminal state.
- Drawback of A: both service calls can remain open for several seconds, so clients need an adequate RPC timeout; implementation needs separate callback groups plus sequence IDs/atomics. The enable caller receives failure even though the interruption was intentional and the final machine state may be safely disabled.
- Alternative B: return `ok=true` from enable as soon as the current batch finishes and then process disable.
- Benefit of B: an enable call is not reported as failed merely because a later disable superseded it.
- Drawback of B: `ok=true` would claim successful enable even though the full five-batch operation never reached `ENABLED`, violating the atomic operation meaning and potentially allowing an orchestration layer to advance incorrectly.
- Alternative C: make services return immediate request acceptance and observe final state only through diagnostics.
- Benefit of C: callbacks never wait and default serial callback behavior is harmless.
- Drawback of C: changes the approved `RtEnable` response from a hardware result into an enqueue acknowledgement, so its failure batch/joint/statusword fields cannot fulfill `REQ-IF-001`.
- Question: approve A, including `stage="preempted_by_disable"` for the interrupted enable response, or choose a different return semantic?
- Decision: approve A exactly as proposed. Services remain synchronous to their terminal hardware result and use separate mutually-exclusive callback groups. A disable arriving during enable finishes the current enable batch, prevents any next enable batch, and runs the approved state-aware group disable. The interrupted enable returns `ok=false`, `failed_batch=-1`, empty `failed_joint`, `status_word=0`, and `stage="preempted_by_disable"`; the disable call returns the actual terminal disable result. RT communication remains lock-free and allocation-free through numeric sequence-tagged request/result records, with strings formed only in the non-RT callback.
- Unblocked scope: the opposite-direction enable-to-disable preemption and its two service responses are frozen for T-010.

## BQ-036 — Duplicate same-direction enable/disable calls during an active operation are not frozen [RESOLVED 2026-07-21]

- Evidence: spec section 8 defines an enable call in the already terminal `ENABLED` state as idempotently successful, but says nothing about a second `/rt/enable` received while the first call is still `ENABLING`, or a second `/rt/disable` received while `DISABLING`. The empty request carries no command ID, so the controller cannot distinguish a transport/client retry from a separately intended duplicate operation.
- Proposed decision A (recommended): coalesce duplicate same-direction calls into the active operation. Do not restart a batch/stage, reset any timeout, or create a second FSM. Each waiting callback attaches to the active numeric operation sequence and returns the same final structured result when that operation finishes. An enable already in terminal `ENABLED` and a disable already in terminal disabled/IDLE return immediate idempotent success. This rule does not change BQ-035's opposite-direction disable preemption or BQ-031's rejection of reset while another operation is active.
- Benefit of A: makes ordinary service retries idempotent, prevents a retry from extending safety timeouts or replaying controlword edges, and gives every caller the real final hardware result.
- Drawback of A: multiple callers can wait on one long operation and cannot tell whether they initiated it; all receive the same failure/preemption result even if they called at different stages.
- Alternative B: immediately reject the duplicate with `ok=false`, `stage="operation_in_progress"`.
- Benefit of B: ownership is explicit and the second caller returns promptly.
- Drawback of B: a harmless retry appears as an operational failure, and clients need additional retry/state logic despite the service having no request ID or status query dedicated to this purpose.
- Question: approve A—coalesce same-direction calls without restarting or extending the active FSM—or choose B and reject them as busy?
- Decision: choose B. A same-direction request received while its operation is active returns immediately with `ok=false`, `failed_batch=-1`, empty `failed_joint`, `status_word=0`, and `stage="operation_in_progress"`. It does not attach to the operation, restart any batch/stage, replay a controlword edge, or reset/extend a timeout. Calls made after the corresponding terminal state has been reached retain immediate idempotent success.
- Benefit of the selected policy: only the original caller owns and waits for the active operation, while duplicate traffic has no timing or controlword side effect.
- Drawback of the selected policy: an ordinary client retry is reported as busy/failure and must be retried after the original call completes; without a command ID, it cannot recover the original response through the duplicate call.
- Unblocked scope: same-direction concurrency is frozen for `/rt/enable` and `/rt/disable` in T-010.

## BQ-037 — `/rt/enable` received while the group is disabling is not frozen [RESOLVED 2026-07-21]

- Evidence: BQ-035 defines the safety-direction transition from an active enable into disable. Neither the frozen specification nor the service contract defines the reverse case: a new `/rt/enable` received while the standard downward sequence is still `DISABLING`. Reversing immediately would leave axes at mixed CiA402 states; queueing it would cause later motion authority to arise from a request made before disable reached its terminal state.
- Proposed decision A (recommended): reject enable immediately while `DISABLING` with `ok=false`, `failed_batch=-1`, empty `failed_joint`, `status_word=0`, and `stage="operation_in_progress"`. Continue the existing disable sequence unchanged. After all axes reach Switch On Disabled and the disable call returns, the caller must issue a fresh `/rt/enable`; that new call reruns preload validation and the complete five-batch sequence.
- Benefit of A: the downward safety transition cannot be reversed or followed by an old queued request; every later enable is based on terminal disabled state and fresh position feedback/preload checks.
- Drawback of A: an operator/orchestrator must wait and retry explicitly, increasing recovery latency and requiring it to handle a busy response.
- Alternative B: queue one enable request and start it automatically as soon as disable succeeds.
- Benefit of B: reduces caller orchestration and can shorten turnaround.
- Drawback of B: a stale request can re-enable automatically after the caller's context has changed, and a failed disable creates ambiguous ownership/cancellation semantics. It also bypasses the requirement for a fresh post-disable service decision.
- Question: approve A—reject enable while disabling and require a fresh call after terminal disable—or choose B and queue one automatic re-enable?
- Decision: approve A. While the group is `DISABLING`, `/rt/enable` returns immediately with `ok=false`, `failed_batch=-1`, empty `failed_joint`, `status_word=0`, and `stage="operation_in_progress"`; it neither reverses nor queues behind the downward sequence. A fresh enable call is required only after terminal disable, and that call reruns the full preload validation and five-batch enable sequence.
- Unblocked scope: disable-to-enable opposite-direction concurrency is frozen for T-010.

## BQ-038 — `/rt/disable` behavior during an active Fault Reset is not frozen [RESOLVED 2026-07-21]

- Evidence: BQ-031/032 keep all non-faulted axes non-enabled during `/rt/reset_fault`, while targeted Fault axes receive one low cycle followed by `0x0080` until they clear or time out. BQ-031 rejects a reset request when another operation is active, but does not specify the reverse case of a disable request arriving during reset. A disable is normally the higher-priority safety-direction request, yet a still-faulted axis cannot satisfy the normal disable terminal confirmation `statusword & 0x004F == 0x0040` without a successful reset.
- Proposed decision A (recommended): allow `/rt/disable` to preempt Fault Reset. On the next RT update, cancel the reset attempt and set every controlword to `0x0000`; do not issue another bit-7 edge. The reset call returns `ok=false`, `failed_batch=-1`, empty `failed_joint`, `status_word=0`, and `stage="preempted_by_disable"`. The disable call evaluates/executes the state-aware downward sequence for every non-faulted axis. It returns success only if all 13 axes reach `0x0040`; if a canceled target remains in Fault, disable returns the first such joint/raw statusword with `stage="fault_requires_reset"`, while keeping all commands at `0x0000`. A later explicit reset call is required.
- Benefit of A: `/rt/disable` always has priority and immediately removes the reset bit; no recovery action continues after an operator asks to disable. The response remains truthful about Fault axes that cannot reach the requested terminal state.
- Drawback of A: disable can intentionally interrupt a nearly completed reset and then return failure even though no axis is enabled and all controlwords are zero. Operations must distinguish “electrically commanded non-enable” from “all axes confirmed Switch On Disabled.”
- Alternative B: reject disable with `stage="operation_in_progress"` and let the bounded reset finish, after which the caller retries disable.
- Benefit of B: avoids interrupting the single reset edge and usually leaves axes in a state where terminal disable can be confirmed.
- Drawback of B: a safety-direction disable request cannot stop the ongoing reset attempt for up to the remaining four-second timeout.
- Question: approve A—disable preempts reset immediately, with a truthful failure if a Fault axis cannot confirm `0x0040`—or choose B and reject disable as busy until reset completes?
- Decision: approve A. `/rt/disable` preempts an active Fault Reset on the next RT update, cancels the reset attempt, writes `0x0000` to every axis, and emits no further bit-7 edge. The reset response is `ok=false`, `failed_batch=-1`, empty `failed_joint`, `status_word=0`, `stage="preempted_by_disable"`. Disable completes the normal state-aware descent for non-faulted axes and succeeds only if all thirteen confirm `0x0040`; any remaining Fault produces the first joint/raw statusword with `stage="fault_requires_reset"`, while all controlwords stay at `0x0000`.
- Unblocked scope: disable priority over reset and the truthful terminal response are frozen for T-010.

## BQ-039 — `/rt/enable` received during an active Fault Reset is not frozen [RESOLVED 2026-07-21]

- Evidence: Fault Reset is a bounded recovery operation that explicitly must not continue automatically into enable. The specification rejects a new reset while another FSM operation is running, but does not say whether an enable arriving during reset is rejected, queued, or starts immediately after reset clears the final Fault.
- Proposed decision A (recommended): reject `/rt/enable` immediately while Fault Reset is active with `ok=false`, `failed_batch=-1`, empty `failed_joint`, `status_word=0`, and `stage="operation_in_progress"`. Do not queue it and do not alter the reset attempt. After reset completes successfully and all targeted axes confirm `0x0040`, the caller must make a fresh enable request, which reruns preload validation and the complete five-batch sequence.
- Benefit of A: preserves the approved “reset never auto-enables” boundary, prevents a stale enable from taking effect after delayed recovery, and ensures enabling always reflects a fresh post-reset operator/orchestrator decision.
- Drawback of A: adds one explicit round trip and requires callers to wait/retry after reset.
- Alternative B: queue one enable and start it automatically after successful reset.
- Benefit of B: shorter recovery workflow.
- Drawback of B: semantically couples reset to automatic enable, contrary to BQ-031/032, and introduces cancellation/stale-request behavior if reset is slow or the operating context changes.
- Question: approve A—reject enable during reset and require a fresh post-reset call—or choose B and queue automatic enable?
- Decision: approve A. An enable call received during active Fault Reset returns immediately with `ok=false`, `failed_batch=-1`, empty `failed_joint`, `status_word=0`, and `stage="operation_in_progress"`. It is not queued and does not alter reset. A fresh post-reset enable call is required and reruns preload validation plus the complete five-batch sequence.
- Unblocked scope: enable concurrency during reset is frozen for T-010.

## BQ-040 — Fault Reset called when no axis is in Fault has no defined result [RESOLVED 2026-07-21]

- Evidence: `/rt/reset_fault` is a group service with an empty request that scans all thirteen EtherCAT axes and targets only axes whose masked statusword is Fault. BQ-031/032 define success when every targeted axis clears Fault and reaches `0x0040`, but do not define the empty-target case. Emitting the approved low/high reset sequence when no axis is in Fault would be unnecessary and could create a bit-7 edge on a drive that was not meant to be reset.
- Proposed decision A (recommended): treat an empty target set as immediate idempotent success: return `ok=true`, `failed_batch=-1`, empty `failed_joint`, `status_word=0`, and `stage="already_clear"`. Do not start the RT reset FSM and do not change any controlword.
- Benefit of A: repeated/operator-safe reset calls have no hardware side effect, and the service means “the group has no outstanding Fault requiring reset” on return.
- Drawback of A: a caller cannot use `ok=false` to detect that no reset action was actually performed; it must inspect `stage="already_clear"` if that distinction matters.
- Alternative B: return `ok=false`, `stage="no_fault_present"` without writing anything.
- Benefit of B: clearly tells the caller its requested action had no target.
- Drawback of B: makes an already-safe postcondition look like a recovery failure and complicates idempotent operations/scripts.
- Question: approve A—no Fault means side-effect-free idempotent success—or choose B and report it as a no-target failure?
- Decision: approve A. If no axis is in Fault, `/rt/reset_fault` returns immediately with `ok=true`, `failed_batch=-1`, empty `failed_joint`, `status_word=0`, and `stage="already_clear"`; it starts no RT reset state and changes no controlword.
- Unblocked scope: the empty-target reset result is frozen for T-010.

## BQ-041 — EtherCAT single-axis runtime Fault has no group-stop owner after watchdog removal [RESOLVED 2026-07-21]

- Evidence: frozen `REQ-SAFE-003` requires a Fault to stop/hold all thirteen EtherCAT axes and then reach terminal disable. The standalone watchdog was removed by BQ-006. Pinned upstream ICube decodes each drive's statusword but neither turns a CiA402 Fault into a controller-manager hardware error nor stops peer axes. `dual_arm_jtc` does not claim statusword/controlword, so it continues sampling and writing the active trajectory for healthy axes. The authoritative fork's preload/takeover patch is not a runtime group-stop mechanism: `position_command_takeover_allowed_` becomes permanently true after initial synchronization and is not cleared on a later Fault.
- Conflict: without a new owner, one EtherCAT drive can enter Fault while the other twelve remain Operation Enabled and follow the old FJT, directly violating the group safety requirement. Merely writing Quick Stop controlwords also leaves the JTC goal and its cached command alive for a later re-enable.
- Proposed decision A (recommended): make the existing `enable_manager` the EtherCAT group-Fault owner. During `ENABLING` or `ENABLED`, the first observed `Fault Reaction Active` or `Fault` statusword latches a group fault in the RT update and immediately commands `0x0002` to every non-faulted axis (the faulted axis receives non-enabling `0x0000`). It signals a non-RT callback/worker to request strict deactivation of `dual_arm_jtc` through controller_manager, so the active FJT is terminated and its old position command cannot resume. After JTC deactivation is confirmed, `enable_manager` drives healthy axes to terminal Switch On Disabled and leaves Fault axes at `0x0000`, reporting the original first-fault joint/statusword. Recovery remains explicit: correct the cause, call group `/rt/reset_fault`, then make a fresh `/rt/enable`; no old trajectory is replayed. The exact Quick-Stop-to-terminal transition and JTC reactivation order are deferred to the next questions rather than guessed.
- Benefit of A: detects the fault in the existing 250 Hz control update, stops healthy peers through the controller that already owns every controlword, and explicitly destroys the stale active trajectory before recovery. It adds no watchdog package, raw bus observer, or hardware-plugin Fault policy.
- Drawback of A: `enable_manager` gains a non-RT controller-manager interaction and fault recovery becomes disruptive: the FJT is aborted, JTC must later be reactivated, and any controller-switch failure needs a latched diagnostic/restart path. This remains a software protective stop, not a rated safety function.
- Alternative B: patch ICube to return hardware `ERROR` when any drive statusword enters Fault and rely on controller_manager lifecycle/error handling to deactivate controllers.
- Benefit of B: uses controller-manager hardware-error propagation rather than a custom controller-to-manager request.
- Drawback of B: is a broader hardware-plugin semantic change, does not by itself specify the peer-axis Quick Stop/terminal-disable controlwords or structured first-fault response, and can tear down the whole EtherCAT hardware component before `/rt/reset_fault` can operate.
- Rejected minimal behavior: issuing group `0x0002` while leaving JTC active does stop present drive operation but preserves the old goal/command, creating a possible stale-command takeover after reset/re-enable.
- Question: approve A and assign EtherCAT group-Fault handling to `enable_manager`, including non-RT JTC deactivation and explicit reset/re-enable recovery, or choose B and move fault escalation into the ICube hardware lifecycle?
- Decision: approve A. `enable_manager` owns EtherCAT group-Fault reaction. The first `Fault Reaction Active` or `Fault` observed during `ENABLING`/`ENABLED` latches the first-fault joint/statusword, commands group Quick Stop/non-enable in RT, and triggers non-RT strict deactivation of `dual_arm_jtc`. Recovery never replays the old FJT and requires explicit cause correction, group reset, fresh enable, and a new trajectory. Do not convert a drive Fault into an ICube hardware lifecycle `ERROR` for this mechanism and do not recreate a watchdog package.
- Unblocked scope: the group-Fault owner and JTC-abort requirement are frozen; the exact Quick-Stop-to-terminal sequence and controller reactivation remain the next decisions.

## BQ-042 — Quick-Stop-to-terminal-disable sequence after an EtherCAT group Fault is not frozen [RESOLVED 2026-07-21]

- Evidence: normal `/rt/disable` starts from Operation Enabled and uses the approved `0x0007 -> 0x0006 -> 0x0000` sequence. After BQ-041's emergency `0x0002`, healthy drives normally enter Quick Stop Active (`statusword & 0x006F == 0x0007`), whose standard non-reenabling exit to Switch On Disabled is Disable Voltage (`0x0000`), not replaying the normal three-step path. Depending on the configured drive quick-stop option, an axis may instead already fall to a lower/terminal state. The EtherCAT quick-stop acknowledgement duration and the behavior when JTC deactivation fails were not frozen.
- Proposed decision A (recommended): on the first group Fault, hold healthy Operation Enabled axes at `0x0002` until each reports Quick Stop Active or an already-lower safe state, using the existing `disable_stage_timeout: 4.0s` rather than inventing a new timeout. Fault/Fault-Reaction axes receive `0x0000`. Once an axis is Quick Stop Active, command the standard `0x0000` Disable Voltage and require `statusword & 0x004F == 0x0040`; axes already in Switched On or Ready follow the BQ-030 state-aware downward steps, and axes already at `0x0040` are complete. Run the non-RT strict JTC-deactivation request in parallel. Even if JTC deactivation fails/times out, still complete the hardware terminal-disable sequence because position commands cannot move a drive held at controlword `0x0000`; then latch `stage="jtc_deactivate_failed"`, reject reset/enable recovery in this process, and require a full rt-control restart so no cached JTC trajectory survives. Preserve the original first drive Fault as the primary diagnostic and add the JTC-switch failure as a recovery blocker.
- Benefit of A: follows the correct CiA402 exit for Quick Stop Active, does not leave healthy drives powered in Quick Stop indefinitely, and still reaches the safest available hardware state if ROS controller switching fails. Reusing four seconds avoids an unapproved new safety timeout.
- Drawback of A: `0x0000` removes drive voltage/torque after the configured Quick Stop; the exact mechanical brake/coast behavior remains drive-dependent and must be verified. A JTC-deactivation failure forces a disruptive full container restart even if all axes reached `0x0040`.
- Alternative B: if JTC deactivation is not confirmed, keep healthy axes indefinitely at `0x0002` and do not proceed to `0x0000`.
- Benefit of B: preserves the drive's Quick Stop active torque/braking behavior while the software trajectory owner is uncertain.
- Drawback of B: fails the approved terminal-disable requirement, may leave drives energized indefinitely, and makes recovery dependent on a controller-manager operation that is not needed to physically remove drive voltage.
- Question: approve A—state-confirm Quick Stop, then use standard Disable Voltage to terminal state even if JTC deactivation fails, with restart required on that failure—or choose B and remain latched in Quick Stop until JTC is confirmed inactive?
- Decision: approve A. Healthy enabled axes hold `0x0002` for state confirmation using the existing four-second stage timeout, then Quick Stop Active exits through standard `0x0000` Disable Voltage to confirmed `0x0040`; lower-state axes join the BQ-030 path and Fault axes stay at `0x0000`. JTC deactivation runs in parallel. A JTC deactivation failure does not prevent terminal hardware disable, but latches `jtc_deactivate_failed`, blocks reset/enable in that process, and requires a full rt-control restart.
- Unblocked scope: the emergency Quick-Stop-to-terminal-disable path and controller-switch failure policy are frozen for T-010.

## BQ-043 — The authoritative preload patch is one-shot across disable/re-enable cycles [RESOLVED 2026-07-21]

- Evidence: BQ-002/frozen `REQ-ECAT-005` require raw `0x6064 -> 0x607A` preload before permitting `0x000F`. The reviewed authoritative fork sets `target_position_preloaded_` after its first successful write and sets `position_command_takeover_allowed_` after the first close controller command, but never clears either flag when a drive leaves Operation Enabled, is normally disabled, enters Fault, or is reset. Therefore a second enable in the same process can retain a stale target/preload proof and can immediately pass an old JTC command. This fork behavior is not sufficient for the approved repeated recovery workflow.
- Proposed decision A (recommended): treat preload and controller takeover as per-enable-session state. On any confirmed transition out of Operation Enabled, and also during controller activation/initialization before the first enable, clear `target_position_preloaded_`, its stored preload value, `position_command_takeover_allowed_`, and `position_command_synchronized_`. While the axis is non-enabled and valid fresh process data is available, force raw actual `0x6064` into `0x607A`; mark preload complete only after that PDO write has executed. Keep overriding the position command with this held preload until a newly activated JTC writes a finite command within the exact one-degree threshold of current actual position. Only then allow position-command takeover for that enable session. Fault Reset alone never restores the old takeover permission.
- Benefit of A: enforces preload on every normal re-enable and Fault recovery, prevents an old JTC command from surviving across a disabled interval, and matches the literal “before permitting `0x000F`” requirement rather than merely protecting process startup.
- Drawback of A: this is a deliberate correction to the authoritative fork rather than a byte-for-byte copy. A displaced unpowered axis causes the next enable to hold its new physical position, so the caller must send a newly admitted trajectory; takeover may remain blocked until JTC is active and has seeded a close hold command.
- Alternative B: retain the fork's one-shot latches and rely on JTC deactivation/restart to clear trajectory state.
- Benefit of B: smallest source delta from the authoritative fork.
- Drawback of B: a normal disable/re-enable without process restart can reuse stale `0x607A` and permanent command permission; it violates the approved preload requirement and can produce a jump if an axis moved while disabled.
- Question: approve A and make preload/takeover proof session-scoped, or retain the fork's one-shot behavior B?
- Decision: approve A. Preload value/proof and position-command takeover/synchronization are cleared on every confirmed exit from Operation Enabled and during initial activation. Every enable session performs a new raw actual-to-target preload and permits controller takeover only after a newly seeded finite JTC command is within the exact one-degree threshold. Fault Reset never restores the prior session's permission.
- Unblocked scope: repeated normal enable and Fault-recovery preload semantics are frozen; JTC lifecycle ordering remains separate.

## BQ-044 — JTC lifecycle ordering around normal enable/disable and Fault recovery is not frozen [RESOLVED 2026-07-21]

- Evidence: pinned JTC 2.53.2 rejects FJT goals while INACTIVE, aborts its active goal in `on_deactivate()`, and seeds a hold trajectory in `on_activate()`. Its parameter `set_last_command_interface_value_as_state_on_activation` can instead seed from stale command-interface values; that behavior is unsafe after a disabled interval. BQ-043 now requires a new close command before each session's takeover. Leaving JTC continuously active across `/rt/disable` retains its last hold/trajectory command and provides no lifecycle event to reseed it from current actual position.
- Proposed decision A (recommended): couple `dual_arm_jtc` lifecycle to the existing enable manager without adding another public service. Bringup configures/spawns JTC INACTIVE and sets `set_last_command_interface_value_as_state_on_activation: false`. For normal `/rt/disable`, non-RT logic strictly deactivates JTC (aborting any active FJT) and then the RT FSM completes the approved state-aware hardware disable; if deactivation fails, hardware disable still completes but recovery is restart-only, matching BQ-042. For `/rt/enable`, require JTC to be INACTIVE, perform fresh per-session preload and all five hardware enable batches while the plugin holds the preload target, then strictly activate JTC. Activation reads the thirteen actual positions and seeds a current-position hold; the enable service returns `ok=true` only after JTC is ACTIVE. If JTC activation fails or its final state is ambiguous, execute BQ-030 rollback to terminal hardware disable, return `stage="jtc_activate_failed"`, and require restart if the controller state cannot be confirmed INACTIVE. Fault recovery uses exactly the same reset -> fresh enable -> final JTC activation sequence.
- Benefit of A: no trajectory can be accepted during partial hardware enable or while disabled; every successful enable ends with a freshly seeded stock JTC hold, and every disable destroys the old active goal. It uses existing lifecycle behavior rather than another command-permit topic/interface.
- Drawback of A: every normal enable/disable now includes a controller-manager switch and aborts any active FJT. Hardware is Operation Enabled holding its preload briefly before JTC activation; activation failure adds rollback latency and may force a full restart if lifecycle state is uncertain.
- Alternative B: leave JTC ACTIVE across hardware disable/re-enable and add a new enable-state admission/command-reseed gate to the JTC patch.
- Benefit of B: avoids controller lifecycle switching and keeps the action server continuously active.
- Drawback of B: broadens the JTC patch and needs a new cross-controller state channel; old goals/commands must be explicitly invalidated and the action can otherwise accept motion while hardware is only partially enabled.
- Question: approve A—JTC inactive while hardware is disabled/being enabled, activated only after all thirteen axes hold in Operation Enabled, and deactivated on every disable—or choose B and keep JTC continuously active with an additional gate?
- Decision: approve A. Bringup leaves `dual_arm_jtc` INACTIVE and configures `set_last_command_interface_value_as_state_on_activation: false`. Normal disable deactivates JTC and aborts its active FJT before/alongside terminal hardware disable. Enable performs fresh preload and all five hardware batches first, then activates JTC from actual state; success is returned only after JTC is confirmed ACTIVE. Activation failure invokes BQ-030 rollback and ambiguous lifecycle state requires restart-only recovery.
- Unblocked scope: JTC lifecycle coupling and stale-goal disposal are frozen for T-007/T-010.

## BQ-045 — EtherCAT process-data freshness during preload and enable is not frozen [RESOLVED 2026-07-21]

- Evidence: BQ-020 adds read-only `ethercat_domain/process_data_age_ms`, updated only after `EC_WC_COMPLETE`, because ICube otherwise continues exposing a cached process image when Working Counter is incomplete. BQ-002 applies a 500 ms maximum age when admitting FJT goals. The new per-session preload reads cached `0x6064` and writes `0x607A`, but no decision currently prevents a stale cached actual position/statusword from being used to advance the five-batch enable FSM.
- Proposed decision A (recommended): let `enable_manager` also read the already approved `ethercat_domain/process_data_age_ms` interface; this adds no PDO or new state interface. Before starting preload/enable and throughout `ENABLING`, require the value to be finite and `<= 500 ms`. If it is missing/non-finite/older than 500 ms at any enable update, stop advancing immediately, return/record `stage="stale_process_data"`, and execute BQ-030 state-aware rollback for any axes already advanced. While disabled and data is fresh, the BQ-043 preload continuously refreshes raw `0x607A` from the latest `0x6064`, so a previously cached write cannot authorize the next transition. After the service has completed and the system is fully `ENABLED`, retain frozen `REQ-ECAT-009`: age/WC loss is diagnostic WARN and rejects new FJT admission but does not introduce a new host-side automatic runtime stop; the drive-side SyncManager watchdog from BQ-033 remains the broken-link protection.
- Benefit of A: the safety-critical enable decision and preload cannot be based on an arbitrarily old process image, and it reuses the same transport-age proof/tolerance already approved for trajectory admission. It does not alter the frozen runtime WC policy.
- Drawback of A: any configured slave causing domain WC to remain incomplete for over 500 ms aborts the whole enable or rolls back a partially enabled group. The enable manager gains one additional read-only state-interface dependency.
- Alternative B: use statusword/position values without checking domain age during enable.
- Benefit of B: fewer controller dependencies and transient WC issues do not interrupt enable.
- Drawback of B: cached statusword and `0x6064` can look valid indefinitely, so the service could report a successful preload/transition against data received before the current enable attempt.
- Question: approve A and apply the existing 500 ms complete-domain freshness gate throughout `ENABLING`, without changing the post-enable WARN-only runtime policy, or choose B and allow cached values during enable?
- Decision: choose B. `enable_manager` does not claim or check `ethercat_domain/process_data_age_ms` during preload or `ENABLING`; it advances from the exposed cached `0x6064` and statusword values. The existing 500 ms domain-age interface remains only in the patched JTC's new-goal admission path, and runtime WC/age handling remains WARN-only plus the drive-side SyncManager watchdog.
- Benefit of the selected policy: enable has no extra domain-age dependency and transient/incomplete WC age alone does not abort or roll back the five-batch sequence.
- Drawback explicitly accepted: because ICube exposes the cached process image when WC is incomplete, preload and enable confirmation can use position/statusword data older than the current service attempt; the service has no transport-freshness proof for those values.
- Unblocked scope: no EtherCAT age interface is added to `enable_manager`; BQ-020's JTC-only consumer remains unchanged.

## BQ-046 — Stock `Cia402System` has no automatic non-RT init/mode activation path [RESOLVED 2026-07-21]

- Evidence: BQ-003 assigns Node 1 PP and Nodes 2/3 PV selection to CANopen hardware activation. Pinned `Cia402System::on_activate()` only delegates to `CanopenSystem` and never calls `init_motor()` or `set_operation_mode()`. `operation_mode` in `bus.yml` is not consumed. The available `init_cmd`/mode command interfaces are processed in `Cia402System::write()`; commanding `init_cmd` there invokes blocking `Motor402::handleInit()/switchState()` from the controller-manager read/update/write thread, violating frozen `REQ-RT-004`. Stock `Cia402DeviceController` merely exposes services that pulse those same command interfaces and does not move the blocking work out of `write()`.
- Conflict: BQ-005 otherwise permits no activation-semantic dependency patch, so the approved “hardware activation owns mode/init” behavior currently has no executable owner. Leaving it manual would make startup order and partial-node activation external to the hardware lifecycle.
- Proposed decision A (recommended): extend the existing narrowly pinned ros2_canopen overlay only at `Cia402System::on_activate()`/non-RT setup. For each configured joint, call the existing upstream `init_motor()` unchanged, then select the joint parameter `operation_mode` (`1` for Node 1; `3` for Nodes 2/3) using the existing upstream `set_operation_mode()` confirmation. After mode selection, seed Node 1's PP target from its cached actual `0x6064` and seed both track PV targets to zero through the existing upstream `set_target()` API; no custom CiA402 transition or PP handshake is introduced. Activation fails if any init/mode/seed API reports failure and reports the first node/stage. Keep `Motor402::handleInit()`'s upstream order—including enable before mode/target—because BQ-005 explicitly chose it; powered activation remains blocked until the drive-adaptation table/readback proves that this interval cannot cause motion and that updown holds safely with no new external command.
- Benefit of A: puts blocking initialization in a non-RT lifecycle callback, requires no new package/process/controller, makes every hardware activation deterministic, and reuses the selected upstream state machine/mode/target APIs rather than reimplementing CiA402.
- Drawback of A: expands the downstream ros2_canopen patch beyond BQ-006's callback exposure. Because upstream `handleInit()` enables before the safe targets are seeded, software alone still cannot prove no movement during that interval; production remains dependent on drive configuration and commissioning evidence. Target seeding is accepted by the upstream API but has no new device readback acknowledgement.
- Alternative B: keep ros2_canopen untouched and add a separate non-RT startup coordinator that invokes each driver's stock init/mode services in order, then seeds the targets.
- Benefit of B: no ros2_canopen source change.
- Drawback of B: adds another node/process or launch-time service orchestration, creates partial-start/retry ownership outside the hardware lifecycle, and conflicts with the approved placement of mode selection in hardware activation.
- Rejected command-interface path: a controller pulsing `init_cmd` causes the blocking init call in the RT `write()` path and is therefore not allowed.
- Question: approve A and authorize this narrow `Cia402System::on_activate()` orchestration patch while retaining upstream `Motor402` semantics, or choose B and introduce an external non-RT activation coordinator?
- Decision: approve A. The pinned overlay may orchestrate existing upstream `init_motor()`, configured mode selection, and initial safe target seeding from non-RT `Cia402System::on_activate()`. Node 1 uses PP/current-position seed; Nodes 2/3 use PV/zero seed. Upstream `Motor402` transition and PP handshake semantics remain unchanged, and powered activation stays gated by drive-adaptation/readback testing of the enable-before-target interval.
- Unblocked scope: CANopen init/mode ownership has an executable non-RT hardware-lifecycle path; matching deactivation remains separate.

## BQ-047 — Stock CANopen hardware deactivation stops communication without disabling motors [RESOLVED 2026-07-21]

- Evidence: pinned `CanopenSystem::on_deactivate()` is an empty success and `Cia402System::on_deactivate()` only delegates to it. Cleanup/shutdown cancels the executor and destroys the device container; it does not stop targets or run a CiA402 motor shutdown. Upstream `Motor402::handleShutdown()` already changes to No Mode and uses its standard `switchState(Switch_On_Disabled)` path, but `NodeCanopen402Driver`/`Cia402Driver` do not expose that method to `Cia402System`. The approved normal path reserves Quick Stop for emergency/fault handling rather than routine deactivation.
- Proposed decision A (recommended): extend the same narrow overlay with an internal `shutdown_motor()` facade that only forwards to existing upstream `Motor402::handleShutdown()`. In non-RT `Cia402System::on_deactivate()` (and the still-live portion of shutdown/cleanup), first seed Node 1 PP target from cached actual position and both track PV targets to zero, allow the already frozen `20 ms` CANopen stage hold for those targets to be processed, then call `shutdown_motor()` for all three nodes and require their standard transition to Switch On Disabled. Do not use Quick Stop on this normal path. Attempt all nodes even after one failure; if any target/shutdown step fails, request the already approved all-node NMT Stop, return lifecycle ERROR, and record the first node/stage. Only after these best-effort motor steps does communication teardown proceed. Repeated deactivation when nodes are already Switch On Disabled is idempotent.
- Benefit of A: normal container/controller-manager shutdown uses the existing upstream CiA402 state machine, sets safe motion targets before mode removal, and does not silently drop CAN communication while drives remain enabled. It adds no public service or custom controlword sequence.
- Drawback of A: deactivation blocks its non-RT lifecycle callback for at least the 20 ms target hold plus upstream state-transition waits. Updown hold uses the cached `0x6064` without a freshness gate per BQ-014/BQ-045 policy. Abrupt SIGKILL/power loss can still bypass lifecycle cleanup and depends on drive-side heartbeat/NMT reaction.
- Alternative B: leave stock deactivation/cleanup unchanged and rely solely on loss-of-master heartbeat after communication teardown.
- Benefit of B: no additional ros2_canopen patch or shutdown latency.
- Drawback of B: routine orderly stop intentionally leaves motor state uncontrolled until the 5000 ms heartbeat consumer expires, and may retain nonzero track targets during that interval.
- Question: approve A and use upstream `handleShutdown()` through a narrow non-RT facade after current/zero target seeding, or choose B and rely on the five-second drive heartbeat timeout for routine teardown?
- Decision: approve A. Normal CANopen lifecycle deactivation seeds updown current position and both track zero targets, allows the frozen 20 ms processing hold, then calls a narrow internal facade to upstream `Motor402::handleShutdown()` for all three nodes before communication teardown. It attempts every node, uses all-node NMT Stop and lifecycle ERROR on any failure, and does not use Quick Stop on the routine path.
- Unblocked scope: deterministic non-RT CANopen motor shutdown is frozen for T-007/T-014 and container lifecycle handling.

## BQ-048 — Updown travel limit is `0.92 m` in hardware authority but `0.99 m` in description source [RESOLVED 2026-07-21]

- Evidence: the hardware mapping export, explicitly designated as the unique authority for hardware numbers, freezes Node 1 updown's current range as `[0.0, 0.92] m`. The supplied description source writes `<limit lower="0" upper="0.99">` for the active `updown` prismatic joint and documents `0.99`. This is a real 70 mm mismatch. If description/MoveIt advertises 0.99 while rt-control follows hardware authority, plans in `(0.92, 0.99]` are kinematically accepted but physically outside the approved range.
- Proposed decision A (recommended, follows source priority): apply a documented hardware-authority overlay when migrating the active robot description: set the active updown URDF upper limit to exact text `0.92` and use `[0.0, 0.92] m` as the rt-control updown target-admission range. Preserve the vendor/raw source file as provenance rather than silently rewriting it; record the single intentional source delta and require motion/MoveIt configuration to consume the corrected active description. Do not change the conversion factor `1,000,000 counts/m`.
- Benefit of A: planning, admission, and the declared production hardware range agree, so rt-control rejects unreachable/unauthorized lift targets before transmitting them. It obeys the declared hardware-number authority.
- Drawback of A: changes one number from the pinned description source and reduces its advertised travel by 70 mm. If `0.99 m` is later proven mechanically valid, the hardware authority and all consumers must be revised together rather than merely changing URDF.
- Alternative B: retain `0.99 m` in the active description and allow commands to that limit.
- Benefit of B: preserves the supplied description byte value and its nominal larger workspace.
- Drawback of B: overturns the hardware authority without commissioning evidence and can command beyond the currently approved production range.
- Question: approve A and freeze the active updown range at `[0.0, 0.92] m`, or choose B and explicitly supersede the hardware mapping with `0.99 m`?
- Decision: supersede both prior values and freeze the production updown range as exact `[0.0, 0.8] m`. Apply `upper="0.8"` to the active migrated URDF and use the same inclusive range in rt-control command admission and motion/MoveIt configuration. Preserve the original vendor/raw `0.99` artifact as provenance and record that the hardware mapping's prior `0.92` value has been explicitly replaced by this user-approved production limit. Keep the exact position conversion at `1,000,000 counts/m`.
- Benefit of the selected range: leaves an additional 120 mm margin below the former hardware-authority limit and makes every active consumer use one conservative value.
- Drawback of the selected range: reduces usable lift travel by approximately 13% relative to `0.92 m` and is an explicit hardware-number authority change that must be propagated beyond this repository.
- Unblocked scope: T-003/T-007 and updown command validation use `0.0..0.8 m`; no implementation may retain `0.92` or active `0.99` as the production command limit.

## BQ-049 — Updown non-finite/out-of-range command handling is not frozen [RESOLVED 2026-07-21]

- Evidence: `UpdownCommand` carries two `float64` fields and one raw `uint32`. BQ-048 now freezes the final target range at `[0.0, 0.8] m`, while BQ-015 requires the expected start to be within `0.05 m` of actual position. The contract does not define NaN/infinity, target clamping, an expected start outside the production range, or a `profile_velocity_raw` numeric range. No authoritative minimum/maximum/zero semantics for LD2 `0x6081` exists.
- Proposed decision A (recommended): reject rather than clamp whenever `expected_start_position`, final `position`, or actual `0x6064` is non-finite; reject the final target unless it is inclusively within `[0.0, 0.8] m`; then apply the existing `abs(expected_start_position - actual) <= 0.05 m` check. Do not independently range-limit `expected_start_position`: this allows an axis whose measured position is slightly outside the production range to be commanded back to a valid final target, while the proximity check still proves the command was planned from that actual state. Pass the full `uint32 profile_velocity_raw` value—including zero—unchanged to `0x6081` until vendor/readback evidence defines valid bounds; do not invent a clamp or default. Any rejection leaves the currently active PP target/velocity unchanged.
- Benefit of A: prevents NaN conversion and out-of-range motion without silently altering the requested target, preserves a path back into the approved range, and avoids guessing vendor velocity semantics.
- Drawback of A: an expected start outside `[0.0,0.8]` can still pass if it matches actual position, and `profile_velocity_raw=0` is forwarded even though its physical meaning remains unverified. These cases must be covered by commissioning evidence/operator procedure.
- Alternative B: also require `expected_start_position` inside `[0.0,0.8]` and reject `profile_velocity_raw=0`.
- Benefit of B: every command field looks conventionally valid and zero-speed moves are blocked.
- Drawback of B: can prevent a controlled command back into range after overshoot/manual displacement and invents a vendor-specific rule for `0x6081=0` without authority.
- Question: approve A—strict finite/final-target validation, no clamping, expected start allowed outside range only when close to actual, and raw velocity passed unchanged—or choose B?
- Decision: approve A. Reject non-finite `expected_start_position`, final `position`, or actual position; reject a final target outside inclusive `[0.0,0.8] m`; do not clamp. Apply the existing `0.05 m` expected-start proximity check without independently range-limiting `expected_start_position`, so an axis slightly outside the production range can be commanded back to a valid final target. Forward the complete `uint32 profile_velocity_raw`, including zero, unchanged until authoritative vendor evidence defines bounds. A rejected command leaves the active PP target and profile velocity unchanged.
- Unblocked scope: numeric validation and rejection semantics are frozen for the updown command admission layer in T-007.

## BQ-050 — CANopen group Stop does not invalidate command interfaces in stock `Cia402System` [RESOLVED 2026-07-21]

- Evidence: BQ-006/BQ-026 require a track heartbeat timeout or EMCY to issue all-node NMT Stop and latch recovery until a full rt-control restart. The pinned master implements `stop_all_nodes: true` for mandatory-node errors, and the approved narrow callback path covers the required track EMCY case. The pinned `Cia402System` receives each node's existing NMT-state callback, but its stock `write()` continues processing NMT start/reset, init/recover/mode requests and calling `set_target()` every update regardless of the reported NMT state. Therefore NMT Stop prevents present PDO execution, but the software command-interface values and one-shot requests remain live inside the same process.
- Proposed decision A (recommended): add a process-lifetime CAN group-stop latch inside the already approved narrow `Cia402System` overlay. Arm it only after successful hardware activation. If either mandatory track reports unexpected `NmtState::STOP`, latch the whole three-node group; the master remains the owner that performs the actual NMT Stop. From the next `write()`, suppress all external NMT start/reset, init/recover, mode-switch, generic TPDO and motion-target dispatch for all three nodes; overwrite the in-process updown target cache with the latest cached actual position and both track target caches with zero on every cycle. Log the latch once. Normal lifecycle teardown may still attempt the BQ-047 upstream `handleShutdown()` cleanup, but neither deactivation nor reactivation clears the latch; only destruction/recreation in a full rt-control process restart does. Do not add a public command-result/status interface.
- Benefit of A: a controller or stale client cannot stage a target, recovery command, or NMT Start behind the stopped bus; no pre-fault or post-fault command can reappear through a same-process lifecycle cycle. It enforces the already approved restart-only recovery boundary without another watchdog or ROS process.
- Drawback of A: controllers/topics can continue accepting data while the hardware overlay silently suppresses it, because BQ-016 explicitly chose no command status/result. The cached updown hold position may itself be old, although it is not transmitted while NMT Stop is latched. A benign/manual NMT Stop of either track after activation also makes the process restart-only.
- Alternative B: do not add a latch; rely on NMT Stop to block PDO execution and on the required full process restart to destroy all cached commands before any recovery.
- Benefit of B: smaller ros2_canopen overlay and behavior closest to upstream.
- Drawback of B: stock `write()` keeps accepting and attempting targets and even NMT Start/recover operations after the group stop; correctness then depends entirely on master/drive rejection and on every recovery path truly replacing the process before operational state can return.
- Question: approve A—latch unexpected track NMT Stop for the rest of the process and suppress/overwrite all three nodes' commands—or choose B and rely only on NMT Stop plus process restart?
- Decision: choose B. Do not add a process-lifetime command-suppression latch to `Cia402System`. The approved master/callback behavior issues all-node NMT Stop for the required mandatory-track fault events, and recovery remains a complete rt-control process restart that destroys the hardware object and all cached command-interface values before operation can resume. Do not add a same-process NMT recovery path.
- Benefit of the selected policy: keeps the ros2_canopen overlay smaller and leaves ordinary target dispatch behavior aligned with upstream while NMT state provides the execution barrier.
- Drawback explicitly accepted: during the stopped interval, stock `Cia402System::write()` may continue attempting target, NMT Start, recover, or mode operations from live command interfaces. Safety therefore depends on the master/drives honoring NMT Stop and on operational recovery never occurring without full process replacement.
- Unblocked scope: no CAN command-cache latch or suppression layer is added for T-007/T-014; full process restart remains the only recovery route after the approved group stop.

## BQ-051 — Partial CANopen hardware activation failure has no rollback policy [RESOLVED 2026-07-21]

- Evidence: BQ-046 makes `Cia402System::on_activate()` initialize and enable Nodes 1/2/3 sequentially, then select modes and seed safe targets. Pinned upstream `RobotSystem::on_activate()` returns immediately on the first `init_motor()` failure and does not shut down motors already enabled earlier in the loop. A ros2_control lifecycle activation failure does not itself guarantee that those earlier drives receive CiA402 disable commands. The already approved overlay exposes upstream `Motor402::handleShutdown()` as `shutdown_motor()` for BQ-047.
- Proposed decision A (recommended): stop activation at the first failed init/mode/seed stage and run a best-effort rollback inside the same non-RT `on_activate()` call. First request the safe targets that are valid at that point (cached-current hold for Node 1 when finite; zero for both tracks), allow the already frozen 20 ms processing hold, then call `shutdown_motor()` for all three nodes, in reverse order for nodes whose activation stages completed and then for any remaining node that can respond. Attempt every node even after a rollback failure. Return activation ERROR after rollback. If any safe-target or shutdown operation fails, additionally request all-node NMT Stop and report both the original activation failure and the first rollback failure; no automatic retry or partial operation is allowed.
- Benefit of A: a later-node failure does not intentionally leave an earlier drive Operation Enabled, and the rollback reuses the selected upstream standard shutdown path before falling back to the approved group NMT Stop. It keeps all blocking work in the non-RT lifecycle callback.
- Drawback of A: a failed activation takes longer because rollback includes the 20 ms hold and bounded upstream state transitions. A drive that failed before current-position feedback became finite cannot receive a proven updown hold target, so the fallback still depends on NMT Stop/drive configuration if its shutdown also fails.
- Alternative B: on the first activation failure, skip standard rollback and immediately request all-node NMT Stop, then return activation ERROR.
- Benefit of B: fastest and simplest failure exit, with no extra target or state-transition attempts on a partially initialized bus.
- Drawback of B: NMT Stop is a communication-state action rather than confirmed CiA402 Switch On Disabled; previously enabled drives may remain electrically enabled, and their mechanical stop behavior depends on the drive-side heartbeat/NMT configuration.
- Question: approve A—best-effort standard shutdown of the whole three-node group with NMT Stop fallback—or choose B and use immediate NMT Stop only?
- Decision: approve A. Stop at the first failed CAN init/mode/seed stage, apply every valid safe target, hold for 20 ms, and then make a best-effort `shutdown_motor()` attempt for all three nodes. Process all rollback targets even after one fails. If any safe-target or shutdown step fails, additionally request all-node NMT Stop. Return lifecycle activation ERROR with the original activation failure retained as primary and the first rollback failure recorded as secondary; do not retry activation or permit partial operation.
- Unblocked scope: partial CAN hardware activation failure and rollback are frozen for the ros2_canopen overlay in T-007/T-014.

## BQ-052 — Normal EtherCAT disable-stage timeout has no bounded terminal action [RESOLVED 2026-07-21]

- Evidence: normal `/rt/disable` is frozen as the confirmed standard sequence `0x0007 -> 0x0006 -> 0x0000`, with a 4.0 s timeout per stage. The earlier AMB-010 record explicitly says not to skip a stage after timeout, but neither the specification nor later decisions say what the RT FSM commands after that timeout. Returning immediately while leaving the timed-out stage command active can leave an axis electrically Switched On; waiting forever violates the configured timeout and the synchronous final-result service contract. JTC is already strictly deactivated by BQ-044, so no trajectory remains active at this point.
- Proposed decision A (recommended safety fallback): preserve the first timed-out joint/raw statusword/stage as the `/rt/disable` failure, but switch the whole group from the failed normal path into the already approved BQ-042 emergency terminal-disable path. Operation Enabled healthy axes receive `0x0002` until Quick Stop Active using the same 4.0 s bound, then `0x0000` until `0x0040`; lower-state axes continue state-aware downward handling, and Fault axes receive `0x0000`. Attempt every axis independently. The service returns `ok=false` after this bounded cleanup attempt even if all axes eventually reach `0x0040`, because the requested normal sequence failed. If any axis still cannot confirm the terminal state, latch its first cleanup failure, reject later enable/reset in this process, and require hardware intervention/full rt-control restart.
- Benefit of A: a failed normal transition does not deliberately leave healthy peers powered, and it reuses an already approved emergency path rather than inventing another controlword sequence. The response truthfully retains the original normal-disable failure.
- Drawback of A: this explicitly overrides AMB-010's “do not skip after timeout” rule for the abnormal fallback. Quick Stop/Disable Voltage may produce a more abrupt, drive-dependent mechanical response, and the call can take additional emergency-stage timeout periods before returning.
- Alternative B: strictly obey the no-skip rule. On timeout, return `ok=false`, continue writing the timed-out stage's controlword to that axis, allow other axes to finish their normal paths, reject all later enable/reset operations, and require operator/hardware intervention; do not invoke Quick Stop or advance that failed axis to `0x0000`.
- Benefit of B: never bypasses the standard confirmed transition order and avoids an automatic abrupt fallback.
- Drawback of B: the failed axis may remain Switched On or Ready indefinitely, so software cannot guarantee terminal disable and process restart/communication loss may still leave its electrical/mechanical behavior dependent on the drive.
- Question: approve A—on normal disable timeout, run the existing emergency group terminal-disable fallback while returning failure—or choose B and freeze the failed axis at its timed-out standard stage for manual intervention?
- Decision: approve A after clarification. An enable-stage failure first uses BQ-030's standard state-aware rollback. If a normal disable/rollback stage itself times out, preserve that original failure but move the whole group through BQ-042's bounded emergency Quick-Stop-to-Disable-Voltage path and attempt to confirm every axis at `0x0040`. The service remains `ok=false` even when emergency cleanup reaches the terminal state. A subsequent launch may treat the group as cleanly awaiting enable only when all thirteen axes actually report `0x0040`; Ctrl+C/process restart must not claim or synthesize that state. A real Fault remains Fault and requires explicit `/rt/reset_fault`; lost communication or an unconfirmed terminal state remains an operator/hardware blocker.
- Benefit of the selected policy: maximizes the chance that an interrupted/failed enable or disable leaves every reachable healthy axis electrically disabled before process exit, instead of preserving a timed-out Switched On stage.
- Drawback explicitly accepted: the abnormal fallback can be mechanically more abrupt and may extend shutdown by additional stage timeouts; it still cannot guarantee a clean state when communication is unavailable or a drive is genuinely faulted.
- Unblocked scope: the bounded fallback and the next-launch terminal-state precondition are frozen for T-010 and shutdown handling.

## BQ-053 — EtherCAT launch startup behavior for residual CiA402 states is not frozen [RESOLVED 2026-07-21]

- Evidence: BQ-030/BQ-052 attempt to leave all axes at Switch On Disabled before a failed service or orderly Ctrl+C completes, but SIGKILL, communication loss, or an exhausted stage timeout can interrupt that cleanup. On the next launch, `auto_state_transitions: false` prevents ICube from normalizing states automatically, JTC starts INACTIVE, and the specification does not say whether `enable_manager` may write controlwords before the first service. A drive can therefore be observed in Ready to Switch On, Switched On, Operation Enabled, Quick Stop Active, Fault Reaction Active, or Fault rather than the intended clean `0x0040` state.
- Proposed decision A (recommended, matches the clarified clean-start goal): when `enable_manager` activates, enter a startup-sanitize FSM before accepting `/rt/enable`. Keep JTC INACTIVE. For non-faulted axes, use the already approved state-aware downward rules: Operation Enabled follows normal `0x0007 -> 0x0006 -> 0x0000`; Switched On/Ready join at their matching step; Quick Stop Active receives `0x0000`; already-disabled axes remain `0x0000`. Use the existing 4.0 s stage timeout and require all axes to confirm `statusword & 0x004F == 0x0040`. Fault/Fault-Reaction axes receive `0x0000` but are not reset automatically; expose the existing `/rt/reset_fault`, and only after explicit reset plus terminal confirmation may the manager become clean/IDLE. Reject `/rt/enable` with `stage="startup_not_clean"` until this completes. A sanitize timeout remains blocked and must not be reported as a clean launch.
- Benefit of A: a process restart cannot accidentally inherit powered residual states as if they were a valid enable session; every accepted enable begins from a confirmed all-axis disabled baseline. It also makes orderly and interrupted shutdown converge on the same condition without automatic Fault Reset.
- Drawback of A: launch activation itself writes controlwords and may remove holding torque from a residual powered axis before an operator calls a service. Startup can remain unavailable for multiple four-second stages, and a real Fault still needs explicit reset/operator action rather than becoming automatically clean.
- Alternative B: startup is read-only. Require all axes already to report `0x0040`; otherwise reject `/rt/enable` and require the operator to call `/rt/disable` or `/rt/reset_fault` explicitly to normalize the group.
- Benefit of B: launch never changes actuator power state without an explicit service request, preserving operator control over a mechanism that might need temporary holding torque.
- Drawback of B: restarting launch does not itself produce the requested clean awaiting-enable state; residual powered states remain until an operator notices and issues the correct service, and `/rt/disable` must be callable from an unclean startup state.
- Question: approve A—automatically sanitize every non-faulted residual state to confirmed `0x0040` at startup while requiring explicit reset for real Faults—or choose B and make startup read-only until an operator calls a service?
- Decision: approve A. `enable_manager` starts in a startup-sanitize state with JTC INACTIVE, automatically applies the approved state-aware downward sequence to every reachable non-faulted residual state, and requires all thirteen axes to confirm `0x0040` before entering clean/IDLE or accepting `/rt/enable`. Fault/Fault-Reaction axes stay at `0x0000` and require the explicit group reset service; no startup auto-reset is allowed. Sanitize timeout or unconfirmed status keeps startup blocked and is never reported as clean.
- Unblocked scope: clean startup normalization and enable admission precondition are frozen for T-010/bringup.

## BQ-054 — A launch `OnShutdown` service call cannot reliably finish EtherCAT disable [RESOLVED 2026-07-21]

- Evidence: the specification says launch registers a shutdown handler that calls `/rt/disable`. In pinned Humble `ros2_control_node`, the controller manager's context pre-shutdown callback removes/cancels the executor, `rclcpp::ok()` becomes false so the 250 Hz read/update/write thread exits, and only then active controllers are deactivated and hardware components shut down. `enable_manager` needs continued RT `update()` cycles to execute and confirm its disable FSM. A launch `OnShutdown` action is dispatched as the same shutdown event also signals child processes, so it cannot guarantee that the service request completes before controller_manager stops. Controller `on_deactivate()` cannot replace the FSM because it has no subsequent update cycles and blocking there would violate the control-thread constraint.
- Proposed decision A (recommended): make the supported rt-control entrypoint a signal-gating wrapper, not raw `ros2 launch`. Keep it in the existing bringup/enable-manager packages: the wrapper starts the launch child in a separately signalled process group and traps orderly SIGINT/SIGTERM first; a small one-shot C++ client installed by the existing `enable_manager` package calls `/rt/disable` and waits for its bounded final result while controller_manager and the RT loop remain alive. Only after the service returns does the wrapper forward SIGINT to the launch process group. If the service is unavailable or returns failure, record that result, still shut the process group down, and rely on BQ-053 startup sanitization rather than claiming a clean exit. No new long-running ROS node or package is added. SIGKILL, host power loss, and a second external forced kill remain outside this orderly guarantee. The wrapper/client wait and container stop-grace bound are deferred to the next timing decision rather than guessed.
- Benefit of A: an ordinary Ctrl+C or container SIGTERM gives the already approved disable FSM actual RT cycles to finish and confirm states before controller_manager teardown. It avoids a controller_manager/ICube patch and adds no resident coordination node.
- Drawback of A: operators and Compose must use the supported wrapper; running raw `ros2 launch` or killing the controller-manager PID bypasses orderly cleanup. The wrapper adds PID/signal-forwarding logic and shutdown latency, and hard power/SIGKILL still depends on hardware safety plus next-start sanitization.
- Alternative B: remove the unreliable launch service claim and rely only on controller-manager's stock shutdown plus BQ-053 normalization at the next startup.
- Benefit of B: no wrapper/helper and Ctrl+C behavior remains stock ROS 2.
- Drawback of B: Ctrl+C cannot promise standard disable or terminal confirmation before communication stops; powered residual states are intentionally left for drive reaction and next launch to handle.
- Question: approve A—use a signal-gating entrypoint plus one-shot C++ `/rt/disable` client before forwarding shutdown—or choose B and make clean state a next-launch-only responsibility?
- Decision: approve A. The supported rt-control process entrypoint is a signal-gating wrapper that starts launch in a separately signalled child process group, traps orderly SIGINT/SIGTERM, invokes a one-shot C++ `/rt/disable` client installed within the existing `enable_manager` package, and forwards shutdown to launch only after the service returns its final result. Failure/unavailability is recorded but cannot be presented as a clean exit; shutdown still proceeds and BQ-053 handles residual states on the next start. Do not add a resident coordinator node or another package. Raw `ros2 launch`, SIGKILL, and host power loss are explicitly outside the orderly-cleanup guarantee.
- Unblocked scope: orderly signal ownership and shutdown-call ordering are frozen for T-007/T-008/T-010; exact wrapper/grace timing remains BQ-055.

## BQ-055 — Orderly shutdown client and container grace periods are not frozen [RESOLVED 2026-07-21]

- Evidence: BQ-054 requires the wrapper to wait for a final `/rt/disable` result before forwarding SIGINT. The JTC strict switch request accepts an explicit timeout, where zero means infinite. With existing 4.0 s FSM stage bounds, a conservative EtherCAT worst path is: finish the active/preempted enable batch (up to 4 s), strict JTC switch (proposed 4 s), normal three-stage disable (up to 12 s), and BQ-052 emergency cleanup (up to 8 s), totaling 28 s before scheduling/transport margin. After SIGINT is forwarded, BQ-047 sequentially invokes upstream `Motor402::handleShutdown()` for three CAN nodes; its unmodified mode/state implementation contains as many as four separate 5 s waits per node in a worst failure path. Docker Compose's ordinary 10 s stop grace can therefore SIGKILL the process before either cleanup path completes. Adding `stop_grace_period` also exceeds BQ-007's previously authorized build-wiring-only Compose additions and needs explicit approval.
- Proposed decision A (recommended): set the strict JTC lifecycle switch timeout to the already approved `4.0 s`; set the one-shot shutdown client's final `/rt/disable` wait to `30 s`; authorize Compose `stop_grace_period: 100s`. The wrapper records a 30 s client timeout as unclean, forwards SIGINT, and the remaining grace allows up to 60 s of sequential stock CAN shutdown plus approximately 10 s process/scheduling margin. A second user-forced SIGKILL still overrides this bound. Keep all component-internal timeouts unchanged; do not parallelize upstream CAN shutdown calls merely to shorten the grace.
- Benefit of A: every wait is finite, normal Ctrl+C/container stop has enough derived time for both the EtherCAT FSM and unmodified upstream CAN shutdown, and a hung process is still forcibly terminated after a known bound. It preserves the chosen upstream CAN behavior.
- Drawback of A: a pathological `docker compose stop` can take up to 100 seconds. The 30/100 s values are deployment policy derived from software worst paths rather than hardware measurements and should be tightened only after commissioning evidence.
- Alternative B: keep the 4 s JTC switch and 30 s helper wait but retain Compose's short/default stop grace, accepting forced termination of CAN shutdown after the grace expires.
- Benefit of B: containers stop faster and no new Compose runtime field is added.
- Drawback of B: directly defeats the clean-exit objective whenever EtherCAT cleanup runs long or sequential CAN shutdown needs more than the remaining grace; the next launch must repair residual states.
- Question: approve A with exact `4 s / 30 s / 100 s` bounds, or choose B and retain the short Compose shutdown grace?
- Decision: approve A. Use a 4.0 s strict JTC lifecycle-switch timeout, a 30 s one-shot `/rt/disable` client deadline, and explicitly authorize `stop_grace_period: 100s` in Compose. On the 30 s client deadline, mark the exit unclean and forward SIGINT so component shutdown still has the remaining grace; do not parallelize or shorten upstream CAN shutdown behavior. These are provisional deployment bounds that may only be reduced using commissioning timing evidence.
- Unblocked scope: orderly shutdown timing and the additional Compose runtime field are frozen for T-007/T-008/T-010.

## BQ-056 — Service behavior while automatic startup sanitization is active is not frozen [RESOLVED 2026-07-21]

- Evidence: BQ-053 starts an internal state-aware disable/sanitize FSM before the system becomes clean/IDLE. BQ-036 rejects a duplicate service request while a same-direction service operation is active, but startup sanitization has no original service caller. BQ-054's orderly shutdown wrapper can receive SIGINT during this startup interval and must call `/rt/disable`; rejecting it as busy would cause the wrapper to tear down controller_manager while the safety-direction FSM is already trying to reach `0x0040`. Conversely, allowing `/rt/enable` or Fault Reset to modify the startup FSM before non-faulted axes are clean creates mixed operation ownership.
- Proposed decision A (recommended): while `STARTUP_SANITIZING`, reject `/rt/enable` immediately with `stage="startup_not_clean"` and reject `/rt/reset_fault` with `stage="operation_in_progress"`. Allow the first `/rt/disable` call to attach as the sole service owner of the already-running startup sanitize operation without resetting any stage or timeout; it waits for and returns that operation's real final structured result. Any second disable while that attached call is pending follows BQ-036 and returns `operation_in_progress`. If startup reaches a blocked-Fault condition after every non-faulted axis is terminal, return the attached disable as `ok=false`, first Fault joint/raw statusword, `stage="fault_requires_reset"`; only then accept the explicit group reset service. If all axes reach `0x0040`, return disable success and let the wrapper continue orderly shutdown.
- Benefit of A: Ctrl+C during startup waits on the safety work already in progress instead of restarting it or cutting it off, while enable/reset cannot race the normalization. Stage deadlines are not extended by attaching the caller.
- Drawback of A: this is a narrow exception to the general duplicate-call rejection policy and needs one “attached shutdown caller” result slot. A disable call during startup may block for the remaining sanitize duration even though it did not initiate the operation.
- Alternative B: reject every service during `STARTUP_SANITIZING`; callers retry only after startup reaches clean/blocked terminal state.
- Benefit of B: one uniform busy policy and simpler request ownership.
- Drawback of B: an orderly Ctrl+C during startup cannot wait for the in-progress cleanup and will generally be marked unclean before launch teardown interrupts it.
- Question: approve A—let one `/rt/disable` attach to startup sanitization without restarting it—or choose B and reject all services until startup sanitization finishes?
- Decision: choose B. While `STARTUP_SANITIZING`, `/rt/enable`, `/rt/disable`, and `/rt/reset_fault` all return immediately with `ok=false`, `failed_batch=-1`, empty `failed_joint`, `status_word=0`, and `stage="operation_in_progress"`; none attaches, restarts, cancels, or extends the internal sanitize FSM. After the startup operation reaches its clean or blocked terminal state, a caller may issue a fresh applicable service. If the signal-gating wrapper receives Ctrl+C during this interval, its disable call is recorded as an unclean failure and shutdown proceeds; BQ-053 must normalize residual states on the next launch.
- Benefit of the selected policy: startup has one owner and all concurrent services follow one uniform rejection rule without extra waiter state.
- Drawback explicitly accepted: orderly shutdown during startup does not wait for the sanitizer and can interrupt it before `0x0040` confirmation, despite the longer normal-exit mechanism.
- Unblocked scope: startup-service concurrency is frozen for T-010 and the shutdown wrapper.

## BQ-057 — Service behavior during automatic EtherCAT runtime-Fault shutdown is not frozen [RESOLVED 2026-07-21]

- Evidence: BQ-041/042 make `enable_manager` automatically latch a runtime drive Fault, deactivate JTC, Quick Stop healthy peers, and drive the group toward terminal disable without a service caller. A `/rt/disable` can arrive from an operator or the BQ-054 shutdown wrapper while this internal fault-stop FSM is active. BQ-036 only covers duplicate service-owned operations, and BQ-056 explicitly chose rejection for startup sanitization but did not authorize extending that choice to runtime Fault handling. Enable and reset must not race the still-active emergency sequence; the remaining choice is whether one disable caller may wait on the same final outcome.
- Proposed decision A: allow the first `/rt/disable` received during the automatic fault-stop FSM to attach without changing any command, stage, deadline, or first-fault record. It returns `ok=false` with the original first-fault joint/raw statusword and `stage="fault_group_stopped"` after emergency terminal cleanup completes; it cannot return success because the group stop arose from a Fault. Further disable calls follow BQ-036 and return `operation_in_progress`. Reject enable/reset until cleanup is terminal.
- Benefit of A: the shutdown wrapper can wait for emergency hardware cleanup before killing launch, and the caller receives the actual originating Fault rather than a generic busy result.
- Drawback of A: adds another special attached-caller path and can hold `/rt/disable` open through the remaining emergency timeouts even though its final result is necessarily failure.
- Alternative B (consistent with BQ-056): reject `/rt/enable`, `/rt/disable`, and `/rt/reset_fault` with `operation_in_progress` while automatic fault-stop is active. After it reaches terminal disabled/failed state, require a fresh reset/disable request as applicable.
- Benefit of B: one uniform policy for every internally owned FSM and no extra waiter/result slot.
- Drawback of B: Ctrl+C during fault cleanup immediately becomes an unclean exit and may interrupt the Quick-Stop-to-terminal sequence; the next launch must sanitize whatever state remains.
- Question: choose A and allow one disable caller to wait on automatic Fault cleanup, or choose B and reject all services consistently with startup sanitization?
- Decision: choose B. While the internally owned runtime-Fault shutdown is active, `/rt/enable`, `/rt/disable`, and `/rt/reset_fault` all return immediately with the standard `operation_in_progress` response and do not attach, cancel, restart, or extend the emergency FSM. Applicable recovery services require a fresh call only after its terminal state. A shutdown-wrapper disable during this interval is therefore recorded as unclean and launch teardown proceeds.
- Benefit of the selected policy: internally owned startup and runtime-Fault operations now share one simple service-rejection rule.
- Drawback explicitly accepted: Ctrl+C can interrupt an in-progress emergency terminal-disable sequence; residual state is deferred to next-launch sanitization and drive-local safety behavior.
- Unblocked scope: runtime-Fault service concurrency is frozen for T-010/shutdown handling.

## BQ-058 — `/rt/disable` arriving during final JTC activation is not frozen [RESOLVED 2026-07-21]

- Evidence: BQ-035 says a disable received during enable finishes the currently active hardware batch, starts no next batch, and then enters `DISABLING`. BQ-044 adds a final non-RT strict activation of `dual_arm_jtc` after all five hardware batches, with the BQ-055 4.0 s switch timeout; this is part of the synchronous enable operation but is not an enable batch. A controller-manager switch request cannot be safely canceled midway, and rejecting the disable until it returns weakens the already approved higher-priority safety-direction request.
- Proposed decision A (recommended): accept `/rt/disable` during pending JTC activation as BQ-035 preemption. Do not issue another controller-manager request concurrently. Wait only for the already bounded activation request to return. If JTC is then ACTIVE, immediately request strict deactivation; if it is confirmed INACTIVE, skip that request; if its state is ambiguous, continue hardware terminal disable but latch the existing restart-only controller-state failure. In all cases run the normal state-aware hardware disable after resolving/expiring the 4 s switch. The enable caller returns `stage="preempted_by_disable"`; the disable caller returns the actual disable result. No trajectory goal is accepted as a successful enable outcome in between—the enable service never returns success once preemption is recorded.
- Benefit of A: preserves disable priority across the entire enable transaction, avoids two concurrent switch requests, and guarantees that a late preemption cannot be reported as successful enable.
- Drawback of A: the disable may wait up to the remaining 4 s for an activation that it no longer wants, then incur another deactivation switch and the full hardware-down sequence. For a brief interval JTC can become ACTIVE before being immediately deactivated.
- Alternative B: reject disable with `operation_in_progress` while JTC activation is pending; the caller retries only after enable returns.
- Benefit of B: simpler controller-switch ownership and no immediate activate-then-deactivate sequence.
- Drawback of B: a safety-direction disable cannot take ownership for up to 4 s, and the original enable can return success just before the retry even though the disable request already arrived.
- Question: approve A—record disable preemption, wait for the bounded JTC switch, then deactivate/disable—or choose B and reject disable until JTC activation finishes?
- Decision: choose B. While the final strict JTC activation request is pending, `/rt/disable` returns immediately with the standard `operation_in_progress` response and does not preempt or queue. The original enable operation completes with its actual activation result; only a fresh disable call made afterward may begin the normal downward path. A shutdown-wrapper call in this interval is treated as unclean and does not wait/retry automatically.
- Benefit of the selected policy: only one controller-manager switch operation owns the final enable phase and no activate-then-immediate-deactivate branch is added.
- Drawback explicitly accepted: disable priority is suspended for up to the 4.0 s JTC switch bound, and a Ctrl+C in this window can tear down after a busy response rather than performing the orderly service path.
- Unblocked scope: final-enable switch concurrency is frozen for T-010/shutdown handling.

## BQ-059 — Unexpected loss of EtherCAT Operation Enabled without Fault is not covered [RESOLVED 2026-07-21]

- Evidence: BQ-041 triggers automatic group shutdown only when an axis reports Fault Reaction Active or Fault during `ENABLING`/`ENABLED`. CiA402 can also leave Operation Enabled without setting Fault—for example into Quick Stop Active, Switched On, Ready to Switch On, or Switch On Disabled because of a local safety input, watchdog reaction, or external state transition. While `enable_manager` still believes the group is `ENABLED`, JTC remains ACTIVE and the other twelve axes can continue sampling the trajectory. No standalone watchdog remains, and the frozen documents do not define whether a non-Fault state loss is a group event.
- Proposed decision A (recommended): while the manager is stably `ENABLED` and no commanded disable/fault transition is active, require every axis each RT cycle to match Operation Enabled (`statusword & 0x006F == 0x0027`). The first recognized non-Fault mismatch latches the offending joint/raw statusword as `stage="unexpected_drive_state"`, strictly deactivates JTC, and runs the BQ-042 group emergency terminal-disable path. Do not auto-reset. If no axis is in Fault after cleanup, recovery requires a fresh `/rt/enable` (which starts from BQ-053/BQ-043 clean state); if a Fault appears, explicit reset is required first. An unknown/unrecognized statusword is treated the same as a mismatch rather than ignored. Do not add debounce or a new timeout without evidence.
- Benefit of A: a single axis silently losing torque/control authority cannot leave its peers executing a coordinated 13-axis trajectory; all unexpected CiA402 exits converge on the existing group stop and stale-goal disposal path.
- Drawback of A: one transient or torn/cached statusword sample can abort the whole trajectory and force terminal disable/fresh enable. This broadens automatic shutdown beyond the original Fault-only wording and can create nuisance trips during commissioning.
- Alternative B: keep automatic group shutdown limited to Fault/Fault Reaction Active. Report other non-Operation-Enabled states through diagnostics but leave JTC and healthy axes running until an existing hardware/controller error or operator action stops them.
- Benefit of B: adheres narrowly to the currently approved Fault trigger and avoids full-group trips on transient non-Fault states.
- Drawback of B: a coordinated trajectory may continue on twelve axes after one drive has locally Quick-Stopped or disabled, causing large path/pose divergence without an immediate group reaction.
- Question: approve A—treat any unexpected loss of Operation Enabled as a group-stop event—or choose B and stop automatically only on Fault states?
- Decision: approve A. During stable `ENABLED`, every RT update compares the thirteen already-mapped statuswords against Operation Enabled. The first nonmatching or unrecognized state latches `unexpected_drive_state`, deactivates JTC, and enters the existing group emergency terminal-disable path. No debounce is added. A non-Fault terminal result requires a fresh enable; any actual Fault requires explicit reset first.
- Communication/load clarification: this adds no PDO, SDO, state interface, bus frame, or cycle-rate change. The 13 statuswords are already transferred every 4 ms and already claimed by `enable_manager`; the steady-state cost is only thirteen in-memory mask comparisons per 250 Hz update (approximately 3,250 simple comparisons per second). Event handling reuses existing command PDOs and non-RT controller switching.
- Unblocked scope: runtime non-Fault loss-of-enable detection is frozen for T-010/T-012.

## BQ-060 — `turn` is continuous in description but limited to ±179° by hardware authority [RESOLVED 2026-07-21]

- Evidence: the supplied description source declares `<joint name="turn" type="continuous">` with no position limit. The unique hardware-number authority and frozen `REQ-ECAT-007` declare `turn.limit_deg: 179.0`, `lower_position_rad: -3.12413936107`, and `upper_position_rad: 3.12413936107`. Pinned JTC 2.53.2 supports `angle_wraparound`/shortest-angular-distance behavior, but the authoritative controller config does not enable it. If turn is treated as continuous, a goal from approximately +179° to -179° can be transformed into a short path crossing the ±π boundary, outside the frozen ±179° range. If direct numeric difference is used instead, the same physically close wrapped representation fails the 1° admission check.
- Proposed decision A (recommended, follows hardware authority): apply a documented active-description overlay changing `turn` from `continuous` to a bounded `revolute` joint with the exact inclusive position range `[-3.12413936107, 3.12413936107] rad`; propagate the same limits to motion/MoveIt and rt-control. Keep JTC `angle_wraparound=false` for turn and apply the existing 1° admission test as direct absolute numeric difference, exactly like the other bounded joints. Reject any trajectory point outside the turn range rather than wrapping or clamping it. Preserve the pinned raw description as provenance and list this as an intentional hardware-authority delta.
- Benefit of A: kinematics, planning, admission, and the production hardware limit all agree; no trajectory can cross an unapproved wrap boundary merely because the mesh description called the joint continuous.
- Drawback of A: explicitly changes the supplied description's joint type and eliminates continuous shortest-path planning. Commands near opposite ±179° limits require a long in-range move or a different operational plan rather than crossing ±π.
- Alternative B: supersede the hardware mapping limit, retain `turn` as continuous, enable JTC angle wraparound, and use shortest angular distance for its 1° admission check and trajectory interpolation.
- Benefit of B: preserves the description semantics and gives natural short-path motion across ±π.
- Drawback of B: removes a frozen production position boundary without mechanical/drive evidence and can command through a region currently outside the authorized ±179° travel.
- Question: approve A—make active `turn` a bounded revolute ±179° axis with no wraparound—or choose B and explicitly authorize continuous wraparound motion?
- Decision: approve A. The active migrated description defines `turn` as a bounded revolute joint with exact inclusive limits `[-3.12413936107, 3.12413936107] rad`; motion/MoveIt and rt-control use the same range. JTC angle wraparound remains false, the 1° initial-point admission check uses direct absolute numeric difference, and out-of-range trajectory positions are rejected without wrapping or clamping. Preserve the pinned raw description and document this intentional hardware-authority overlay.
- Unblocked scope: turn joint type, planning boundary, and JTC admission/error metric are frozen for T-003/T-007/JTC overlay.

## BQ-061 — Bounded `turn` URDF requires an effort value that has no authority [RESOLVED 2026-07-21]

- Evidence: BQ-060 changes active `turn` from continuous to revolute. URDF requires a revolute `<limit>` to contain lower, upper, effort, and velocity. Hardware authority supplies exact turn position limits and `max_velocity_rad_s: 87.2664626` (explicitly a PLC upper bound, not first-test speed), but supplies no turn maximum effort/torque. The description's arm joints contain model-specific effort values, while its pitch/updown placeholders use `effort="0"`; copying an arm effort to turn would be an unsupported hardware number.
- Proposed decision A: write the active turn limit as lower/upper from BQ-060, `velocity="87.2664626"` from the hardware authority, and `effort="0"` explicitly as an “effort not modeled/not authorized” sentinel consistent with existing description placeholders. Do not expose or enforce an effort command interface, and document that `0` is not a measured zero-torque capability. Motion's initial commissioning remains limited to the already frozen ≤20% velocity scaling; the high PLC velocity limit does not authorize full-speed testing.
- Benefit of A: produces a valid bounded URDF without inventing a torque value and propagates the one available authoritative velocity bound.
- Drawback of A: generic consumers can interpret URDF `effort=0` literally as zero allowable effort, so this description is unsuitable for torque-aware simulation/planning until a real value is supplied. The very large PLC velocity upper bound is also not a safe commissioning speed by itself.
- Alternative B (strict no-placeholder): block the bounded-turn description until the drive/mechanical owner supplies an authoritative maximum effort; do not create the revolute `<limit>` yet.
- Benefit of B: every URDF limit field has real physical meaning and no consumer sees a zero sentinel.
- Drawback of B: BQ-060 cannot be implemented and T-003/T-007 remain blocked even though rt-control is position-only and does not consume effort.
- Question: choose A and authorize `effort="0"` as a documented nonphysical sentinel with authoritative velocity, or choose B and block until turn effort is supplied?
- Decision: choose A. The active turn URDF limit uses exact lower/upper `-3.12413936107`/`3.12413936107`, authoritative `velocity="87.2664626"`, and `effort="0"` as an explicitly nonphysical “not modeled/not authorized” sentinel. Do not add an effort command/state interface or infer torque capability. The velocity remains a PLC ceiling and does not change the ≤20% commissioning restriction.
- Unblocked scope: bounded turn URDF limit fields are frozen for T-003; torque-aware simulation remains outside the authorized scope until real effort evidence exists.

## BQ-062 — Movable `pitch` has no production state source, breaking the RSP TF chain [RESOLVED 2026-07-21]

- Evidence: the supplied description defines `pitch` as a revolute joint from `base_link` to the parent link of `turn`, so all turn/updown/arm transforms depend on its position. Frozen `REQ-CAN-002` prohibits Node 20 from the production CANopen bus/configuration and the implementation spec defers even its read-only diagnostic path; the hardware authority also says its feedback scale is unknown. Frozen `/joint_states` contains 16 production joints (13 EtherCAT + updown + two tracks), not pitch. `robot_state_publisher` publishes a movable joint transform only after receiving that joint's state, so the approved RSP cannot produce a complete `base_link -> pitch -> turn -> ...` TF subtree. Publishing a made-up pitch state or adding Node 20 now would violate the frozen boundary.
- Proposed decision A (only valid if the production mechanism is physically locked at zero for this stage): apply an active-description overlay changing `pitch` to a fixed joint at the source joint's zero pose. Remove it from active MoveIt variables and publish the transform through RSP as static. Preserve the raw source's revolute joint as provenance. Reverting to movable pitch later requires a separately approved read-only physical state source and scale.
- Benefit of A: produces a complete, single-source TF tree with no fabricated runtime feedback or Node 20 write path, and matches a genuinely locked mechanism.
- Drawback of A: if physical pitch is not locked exactly at zero, every downstream arm/lift transform and collision model is wrong; motion planning cannot use pitch at all in this phase.
- Alternative B (strictly preserves physical model): keep `pitch` revolute and do not publish a constant substitute. Mark the complete RSP subtree/T-007 as blocked until an authoritative Node 20 angle conversion and read-only state transport are supplied and approved; Node 20 still receives no NMT/controlword/mode/target/SDO write.
- Benefit of B: never lies about robot geometry and preserves the future movable joint correctly.
- Drawback of B: the production TF tree remains incomplete and T-007/RSP cannot be accepted in this phase without new hardware evidence.
- Question: is pitch mechanically fixed at zero for this production phase, approving A; or choose B and keep the TF/bringup work blocked until real read-only pitch feedback is available?
- Decision: approve A. For this production phase pitch is treated as mechanically fixed at the source joint's zero pose. The active migrated URDF changes it to a fixed joint, removes it from active MoveIt variables, and lets RSP publish that transform statically. Preserve the raw revolute description as provenance. Do not add Node 20 to bus configuration or create any write/read-control path for it in this phase.
- Unblocked scope: the base-to-turn TF chain and production RSP model are frozen for T-003/T-007; future movable pitch support requires a separately approved real state source.

## BQ-063 — Track position scale is unspecified but stock ros2_canopen invents `0.001` [RESOLVED 2026-07-21]

- Evidence: frozen `REQ-IF-002` requires position for all 16 production joints plus track velocity. The hardware authority supplies only the track velocity conversion `scale_vel_to_dev: -731746.8647903234` and `scale_vel_from_dev: -1.36659280431156e-6`; TPDOs also contain raw `0x6064` track position. Pinned ros2_canopen always exports a position state for Cia402 nodes and, when no position scale is configured, silently defaults `scale_pos_from_dev` to `0.001` and `scale_pos_to_dev` to `1000.0`, which have no authority. The production convention deliberately uses linear track speed as the joint velocity and `wheel_radius=1.0`, so accumulated linear displacement is numerically the matching logical joint position.
- Proposed decision A (recommended): explicitly set each track's position scales equal to its already frozen velocity/count scales: `scale_pos_to_dev: -731746.8647903234` and `scale_pos_from_dev: -1.36659280431156e-6`, with zero offsets. Treat `0x6064` as accumulated track displacement in metres; under the intentional `wheel_radius=1.0` convention the same numeric value is the continuous joint coordinate consumed by JSB/RSP. Position commands remain unclaimed/unused in PV operation. Keep the existing T-014 low-speed sign/ratio verification as a commissioning gate and record rollover behavior.
- Benefit of A: eliminates ros2_canopen's unauthorized defaults, preserves the frozen 16-position `/joint_states` contract, and uses no new numeric constant—the derivative of position has the same unit scale as the already authorized velocity.
- Drawback of A: the published track “position” is a logical accumulated linear displacement rather than physical motor/output-shaft radians; its absolute origin is arbitrary and signed. Raw int32 `0x6064` eventually rolls over, and the exact sign/ratio still depends on the pending T-014 commissioning result.
- Alternative B: do not assert a position scale. Patch/specialize the hardware interface so PV track nodes expose velocity but no position, and supersede `/joint_states` to contain 14 positions plus two track velocities until a position conversion is separately proven.
- Benefit of B: publishes no unverified accumulated position and avoids rollover being mistaken for physical angle.
- Drawback of B: changes frozen `REQ-IF-002`, broadens the ros2_canopen overlay, and leaves continuous track-joint TF unavailable unless the active description also fixes those visual joints.
- Question: approve A—reuse the exact authorized count scale for accumulated track position under `wheel_radius=1.0`—or choose B and remove track position from the production interface?
- Decision: approve A. Explicitly configure both track position scales using the same exact authorized count conversion as velocity, with zero offsets, and interpret `0x6064` as accumulated logical track displacement under the intentional `wheel_radius=1.0` convention. Position command interfaces remain unused in PV operation. The subsequently requested physical sprocket-size change can supersede the numeric scale only after BQ-064 resolves whether `0.2088 m` is a radius or diameter; position and velocity scales must then change together from the same approved geometry.
- Unblocked scope: track position-interface semantics are frozen; final numeric factors await the new physical-dimension adjudication.

## BQ-064 — New track sprocket dimension changes the meaning of the frozen conversion [RESOLVED 2026-07-21]

- New direction: replace the track driving-sprocket dimension `0.174` with `0.2088`, reduce the chassis linear-velocity limit to `0.3 m/s`, reduce the angular-velocity limit to `0.3 rad/s`, and set the corresponding acceleration limits to twice the velocity limits. These explicitly requested values supersede the older frozen limits once their units and limiter semantics are resolved.
- Inconsistency: the hardware authority labels `0.174 m` as the wheel dimension and derives the frozen raw scale as `-40 * 10000 / (pi * 0.174)`. That formula uses `0.174 m` as the **diameter**, even though the new direction says the driving-wheel **radius** changes from `0.174` to `0.2088`. Treating the new number as a radius versus a diameter changes every track position/velocity raw conversion by exactly a factor of two.
- Proposed decision A (literal new wording): freeze the physical driving-sprocket radius as `0.2088 m`, hence diameter `0.4176 m`. Configure both position and velocity scales as `scale_*_to_dev = -304894.5269959681` raw per metre (or metre/second) and `scale_*_from_dev = -3.27982273034774e-6` metre (or metre/second) per raw count. Keep the logical diff-drive `wheel_radius=1.0` convention.
- Benefit of A: follows the newly stated word “radius” literally and gives one consistent physical geometry for position and velocity conversion.
- Drawback of A: this changes the old physical diameter from `0.174 m` to `0.4176 m`, a 140% increase, rather than changing like-for-like from `0.174` to `0.2088`; an unintended interpretation would halve commanded/observed linear motion relative to decision B.
- Alternative B (same field as the authority): freeze `0.2088 m` as the new physical driving-sprocket **diameter**, hence radius `0.1044 m`. Configure both position and velocity scales as `scale_*_to_dev = -609789.0539919361` and `scale_*_from_dev = -1.63991136517387e-6`. Keep the logical diff-drive `wheel_radius=1.0` convention.
- Benefit of B: replaces the authority's existing diameter with another diameter; the physical size rises by 20%, which is a like-for-like update.
- Drawback of B: contradicts the new wording “radius `0.2088 m`” and would be wrong by a factor of two if `0.2088 m` is truly the measured radius.
- Common effect: BQ-063 remains valid semantically, but its numeric position and velocity factors change together to the selected geometry. Do **not** set `diff_drive_controller.wheel_radius` to the physical radius: its frozen `1.0` is what makes the hardware joint command/state units directly metres and metres/second; changing it too would apply the wheel-radius conversion twice.
- Question: is `0.2088 m` the measured physical **radius** (choose A), or the measured physical **diameter** replacing the old `0.174 m` diameter (choose B)?
- Decision: choose A. Freeze the physical driving-sprocket radius as exact `0.2088 m` and diameter as `0.4176 m`. Both track nodes use `scale_pos_to_dev` and `scale_vel_to_dev` equal to `-304894.5269959681`; both inverse factors equal `-3.27982273034774e-6`, with zero offsets. Keep the logical diff-drive `wheel_radius=1.0` unchanged. This explicitly supersedes the old `0.174 m` diameter and its `-731746.8647903234` conversion in the hardware authority and frozen REQ-CAN-003.
- Benefit of the selected policy: the CANopen raw conversion now matches the newly confirmed measured physical radius, and position/velocity retain identical derivative-consistent scales.
- Drawback explicitly accepted: this is a large physical-geometry change (old diameter `0.174 m` to new diameter `0.4176 m`) and deliberately invalidates the old frozen conversion; commissioning must reverify direction and measured travel before motion acceptance.
- Unblocked scope: final track position/velocity CANopen conversion is frozen for T-006/T-014.

## BQ-065 — Chassis twist limits do not by themselves limit each track to `0.3 m/s` [RESOLVED 2026-07-21]

- Evidence: the new direction states linear velocity limit `0.3 m/s` and angular velocity limit `0.3 rad/s`. With the frozen `wheel_separation=0.82 m`, standard differential-drive inverse kinematics produce `v_left/right = linear.x ∓ angular.z * 0.41`. A simultaneous corner command `(0.3 m/s, 0.3 rad/s)` therefore commands one track at `0.177 m/s` and the other at `0.423 m/s`, even though each chassis-axis limit is individually satisfied. Under BQ-064 the outer command is approximately `-128970` raw rather than the `-91468` raw corresponding to `0.3 m/s`.
- Proposed decision A (standard diff-drive parameter meaning): apply independent symmetric chassis command limits `linear.x ∈ [-0.3, 0.3] m/s` and `angular.z ∈ [-0.3, 0.3] rad/s`. Do not add a per-track `0.3 m/s` limiter; combined commands may reach `0.423 m/s` on the outside track. The old hardware limit was `0.6 m/s`, so this combined maximum remains below that prior authorized physical-track speed, although that fact does not turn `0.3` into a per-track bound.
- Benefit of A: maps the requested linear/angular limits directly onto stock `diff_drive_controller` parameters, adds no custom saturation path, and permits both requested maxima simultaneously.
- Drawback of A: an individual track can exceed `0.3 m/s` by 41% during combined translation and rotation; anyone reading `0.3 m/s` as a hard actuator/track ceiling would get the wrong behavior.
- Alternative B (hard physical-track ceiling): treat `0.3 m/s` as the maximum magnitude of either track. After inverse kinematics, uniformly scale both track velocities whenever `max(abs(v_left), abs(v_right)) > 0.3`, preserving curvature. At the simultaneous maximum corner, both chassis components reduce by factor `0.3/0.423 = 0.709219858156`, yielding approximately `linear.x=0.212765957447 m/s` and `angular.z=0.212765957447 rad/s`.
- Benefit of B: neither track ever exceeds the requested `0.3 m/s`, and uniform scaling preserves the commanded curvature/sign relationship.
- Drawback of B: the two advertised chassis maxima cannot be achieved simultaneously, and pinned stock `diff_drive_controller` does not expose this coupled wheel-speed limiter as a parameter, so it requires a narrow controller overlay/patch plus tests.
- Question: choose A and interpret `0.3 m/s`/`0.3 rad/s` as independent chassis-twist limits, or choose B and additionally enforce `0.3 m/s` as a hard limit on each physical track?
- Decision: choose A. Configure stock `diff_drive_controller` with independent symmetric chassis limits `linear.x.min_velocity=-0.3`, `linear.x.max_velocity=0.3`, `angular.z.min_velocity=-0.3`, and `angular.z.max_velocity=0.3`, with both velocity-limit enable flags true. Do not add a coupled per-track limiter. At simultaneous maxima the standard inverse kinematics may command one physical track at `0.423 m/s`; this is explicitly permitted.
- Benefit of the selected policy: the requested chassis limits map directly to supported upstream parameters with no controller patch, and either maximum remains usable while the other component is nonzero.
- Drawback explicitly accepted: the stated `0.3 m/s` is not a hard actuator/track-speed ceiling; a combined command can drive the outer track 41% faster.
- Unblocked scope: chassis velocity-limiter semantics are frozen for T-007; acceleration semantics remain in BQ-066.

## BQ-066 — “Acceleration is twice velocity” does not specify the normal deceleration bound [RESOLVED 2026-07-21]

- Evidence: the pinned stock `diff_drive_controller` exposes signed `min_acceleration` and `max_acceleration`, not separate acceleration/deceleration parameters. If only `max_acceleration` is supplied, the upstream `SpeedLimiter` automatically sets `min_acceleration=-max_acceleration`, so the normal slowdown is symmetric. The same limiter runs after a stale `cmd_vel` is replaced with zero, whereas hardware disable, NMT Stop, EMCY/heartbeat handling, and the approved emergency group-stop paths are separate and must not be delayed by this ordinary motion limit.
- Proposed decision A (recommended upstream-native interpretation): enable acceleration limiting and explicitly set linear acceleration to `[-0.6, 0.6] m/s²` and angular acceleration to `[-0.6, 0.6] rad/s²`; leave jerk limiting disabled because no jerk value was authorized. Thus normal acceleration, normal braking, reversal, and `cmd_vel`-timeout ramp-down all use the same magnitude. From maximum speed, an ordinary stop takes at least `0.5 s` and approximately `0.075 m` (or `0.075 rad`) under a constant `0.6` deceleration; reversal from `+0.3` to `-0.3` takes at least `1.0 s`.
- Benefit of A: directly implements “twice” with supported stock parameters, gives predictable bidirectional ramps, and avoids an unbounded command discontinuity that can cause track slip or mechanical shock.
- Drawback of A: an ordinary command stop, including command-timeout zeroing, is intentionally not immediate and can add up to `0.075 m` straight-line stopping distance at the limit. This is not the emergency-stop guarantee.
- Alternative B: apply `+0.6` only to speed increase but do not assume a `-0.6` normal deceleration. Because the pinned controller defaults an omitted minimum to `-max`, B requires either a separately supplied negative-deceleration value or another narrow limiter patch; implementation remains blocked until that value/behavior is frozen.
- Benefit of B: allows normal braking to be made faster than acceleration if that is the intended vehicle behavior.
- Drawback of B: it is not implementable from the current numbers alone and loses the upstream-native symmetric interpretation; unrestricted braking would also create step commands and higher slip/shock risk.
- Question: choose A and make both normal acceleration and deceleration symmetric at magnitude `0.6`, or choose B and provide a distinct normal-deceleration requirement?
- Decision: choose A. Enable the stock acceleration limiters and explicitly configure `linear.x.min_acceleration=-0.6`, `linear.x.max_acceleration=0.6`, `angular.z.min_acceleration=-0.6`, and `angular.z.max_acceleration=0.6`. Leave both jerk-limit flags false. These are ordinary chassis-command limits; emergency hardware group stop, NMT Stop, heartbeat/EMCY reaction, and disable/fault FSMs bypass this ordinary ramp as already frozen.
- Benefit of the selected policy: ordinary acceleration, braking, reversal, and stale-command zeroing have one auditable symmetric upstream-native behavior with no controller patch.
- Drawback explicitly accepted: a normal stop from the configured maximum takes at least `0.5 s` and can travel/rotate approximately `0.075 m`/`0.075 rad`; this must not be presented as emergency-stop performance.
- Unblocked scope: chassis velocity/acceleration limiter values and semantics are frozen for T-007.

## BQ-067 — T-004 `joint_limits.yaml` target path conflicts with its allowed modification scope [RESOLVED 2026-07-21]

- Evidence: implementation spec §4 defines the target ownership as `src/rt_control/rt_control_bringup/config/joint_limits.yaml`, alongside `controllers.yaml`. TASK-RT-004 nevertheless says its allowed modification scope is only `robot_hw_ethercat/`, while also requiring migration of `joint_limits.yaml`. Both instructions cannot be satisfied literally. Creating a copy in both packages would introduce two editable numeric authorities and an unspecified consumer precedence.
- Proposed decision A (recommended, follows the target architecture): place the one production `joint_limits.yaml` at `src/rt_control/rt_control_bringup/config/joint_limits.yaml` and narrowly expand T-004's allowed paths to that single bringup file. Keep slave profiles and `ecat.ros2_control.xacro` in `robot_hw_ethercat`; T-007's controller configuration consumes the bringup-owned limit file. Do not create another copy in the hardware package.
- Benefit of A: preserves the explicit target directory architecture and gives controller/MoveIt-facing runtime configuration one authoritative limit file adjacent to `controllers.yaml`.
- Drawback of A: T-004 touches one file outside the execution-plan line that says only `robot_hw_ethercat/`, so that path whitelist must be explicitly superseded.
- Alternative B (follows the task path whitelist): place the only `joint_limits.yaml` at `src/rt_control/robot_hw_ethercat/config/joint_limits.yaml` and supersede spec §4's bringup location. T-007/launch must resolve the limits from the hardware configuration package.
- Benefit of B: T-004 stays entirely inside its stated allowed package and keeps hardware numeric configuration together.
- Drawback of B: violates the documented package ownership, couples controller/MoveIt limits to the EtherCAT hardware package, and requires later launch/controller references to diverge from spec §4.
- Rejected implicit option: do not install two copies; synchronization and precedence are not specified and would undermine the exact-text migration gate.
- Question: choose A and authorize the single bringup-owned `joint_limits.yaml`, or choose B and move the authoritative limit file into `robot_hw_ethercat`?
- Decision: choose A. The single production `joint_limits.yaml` is owned by `src/rt_control/rt_control_bringup/config/joint_limits.yaml`. T-004 receives a narrow path-whitelist exception for that one file; its slave profiles and EtherCAT ros2_control xacro remain under `robot_hw_ethercat`. Do not create a second limits file in the hardware package.
- Benefit of the selected policy: controller and launch configuration retain the target package ownership documented by implementation spec §4, with one exact-text numeric authority.
- Drawback explicitly accepted: T-004 modifies one bringup-owned file outside the execution plan's original `robot_hw_ethercat/`-only line.
- Unblocked scope: T-004 and dependent T-005/T-007/T-010/T-012 work may proceed.
## BQ-068 — 250 Hz Controller Manager 无法用官方分频得到精确 100 Hz JSB [RESOLVED 2026-07-21]

- 状态：**RESOLVED**
- 冲突：冻结 `REQ-RT-001` 要求 Controller Manager 为 250 Hz，冻结 `REQ-IF-002` 要求
  `/joint_states` 为 100 Hz。ros2_control Humble 基线的 per-controller `update_rate`
  采用整数分频；250/100 不是整数，因此配置 `joint_state_broadcaster.update_rate: 100`
  时，源码明确将实际频率调整为 `250 / floor(250/100) = 125 Hz`。
- 证据：`controller_manager.cpp` 的 controller update-rate 校验与 update loop（上游只读基线
  `/home/kkozia/rt_control_refs/ros2_control@e65ddd72804f3f2d9b19e533a15ed436b2f3fc42`）。
- Decision：将 `/joint_states` 的生产频率由原冻结的 100 Hz 改为 **50 Hz**，明确取代
  `REQ-IF-002` 中的 100 Hz 数值。Controller Manager 保持 250 Hz；给官方
  `joint_state_broadcaster` 配置 `update_rate: 50`，使用精确整数分频 5，不增加发布节流补丁。
- Benefit：完全沿用 Humble 上游调度逻辑，精确得到 50 Hz，并相对原 100 Hz 目标降低一半
  JointState 序列化、DDS 和下游处理负载。
- Drawback：外部关节状态更新周期从原目标的 10 ms 增加到 20 ms；依赖 100 Hz 状态流的域外
  消费者必须接受新契约。250 Hz 控制环和内部硬件反馈采样频率不变。
- Unblocked scope：BQ-068 不再阻塞 T-007；控制器配置和接口检查统一使用 50 Hz。

## BQ-069 — 同一 service 的互斥回调组无法实现“重复调用立即 busy” [RESOLVED 2026-07-21]

- 状态：**RESOLVED**
- 冲突：BQ-035 已批准 `/rt/enable` 与 `/rt/disable` 分别使用独立的
  `MutuallyExclusive` callback group，以便两种服务可以并发、disable 可以打断 enable；
  BQ-036 又要求同方向的第二次调用在第一次仍执行时**立即**返回
  `stage="operation_in_progress"`。在 Humble MultiThreadedExecutor 中，同一个
  MutuallyExclusive group 内的第二个同名 service callback 不会并发进入，而是排队到第一次
  callback 返回；届时操作已经结束，无法再按 BQ-036 立即报 busy。
- 需要裁决：A）将三个 service 各自改为 `Reentrant` callback group，在代码中用原子所有权实现
  同向重复调用立即 busy，同时仍允许 enable/disable 并发；或 B）保留
  `MutuallyExclusive`，接受同向重复请求由 executor 排队，完成后按终态幂等结果返回。
- 权衡：A 精确满足 BQ-036，但放弃了 BQ-035 对 callback-group 类型的字面选择并要求所有共享
  状态均通过原子记录保护；B 保持原 callback-group 设计更简单，但可观察的重复调用语义与已批准
  结果不同。
- Decision：选择 A。`/rt/enable`、`/rt/disable`、`/rt/reset_fault` 分别使用
  `Reentrant` callback group；每个 service callback 先通过原子操作所有权判定，同方向已有操作时
  立即返回 `ok=false, stage="operation_in_progress"`，不得排队后伪装成终态幂等调用。
  enable/disable 之间已冻结的并发、抢占和不抢占窗口保持不变。
- Benefit：精确保留 BQ-036 的可观察立即-busy 语义，并允许独立 service 方向按既有状态机规则
  并发进入，而不依赖 executor 排队时序。
- Drawback：放弃 BQ-035 对 `MutuallyExclusive` 类型的字面选择；共享操作状态必须全部采用原子
  记录或明确的单所有者传递，代码审查和竞态验证面增大。
- Unblocked scope：BQ-069 不再阻塞 T-010；enable_manager 按 Reentrant+atomic 方案实现。

## BQ-070 — enable_manager 诊断如何交给 T-012 未定义 [RESOLVED 2026-07-21]

- 状态：**RESOLVED**
- 冲突：冻结诊断表要求 1 Hz 输出 enable_manager 状态和首个失败字段，但已批准的
  `RtEnable.srv` 只有请求的最终响应，没有状态查询；BQ-024 只冻结了 EtherCAT 快照接口，
  没有冻结 enable_manager 到 `rt_diagnostics` 的状态 topic/state interface。
- 需要裁决：A）由 enable_manager 自己的非 RT 1 Hz timer 直接发布该行标准
  `/diagnostics`（T-012 聚合其余来源，不新增状态 topic）；或 B）新增一个只读状态消息/topic，
  由 `rt_diagnostics` 订阅后统一发布。
- 权衡：A 接口最少、状态所有者直接发布，但该行不经过聚合节点；B 所有行由一个节点统一，
  代价是新增公共消息、topic、QoS 和生命周期契约。
- Decision：选择 A。enable_manager 使用自身普通优先级、非 RT 的 1 Hz timer，直接向标准
  `/diagnostics` 发布 `enable_manager` 行及已冻结的 `state`、`failed_batch`、`failed_joint`、
  `failed_status_word` 字段；不得在 controller `update()` 路径发布、记录日志或分配消息。
  `rt_diagnostics` 继续发布 EtherCAT/CANopen 等其余行，不新增 enable-manager 状态 topic/message。
- Benefit：使用 ROS 标准 diagnostics 的多发布者模型，状态所有者直接生成快照，不新增机内接口、
  QoS 和消息版本维护面。
- Drawback：`/diagnostics` 的生产者不再只有 `rt_diagnostics`，enable_manager 行不会经过该节点的
  统一组装；消费者必须按诊断 name/hardware_id 聚合，而不能假设单一 publisher。
- Unblocked scope：BQ-070 不再阻塞 T-012 的 enable_manager 行，也不新增接口包工作。

## BQ-071 — 当前 ICube 固定提交属于 Jazzy，不能在冻结的 Humble 环境编译 [RESOLVED 2026-07-21]

- 状态：**RESOLVED**
- 冲突：当前审计参考 `/home/kkozia/rt_control_refs/ecat_icube` 固定为官方 `jazzy`
  提交 `7a32bd7b8fc066c6668b1df7446c89aff570bb7d`，其 `EthercatDriver::on_init()`
  使用 Jazzy 的 `hardware_interface::HardwareComponentInterfaceParams`。冻结运行环境是 ROS 2
  Humble；Humble 的 `SystemInterface` 只有 `on_init(const HardwareInfo &)`，因此该官方提交在
  `ros:humble-ros-base` 中原样编译失败。官方仓库另有 Humble 分支，当前头为
  `1390be742986f4e898ca112e49bb24805be9899a`，但它的内部结构早于 Jazzy 的
  `EthercatBusManager` 重构，切换后必须针对 Humble 源重新审计并生成 preload/只读状态补丁。
- 选择 A（推荐，向开源目标发行版靠拢）：固定官方 Humble 提交
  `1390be742986f4e898ca112e49bb24805be9899a`，在该源上重做已批准的原始位置预装载、
  per-enable-session 清理和只读状态接口补丁。
- A 的好处：基础 API 与部署发行版一致，避免维护 Jazzy→Humble 的整层兼容回移，官方分支可
  直接接受 Humble 的 ros2_control ABI。A 的弊端：放弃已经审计过的 Jazzy BusManager 重构，
  需要重新检查旧版 read/write、锁和生命周期实现，补丁落点及测试都要重做。
- 选择 B：继续固定 Jazzy `7a32bd7`，授权增加 Humble API 兼容回移，并继续修复随后发现的
  Jazzy/Humble ABI 差异。
- B 的好处：保留较新的 BusManager 拆分和目前已完成的 Jazzy 源码审计。B 的弊端：形成跨发行版
  私有 backport，当前只证实了第一个 `on_init` 错误，后续可能还有更多编译或运行 ABI 差异，
  维护和验证面明显更大。
- Decision：选择 A。ICube 唯一实现基线改为官方 Humble 提交
  `1390be742986f4e898ca112e49bb24805be9899a`；在该提交上重新审计 read/write、生命周期和锁，
  并重做已批准的原始位置预装载、per-enable-session 清理及
  `ethercat_domain/process_data_age_ms` 只读状态接口补丁。禁止把 Jazzy
  `7a32bd7b8fc066c6668b1df7446c89aff570bb7d` 的 BusManager 结构或兼容回移带入生产分支。
- Benefit：ICube 基础源码与 ROS 2 Humble 的 ros2_control ABI 原生一致，部署和编译目标统一，
  避免维护不可预估的跨发行版私有 backport，并符合向官方开源实现靠拢的原则。
- Drawback：必须放弃此前针对 Jazzy BusManager 的源码审计落点，在旧版 Humble 结构中重新审计
  和实现补丁；两版内部结构不同，不能机械移植补丁。
- Process correction：该发行版不匹配本应在 T-001/T-002 基线核验时报告，实际直到 Humble
  Docker 编译才暴露，属于实现前置检查顺序错误。任何 Jazzy ICube 实验修改均未进入目标仓库或
  提交，因此不存在生产代码回退；后续必须先完成 Humble 原版编译，再开始功能补丁。
- Unblocked scope：ICube overlay 可在固定 Humble 基线上重做；完成前仍阻塞 T-007 全量加载和
  T-012 EtherCAT 数据源验收。

## BQ-072 — Leadshine 原始 EDS 不能被 Humble `dcfgen 2.4.0` 原样解析 [RESOLVED 2026-07-21]

- 状态：**RESOLVED**
- 证据：在恢复后的 Docker 网络环境中安装官方 Humble
  `ros-humble-lely-core-libraries`，按上游 `cogen_dcf()` 流程运行
  `cogen -> dcfgen -rS`。仓库内原始 `LD2-CAN.eds` 的
  `SupportedObjects=0x0003/0x01CA/0x0054` 被 `dcf-tools 2.4.0` 用十进制
  `int(..., 10)` 解析而失败；其 9 个 PDO COB-ID 表达式使用 `$NodeID`，而工具只向
  表达式环境注入大写 `$NODEID`。`-S/--no-strict` 只放宽 lint，不能绕过这两个解析错误。
  EDS 还含 `[607E]` 的 `DataType=INTEGER8, HighLimit=255` lint 警告，但 `-S` 可保留
  原文并继续生成。
- 已做只读验证：仅在一次性临时副本中把三个 `SupportedObjects` 改为数值等价的
  十进制 `3/458/84`，把 `$NodeID` 改成 `$NODEID`，其余内容不变；随后成功生成
  `master.dcf`、`master.bin` 和三个节点 `.bin`。verbose 输出只有每节点
  `0x1016:1=5000 ms consumer(node 100)` 与 `0x1017:0=1000 ms producer`，与已冻结值一致。
  仓库中的权威原始 EDS 未修改，SHA-256 仍为
  `e1abc580b76d0548c5eedfbb6461b9ad3f3607b90ecbc087f042cc1573cf9c08`。
- 选择 A（推荐，最小构建适配）：权威原始 EDS 原封不动入仓并继续用于存档校验；构建时
  复制到 build 目录，严格只执行上述 12 处等值文本规范化，再运行 `cogen` 和
  `dcfgen -rS`。安装生成物和规范化后的运行时 EDS，同时记录源/派生文件哈希及转换日志。
- A 的好处：不改任何对象值、PDO 映射或驱动器参数，已证明可由官方 Humble 工具生成，
  且保留供应商文件作为唯一审计源。A 的弊端：明确例外于“数值文本不得改格式”的一般规则；
  运行时使用一个派生 EDS，构建脚本必须长期维护并验证只发生这 12 处变换；`-S` 仍会接受
  供应商 `[607E]` 元数据不一致警告。
- 选择 B（工具兼容补丁）：不转换 EDS，给 `dcf-tools 2.4.0` 增加兼容解析，使
  `SupportedObjects` 接受基数自动识别并对变量名做大小写兼容，再以原始 EDS 生成。
- B 的好处：构建和运行始终消费字节完全相同的供应商 EDS。B 的弊端：新增第三个上游依赖
  补丁，修改通用 DCF 解析语义，验证和长期维护面明显大于 12 处确定性构建转换。
- 选择 C：不授权任何转换或工具补丁，等待驱动器厂商提供可被 `dcfgen 2.4.0` 接受且经重新
  签核的 EDS；在此之前保持 CANopen DCF/加载/实机任务阻塞。
- Decision：选择 A。供应商原始 EDS 保持字节不变并继续作为唯一硬件审计源；构建脚本先校验其
  SHA-256 必须为 `e1abc580b76d0548c5eedfbb6461b9ad3f3607b90ecbc087f042cc1573cf9c08`，
  在 build 目录复制后仅执行三个 `SupportedObjects` 十六进制到等值十进制转换，以及九个
  `$NodeID` 到 `$NODEID` 的大小写转换。脚本必须验证替换数量恰为 3+9，输出源/派生哈希及转换
  日志；随后用未修改的官方 Humble `cogen` 和 `dcfgen -rS` 生成并安装运行时 bus/EDS/DCF/BIN。
- Benefit：已实证可由官方 Humble 工具链生成，且不修改解析工具、对象值、PDO 映射、默认值或
  驱动器参数；供应商原始文件仍可按固定哈希独立审计。
- Drawback：明确授权一个只限于该固定哈希 EDS 的文本格式例外；运行时 EDS 与供应商文件不再
  字节相同，构建和评审必须长期保留派生哈希/转换日志，`-S` 仍会显示 `[607E]` 元数据警告。
- Unblocked scope：DCF 构建适配可实施；完成生成检查后 BQ-072 不再阻塞 T-007/T-014。

## BQ-073 — 官方 Humble ICube 的 IgH 安装前缀与现有 Dockerfile 不一致 [RESOLVED 2026-07-21]

- 状态：**RESOLVED**
- 证据：按 BQ-071 固定官方 Humble `1390be742986f4e898ca112e49bb24805be9899a` 后，先在
  已完成的 rt-control 镜像中原样编译。官方 `ethercat_interface/CMakeLists.txt` 与
  `ethercat_manager/CMakeLists.txt` 均硬编码 `ETHERLAB_DIR=/usr/local/etherlab`，导出
  `/usr/local/etherlab/include` 并从 `/usr/local/etherlab/lib` 查找 `libethercat`。现有
  `docker/rt-control/Dockerfile` 则以 `./configure --prefix=/usr/local --disable-kernel` 安装，
  实际头文件/库位于 `/usr/local/include`、`/usr/local/lib`。因此官方原版首先在不存在的导出
  include 路径上失败，尚未进入功能补丁编译。
- 选择 A（推荐，向官方 ICube 布局靠拢）：把容器内 IgH 用户库安装前缀改为
  `/usr/local/etherlab`，同步设置 `PATH=/usr/local/etherlab/bin`、
  `LD_LIBRARY_PATH=/usr/local/etherlab/lib` 并重跑 T-008 镜像/版本检查。后续 ICube 不为路径增加
  补丁；T-009 宿主脚本也明确记录其独立安装前缀，避免把容器路径误当宿主 ABI 要求。
- A 的好处：官方 Humble ICube 原版 CMake 可直接查找 IgH，减少一个非功能性下游源码补丁；
  EtherLab 文件集中在专用目录，来源和卸载边界更清楚。A 的弊端：修改已验证的 Docker 镜像布局，
  必须重建镜像并复核 `ethercat` CLI、动态库搜索路径以及宿主/容器版本一致性命令。
- 选择 B：保留容器 `/usr/local` 前缀，给官方 ICube 的两个 CMakeLists 增加可配置
  `ETHERLAB_DIR` cache 变量，并在构建时传 `/usr/local`。
- B 的好处：现有 Dockerfile、CLI 和动态库路径保持不变，补丁是常见的 CMake 可配置化改动。
  B 的弊端：新增与机器人功能无关的 ICube 下游补丁；任何遗漏的硬编码路径仍可能在后续包或导出
  配置中出现，且偏离“官方 Humble 原版先编译”的目标。
- Decision：选择 A。容器内 IgH 用户态源码仍由 Dockerfile 构建，但安装前缀改为
  `/usr/local/etherlab`；设置 `PATH=/usr/local/etherlab/bin` 与
  `LD_LIBRARY_PATH=/usr/local/etherlab/lib`。官方 Humble ICube 不携带 ETHERLAB_DIR 路径补丁。
- Benefit：部署布局与官方 ICube 的原生查找路径一致，少维护一个无关功能的下游补丁；头文件、
  库和 CLI 均位于独立 EtherLab 目录。
- Drawback：T-008 镜像需要重建，并重新核验 CLI、动态库和版本一致性；现有 `/usr/local` 布局
  不再是生产镜像布局。
- Unblocked scope：官方 Humble ICube 原版编译可在新布局镜像中重新验证。

## BQ-074 — 是否在最终运行镜像中保留 IgH 完整源码

- 状态：**RESOLVED 2026-07-21**
- 证据：当前 Dockerfile 在镜像构建过程中克隆并编译 IgH `stable-1.6`，随后执行
  `rm -rf /tmp/ethercat`。因此“在 Docker 内从主站源码构建”已经成立，但最终运行镜像只保留
  `/usr/local/etherlab` 下的用户态安装产物，不保留 Git 工作树。用户提出“肯定要把主站源码放在
  docker 里”，可能表示要求最终镜像也能现场查看源码，现有行为不满足该解释。
- 选择 A：在最终镜像 `/opt/src/igh-ethercat` 保留与安装产物完全对应的固定源码工作树和提交信息，
  不允许运行时修改或从该目录重新安装。
- A 的好处：离线现场可审计具体源码/提交，排障时可直接对照头文件与实现，也更易证明容器用户库
  的来源。A 的弊端：增加镜像体积和文件数量；源码本身不参与运行，保留编译工作树可能让现场人员
  误以为可在生产容器内临时重编覆盖。
- 选择 B：构建后删除源码，仅在镜像标签、versions.env 和构建记录中保存精确提交/版本，最终镜像
  只含运行与后续包编译需要的 bin/lib/include。
- B 的好处：镜像更小、运行内容边界更清楚，避免生产容器内临时编译的误用。B 的弊端：现场离线
  无法直接查看完整主站实现，需要从固定提交另取源码核对。
- Decision：选择 B。IgH 源码只存在于 Docker 构建阶段；安装到
  `/usr/local/etherlab` 后删除 Git 工作树，最终运行镜像只保留运行以及编译 ROS overlay 所需的
  bin/lib/include，并在版本清单和构建记录中保存可核验版本与精确提交。
- Benefit：减小最终镜像并清晰区分运行产物与源码，避免在生产容器中临时修改、重编或覆盖主站。
- Drawback：现场离线时无法直接查看完整 IgH 实现；排障审计需要按记录的固定提交另取源码。
- Unblocked scope：T-008 最终镜像内容可定稿；本裁决不阻塞 ICube Humble 原版及功能补丁编译。

## BQ-075 — IgH `stable-1.6` 是可移动分支，无法仅凭版本名复现构建

- 状态：**RESOLVED 2026-07-22**
- 证据：`versions.env` 当前只有 `IGH_VERSION=stable-1.6`，Dockerfile 使用
  `git clone --depth 1 --branch "${IGH_VERSION}"`。2026-07-21 只读查询官方远端时，该分支指向
  `2f7f884f1c7d377c02a7d627eb06512126a0e50e`；分支今后可移动，同一仓库提交不能保证得到同一主站源码。
  BQ-074 选择 B 又要求删除源码并在版本/构建记录中保存精确提交，因此必须明确是固定输入，还是仅记录
  每次实际解析到的输出提交。
- 选择 A（推荐，可复现）：增加 `IGH_COMMIT=2f7f884f1c7d377c02a7d627eb06512126a0e50e`，构建时仍可记录
  人类可读版本 `stable-1.6`，但必须 checkout 并校验这个完整提交；把版本和提交写入镜像 label/清单。
- A 的好处：今后重建消费同一源码，能准确对应已验证的 ICube/IgH ABI；即使分支移动也不改变生产输入。
  A 的弊端：不会自动获取 stable-1.6 后续修复，升级必须显式改提交并重新验证。
- 选择 B（只记录、不固定）：继续从 `stable-1.6` 分支头构建，每次把实际 `git rev-parse HEAD` 写入镜像；
  接受不同时期构建可能使用不同源码。
- B 的好处：自动得到分支后续修复，不需要人工更新提交。B 的弊端：相同配置不能保证复现相同镜像，且未经
  再审批的上游变化会直接进入下一次构建。
- Decision：选择 A。保留 `IGH_VERSION=stable-1.6` 作为人类可读版本，同时固定
  `IGH_COMMIT=2f7f884f1c7d377c02a7d627eb06512126a0e50e`。Docker 构建必须直接 fetch、checkout 并
  校验该完整提交，把两者写入镜像 label 和依赖版本清单；不得在构建时解析可移动分支头作为输入。
- Benefit：所有后续构建使用同一份已知源码，并能将 IgH 安装产物追溯到完整提交，不受官方分支移动影响。
- Drawback：不会自动接收 `stable-1.6` 后续修复；任何升级都必须显式修改提交、重新编译并完成回归验证。
- Unblocked scope：T-008 的 IgH 源码版本输入可定稿并重新验证。

## BQ-076 — 清华 ROS 2 镜像缺少 `ros-humble-lely-core-libraries`

- 状态：**RESOLVED 2026-07-22**
- 证据：按 BQ-073/BQ-075 重建时，固定 IgH 提交已成功编译并安装到 `/usr/local/etherlab`；随后
  `rosdep install` 在清华 ROS 2 apt 镜像报 `Unable to locate package
  ros-humble-lely-core-libraries`。使用相同 `ros:humble-ros-base` 但不替换 ROS 2 软件源，只读运行
  `apt-cache policy` 可得到官方源候选版本 `0.2.13-1jammy.20260304.090440`。因此是清华镜像缺包，
  不是依赖名错误。当前 Dockerfile 选择清华源是早期网络条件下的实现，冻结规格没有规定缺包时的来源优先级。
- 选择 A（推荐，代理已修复）：Ubuntu 软件包继续使用清华镜像，但 ROS 2 apt 和 rosdep 索引恢复官方
  `packages.ros.org`/`raw.githubusercontent.com`，然后通过 apt 安装发布版 Lely。
- A 的好处：使用 ROS 官方发布的 Humble 二进制包及依赖解析，不增加 Lely 私有源码构建和补丁，完整工作区
  已实证能解析到所需版本。A 的弊端：镜像构建依赖 GitHub/ROS 官方网络可用性，国内网络速度可能低于镜像。
- 选择 B：保持清华 ROS 2/rosdep 镜像，另行固定并从源码构建 `lely_core_libraries`。
- B 的好处：继续使用国内镜像完成主要 apt/rosdep 流程。B 的弊端：新增一个必须固定、编译、安装和验证的
  源码依赖；其 ABI/安装布局必须与 ros2_canopen Humble 重新核对，维护面更大。
- 不建议混用条件式 fallback：同一 Dockerfile 根据镜像当时是否缺包在清华和官方 apt 间切换，会使实际二进制
  来源随时间变化，不利于复现和审计。
- Decision：选择 A。Ubuntu apt 继续使用清华镜像；ROS 2 apt 保持基础镜像中的官方
  `packages.ros.org`，rosdep 保持官方 `raw.githubusercontent.com` 和官方 rosdistro 索引。Lely 使用
  官方 Humble 二进制发布包，不增加源码构建。
- Benefit：恢复已实证存在的官方 Lely 包并沿用 ROS Humble 发布体系，避免增加一个源码依赖及其 ABI、安装和
  补丁维护工作。
- Drawback：Docker 构建依赖 ROS/GitHub 官方网络可用性，国内下载可能比镜像更慢。
- Unblocked scope：可重新执行 T-008 镜像构建及全工作区依赖/编译检查。

## BQ-077 — 官方 ROS apt 可达，但 Docker 构建内 GitHub Raw 的 rosdep 索引连续超时

- 状态：**RESOLVED 2026-07-22**
- 证据：BQ-076 A 实施后，`packages.ros.org` 已成功下载 ROS 2 软件包，原先缺失的 Lely 二进制包
  可以解析；但 `rosdep update` 两次连续在 `raw.githubusercontent.com/ros/rosdistro` TLS 读取阶段
  超时，一次只读到两个 YAML，另一次只读到一个。Docker daemon 显示 HTTP/HTTPS proxy 为
  `127.0.0.1:10808`，但构建 `RUN` 环境没有 `HTTP_PROXY/HTTPS_PROXY`；容器内的 `127.0.0.1` 也不是
  宿主代理。因此“Docker 代理已修复”目前覆盖镜像拉取，不等于构建步骤能使用该宿主回环代理。
- 选择 A（推荐，最小来源拆分）：保持 ROS 二进制 apt 为官方 `packages.ros.org`，仅把 rosdep YAML 与
  rosdistro 索引恢复到清华镜像。rosdep 索引只把包键映射到 apt 包名，最终 Lely/ROS 二进制仍由官方 apt
  下载。
- A 的好处：沿用此前已成功的索引访问路径，不暴露或硬编码宿主代理地址；同时解决清华 apt 缺 Lely和
  GitHub Raw 超时两个问题。A 的弊端：依赖清华 rosdep/rosdistro 镜像同步时效，索引与官方 apt 可能短暂
  不同步。
- 选择 B：保持 rosdep/rosdistro 全部官方，暂停镜像构建，先由用户提供一个从 Docker build 网络可达的
  代理地址或系统级 BuildKit 代理配置，再重试。
- B 的好处：所有 ROS 元数据和二进制均来自官方端点。B 的弊端：当前任务继续阻塞；不能把 daemon 的
  `127.0.0.1:10808` 直接作为容器代理，代理地址和凭据也不应猜测或写入仓库。
- Decision：选择 A。ROS 2 apt 保持官方 `packages.ros.org`；仅将 rosdep YAML URL 重写到清华
  `github-raw` 镜像，并将 `ROSDISTRO_INDEX_URL` 指向清华 rosdistro index。不得把宿主回环代理地址或凭据
  写入仓库/Dockerfile。
- Benefit：最终 ROS/Lely 二进制仍由官方 apt 发布源提供，同时避开 Docker build 网络对 GitHub Raw 的
  连续超时，也不引入环境相关代理配置。
- Drawback：依赖清华 rosdep/rosdistro 元数据镜像的同步时效；镜像与官方 apt 短暂不同步时仍可能解析失败。
- Unblocked scope：可继续 T-008 全镜像构建及依赖、标签、IgH 安装布局检查。

## BQ-078 — ICube Humble 的原始位置预装载与发送确认边界 [RESOLVED 2026-07-22]

- 状态：**RESOLVED — HIGH-RISK（安全关键下游补丁）**
- 证据：官方 Humble ICube 在 `read()` 中保存换算后的 `last_position_`，但没有为外部
  `control_word` 所有者提供“本次使能会话的 0x607A 已按最新 0x6064 原始值装载并至少进入一次
  EtherCAT 发送”的证明。仅在 `write()` 内给 0x607A 赋值不能区分尚未调用
  `ecrt_master_send()` 与已提交给主站的帧，也不能防止上一使能会话的旧标志授权下一次 0x000F。
- 自主裁决：在固定 Humble 提交上保持 position-only PDO，不新增 PDO/SDO；以驱动器原始
  `0x6064` 精确复制到 `0x607A`。补丁采用两阶段确认：slave 写入本周期预装载后只登记候选，
  `EcMaster` 完成 `ecrt_master_send()` 后再通知 slave 把该候选确认为已发送。外部控制字在本会话
  首次确认前把 `0x000F` 钳制为 `0x0007`；确认至少一个更早周期的预装载帧后才允许 0x000F。
  离开 Operation Enabled、硬件 deactivate 或新 activate 时清空本会话确认；位置命令只有在与
  当前反馈相差不超过精确 `1° = 0.017453292519943295 rad` 时才完成接管。硬件 activate 等待预装载
  的上限采用已批准的 `5000 ms`，但不等待 Operation Enabled，后者仍由 enable_manager 管理。
- Benefit：每次使能都由最新原始反馈自动预装载，避免编码器比例/offset 的往返舍入，也保证 0x000F
  不会与首个预装载值同帧越过；用户无需先发单点 FJT。
- Drawback：确认边界只能证明数据已经交给 IgH 的 `ecrt_master_send()`，不能证明从站物理接收或
  内部接受；补丁横跨 ICube master/slave/驱动层，未来升级官方提交时必须重新审计。为满足 RT
  规则，周期路径不记录日志，现场定位只能依赖只读状态接口和非 RT diagnostics。
- 验证边界：补丁 SHA-256 为
  `70b18622069420185d8f94b7b1ffe546cea52795df5525884ba84b1295267d64`；已在固定 Humble 原版上
  `git apply --check`，且其最小上游安全测试为 91 tests、0 errors、13 skipped。实机接收与无跳变
  仍由 T-013 证明。

## BQ-079 — 生产源码 overlay 的固定提交与维护边界 [RESOLVED 2026-07-22]

- 状态：**RESOLVED**
- 自主裁决：生产构建只导入三份运行依赖源码并固定完整提交：ICube Humble
  `1390be742986f4e898ca112e49bb24805be9899a`、ros2_canopen Humble
  `fef50e54b1c94c50e908e2c5d0b8888eed907e8d`、ros2_controllers Humble
  `cbcf66218ff43353f9fb5fe7a2c33f458d578d73`。三份补丁在 Docker 构建中先执行
  `git apply --check` 再应用。`/home/kkozia/robot_driver@6bc94cd` 继续只读，只作为迁移/差异权威，
  不作为生产 runtime vendor，也不写入 `deps.repos`。
- Benefit：构建输入可复现，目标运行时不会悄然链接只读存量 fork；每个偏离上游的行为都有单独补丁
  和固定落点。
- Drawback：镜像必须从源码构建三套 overlay，构建时间、网络依赖和升级维护成本均增加；任何上游
  升级都不能仅改分支名，必须重新 apply-check、编译和审计补丁语义。

## BQ-080 — enable_manager 超时、控制器切换歧义和复位终态 [RESOLVED 2026-07-22]

- 状态：**RESOLVED — HIGH-RISK（全组使能/去使能状态机）**
- 自主裁决：RT `update()` 仅用固定数组、标量和原子标志推进状态机；service 回调以 2 ms 普通线程
  轮询最终结果，单次最多 30 s。30 s 只是调用方等待上限，不在后台取消一个仍安全推进的硬件
  状态机。常规下使能按状态感知的标准 `0x0007 -> 0x0006 -> 0x0000` 执行；任一阶段 4 s 超时后，
  全组进入有界 `0x0002` Quick Stop，再继续 terminal disable。任何轴 Fault/Fault Reaction 或
  ENABLED 中意外离开 Operation Enabled 都触发相同全组路径，并在非 RT 线程严格停用 JTC。
- 自主裁决：controller-manager switch 的明确 `ok=false` 记为失败；service 不可用、future 超时或
  同时已有 switch 无法判断服务端最终状态，记为 **ambiguous**。激活歧义立即锁存
  `restart_required`，并发起一次尽力 JTC deactivate；本进程拒绝再次 enable/reset，避免不确定的
  JTC 生命周期和缓存轨迹被复用。
- 自主裁决：`/rt/reset_fault` 为全组操作，先全组至少一个 0x0000 周期，仅对当前 Fault/Fault
  Reaction 目标轴发 0x0080；成功条件不是“原目标已清”，而是全部 13 轴都已到 0x0040。复位期间
  新出现的另一轴故障因此不能被误报为成功。
- Benefit：任何普通失败都优先完成硬件去使能；无法证明 JTC 状态时选择重启边界，不会在不确定的
  控制器状态下恢复运动；全组复位结果与“干净待使能状态”一致。
- Drawback：短暂的 controller-manager 响应延迟也可能升级为必须重启，恢复较保守；service 客户端
  30 s 超时后后台 FSM 仍可能完成，调用方需看 diagnostics 再决定重试；Quick Stop/disable 的实物
  减速度和抱闸效果仍必须由 T-013 验证，软件路径不是认证安全功能。

## BQ-081 — T-012 诊断聚合只使用现有权威来源 [RESOLVED 2026-07-22]

- 状态：**RESOLVED**
- 自主裁决：`rt_diagnostics` 只订阅 JSB 的 `/dynamic_joint_states` 与标准
  `/diagnostics`。EtherCAT 行使用 BQ-024 的只读接口；CANopen 行只接受 hardware_id 1/2/3 且含
  上游 native NMT/EMCY/CiA402 字段的状态，再规范化到 `/robot/rt_control/canopen/node_<id>`。
  本节点自己发布的 `hardware_id=robot-001` 行不会被再次采集，避免自反馈。源快照 3 s 未更新即
  STALE；WC 只在累计值相对上一秒增长时 WARN。enable_manager 按 BQ-070 自己发布其诊断行。
- Benefit：不再造 raw CAN 观察器、心跳年龄或 SDO 计数，也不把 diagnostics 变成第二套安全判断；
  EtherCAT、CANopen 和使能状态都来自实际执行者。
- Drawback：`/diagnostics` 是多发布者模型，消费者必须按 name/hardware_id 聚合；CAN 行的字段和
  可见性依赖 pinned ros2_canopen native diagnostics，JSB 未运行时 EtherCAT 全部 STALE。

## BQ-082 — 生产/Mock 启动链与干净停止入口 [RESOLVED 2026-07-22]

- 状态：**RESOLVED**
- 自主裁决：`use_mock_hardware` 默认 `false`，生产 xacro 只装载真实 ICube 和 Cia402System；显式
  `true` 时才装载接口镜像用 GenericSystem。启动时 JTC 必须先成功配置为 INACTIVE，随后才加载
  enable_manager；JTC spawner 失败则关闭整次 launch，不留下可使能的半启动系统。JSB、updown、
  diff-drive 默认 ACTIVE，JTC 默认 INACTIVE。容器唯一支持的入口是 `rt_control_start`：launch 位于
  独立进程组，收到 INT/TERM 先调用 `/rt/disable` 的一次性客户端，再把 SIGINT 转发给 launch。
- Benefit：Mock 可执行冻结的控制器加载检查而不伪装生产总线；生产启动不会在 JTC 配置失败时开放
  使能；容器停止有明确的标准下使能机会。
- Drawback：Mock 只验证接口和生命周期加载，不模拟真实 CiA402 状态跳转、CANopen 心跳或机械响应；
  直接绕过包装脚本运行 `ros2 launch` 不享受 orderly disable，必须作为不受支持的调试方式标注。

## BQ-083 — Docker build 的宿主网络与代理暴露边界 [RESOLVED 2026-07-22]

- 状态：**RESOLVED — HIGH-RISK（构建网络边界）**
- 证据：直接 VCS 拉取在当前网络多次卡住，而用户确认本机代理和 Docker 已恢复；宿主代理监听
  `127.0.0.1:10808`，普通 Docker build 网络中的回环并不指向宿主。
- 自主裁决：compose 的 build 使用 `network: host`；可选环境变量
  `RT_CONTROL_BUILD_PROXY` 只映射到 Docker/BuildKit 预定义的 `HTTP_PROXY/HTTPS_PROXY` build args，
  Dockerfile 不声明同名 `ARG`，避免代理值进入镜像历史。仅 `vcs import` 那个 RUN 继承代理；apt、
  IgH fetch/build、rosdep 和 colcon 的 RUN 均先显式清除大小写 HTTP(S)/ALL_PROXY。仓库、镜像 ENV、
  runtime compose 和生成文档中均不写代理地址或凭据。
- Benefit：当前 GitHub 源码拉取可稳定完成，同时代理配置保持为构建机临时输入，不进入生产镜像或
  远程仓库。
- Drawback：BuildKit 构建阶段获得宿主网络可见性，隔离弱于默认 bridge；源码拉取仍依赖操作者
  正确提供本机代理，且代理服务能观察该阶段访问的三个公开 Git 仓库。

## BQ-084 — 生产镜像排除测试依赖与测试目标 [RESOLVED 2026-07-22]

- 状态：**RESOLVED**
- 自主裁决：Docker 中先用 `colcon list --packages-up-to` 得到生产包闭包，只对这些路径执行
  `rosdep install`，dependency types 限于 build/buildtool/export/exec；colcon 统一传
  `-DBUILD_TESTING=OFF`。规则 16 允许的 ICube 安全补丁最小上游测试已在独立固定源码工作区运行，
  不把 test/doc/lint 工具链带入最终镜像。
- Benefit：生产镜像依赖面和构建时间下降，严格遵守“不建设测试体系”的冻结范围；实际 runtime
  包闭包仍由 rosdep 解析而非人工漏列。
- Drawback：生产 Docker build 本身不会发现仅在上游测试目标中发生的编译回归；补丁升级时必须在
  镜像外重新执行相应的最小安全测试并留证。

## BQ-085 — JTC admission 快照竞争采用有界拒绝 [RESOLVED 2026-07-22]

- 状态：**RESOLVED**
- 证据：JTC 的 FJT goal callback 在非 RT executor 中读取 RT `update()` 写入的 13 轴位置和反馈
  age。seqlock 风格快照若使用无限循环，在 update 恰好写到奇数序列后停止或极端争用时可能永久
  阻塞 action goal callback。
- 自主裁决：goal callback 最多尝试 1000 次取得序列号前后相同且为偶数的快照；未取得就以
  `FJT_REJECT reason=feedback_snapshot_unavailable` 拒绝。成功快照仍必须满足 age <= 500 ms 且
  每轴第一点误差 <= 1°。不在 RT 路径增加锁、日志、等待或分配。
- Benefit：任何控制环停顿都不会把 FJT 准入线程永久挂死；失败是明确的拒绝，旧轨迹不会被接受。
- Drawback：在极端调度争用下，即使下一瞬间就能获得一致快照，也可能产生一次保守的假拒绝；上层
  必须基于新状态重新发送目标，而不是假设 action server 会排队等待。

## BQ-086 — Humble launch 的 xacro 输出必须显式按字符串参数传递 [RESOLVED 2026-07-22]

- 状态：**RESOLVED**
- 证据：生产 xacro 单独展开并通过 `check_urdf`，但首次 mock 启动在 launch 参数装配阶段失败：
  `launch_ros` 尝试把完整 URDF 当作 YAML 解析，并提示字符串参数应显式声明类型。
- 自主裁决：只在 `rt_control.launch.py` 中用 Humble 提供的 `ParameterValue(..., value_type=str)` 包装
  `Command(xacro ...)` 的结果，不修改 URDF、控制器参数或硬件选择逻辑。
- Benefit：消除不同 XML 内容触发 YAML 隐式解析的歧义，真实和 mock 启动共用同一条确定的字符串参数路径。
- Drawback：launch 文件增加一个 Humble API 依赖；升级 ROS 发行版时需确认该 API 的兼容性，但没有
  控制周期运行时开销。

## BQ-087 — 生产容器 PID 1 必须是失能包装脚本 [RESOLVED 2026-07-22]

- 状态：**RESOLVED — HIGH-RISK（生产停止信号链）**
- 证据：首次 mock 停止验证中，镜像默认命令是 `ros2 run rt_control_bringup rt_control_start`；容器
  PID 1 因而是 ROS CLI 启动器，不是安装后的 Bash 包装脚本。Docker SIGTERM 未触发包装脚本的 trap，
  40 s 后容器被 SIGKILL，退出码为 137，日志中也没有 `/rt/disable` 结果。
- 自主裁决：Docker CMD 直接执行 merge-install 下的
  `/opt/rt_control_ws/install/lib/rt_control_bringup/rt_control_start`。entrypoint 最终 `exec` 该文件，
  使包装脚本成为 PID 1；调试时也必须直接运行该已安装文件，不再用 `ros2 run` 包一层。
- Benefit：INT/TERM 直接进入包装脚本，先完成有界 `/rt/disable`，再向 launch 进程组转发 SIGINT；Docker
  的 `stop_grace_period` 因此覆盖真正的硬件失能窗口。
- Drawback：CMD 与当前 `/opt/rt_control_ws/install` merge-install 路径绑定；若未来改变工作区安装前缀或
  包可执行文件布局，镜像会在启动时明确失败并需要同步修改。本路径不是硬件断电保证，实机终态仍由
  T-013 验证。

## BQ-088 — 上游补丁载荷的空白行不做机械改写 [RESOLVED 2026-07-22]

- 状态：**RESOLVED**
- 证据：`git diff --cached --check` 会把 ICube 补丁中表示上游源文件空白行的 `+ ` 报为 trailing
  whitespace；这些字符属于已在冻结提交上生成、应用、编译和测试过的补丁载荷，补丁 SHA-256 也已写入审计记录。
- 自主裁决：仓库自身源代码继续执行 `git diff --check`；补丁目录从该机械空白检查中排除，改用冻结上游
  HEAD、`git apply --check` 和已记录 SHA-256 三项联合校验。不为满足样式检查重写补丁载荷。
- Benefit：保留已验证补丁的逐字身份和可追溯哈希，同时仍对实际仓库源代码执行严格空白检查。
- Drawback：对整个暂存区直接运行无排除项的 `git diff --check` 会返回非零；审阅者必须使用文档中的分层
  命令，且未来重新生成补丁时仍需人工区分载荷空白与仓库源代码空白。

## BQ-089 — Compose 必须以仓库根目录作为 project directory [RESOLVED 2026-07-22]

- 状态：**RESOLVED**
- 证据：从仓库根直接执行 `docker compose -f docker/compose.yaml ...` 时，当前 Compose 将首个 compose
  文件所在的 `docker/` 作为默认 project directory，因而把 `dockerfile: docker/rt-control/Dockerfile`
  错解为 `docker/docker/rt-control/Dockerfile`；失败发生在构建上下文解析阶段，没有执行镜像步骤。
- 自主裁决：生产构建、config 和 up/down 统一使用 `tools/rt_control_compose.sh`；该冻结包装器显式传入
  `--project-directory .`、`--env-file versions.env`，并从 Git HEAD 生成镜像标签。不再把裸 `-f` 命令
  作为受支持入口。
- Benefit：无论调用者当前目录和 Compose 默认规则如何，context、Dockerfile、版本文件和镜像标签都绑定
  到同一仓库根目录。
- Drawback：操作者必须经过包装器并预先提供已在目标机验证的 `RT_CONTROL_CPUSET`；绕过包装器的临时调试
  命令需要自行完整复现 project-directory、env-file 和标签参数。

## BQ-090 — T-009 实机内核与 EtherCAT MAC 覆盖冻结初值 [RESOLVED 2026-07-23]

- 状态：**RESOLVED — 用户明确批准的实机覆盖**。
- 证据：`alfa-two` 实机运行 Ubuntu Pro `6.8.1-1056-realtime`，`/sys/kernel/realtime=1`；
  I210 `enp3s0` 的固化 MAC 为 `8c:59:3c:15:01:f8`。原 REQ-RT-002/REQ-ECAT-008 写的是
  5.15-rt 与旧 MAC `8c:59:3c:14:ff:d3`，在本机均不成立。用户已明确批准 HWE 6.8 RT 与新 MAC。
- Decision：只对 T-009 主机实装值应用上述覆盖；master ID 0、`enp3s0` 专用且无 IP/DHCP/DDS、
  IgH stable-1.6+ec_igb 与其余总线约束不变。旧值保留在原始证据文档中，不静默改写。
- Benefit：部署绑定实际硬件和已成功启动的受支持 Ubuntu Pro RT 包，不会把主站指向不存在的 MAC。
- Drawback：偏离原冻结主机版本，所有 out-of-tree 模块必须针对 6.8 RT 单独编译验证；换机不得复用该 MAC。

## BQ-091 — `alfa-two` 隔离 CPU 与 SMT 策略 [RESOLVED 2026-07-23]

- 状态：**RESOLVED — HIGH-RISK（启动参数/调度拓扑）**。
- 证据：i7-14700 的 CPU 0-15 是八个双线程 P 核，CPU 16-27 是十二个单线程 E 核；
  CPU 14/15 同属 P-core 7。当前 cmdline 无 isolcpus/nohz_full/rcu_nocbs/irqaffinity。
- Decision：控制容器 cpuset 固定 CPU 14；启动参数采用
  `nosmt isolcpus=domain,managed_irq,14 nohz_full=14 rcu_nocbs=14 irqaffinity=0-13,15-27`；
  Controller Manager 使用已批准的 PROVISIONAL `SCHED_FIFO 80`。CPU 15 及其他 SMT 次线程由
  `nosmt` 下线，避免同物理核争用。
- Benefit：控制环独占完整 P 核，普通调度、IRQ、RCU callback 和 SMT sibling 均不与其竞争。
- Drawback：全机少八个 SMT 逻辑线程且 CPU 14 不供普通工作负载使用；需重启生效，换 CPU/BIOS 后必须重探测。

## BQ-092 — NVIDIA 595-open 强制 RT 构建与 PCIe 错误隔离 [RESOLVED/REVISED 2026-07-24]

- 状态：**RESOLVED — HIGH-RISK（官方不支持的 RT GPU + 实机 PCIe 异常）**。
- 证据：595 闭源和开源 DKMS 均以“does not support real-time kernels”拒绝 PREEMPT_RT；
  版本限定覆盖与 GCC 12 可成功构建 595-open，`nvidia-smi` 正常识别 RTX A2000。但加载后 AER
  correctable 总数约 80486174，主要为 RxErr，并含 BadTLP/BadDLLP。首次卸载模块后 5 秒增量为 0，
  但重启后在 NVIDIA 模块未加载时仍复现：GPU 位于 D0/Gen4 x8，3 秒新增 10414 个 correctable，
  HDMI 音频 function 仍绑定 `snd_hda_intel`。因此“仅卸载/屏蔽 NVIDIA 驱动即可止错”的早期结论
  已被实测推翻，驱动不是根因。
- Decision：保留 `/etc/dkms/nvidia-595.71.05.conf` 供可审计重建，但以
  `/etc/modprobe.d/nvidia-rt-quarantine.conf` 阻止 NVIDIA 自动/直接加载；GDM 使用 Intel i915。
  另外用 `gpu-pcie-quarantine.service` 在启动时严格校验 `01:00.1=10de:2291`、
  `01:00.0=10de:25b8`；全 PCI sysfs 预扫发现任一目标 ID 移到其他 BDF 时 fail closed，并校验独占上游
  根端口 `00:01.0=8086:a70d`，bus 01 出现任何非预期 child 同样拒绝执行。物理重新插拔后的复测推翻了
  “只移除两个 endpoint 足以止错”的早期观测：两个 endpoint 已从 sysfs 消失时，根端口仍持续收到 AER；
  改为移除经过完整拓扑校验的独占根端口后，AER 与日志增长才同时停止。EtherCAT、SSH、GDM/i915 均保持正常。
  不用 `pci=noaer` 或关闭 AER 隐藏问题。
  物理链路/供电检查和受控 Gen3 复测通过前不得解除两层隔离。
- 启动时序修正：首次持久服务验证虽然最终摘除设备，但因错误地排在 `systemd-udev-settle` 之后，启动早期
  仍产生 389349 条 AER 日志，属于不合格证据。unit 改为 `DefaultDependencies=no`、由 `sysinit.target`
  拉起、仅等待根文件系统 remount，并排在 `systemd-udev-trigger` 之前；这样在任何 udev 驱动绑定前先完成
  全 PCI ID/BDF 校验和移除。第二次重启中 unit 在 userspace 约 0.47 s 开始、107 ms 完成，启动时间由
  约 86 s 降至 52 s，移除后根端口 15 s 增量为 0；但该 boot 仍产生 127072 条 AER 日志，证明错误已在
  PCI 枚举/内核早期大量形成。服务保留为部分 containment，**不得据此宣称 GPU/RT 主机生产就绪**；
  必须断电检查插接/载板/供电，或 BIOS 强制 Gen3 后重测。用户态继续提前已无可信收益，且仍禁止
  `pci=noaer` 掩盖。重新插拔后的受控 PCI rescan 仍以 Gen4 x8 建链，2 秒产生 2360 条 AER 匹配日志，
  明确为 correctable Physical Layer `RxErr`，因此插拔动作未修复链路；下一硬件试验必须是 BIOS 固定 Gen3
  后冷启动复测，或交叉更换载板/插槽/显卡。
- 后续用户报告已更换接口/Gen3 后的冷启动，实机仍枚举为同一 `00:01.0 -> 01:00.x` 拓扑；启动隔离前产生
  2462 条 AER 匹配日志。受控 rescan 显示 GPU `LnkCap=16GT/s x16`、`LnkSta=16GT/s x8`，根端口
  `LnkCap=32GT/s x16`、`LnkSta=16GT/s x8`，因此实际仍是 **Gen4 x8**，不是 Gen3。短窗口继续产生
  2295 条 correctable Physical Layer `RxErr`，VGA 与 HDMI-audio function 均有报告，随后根端口重新隔离。
  本次证据只说明更换动作未改变可见 BDF/协商代际，**不得误写成“Gen3 也失败”**；只有现场读到
  `LnkSta: Speed 8GT/s` 后，才开始正式 Gen3 AER 验收。
- 2026-07-24 用户最终裁决：**本台 rt-control 工控机不再使用 NVIDIA GPU**。因此取消 Gen3、驱动加载、
  CUDA 与 GPU-load 验收，不再以“修复后可用 GPU”作为 T-009 前提；但这不等于允许带故障卡参加最终 RT
  验收。断电物理拆除前继续保留 modprobe 与根端口两层隔离，只允许在确认 `00:01.0` 已摘除后进行受控
  EtherCAT/CAN bring-up；最终 30 分钟 250 Hz/cyclictest 必须在故障卡物理移除、冷启动 NVIDIA 链路 AER
  为零后执行。Benefit：彻底消除已知 AER 风暴和官方不支持的 NVIDIA+PREEMPT_RT 组合对实时/日志的影响。
  Drawback：本 IPC 永久不提供 CUDA、NVIDIA 显示或 HDMI 音频；需要 GPU 的感知负载必须迁移到其他硬件。
- Benefit：真正停止会污染日志和实时性的错误风暴，同时保留已安装驱动及明确恢复路径；设备 ID 校验可避免
  PCI 地址变化时误删其他设备。
- Drawback：隔离期间无 NVIDIA 计算、HDMI 音频和独显输出；服务依赖本机固定 BDF，目标 ID 出现在其他 BDF
  时会失败而不猜测；
  即使 PCIe 修复，595+PREEMPT_RT 仍不受 NVIDIA 官方支持，必须以 idle/GPU-load cyclictest 对照决定
  是否可用于生产。重新插拔后的 endpoint-only 隔离失效使当前 boot 出现 8112545 条 AER 匹配记录，
  `syslog`/`kern.log` 各增至约 82 GB、`/var/log` 共 169 GiB。用户明确要求删除测试产生的大日志后，已在
  根端口止错条件下清空活动文件、删除八个超过 100 MB 的 AER 膨胀轮转文件，并执行 journal vacuum。
  随后用真实 PCI rescan 验证新版完整拓扑校验/根端口移除脚本：service exit 0、根端口与 NVIDIA endpoint
  均消失，短暂枚举产生的 1655 条 AER 匹配日志也已清理。最终 journal 占 40 MB、`/var/log` 占 602 MB，
  根分区可用 371 GB（17% used），活动日志 5 秒无增长。代价是混入这些文件的旧日志也被删除，故关键诊断
  数字保留在本裁决记录中。

## BQ-093 — `alfa-two` Docker 来源、权限与受限网络部署 [RESOLVED 2026-07-23]

- 状态：**RESOLVED — 自主部署裁决（安全边界已记录）**。
- 证据：实机为 Docker 官方支持的 Ubuntu 22.04 amd64，安装前无 `docker.io`、`containerd`、`runc`
  等冲突包，UFW 未启用。工控机直连 `download.docker.com` 被重置，Docker Hub 请求超时；WSL
  `127.0.0.1:10808` 可访问官方源。官方文档要求生产机使用签名 apt repository，而 convenience script
  只建议测试/开发使用。
- Decision：使用 Docker 官方签名 apt repository，安装精确实测版本 Docker CE 29.6.2、containerd 2.2.6、
  Buildx 0.35.0、Compose 5.3.1；不使用 convenience script，也不执行系统整体升级/自动清理。下载时仅建立
  工控机 `127.0.0.1:18080` 到 WSL 代理的临时 SSH reverse forward，完成后关闭，不写入 apt、daemon、
  镜像或仓库。保持 rootful Docker，但不把 `alfa` 加入等同 root 权限的 `docker` 组，生产操作显式使用
  `sudo`。通过 FIFO+SSH 流式导入已验证镜像，不落大 tar 文件。对 Docker CE/CLI、containerd、Buildx、
  Compose 和 rootless extras 设置可逆 apt hold；维护升级必须先 unhold，完成复测后重新 hold。
- Benefit：软件来源、签名、确切版本和回滚包边界可审计；没有持久代理或扩大的本地用户权限；离线导入的
  image ID 与 WSL 完全一致。
- Drawback：`alfa` 不能直接操作 Docker socket，命令需 `sudo`；工控机尚不能直接从 Docker Hub 拉取，
  后续升级/新镜像仍需临时受控传输。Docker 发布端口会改变防火墙路径，未来若新增端口必须在
  `DOCKER-USER` 链单独审查；当前 Compose 使用 host network 但未声明发布端口。apt hold 同时阻止自动获取
  Docker 安全更新，需纳入显式维护窗口，不能无限期不审查。

## BQ-094 — IgH 安装脚本在 `ec_igb` 已接管 I210 后的幂等校验 [RESOLVED 2026-07-23]

- 状态：**RESOLVED — 实机复查发现的脚本缺陷修正**。
- 证据：首次启动成功后 `ec_igb` 接管 I210，普通 netdev `enp3s0` 不再出现在 `/sys/class/net`；旧脚本
  仅从该路径读 MAC，因此合法复跑会在构建前误报接口缺失。当前主站仍能给出精确
  `Main: 8c:59:3c:15:01:f8 (attached)`，且 `/etc/ethercat.conf` 保存同一 MAC。
- Decision：首次安装继续以 netdev MAC 精确匹配且无 IPv4/IPv6 为门禁；接口已消失时，只有
  `ethercat.service` active、既有 CLI 可执行、配置行逐字匹配、主站报告同一 MAC attached 四项同时成立，
  才认定是 `ec_igb` 合法接管并允许复建。脚本不为幂等校验自动停止主站；显式 `--start` 才在安装末尾
  重启服务，已接管路径跳过无意义的 `nmcli device set`。
- Benefit：内核更新后的重复构建不再误失败，同时不会为了检查而中断可能在线的 EtherCAT 主站，也不会把
  “接口消失”宽松解释成任意硬件状态。
- Drawback：已接管路径依赖既有 IgH CLI 与配置作为交叉证据；若旧安装损坏到无法提供四项证据，脚本会
  fail closed，需要维护窗口中先恢复 netdev/服务，而不会自行卸载模块猜测处理。

## BQ-095 — CPU governor 在预备延迟通过后的保留策略 [RESOLVED 2026-07-23]

- 状态：**RESOLVED — 自主实时调优裁决**。
- 证据：保持实机默认 `intel_pstate`、`powersave` governor、`balance_performance` EPP，不修改 C-state 或
  RT runtime；CPU14 上以 FIFO90、1 ms interval、mlockall 运行 30 分钟宿主 preliminary cyclictest，
  1,800,000 周期结果为 Min 1 / Avg 4 / Max 18 µs，低于 100 µs 门槛；期间无调度节流、RCU stall、
  lockup、AER 或内核 warning。
- Decision：保留默认 governor/EPP，不为进一步压低预备数字强制全机/单核 performance，也不改 C-state
  和 `sched_rt_runtime_us`。该结果只证明空闲宿主基线，不能替代实总线 250 Hz 控制空跑的最终 30 分钟验收。
- Benefit：已经满足基线门槛，同时减少常驻功耗、热量和因温度降频带来的长期不确定性，避免引入规格外启动参数。
- Drawback：真实感知/控制负载下的最坏延迟仍未知；若最终联合验收超过 100 µs，必须基于 trace/IRQ/热数据
  重新裁决，不能把本次 18 µs 当作生产保证。

## BQ-096 — 最终“容器内 cyclictest”工具注入方式 [RESOLVED 2026-07-23]

- 状态：**RESOLVED — 不修改冻结生产镜像/Compose 的验收工具方案**。
- 证据：生产镜像未内置 `cyclictest`，但已有兼容 `libnuma`；宿主提供 rt-tests 2.2-1 / cyclictest 2.20。
  以同一生产 image ID 启动一次性诊断容器，只读绑定 `/usr/bin/cyclictest`、映射
  `/dev/cpu_dma_latency`、CPU14、FIFO90、`SYS_NICE`/`IPC_LOCK` 与相同 ulimit，5 秒 5,000 周期结果
  Min 1 / Avg 3 / Max 12 µs，并确认 PM QoS 设为 0 µs。
- Decision：最终联合验收使用上述独立诊断容器，与 250 Hz 生产容器同时运行；不向生产镜像安装 rt-tests，
  不给冻结 Compose 增加诊断 device/volume。证据必须同时记录生产 image ID、宿主 rt-tests 包版本和完整命令。
- Benefit：验证发生在容器调度/namespace 路径中，又不污染生产镜像或永久扩大其设备权限；工具可独立移除。
- Drawback：工具二进制来自宿主而非镜像供应链，且 FIFO90 的诊断线程会短暂抢占优先级80的控制环；这正是
  联合延迟压力的一部分，但最终结果必须和 250 Hz overrun/总线状态一起解释。

## BQ-097 — CANable 2.0 的 SLCAN commissioning 与生产 `gs_usb` 路径 [RESOLVED 2026-07-24]

- 状态：**RESOLVED — HIGH-RISK（需现场刷写适配器固件，尚未执行）**。
- 证据：实机 USB `16d0:117e`、序列号 `208031C05230` 运行 CANable 2.0 原厂 SLCAN 固件；现场手工进程为
  `slcand -o -c -s6 /dev/ttyACM0 can0`。官方 CANable 命令表确认 `S6=500 kbit/s`；被动监听收到
  `0x701/0x702/0x703/0x714`，数据均为 `0x7F`，SocketCAN 错误计数和 TX 均为 0。其中 `0x714` 是权威映射
  已登记的 Node 20 Pitch 只读诊断心跳，不进入生产配置。SLCAN 在适配器内设置 bit timing，Linux 虚拟
  interface 因而报告 `bitrate 0`，且手工 daemon 随桌面会话存在；这不满足 REQ-CAN-001/REQ-DEP-003 的
  宿主 systemd 和 `bitrate 500000` 可核验证据。
- Decision：SLCAN 仅允许继续做不发帧的 commissioning 观察，不改冻结 `hostsetup/can0.service` 去迁就它。
  生产路径采用厂家官方支持的 CANable 2.0 candleLight 固件，使设备由内核原生 `gs_usb` 驱动；RT 内核
  `6.8.1-1056-realtime` 已验证存在 signed in-tree `gs_usb.ko` 且 `CONFIG_CAN_GS_USB=m`。刷写后继续使用
  冻结 unit 通过 netlink 设置 500000，并重新验证设备固定命名、`ip -details`、心跳和零错误。刷写需要现场
  BOOT 按键/拔插配合，当前尚未执行；不得把本次 SLCAN 观察写成 T-009 完成。
- Benefit：回到原生 SocketCAN/systemd/可观测 bitrate 的冻结架构，绕过 `slcand` userspace 串行转发，
  官方说明在高负载下性能更好。
- Drawback：固件刷写有中断和恢复风险，需要现场物理操作；CANable 2.0 candleLight 不支持 CAN-FD，但本项目
  仅使用经典 CAN。依据：`https://canable.io/getting-started.html`、
  `https://github.com/normaldotcom/canable2-fw`。

## BQ-098 — low-latency 迁移后的 NVIDIA PCIe 隔离解除门禁 [BLOCKED 2026-07-24]

- 状态：**BLOCKED — HIGH-RISK（需要用户明确批准一次无隔离启动）**。
- 新裁决与旧裁决关系：用户随后明确选择 Ubuntu HWE low-latency 迁移以恢复 NVIDIA 能力，这覆盖 BQ-092
  中“本 IPC 永久不使用 GPU”的软件部署范围；但它没有证明此前的 Gen4 x8 Physical Layer `RxErr` 已修复，
  也没有自动授权解除为防止 169 GiB 日志膨胀而建立的硬件隔离。
- 已完成证据：移除损坏的 `nvidia-dkms-535`/`nvidia-driver-535` 后，安装并一次性启动
  `6.8.0-136-lowlatency`；`CONFIG_PREEMPT=y` 且无 `CONFIG_PREEMPT_RT`，CPU14 隔离参数原样继承，
  realtime `6.8.1-1056` 与 generic `6.8.0-134` 回退内核均保留。IgH stable-1.6 冻结提交
  `2f7f884f1c7d377c02a7d627eb06512126a0e50e` 已针对新 ABI 重建，`ec_igb` 报告主设备
  `8c:59:3c:15:01:f8 (attached)`；当时现场链路为 DOWN、从站 0 个。安装了 Ubuntu 仓库的
  `nvidia-driver-595-open=595.84-0ubuntu0.22.04.1` 与精确 ABI 的
  `linux-modules-nvidia-595-open-6.8.0-136-lowlatency`，APT 模拟和实装均未引入 DKMS；模块 vermagic 与
  当前内核一致。NVIDIA 官方说明 Turing 及更新架构应使用 open 模块，CUDA 13.2 Update 1 要求驱动至少
  595.58.03，因此 595.84 满足 RTX A2000/CUDA 13.2 软件条件。
- 接口名现场修正：移除旧 ABI 的 `ec_igb` 占用后，权威 MAC `8c:59:3c:15:01:f8` 在 sysfs 唯一对应
  `enp2s0`；`enp3s0` 实际 MAC 为 `8c:59:3c:15:01:f9`。旧记录把 f8 与 `enp3s0` 并列是误判。
  `hostsetup/igh-install.sh` 改为按权威 MAC 唯一解析当前 netdev 名称，发现零个或多个匹配均 fail closed；
  `/etc/ethercat.conf` 仍只以冻结的 `MASTER0_DEVICE` MAC 绑定，不依赖可能随枚举变化的接口名。
- 未通过门禁：`gpu-pcie-quarantine.service` 与 `/etc/modprobe.d/nvidia-rt-quarantine.conf` 仍启用。
  当前内核的受控全局 PCI rescan 没有重新枚举已摘除的 `00:01.0` 根端口，因此 AER 无新增但也未见 GPU，
  该结果只能判定为“未枚举”，不能判定硬件恢复。必须通过一次禁用根端口隔离的重启才能验证
  `lspci`、协商代际、AER 增量和 `nvidia-smi`。
- 用户批准与收紧执行：用户明确批准一次无隔离测试启动。为避免 SSH 恢复前重现持续风暴，实际采用更窄的
  一次性 sysinit 测试窗口：保持 NVIDIA modprobe 禁载，在原隔离服务时序中先保留设备 2 秒、记录 BDF/
  link/AER，再自动调用原脚本摘除根端口并删除测试 drop-in。Benefit：测试窗口有硬上限且自动恢复；
  Drawback：只能采集早期物理层证据，不能在同一次启动直接运行 `nvidia-smi`。
- 测试结果：`6.8.0-136-lowlatency` 于 21:01 启动，但测试包装器第一次读取时
  `00:01.0/01:00.0/01:00.1` 就全部不存在，2 秒后仍不存在；AER 匹配数保持 `4 -> 4`、增量 0。
  包装器成功报告 `root_port_removed=yes` 并自删除 drop-in，日志未失控（`/var/log` 约 855 MiB、journal
  约 288 MiB），系统与 IgH 无 failed unit。该结果是“设备未枚举”，不是“链路通过”；软重启没有恢复此前
  已从 sysfs 摘除的根端口，故不能解除 modprobe 禁载或测试 `nvidia-smi`。
- 剩余问题：RTX A2000 当前是否仍物理安装？若已拆除，本轮 GPU 迁移在硬件门禁处终止；若仍安装，需要现场
  完整断电冷启动（不是 `systemctl reboot`）后重跑同一有界窗口，才能判断物理链路/AER。
- 用户纠正与冷启动证据：用户说明 21:01 测试时显卡实际未安装，随后断电插入并冷启动。默认 GRUB 回到
  `6.8.1-1056-realtime`；隔离服务执行前本次 boot 记录 2,850 条 AER 匹配信息，仍为
  `01:00.0` correctable Physical Layer `RxErr`，随后根端口被成功摘除。插卡还触发 BIOS 自动图形策略：
  冷启动后的 PCI 树完全没有原先的 Intel `00:02.0`，`i915` 无设备可加载；NVIDIA 又被隔离，导致系统最终
  没有任何 DRM GPU。`/dev/dri/card0` 是无 sysfs 后端的 `root:root 0600` 残留节点，GDM/Xorg 因此报
  `Permission denied`、`no primary bus or device found` 与 `no screens found`。这解释了图形界面失败，
  不是解除 modprobe 禁载的理由。
- 新物理门禁：先在 AMI BIOS `RXE26005` 中把集成显卡从 Auto 改为强制 Enabled（常见名称为 Internal
  Graphics/IGD/iGPU Multi-Monitor），并把主显示设为 IGD/IGFX、显示器接主板接口；确切菜单名称由该 OEM
  BIOS 决定，若现场页面不同必须提供照片，不能猜测。只有冷启动后重新出现 `00:02.0`、`i915` 和有效
  DRM 节点，才允许恢复 GDM。NVIDIA 隔离保持不变，因为本次冷启动已经复现硬件错误。
- 暂行决定：确认硬件状态并完成必要冷启动前，不解除 modprobe 隔离、不把 low-latency 写成永久 GRUB 默认，
  也不宣称 GPU 可用。
- Benefit：内核、IgH 与 NVIDIA 软件栈已经可回退地准备完成，同时避免未经授权重现已知 PCIe 风暴。
- Drawback：当前启动是一次性 low-latency；下一次普通重启仍回到 realtime，且隔离期间 `nvidia-smi` 必然
  不可用。完整迁移和 GPU/负载延迟验收仍未完成。

## BQ-099 — T-009 更换到原冻结硬件主机后的覆盖边界 [RESOLVED 2026-07-24]

- 状态：**RESOLVED — 用户授权配置新主机后的实施裁决**。
- 证据：新目标 `ar-Default-string`（`192.168.0.40`）实际运行
  `5.15.0-1032-realtime`，EtherCAT I210 MAC 为冻结值
  `8c:59:3c:14:ff:d3`，15 个从站全部可见；CPU 同为 i7-14700，CPU 14/15
  是一个完整 P-core。RTX 2000 Ada 正常工作且无 AER/Xid。BQ-090、BQ-092、
  BQ-098 的 6.8/new-MAC/GPU-quarantine 证据均明确属于已放弃的 `alfa-two`。
- Decision：新主机恢复原 REQ-RT-002/REQ-ECAT-008 的 5.15 PREEMPT_RT 和
  `8c:59:3c:14:ff:d3`，仍用固定 IgH stable-1.6 提交与 `ec_igb`。同拓扑下
  生产 cpuset 选 CPU 14，并采用已批准的 `nosmt` whole-core 参数；原主机已有的
  C-state=1 参数只保留、不由本任务新增。`alfa-two` 的 GPU 隔离脚本不安装到新机，
  但专有 NVIDIA 模块下的最终 GPU/MoveIt 联合延迟仍必须实测。
- Benefit：主机重新与原冻结内核和硬件身份对齐，避免把 `alfa-two` 的异常与覆盖
  污染到健康主机；CPU14 具有完整物理核隔离。
- Drawback：换机使先前 `alfa-two` 的 Docker/IgH/延迟证据不能直接充当本机验收；
  `nosmt` 会移除八个逻辑线程，NVIDIA 专有模块仍会 taint 实时内核。

## BQ-100 — 两只同型号 CANable 的生产接口身份 [RESOLVED 2026-07-24]

- 状态：**RESOLVED — 实机枚举事实下的 fail-closed 绑定**。
- 证据：两只 `1d50:606f` CANable 2.0 均由 `gs_usb` 驱动。当前 `can0` 序列号
  `004D00675230500720333159` 正在收发 rt-control Node 1/2/3 数据；当前 `can1`
  序列号 `003000265230500720333159` 由现有 BMS 节点使用。仅靠内核枚举顺序会在
  换 USB 口或启动时序变化后交换名称。
- Decision：用一次性 systemd 命名服务按两个精确 USB 序列号事务式重命名：前者
  固定 `can0`，后者固定 `can1`，再由冻结 `can0.service` 设置 500 kbit/s。任一
  设备缺失、重复或序列号变化均拒绝启动，不猜测另一只适配器。命名过程保留执行前
  的 UP/DOWN 状态，避免实装检查额外改变 BMS 链路状态。
- Benefit：彻底消除同型号适配器枚举交换导致向错误物理总线发控制帧的风险。
- Drawback：两个指定适配器成为启动依赖；维护更换 CANable 后必须人工更新并重新
  审核序列号，不能自动接受新硬件。

## BQ-101 — Jammy AppArmor 不识别 Cursor 的 `userns` 规则 [RESOLVED 2026-07-24]

- 状态：**RESOLVED — 宿主安全服务修复**。
- 证据：`apparmor.service` 唯一失败原因是
  `/etc/apparmor.d/cursor-sandbox:11` 的 `userns,`；Ubuntu 22.04 自带解析器
  报 `unexpected TOK_END_OF_RULE`，导致整次 profile reload 非零。该 profile 自
  2026-01-23 起未成功加载，而不是 rt-control/Docker 新引入的问题。
- Decision：备份原 profile，只注释 Jammy 不支持的 `userns,` 一行并保留其余规则，
  以 `apparmor_parser` 解析成功和 `apparmor.service` active 为门禁。不禁用整个
  AppArmor 服务，也不为 rt-control 添加宽松 profile。
- Benefit：恢复全机 AppArmor profile 的正常加载，优于继续容忍失败服务或整体禁用。
- Drawback：Cursor profile 在本机不包含较新 AppArmor 的 user-namespace 专用规则；
  升级到支持该语法的系统后需要重新评估，而不能永久沿用注释。

## BQ-102 — IgH 1.6 默认允许实时上下文 syslog [RESOLVED 2026-07-24]

- 状态：**RESOLVED — REQ-RT-004 的直接实施约束**。
- 证据：固定 IgH 1.6.10 提交的 `configure` 默认报告
  `whether to syslog in realtime context... yes`，并在 `master/domain.c`、
  `master/master.c` 和 datagram 路径通过 `EC_RT_SYSLOG` 编译实时日志。冻结规则 9
  明确禁止 RT read/update/write 路径日志；宿主模块无需实时日志才能提供已有的非 RT
  启动/状态变化证据。
- Decision：宿主 IgH 构建显式使用 `--disable-rt-syslog`，容器用户态库继续保持同一
  1.6.10 ABI/提交。首次发现默认值时尚未进行任何运动或激活应用；随后立即用该开关
  重建并重启空闲主站，以最终模块配置为验收对象。
- Benefit：消除内核实时数据路径调用 syslog 的延迟和分配风险，直接满足 REQ-RT-004。
- Drawback：少数只在 `EC_RT_SYSLOG` 下产生的运行期详细日志不可用；故障定位依赖已冻结
  的主站状态、WC/diagnostics 与 commissioning CLI，而不能临时在生产模块中打开实时日志。

## BQ-103 — Docker 安装脚本复跑的网络与包升级边界 [RESOLVED 2026-07-24]

- 状态：**RESOLVED — 实机幂等性复跑发现并收紧**。
- 证据：Docker 已按固定指纹和版本成功安装后，第一版脚本复跑仍重新下载官方 GPG key；本机到
  `download.docker.com` 的 TLS 连接瞬时 reset，使脚本退出。此前置步骤还因普通
  `apt-get install` 语义把已安装的 `curl`、`libcurl4`、`libcurl4-openssl-dev` 从 Ubuntu
  `.23` 升到 `.25`。Docker、EtherCAT、CAN 和启动配置均未改变，但这证明原脚本不满足“复跑不扩大变更”。
- Decision：若 `/etc/apt/keyrings/docker.asc` 已存在，则离线读取并逐字核对冻结指纹后复用，绝不为复跑
  再下载；仅在文件缺失时通过 TLS 获取。前置包安装增加 `--no-upgrade`，Docker 六包继续精确版本安装和
  `apt-mark hold`。修正版实机复跑退出 0，零升级、零新装、零卸载。已有 RealSense 源缺 key
  `FB0B24895113F120` 的 apt 警告不属于 rt-control，不越权修复。
- Benefit：断续网络下已配置主机仍可验证并复用可信 key，维护复跑不会顺带升级宿主依赖；固定版本与 hold
  继续阻止未验漂移。
- Drawback：Docker 官方未来轮换签名 key 或发布安全更新时脚本会 fail closed，必须人工审核并更新冻结指纹/
  版本；本次意外发生的三个 Ubuntu curl 包升级不可无损回滚，已作为部署事实保留。

## BQ-104 — T-009 的 250 Hz 空跑不得提前激活真实 CANopen 硬件 [RESOLVED 2026-07-24]

- 状态：**RESOLVED — 按任务安全门禁收窄为同镜像 mock 空跑**。
- 证据：固定 ros2_canopen 补丁中的 `Cia402System::on_activate()` 会按 Node 1/2/3 调用
  `init_motor()`、写运行模式并预装当前位置/零速；因此即使不调用 `/rt/enable`，直接启动默认生产 launch
  也会进入真实 CAN 驱动激活流程。这已跨过 T-009“非使能启动”边界，而真实 EtherCAT/CAN 使能、无跳变和
  PP/PV 行为分别属于 T-013/T-014。
- Decision：T-009 的 §14 ⑦ 使用**同一生产镜像**、同一 CPU14 cpuset、host IPC/network、
  `CAP_SYS_NICE`/`CAP_IPC_LOCK` 和 RT ulimit，但 launch 显式传
  `use_mock_hardware:=true`，ROS Domain 改为 142，且不映射 `/dev/EtherCAT0`。并行 cyclictest 仍以
  FIFO90 给 250 Hz/FIFO80 控制环施压。默认生产 Compose 只创建并检查安全字段，保持 `created`、从未启动。
- Benefit：覆盖真实镜像、Controller Manager 频率/优先级、容器调度和宿主延迟路径，同时物理上不能访问
  EtherCAT 设备，也不会实例化 ros2_canopen 真实插件，避免越过带电门禁。
- Drawback：mock 结果不能证明 PDO/SYNC/WC、真实驱动激活、自动预装载或机械无跳变；这些证据仍必须在
  T-013/T-014 按急停和支撑条件完成，不能把 T-009 soak 写成最终实总线验收。

## BQ-105 — T-009 soak 启动窗口中工控机三个网络地址同时失联 [RESOLVED 2026-07-24]

- 状态：**RESOLVED — 操作者确认是现场误断电，不是软件、内核或自动休眠故障**。
- 证据：最后确认的生产容器状态是 `created` 且从未启动。发送 mock soak 命令后，既有 SSH 会话停止返回，
  Wi-Fi `192.168.0.40`、I219 `192.168.1.5` 和辅助 I210 `192.168.1.104` 均无 ARP/ICMP；同一 WSL 到
  网关和公网正常。向已知 I219/I210/Wi-Fi MAC 多次发送 WoL 后仍未恢复。由于失联前命令回显未返回，当前
  不能证明 mock/cyclictest 容器是否真正创建或开始运行，也不能把本轮计为 soak。
- 已知安全边界：mock 命令未映射 `/dev/EtherCAT0`，使用 `use_mock_hardware:=true`；生产容器在最后可观测点
  仍是 `created`。未调用 `/rt/enable`，未发布 FJT/updown/履带命令。
- 恢复结果：冷启动后运行 `6.8.1-1056-realtime` 且 `/sys/kernel/realtime=1`；CPU14 隔离参数、systemd、
  Docker、EtherCAT、CAN 和 `hostsetup/verify-host.sh` 全部恢复通过。当前 boot 无 OOM、panic、watchdog、
  suspend 或 NVIDIA Xid。`last -x` 的前一会话中断与操作者确认的断电相符。Docker 证明 mock 与 cyclictest
  容器从未创建，生产容器在该事件前仍为 `created`，因此没有遗留测试负载可清理。
- Decision：把该失联归因为已确认的外部误断电；不修改休眠、内核、网络或容器配置，不把未开始的窗口计入
  soak。T-009 的 30 分钟 mock+cyclictest 验收仍须另行完整执行。
- Benefit：恢复检查排除了可见的软件崩溃和遗留负载，同时避免因一次已知误断电扩大宿主配置变更。
- Drawback：断电前没有可用的 mock soak 数据；T-009 仍不能仅凭本次恢复检查宣告完成。

## BQ-106 — ros2_canopen 动态 Master 组件未进入生产镜像构建闭包 [RESOLVED 2026-07-24]

- 状态：**RESOLVED — 补齐已冻结上游实现所需的运行组件，不改变总线配置或协议语义**。
- 证据：首次真实生产启动在 `DeviceContainer` 解析现有 `bus.yml` 后终止，原始异常为
  `Could not find requested resource in ament index`。启动日志同时明确显示请求的插件是
  `package: canopen_master_driver`、`driver: ros2_canopen::MasterDriver`。当时镜像只显式构建
  `canopen_ros2_control` 及其静态依赖；`canopen_master_driver` 是由配置在运行时动态加载的 sibling package，
  不在 `--packages-up-to canopen_ros2_control` 的依赖闭包中。故障发生在硬件组件加载和激活之前：EtherCAT
  保持 `Idle / Active:no`，CAN 节点保持 PRE-OP，未调用 `/rt/enable`，也未发生运动。
- Decision：把固定上游源码中的 `canopen_master_driver` 加入 Dockerfile 现有
  `rt_control_packages` 白名单，让 rosdep 和 colcon 同时覆盖该官方运行组件；保持 `bus.yml`、DCF、PDO/SDO、
  NMT、CiA402 握手和所有超时值原样。重建后先检查 ament 索引和 package prefix，再进行一次生产启动复测。
- Benefit：DeviceContainer 可按已批准的 ros2_canopen 标准架构加载 Master；修复范围只覆盖缺失的运行依赖，
  不引入自研主站或协议分支。
- Drawback：生产镜像增加一个上游包及其少量体积和构建时间；该修复只能证明组件可加载，真实 NMT/PDO/EMCY
  与机械无跳变仍须 T-013/T-014 实机证据。

## BQ-107 — Lely 初始化无条件修改 CAN TX queue 需要容器 NET_ADMIN [RESOLVED 2026-07-24]

- 状态：**RESOLVED — 用户批准采用 Lely 自身默认 `txqueuelen=128`，宿主预配置且容器不扩权**。
- 证据：Master 组件补齐后的首次启动已成功实例化 `ros2_canopen::MasterDriver`，随后在
  `lely::io::CanController("can0")` 抛出 `Operation not permitted`。同一生产安全上下文可创建并 bind
  `PF_CAN/SOCK_RAW`，内核无 AppArmor/seccomp DENIED；直接调用固定镜像的
  `io_can_ctrl_create_from_name("can0", 0)` 稳定返回 `errno=1`。固定 Lely 源码把参数 0 展开为默认
  `LELY_IO_CAN_TXLEN=128`，然后无条件调用 `io_if_set_txqlen()`；实机 `can0` 当时为 10。该 netlink 写需要
  `CAP_NET_ADMIN`，而生产容器只有默认 capability 加已冻结的 `IPC_LOCK`/`SYS_NICE`，BQ-007 明确禁止扩大
  device/capability 权限。失败前 CAN 节点始终 PRE-OP、TX=0，EtherCAT 保持 `Idle / Active:no`，未使能或运动。
- Decision：用户批准使用上游自身默认队列长度 128。`can0.service` 在接口拉起前以宿主 root 设置
  `txqueuelen 128`；固定 Lely 仅增加窄兼容补丁：先读取当前值，已经等于 128 时跳过重复写，不一致时仍执行
  原上游设置并在无权限下 fail closed。正式容器不增加 `NET_ADMIN`，不改 host network、bitrate、PDO/SDO、
  NMT 或 CiA402 行为。补丁同时固定到现有 Lely commit，并做 apply-check。
- Benefit：保留非 privileged 和最小 capability 边界；队列长度仍严格等于上游原本会设置的 128，避免沿用
  宿主默认 10 带来的高负载 TX 排队不足，同时启动对宿主漏配保持失败可见。
- Drawback（重点）：新增一个固定 Lely 源码补丁和宿主/容器联合前置条件；升级 ros2_canopen/Lely 时必须重核
  补丁。128 会允许更多待发 CAN 帧排队，突发拥塞时最坏排队时延高于 10；本系统 50 Hz、经典 CAN 500 kbit/s
  下接受该上游默认取舍。选择不授予 `NET_ADMIN` 也意味着容器不能自行修复错误的宿主队列配置。

## BQ-108 — 节点驱动 Boot 等待误落入 ros2_canopen 的 20 ms 默认值 [RESOLVED 2026-07-25]

- 状态：**RESOLVED — 按用户授权的后续自主实施裁决，并由两轮实机抓包收敛到主站同值窗口**。
- 证据：生产镜像已成功打开 `can0` 并加载 Master；实机 Node 2 收到 NMT reset 后回传
  `0x702#00`，并正常响应 `0x1000`/`0x1018` SDO。CAN 始终 ERROR-ACTIVE、总线错误计数为 0，EtherCAT
  保持 `Idle / Active:no`。但三个 Boot attempt 分别只等待约 20 ms 后中止。固定 ros2_canopen 源码证明
  `master.boot_timeout`（默认 2000 ms）与设备 `boot_timeout_ms` 相互独立；后者未配置时默认 20 ms。实测该节点
  从 reset 到 Boot-up 约 103 ms，故 20 ms 必然早退。固定上游的 CiA402 测试配置已显式使用
  `boot_timeout_ms: 1000`，冻结 REQ-CAN-004 同时规定单 transition 上限为 1 s。
- 首轮修正与反证：先按上游 CiA402 测试配置及冻结的 1 s transition 数值试用
  `boot_timeout_ms: 1000`。镜像构建和参数核对通过，但实机仍严格每 1000 ms 超时。完整抓包证明 Node 2
  的 `0x1000`、四个 `0x1018` 身份子项、`0x1016` 和 `0x1017` 交换均成功且无 SDO abort；每次 NMT reset
  到 Boot-up 约 120 ms，而生产 heartbeat 周期本身为 1000 ms。Lely 的 Boot 完成还在等待下一帧 PRE-OP
  heartbeat，因此 1000 ms 窗口会在该心跳前约 120 ms 必然到期。冻结的 1 s deadline 属于后续 CiA402
  transition，不是 NMT Boot 总窗口，不能混用。
- Final decision：在 `bus.yml` 的设备 defaults 中显式设置 `boot_timeout_ms: 2000`，与 ros2_canopen Master
  的默认 2000 ms Boot 窗口一致，给 Node 1/2/3 留出一个完整 1000 ms heartbeat 周期及 reset/config 余量。
  保持 SDO 500 ms、三次 Boot attempt、NMT reset、心跳 1000/5000 ms 和所有 CiA402/PDO 语义不变。先做
  配置生成/构建检查，再重新启动；只有三个节点均 Boot 成功且现场状态无异常时才继续 `/rt/enable`。
- Benefit：等待条件与实际生产 heartbeat 时序及主站既有窗口一致，仍会在有界时间内 fail closed；实机报文已
  排除总线错误、身份 SDO 失败和心跳配置写失败。
- Drawback（重点）：真正不响应的节点每次 Boot attempt 最多等待 2 s，三次尝试会令单节点失败路径增加到约
  6 s；若多个节点依次失败，整体启动诊断会相应变慢。该值解决的是 NMT Boot 时序，不证明驱动器 PDO、
  CiA402 或机械行为正确，后续实机门禁仍必须逐项通过。

## BQ-109 — 工控机直接构建的本机代理适配 [SUPERSEDED 2026-07-25]

- 状态：**SUPERSEDED — 不再为本次工控机代理修改生产 Dockerfile，改为开发机构建后传输镜像**。
- 证据：工控机 `mihomo` 在 `127.0.0.1:7897` 提供 HTTP/mixed proxy，经它访问 Docker Registry 返回标准
  认证挑战；`10808` 未监听。未设代理时，Docker Registry 的 DNS/IPv6 路径超时或被 reset。临时给 daemon
  和 buildx 客户端注入 7897 后 Registry 元数据成功，但 Dockerfile 的四个 `RUN` 均首先执行
  `unset HTTP_PROXY HTTPS_PROXY ...`，使 apt/git 继续直连；ROS 包下载因此极慢。Compose 已把显式
  `RT_CONTROL_BUILD_PROXY` 作为 Docker 预定义的 `HTTP_PROXY`/`HTTPS_PROXY` build args 传入，且 build
  network 已是 host，容器可访问宿主 `127.0.0.1:7897`。
- Final decision：撤销 Dockerfile 的代理语义调整，恢复所有构建 `RUN` 主动清空代理变量的原行为；本次在
  WSL 开发机按最终 feature commit 构建、核验，再以压缩镜像流经 SSH 传到工控机。工控机临时注入的 Docker
  daemon 代理已经 `unset-environment` 并重启，`docker info` 的三项代理字段均为空。
- Benefit：生产 Dockerfile 不再承载某台工控机的网络规避逻辑，构建路径保持冻结实现之前已验证的简单行为，
  同时避开工控机代理的随机 502。
- Drawback（重点）：每次镜像变化需要在开发机重新构建并传输较大的镜像归档，占用额外时间、带宽和两端磁盘；
  工控机原地构建暂不作为本次 commissioning 的可靠路径。

## BQ-110 — 为工控机代理增加下载重试工具 [REJECTED 2026-07-25]

- 状态：**REJECTED — 实验和反证保留，但不把专用重试工具留在实现中**。
- 证据：`b25578f` 在工控机直接构建的首轮下载中，80 个 ROS/Ubuntu 包里有 4 个经
  `127.0.0.1:7897` 返回 502；保持源码、固定依赖和命令不变重试后，首轮失败的 4 个包均成功，另一个 ROS
  包随机返回 502。两轮都在第一条 `apt-get install` 结束，未进入源码编译，符合瞬时代理/上游错误而非固定
  包损坏。
- 修正证据：首次实现采用 apt 原生 `Acquire::Retries "5"`，但第三轮实机构建中单个 ROS 包返回 502 后 apt
  仍直接退出，证明该配置未把当前代理的 502 判为可重试错误；不得把“已配置”误记作“已生效”。同一失败包
  随后用 curl 经 HTTP 代理连续三次返回 200；ROS 仓库的 HTTPS 证书与 `packages.ros.org` 主机名不匹配，
  因此不能通过跳过证书校验强改 HTTPS。
- Final decision：撤销 apt 原生重试配置和 `rt-control-retry-command` 方案，不增加脚本、不包装 rosdep/apt；按
  BQ-109 改走开发机本地构建和镜像传输。该决定不改变任何运行时代码、依赖版本或硬件控制语义。
- Benefit：避免为了单机网络故障增加并维护自定义下载控制层，Dockerfile 回到既有、容易审计的路径。
- Drawback（重点）：若以后再次要求工控机直接构建，偶发 502 仍可能让构建失败，需要从基础网络或代理服务
  侧解决；本次不声称已经修复工控机的代理可靠性。

## BQ-111 — 全量构建受 GitHub 直连阻断时复用已验证镜像 [RESOLVED 2026-07-25]

- 状态：**RESOLVED — 仅对唯一运行差异做离线增量重建，并保留基镜像与生成物核验证据**。
- 证据：开发机按最终源码全量构建时，基础系统、IgH 和前置层均命中缓存，但三个固定 GitHub 仓库在既有
  `vcs --retry 5` 后分别以 GnuTLS `-110` 或 443 timeout 失败。`7de7d99` 已验证镜像 ID 为
  `sha256:143af46296c8526d121b20a62df880ecf17e7767effc85c7b84db64e9fc1a34b`。从该提交到当前提交，Markdown
  被 `.dockerignore` 排除，唯一进入运行镜像的源码差异是 `robot_hw_canopen/config/bus.yml` 将三节点
  `boot_timeout_ms` 从 1000 改为 2000；Dockerfile 最终内容与 `7de7d99` 相同。
- 修正证据：首个离线派生试验在生成目标后执行整个包的 `cmake --install`，全安装前缀 1633 个文件的哈希
  比较发现，除预期 bus.yml 外还改写了 `robot_hw_canopen/package.dsv`，删除了原有环境 hook 条目。该试验镜像
  因存在未授权运行环境差异而拒绝上传。
- Final decision：以该固定镜像 ID 为基础，离线覆盖镜像内的 bus.yml 源文件，调用镜像已有的
  `canopen_runtime_config` CMake target 重新运行 EDS normalization、cogen 与 dcfgen；随后只把 CMake 中
  `CANOPEN_GENERATED_FILES` 对应的 8 个运行文件以 `copy_if_different` 更新到安装前缀，不执行整包 install。
  以最终 feature commit 标记派生镜像。传输前核对源码哈希、安装前缀三节点的 2000 ms 值、生成 DCF/bin
  文件、依赖版本和全安装前缀哈希差异。该方法仅适用于这次已证明的单配置差异，任何 C++、依赖或其他配置
  变化都必须恢复全量构建。
- Benefit：完全离线复用已编译且已通过上一轮镜像检查的代码和固定依赖，只重跑真正受影响的标准生成目标，
  避免为部署网络增加生产代码或跳过配置生成。
- Drawback（重点）：这不是从空缓存开始的全量重建，可信性依赖固定基镜像及“唯一运行差异”的审计结论；
  因此必须把基镜像 ID、提交间文件差异和派生层核验一起归档，不能把该流程泛化为后续发布方式。

## BQ-112 — LD2 的 0x6060 周期 PDO 缓存、Homing 宣告与 Lely 析构线程 [RESOLVED 2026-07-25]

- 状态：**RESOLVED — 按用户授权的后续自主裁决收敛，并完成一次不下发运动命令的实机启停复测**。
- 证据：只调用上游 `set_operation_mode()` 时，驱动器短暂进入 Node 1 PP / Node 2、3 PV，随后 Lely
  已映射 RPDO 的本地对象字典仍以默认值 `0` 周期发送 `0x6060`，把运行模式覆盖回 No Mode；LD2 在已使能
  状态下因此报 `0x603F=0x5201`（Er870，不支持的控制模式）。实机 `0x6502` 为只读
  `0x0003002D`，仍宣告 Homing，不能按 BQ-005 原计划通过启动写或驱动器持久参数清除。上游
  `Motor402::handleShutdown()` 又先切换 No Mode、再进入 Switch On Disabled，会在正常停机重现同一错误。
  最后，`Motor402` 的模式对象持有 `LelyDriverBridge` 的最后一个共享引用；若直到 ROS 信号线程析构才释放，
  Lely 会触发 `ev_fiber_exec_fini` 的线程归属断言。
- Final decision：保留 Node ID 到模式的 fail-closed 固定映射并在非实时硬件激活时严格校验：Node 1 只能为
  PP=`1`，Node 2、3 只能为 PV=`3`，且必须恰有这三个节点。在调用上游 `init_motor()` 前，使用已有
  `tpdo_transmit()` 路径把 `0x6060` 的 1/3/3 写入相应已映射 RPDO 缓存；不增加 PDO、不运行时重映射、
  不增加 SDO 回读门禁。由于 `0x6502` 只读，在该固定系统中只禁止 `Motor402` 自动分配/执行 Homing，
  其余标准模式检测不变。正常停机保留当前固定模式，只执行上游标准 CiA402 状态转换到
  Switch On Disabled，随后全节点 NMT Stop；不再先写 `0x6060=0`。在 driver cleanup 阶段、主站 event loop
  尚存活时，把 `Motor402` 投递到同一个 Lely executor 线程释放，再清理 master。
- 与既有裁决的关系：本裁决只取代 BQ-005/BQ-046/BQ-047 中“Motor402 完全不修改”、通过驱动器清除
  Homing 宣告，以及停机先进入 No Mode 的部分。CiA402 controlword/statusword 标准转换、PP bit-12 握手、
  20 ms 安全目标处理窗口、心跳/EMCY 的上游归属均保持不变。
- 实机复测：启动日志记录 Node 1/2/3 缓存分别预置 1/3/3；抓包 `0x281/0x282/0x283` 的
  `0x6061` 分别回报 1/3/3，周期 `0x401/0x402/0x403` 继续携带 1/3/3，履带目标持续为零，未出现
  非零 EMCY。`docker stop --timeout 60` 在
  1.903 s 返回、容器 exit 0；三节点随后至少连续 5 s 回报 heartbeat state `0x04`（NMT Stopped），
  CAN 保持 ERROR-ACTIVE，bus-error/error-warning/error-passive/bus-off 和 dropped 计数均无新增，且不再出现
  Lely 析构断言。证据位于工控机
  `/var/lib/rt-control/commissioning/mode-sdo-debug-20260725/thread-cleanup-*`；candump 与本轮 Docker 日志
  SHA-256 分别为 `a43b8f832e2a061c07ebef99861535f3b48c4725e855e391efd84bf8bbfffb34`、
  `1e346d5924779cb91c238bd38be049bf669f7834e715b4b81aceb4ae52322f79`。
- Benefit：模式值与每个物理 Node 确定绑定，周期 PDO 不会再把模式覆盖为 0；不增加 CAN 帧种类或 50 Hz
  周期负载；正常停机不再制造 Er870，且 Lely 对象在其要求的线程内销毁，容器能正常退出。
- Drawback（重点）：固定 ros2_canopen overlay 扩大到三个很窄但必须长期 rebase 的兼容点；软件不再把
  `0x6502` 的 Homing 位当作本机可执行能力的唯一真相。Node ID/用途变更会 fail closed，必须修改并重验映射。
  本轮仅证明无外部命令下的激活、保持和清理，不证明 updown PP 打点速度/bit-12、履带方向/比例，或真实
  heartbeat/EMCY 丢失时的机械停车反应；这些仍属于 T-014 未完成的带支撑实机验收。

## BQ-113 — EtherCAT 预装载窗口早于完整过程数据 [RESOLVED 2026-07-25]

- 状态：**RESOLVED — HIGH-RISK（真实 13 轴启动门禁）**。
- 证据：原 ICube `on_activate()` 在 master activate 后立即启动 5 s 预装载期限；本机 13 个已配置运动从站
  首次达到 AL=OP 和 Domain `39/39` 需要约 26 s。原路径因此可能在 WC 不完整、`0x6064` 仍是缓存值时耗尽
  预装载期限，或者错误地用陈旧反馈授权后续使能。环位置 13 是未配置的 hub，正常保持 PREOP，不能把全环
  15 个位置都要求为 OP。
- 自主裁决：给 ICube overlay 增加必填 `startup_bus_timeout_ms=70000`。硬件激活先循环现有
  `master_.update()`，要求 13 个已配置 slave handle 全部 AL=OP 且 `processDataAgeMs()` 为有限值（即至少一次
  `EC_WC_COMPLETE`）；超时直接返回 ERROR。满足后才单独启动原 `preload_timeout_ms=5000`，保持 raw
  `0x6064 -> 0x607A` 和后续接管语义不变。不配置或猜测位置 0/13，不新增 PDO/SDO/state interface。
- 实机结果：run 6/run 7 均先打印 `All configured EtherCAT slaves are OP with a complete working counter`，
  再完成系统激活；复位、五批使能和标准失能均返回 success，运行过程 Domain 为 `39/39`。
- Benefit：预装载证明来自完整且新鲜的过程数据，失败保持有界并 fail closed；不把非运动 hub 强行改为 OP。
- Drawback（重点）：断线或从站异常时，启动失败最多延迟 70 s 才显现；`on_activate()` 的既有阻塞初始化窗口
  相应变长。该门禁只解决首次完整数据，不解决 BQ-114 的控制循环接管同步波动。

## BQ-114 — ICube 激活循环交接时的一次性 DC/WC 同步级联 [RESOLVED/HIGH-RISK 2026-07-25]

- 状态：**RESOLVED/HIGH-RISK MITIGATION — 已实机验证，不表述为 DC/application-time 根因修复，T-013 仍受低速运动验收约束**。
- 证据：BQ-113 门禁确认全部已配置从站 OP/WC complete 后，`on_activate()` 返回并由 controller-manager 的
  `read()/update()/write()` 循环接管。此时 IgH 仍会依次报告多个从站 AL `0x001A Synchronization error`，
  Domain 短暂降到不完整 WC；run 7 轮询依次见 `27/39 -> 16/39 -> 34/39 -> 39/39`，随后保持完整。累积
  `wc_error_count` 增长后停止增长。增加 5 s “返回前稳定窗口”的 0003 实验补丁没有阻止交接后的级联，故已
  删除，正式实现只保留 BQ-113 的 0002。
- 伴随现象：四个 Ti5 的 PDO mapping clear/write 返回只读对象 abort `0x06010002`，但内核同时打印当前
  `0x1601={6040,607A}`、`0x1A01={6041,6064}` 与冻结目标完全一致。当前不改 mapping、不屏蔽内核告警。
- 用户批准的缓解：在 8 个共享从站 profile 的启动 SDO 中把 `0x10F1:02` 写为 100。250 Hz 下这是约 400 ms
  的连续同步错误计数容差；9 个 ZeroErr 和 4 个 Ti5 的实机只读上传均确认当前值为 100。保持冻结 DC、PDO、
  250 Hz 和 BQ-113 完整 WC 门禁，不增加固定 sleep，也不隐藏内核 AL/WC 错误。
- 迁移门审计：`diff_legacy.py` 仅将上述 `0x10F1:02=100` 作为 BQ-114 批准的列表头部 overlay；ZeroErr
  必须为 `uint16`、Ti5 必须为 ESI 对齐的 `uint32`。其他 SDO 顺序、类型或数值差异仍然失败。纳入该 overlay
  后，冻结基线比较覆盖 13 个逻辑轴和 `joint_limits.yaml` 共 813 个语义值。
- 独立退出修正：保留 ICube `stop()` 只置 `running_=false`，因为它可由周期循环 callback 调用，不能加入阻塞
  主站操作；新增非实时 `EcMaster::deactivate(5000)`，只由 `EthercatDriver::on_deactivate()` 调用。该路径先
  `ecrt_master_deactivate()`，每 20 ms 轮询 13 个已配置运动从站并要求精确 PREOP，超时返回 lifecycle ERROR；
  只有随后析构才 `ecrt_release_master()`。这与生产退出日志中 resource_manager 先调用 hardware deactivate、
  再 shutdown 的既有顺序一致。
- 实机复测：工控机镜像
  `sha256:b8cc0ad9691e9fa34062338501ef1b26d9f8de6cf66eda62fca9802664a96cc5` 连续两轮分别在约
  49 s、52 s 达到 13 个配置位置 OP 和 `39/39` 后才完成激活；从启动到停用、release 的内核
  `0x001A/Synchronization error` 计数均为 0。两次 lifecycle 停用命令分别在 1.37 s、1.05 s 返回，驱动日志中
  `Stopping` 到 `successfully stopped` 均约 41 ms，随后 master inactive、全环 15 个位置 PREOP，SIGINT 后
  容器 exit 0。证据与清单位于工控机
  `/var/lib/rt-control/validation/run-14-bq114-400ms-gated-ecat-only` 和
  `/var/lib/rt-control/validation/run-15-bq114-400ms-gated-ecat-only-repeat`；清单 SHA-256 分别为
  `248da7d6c29ccd742c0db62d4efd06f432cb2044f5c69f593c3185ed0c795624`、
  `85f411497829e7794079991dc6087fec5b8915507e70119d4650e8b7e43a9dbb`。
- Benefit：无需修改 IgH 内核或冻结 DC/PDO，即可跨过已观测的一次性交接波动；完整 WC 门禁仍 fail closed；
  退出明确执行 deactivate→PREOP→release，不再依赖未运行的 ICube `run()` 线程隐式收尾。
- Drawback（重点）：100 是“连续错误周期计数”，400 ms 只是 250 Hz 下的 nominal 换算，不是独立墙钟看门狗；
  小于该窗口的真实同步扰动会延后升级为 `0x001A`，因此这是容差扩大而非根因修复。5 s PREOP 等待会使异常退出
  最多增加 5 s，且失败会令 lifecycle 返回 ERROR。本轮为 EtherCAT-only（CAN component 保持 unconfigured）、
  无使能无运动复测；低速轨迹、turn jog、长期 DC 稳定性和完整生产栈退出仍属于 T-013/T-015 验收。
- 2026-07-27 supersession：以上 `100/400 ms` 是保留不改写的历史实机证据。用户在 BQ-120 复核后明确批准
  新构建把 `0x10F1:02` 提高到 `250`，即 4 ms 周期下 nominal `1000 ms`；其收益、代价和未闭合边界以
  BQ-120 的最新裁决为准。

## BQ-115 — Ti5 对 0x0000 的非激磁终态停留在 Ready To Switch On [RESOLVED 2026-07-25]

- 状态：**RESOLVED — HIGH-RISK HARDWARE EXCEPTION；不得表述为 literal REQ-SAFE-002 已闭环**。
- 证据：四个 Ti5 在 Fault Reset 清除 `0x7500` 后，PDO 持续写 `0x0000`，只读 SDO 同时确认
  `0x6040=0x0000`、`0x603F=0`，但 statusword 保持 Ready To Switch On；额外试验持续写 `0x0002` 也不改变
  状态。相同序列的九个 ZeroErr 均到 Switch On Disabled。供应的 Ti5 协议手册把 Ready To Switch On 定义为
  电机未激磁，实机无命令保持曲线也未见跳变。
- 自主裁决：常规失能、启动 sanitize 和全组 reset 仍对全轴写标准 `0x0000`。只有逻辑轴
  `right_joint2/right_joint3/left_joint2/left_joint3` 可把 Ready To Switch On 作为“已确认非激磁”终态；其余
  九轴仍严格要求 Switch On Disabled。使能序列不变，仍从 Ready 按 `0x0007 -> 0x000F` 进入 Operation
  Enabled；不向 Ti5 增加私有 SDO/PDO 或自动 fault reset。
- 与冻结要求的差异：REQ-SAFE-002、BQ-030/031/032/053/080 中 literal `0x0040` 的终态，只对这四个已实证
  Ti5 收窄为上述非激磁终态。T-013 因此保持 partial，最终安全验收必须把此硬件例外人工签字，不得以 service
  success 隐去差异。
- 余留故障：master release 后四个 Ti5 均以 `0x6040=0x0000` 进入 Fault，`0x603F=0x7500`；下一次启动仍需
  显式 `/rt/reset_fault`。建议在驱动器侧审查 EtherCAT 通讯丢失/主站释放反应映射。好处是有机会消除每次正常
  关停后的虚假 Fault 和复位步骤；弊端是改变驱动器本地失联安全反应，若配置过松会削弱断网保护，因此必须由
  厂商协议/现场断链试验确认，主站现在不猜测也不自动修改。
- Benefit：服务结果与驱动器真实、无激磁的终态一致，不再因永远到不了的 `0x0040` 误超时；四轴仍保持
  controlword `0x0000`，九个 ZeroErr 的严格确认未放宽。
- Drawback（重点）：这是显式硬件兼容例外，并保留启动前人工 reset；若 Ti5 固件对 Ready 状态的扭矩语义与
  手册不一致，软件本身无法检测。最终必须以驱动器参数表和机械断链/急停验收补证。

## BQ-116 — 成功恢复后 diagnostics 仍展示上一轮失败 [RESOLVED 2026-07-25]

- 状态：**RESOLVED — 最小字段清理并完成单次实机复测**。
- 证据：run 6 的 reset/enable/disable service 均返回 success，但 IDLE/ENABLED 的
  `/robot/rt_control/enable_manager` 仍显示旧的 `fault_requires_reset/right_joint2/4616`。原因是
  `recordFailure()` 只在失败时写原子字段，任何成功终态都没有清理它们；硬件状态机本身已成功。
- 自主裁决：新增只写既有原子字段的 `clearFailure()`，在 controller activate 以及 reset、enable、disable 的
  成功/幂等成功路径清为 `success/-1/empty/0`。不增加 topic、接口、锁或动态分配；失败路径和 service 返回值
  不变。
- 实机复测：run 7 的 reset、enable、disable 全部 `ok=true`；ENABLED 和后续 IDLE diagnostics 都为
  `failed_batch=-1`、空 `failed_joint`、`failed_status_word=0`、`stage=success`。Domain 保持 `39/39`；使能保持
  最大反馈波动 0.016479°，失能后 0.004807°；容器 exit 0。
- Benefit：当前健康状态不会被历史故障伪装成活动故障，rqt/上层监控可直接按当前 FSM 判断。
- Drawback：diagnostics 不再承担“最后一次历史故障”存储；成功恢复后历史值被清除。完整历史仍保留在 Docker
  日志和 commissioning 证据中，若未来需要持久故障史应由独立黑匣子需求实现，不能塞回当前实时控制器。

## BQ-117 — Ti5 `0x10F1:02` ESI 类型与实机上传长度不一致 [OPEN/HIGH-RISK 2026-07-25]

- 状态：**OPEN/HIGH-RISK COMPATIBILITY — 不阻塞已验证的值 100 写入，但阻塞宣称 Ti5 对象类型已闭环**。
- 不一致：供应的 `Ti5Robot_JointMotor_2.0.xml` 把 `0x10F1:02 Sync Error Counter Limit` 声明为
  `UDINT/32 bit`，所以 Ti5 YAML 使用 `uint32`；同一实机四个 Ti5 的 SDO upload 却只返回 2 bytes，指定
  `uint32` 读取报 `Data type mismatch`，指定 `uint16` 可读得 `0x0064/100`。ZeroErr ESI 与实机都为 UINT16。
- 当前决策：本轮不擅自把 Ti5 startup SDO 从制造商 ESI 的 `uint32` 改成 `uint16`。实机启动没有该对象的 SDO
  abort，四轴均读回 100，故保留已验证写入；在 Ti5 固件/ESI 版本得到厂商确认前，把类型差异作为显式兼容风险。
- Benefit：不根据一次上传行为静默推翻供应商对象字典，且当前 100 的配置效果已实证。
- Drawback（重点）：Ti5 固件与 ESI 至少一方不一致；更换固件/批次后 4-byte 写入可能被严格拒绝。发布验收应
  固定固件版本，并向厂商确认 `0x10F1:02` 的真实宽度后再统一 ESI 与 YAML。
- 2026-07-27 update：新配置值已按用户批准改为 `250`，但 Ti5 仍保留 ESI 对齐的 `uint32` 类型。历史实机只证明
  `100` 的启动写入没有 abort；因此必须在新镜像首次启动时确认四台 Ti5 均无该 SDO abort，并以 `uint16`
  只读上传得到 `250`。完成前 BQ-117 继续 OPEN，不能把“250 可写”当作已验证事实。
- 2026-07-27 first-image gate result：镜像 `rt-control:4fc8414f67b63bf3a1c4fb4c34eb27fe8caafc9d`
  首次启动没有 `0x10F1:02` startup abort；全部 14 个运动从站（九台 ZeroErr、四台 Ti5、XMC）均以
  `uint16` 只读上传得到 `250`，随后完整进入 OP/WC-complete。由此关闭“新值是否真正写入当前实机”的门禁，
  但不关闭 Ti5 ESI `uint32` 与实机上传 2-byte 的类型矛盾；换固件/批次前仍必须取得厂商确认。

## BQ-118 — Updown 从 CANopen PP 迁移为 EtherCAT CSP [RESOLVED/HIGH-RISK 2026-07-27]

- 状态：**RESOLVED/HIGH-RISK — 用户完成三轮接口、机械和故障语义裁决，并指定按实机 SW 5.11 固定 PDO 生成**。
- 现场权威证据：环位置 15 的从站身份为 Vendor `0x0000034E`、Product
  `0x00445566`、Revision `0x00000000`，设备名 `XMC_ESC`，HW `1.0`，SW `5.11`；启用驱动器参数
  `sysPRM.EtherCATEnable=ON` 并执行 IgH rescan 后进入 PREOP。实机 SII 声明 `PdoAssign=false`、
  `PdoConfig=false`，固定 RxPDO `0x1600` 为 `6040:16,6071:16,60FF:32,607A:32,6081:32,
  6060:8,2302:16`（19 bytes），固定 TxPDO `0x1A00` 为 `6041:16,603F:16,6078:16,
  606C:32,6064:32,6061:8,6000:8,2300:16,2301:16,60FD:32`（24 bytes）。供应商 XML
  SHA-256 为 `4c295d61e87675652e9ba4df2b8e0970cd6bd7a4cbeae9927d04202add0a2b46`，但其中
  RxPDO 末项为 `0x607F:uint32` 且 TxPDO 多出 `0x6077:int16`，分别形成 21/26 bytes，故不得作为
  SW 5.11 运行映射覆盖现场 SII。
- 用户裁决：完整删除 CANopen Node 1、旧 `updown_position` controller、`/updown/command` 和
  `UpdownCommand.msg`；Updown 改为环位置 15 的 4 ms CSP，从 13 轴 JTC 扩为 14 轴，固定顺序为
  `right_joint1..6,left_joint1..6,turn,updown`，继续使用
  `/dual_arm_jtc/follow_joint_trajectory` 且拒绝 partial goal。Updown 加入 14 轴整体使能、失能、
  全组 reset 和任一轴 Fault 全组停车，第五批与 Turn 同时使能。旋转轴第一点容差保持精确 1 degree
  (`0.017453292519943295 rad`)，Updown 为 `0.05 m`，反馈年龄上限 `500 ms`。
- 机械与数值：用户及供应商确认 65536 counts/rev、无减速比、丝杆导程 10 mm/rev、`0x6064`
  向上为正、raw 0 对应 0 m、机械范围 `[0.0,0.8] m`、多圈绝对值且掉电保持，抱闸由驱动器管理。
  因此比例固定为 `6553600 counts/m` 与 `0.000000152587890625 m/count`；目标最大速度
  `0.3 m/s`、最大加/减速度 `0.5 m/s^2`，不增加 jerk 约束或自研插补器。
- 启动与同步：保持 DC `AssignActivate=0x0300`、250 Hz；启动写 `0x60C2:01=4`、
  `0x60C2:02=-3`、`0x6060=8`。沿用 BQ-114 的 `0x10F1:02=100`，即 4 ms 周期下约 400 ms
  同步错误计数容差。固定 RPDO 内的 `0x6060` 每周期保持 CSP=8，其他非控制字段写保守常量 0，
  不新增业务 command topic；固定 TPDO 全量注册但只向 ros2_control 暴露 position/statusword。
- 与旧裁决关系：本裁决按原 spec 已预留的“更换 EtherCAT 电机后并入同一 FJT”接缝，取代 BQ-002、
  BQ-019、BQ-046、BQ-051、BQ-052、BQ-112 及冻结 REQ-CAN-002/003/005/006、REQ-IF-007 中仅与
  CANopen Node 1 PP Updown 有关的部分；两条履带 Node 2/3、标准 ros2_canopen 心跳/EMCY 和配对停车保持不变。
- Benefit：Updown 与双臂/Turn 共用同一 250 Hz 官方 JTC 插补与 EtherCAT 周期，删除跨总线 PP 打点、
  6081 速度和专用 topic 的双轨语义；固定 PDO 与实机 SW 5.11 字节布局逐项一致，周期模式不会回落为 0。
- Drawback（重点）：供应商 XML 与实机固件映射不一致，升级固件或换驱动器时必须重新读取 SII 并逐项复核，
  不能只替换 XML；14 轴任一轴故障会扩大为全组停机。`0.3 m/s`、`0.5 m/s^2` 是轨迹生成/验收上限，
  Humble JTC 本身不会替代 motion 的时间参数化或机械安全链。本裁决当时只完成只读识别；后续 T-019 已证明启动
  SDO、OP、预装载和第五批使能，但没有运动，并新发现 BQ-119/BQ-120，必须解决后才可进入低速空载运动门禁。
- 2026-07-27 supersession：上述 `100/400 ms` 记录的是 T-019 首轮所用镜像，不代表新构建值。新构建统一使用
  `0x10F1:02=250`（nominal 1000 ms），不改变 XMC 固定 PDO、CSP=8、4 ms 周期或任何控制接口。
- 2026-07-27 minimal-motion authorization and result：用户确认现场有硬件急停，当前关节位姿任意方向小幅运动
  均无碰撞，并批准 13 个旋转轴各单程 `0.5 degree`、Updown 单程 `0.05 m`、每单程 `2 s`。执行选择
  完整 14 轴 goal、一次只改变一轴、朝限位区间中点外移后回程；好处是首次运动能量低且故障可定位，弊端是
  不覆盖反方向首次外移、多轴耦合和同时运动负载。28/28 段 action 成功；Updown 最大移动轴跟踪/终点误差为
  `0.00184546718425/0.000252685546875 m`。这只关闭首轮最小 CSP/FJT 运动门禁，不关闭动态精度、故障注入或
  生产轨迹验收；完整记录见 `docs/fjt-14axis-low-speed-commissioning-20260727.md`。

## BQ-119 — 两节点 ros2_canopen 退出存在 polling callback use-after-free [RESOLVED 2026-07-27]

- Evidence：T-019 首次 14 轴使能测试中 `/rt/disable` 已成功，随后 controller manager deactivate
  `canopen_mobile_axes`。清理 `left_track_joint` 时，MultiThreadedExecutor 的并发 timer 仍在执行
  `NodeCanopen402Driver::poll_timer_callback()`，最终在已经失效的
  `LelyDriverBridge::get_id()` 访问地址 `0x2d8` 并 SIGSEGV。`ros2_control_node` exit `-11`，外层容器却 exit 0。
- Impact：CANopen cleanup 抢先崩溃使 EtherCAT hardware 没有进入正常 deactivate；主站被进程释放时 14 个运动从站
  出现 `0x001B Sync manager watchdog`，最终虽回 PREOP，但不满足干净退出契约。容器 exit 0 不能作为成功依据。
- Question：应在哪个上游生命周期层完成 timer cancel、executor removal/barrier 和 bridge/Motor402 release 的严格排序，
  同时不让已有 cleanup 投递任务死锁？
- Preferred direction：窄补丁先取消 polling timer、阻止新 callback、等待 executor 中已开始 callback 排空，再销毁
  driver bridge。Benefit 是针对实际 use-after-free 且保留开源 CiA402 行为；drawback 是并发顺序补丁必须做重复启停和
  fault/EMCY 退出压力测试。直接先停整个 executor 更简单彻底，但可能阻塞依赖 executor 完成的 cleanup，死锁风险更高。
- Blocked scope：修复并验证前禁止新的整栈使能、运动或宣称 graceful stop 通过；静态源码审计和 Mock 可继续。
- Root cause：上游 `NodeCanopenDriver::deactivate()` 在 `activated_=false` 后先调用
  `remove_from_master()` 释放 `LelyDriverBridge`，之后才进入派生类 `deactivate(true)` 取消 polling timer 并
  join NMT 线程；这与实机 `get_id()` UAF 栈逐项吻合。仅交换顺序仍不够，因为 `TimerBase::cancel()` 不等待已经
  进入 MultiThreadedExecutor 的 callback 返回。
- 2026-07-27 implementation：用户批准维护窄上游并发补丁。第一道防线把基类顺序改为
  `deactivate(true) -> remove_from_master()`；第二道防线在两履带完成零速、CiA402 shutdown 和全节点 NMT Stop 后，
  先 `executor_->cancel()` 并 join spin thread，确认 ROS callback 全部排空，再执行 `shutdown_drivers()`；Lely master
  仍在驱动 cleanup 完成后由既有 `clean()` 关闭。没有给 RT `read/update/write` 路径增加锁、阻塞或日志。
- Verification：复用 ros2_canopen 现有 GTest 增加严格顺序期望。修复前稳定失败于
  `remove_from_master()` 提前调用；修复后该目标通过，且 `canopen_ros2_control` 完整编译通过。Docker 以锁定上游
  commit 依次应用 `0001/0002/0003`，仓库门禁检查回调排空在 driver release 之前。
- Hardware verification：镜像 `rt-control:4fc8414f67b63bf3a1c4fb4c34eb27fe8caafc9d` 在
  `ar@192.168.0.40` 完成三轮完整启动/停止；第一轮还执行一次全组 reset、14 轴 enable/hold/disable，后两轮不发送
  使能或运动命令。三轮均看到 `Exiting spin thread` 后才 cleanup 两个履带 driver，
  `ros2_control_node` 和容器均正常退出、container exit 0，无 `SIGSEGV`、`get_id()` UAF 或
  `UNCLEAN_SHUTDOWN`。证据归档为
  `/var/lib/rt-control/validation/run-17-bq119-bq122-4fc8414/`，并有
  `evidence-manifest.sha256` 完整性清单。
- Resolution boundary：本项只关闭已复现的 callback/bridge 生命周期竞态，不宣称完成 heartbeat/EMCY 故障注入、
  CAN 断链机械停车或长时间启停压力验收。

## BQ-120 — EtherCAT OP 期间仍周期性成组 datagram timeout [RISK-ACCEPTED/ROOT-CAUSE-LOCALIZED 2026-07-27]

- Evidence：T-019 期间 IgH master `Lost frames` 从 391 增到 399；rt_diagnostics `wc_error_count` 从稳定后的 705
  增到 712。内核在 OP 阶段八次记录 `3` 或 `4 datagrams TIMED OUT`，间隔约 10–100 秒；反馈年龄采样仍约
  `0.005 ms`，16 个位置仍为 OP。启动阶段另有 `850 datagrams TIMED OUT`、DC sync timeout 和部分
  `0x001B` 恢复过程。
- Conflict：BQ-114 的 400 ms 容忍允许系统最终进入 OP，但不能把仍增长的 timeout/WC 计数解释成健康通信；运动时继续
  使用该状态会扩大旧过程数据或同步级联风险。
- Question：丢失来自 NIC/IRQ/CPU 调度、DC 配置、物理链路还是主站/从站状态切换的哪一层？必须用同一镜像联查 IRQ、
  cyclic loop、IgH domain/WC 和链路统计后裁决，不能继续猜测参数。
- Decision boundary：不再增加 `0x10F1:02` 或软件宽容值。Benefit 是不掩盖真实丢帧并保持 400 ms 已批准边界；
  drawback 是在根因修复前会阻塞运动验收。单纯增加宽容度实现最简单，但只延迟失败且扩大风险窗口，明确不采用。
- Blocked scope：允许只读诊断和离线修复；禁止 FJT/Updown 运动，修复后必须证明稳定窗口内 lost/WC 不增长。
- 2026-07-27 superseding user decision：用户明确接受这些短通信抖动不作为控制停机条件，并批准进一步增加驱动同步
  错误计数容差；因此上一条 decision boundary 和 motion block 仅保留为历史记录，不再支配新构建。全部 9 个共享
  profile 的 `0x10F1:02` 改为 `250`，在 4 ms 周期下 nominal 约 `1000 ms`。REQ-ECAT-009 的冻结语义仍是
  AL/link/responding/WC 异常进入 diagnostics WARN、不得自动升级为 FAULT；250 Hz、DC、PDO、完整 WC 启动门禁均不变。
- Benefit：已观测的 12–16 ms 成组 timeout 和控制循环交接抖动不会轻易触发驱动本地同步故障，减少无效启动失败或
  整组掉使能；配置、迁移门和启动 SDO 仍单一一致。
- Drawback（重点）：真实连续同步故障从约 400 ms 延后到约 1000 ms 才由该对象升级，最坏增加约 600 ms 的驱动侧
  反应延迟。该值不是墙钟 watchdog，也不会消除 IgH `datagrams TIMED OUT`、Lost frames 或 WC 计数；软件仍会如实
  WARN。NIC/IRQ/调度与物理链路根因调查继续 OPEN，但不再单独阻塞最小低速验收。
- First-image gate：首次新镜像启动必须确认所有 ZeroErr/XMC 和四台 Ti5 接受值 250；尤其 Ti5 因 BQ-117 只能用
  `uint16` SDO upload 做只读回读。若任何 startup abort、AL 无法稳定 OP 或反馈不新鲜，仍 fail closed，不能以风险接受
  绕过启动门。
- 2026-07-27 short-window result：上述 first-image gate 已通过；14 个运动从站均只读回读 `250` 并进入
  OP/WC-complete。三轮整栈启停期间 IgH `Lost frames` 从基线 `399` 到最终仍为 `399`，退出窗口没有新的 datagram
  timeout 或 WC/同步级联。该短窗口结果允许按用户已接受的风险继续后续最小低速验收，但不能代替 30 分钟联合空跑，
  也不关闭 NIC/IRQ/调度/物理链路的长期根因调查。
- Root-cause localization：停机后的只读 `ethercat crc` 显示只有 slave 2（Ti5 `right_joint2`）port 0
  累计 `CRC=8, PHY=6`；slave 3 及其后的 `FWD=8` 是同一批坏帧的转发计数。`ethercat graph` 证明该入口直接连接
  `slave 1 port 1 -> slave 2 port 0`。这 8 个 CRC 错误与首轮 IgH `Lost frames 391->399` 的增量精确一致，且八组
  OP datagram timeout 同期发生，因此根因已优先定位到这段 MII 物理链路，而不是整个环或 XMC 新分支。
- Exclusion evidence：主站 I210 `03:00.0` PCIe 为 2.5 GT/s x1 正常宽度，PCI Device/AER correctable、nonfatal、fatal
  均为 0；IgH 主站 Tx errors 为 0，timeout 时段没有 link-down。ec_igb 接管后不提供普通 `enp3s0` ethtool 统计，
  所以不能据此排除线缆/接头或相邻从站 PHY。
- Scheduling observation：IgH module 当前额外配置 `run_on_cpu=12`，而仓库/GRUB 只隔离 CPU 14；CPU 12 仍接收普通
  timer/softirq 和 `enp4s0` IRQ。该配置未记录在仓库 host contract，是次级调度风险；但 corrective 三轮在相同配置下
  没有新增 lost frame，且物理 CRC 计数与历史增量一一对应，因此本轮不在线改 CPU 归属来混淆变量。
- Next controlled action：维护窗口断电检查或替换 `slave 1 port 1 <-> slave 2 port 0` 的线缆与两端接头，并检查
  slave 2 入端/slave 1 出端；复电后先记录 CRC 基线，再完成至少 30 分钟 OP 空跑，要求 CRC/PHY/FWD、Lost frames 和
  WC error 都不增长。只有该段稳定后才单独评审 `run_on_cpu=12` 是保留、迁移到隔离 CPU，还是为主站另隔离 CPU；
  不同时改物理链路与调度配置。
- 2026-07-27 minimal-motion result：在用户批准硬件急停、无碰撞位姿、旋转轴 `0.5 degree`、Updown
  `0.05 m` 和单程 `2 s` 后，corrective image 完成 28/28 段逐轴外移/回程 FJT，期间没有 goal/drive
  失败或整组掉使能。但 `Lost frames` 从 399 增至 402，slave 2 port 0 从 `CRC=8, PHY=6` 增至
  `CRC=11, PHY=8`，三次 `4 datagrams TIMED OUT` 中一次发生在 FJT 窗口。该结果证明已批准的
  1000 ms 容忍能跨过本轮短抖动，**不证明链路健康或根因关闭**；物理段维护和 30 分钟零增量门禁保持不变。
- 2026-07-31 scheduling corrective decision：导航负载启动后现场复现 EtherCAT 主站掉链，同时观察到
  `sched: RT throttling activated`、datagram timeout 和 Sync Manager watchdog 同期出现。用户批准把 IgH
  `EtherCAT-OP` 从未隔离 CPU12 收敛到已批准隔离核 CPU14，但该修改必须和 native 线程拆分同批实施，避免
  “EtherCAT-OP + 整棵 rt-control 进程树”共同挤在 CPU14 的危险中间态。冻结实施方式：外层
  `rt_control_start` 只用 `taskset` 约束到 housekeeping CPUs `0,2,4,6,8,10,12,16-27`，启动后由
  `sched_setaffinity` 只把 `ros2_control_node` 中**唯一** `SCHED_FIFO/80` update 线程移到 CPU14；DDS、
  service、CANopen/Lely、诊断和 IO 普通线程保留在 housekeeping。每次 native 脚本调用 `/rt/enable` 前均重新执行
  并验证该 pin，若找不到 FIFO80、匹配到多个、`sched_setaffinity` 失败、affinity 或当前 `PSR` 不为 14，则 fail-fast
  拒绝使能。该方案依赖 `taskset`/`sched_setaffinity` 语义，禁止替换为 cgroup cpuset 或 systemd
  `CPUAffinity=` 实现。RT throttling 二次触发不再依赖 `dmesg`，需用 kprobe/bpftrace 或 “FIFO80 约 95% CPU +
  1 秒节律超时”症状判断。该裁决新增调度拓扑风险控制，不关闭既有物理链路根因和 30 分钟零增量门禁。
- 2026-07-31 target smoke after corrective：目标机源码快进到 `98e9d12`，`ec_master` 重载后
  `/sys/module/ec_master/parameters/run_on_cpu=14`。native `start` 证明外层进程树在 housekeeping CPUs，唯一
  FIFO80 update 线程 `PSR=14`，`EtherCAT-OP PSR=14`；第二次全组 `/rt/reset_fault` 成功后 `/rt/enable`
  成功，14 轴 enabled 契约通过，`rt_control_native.sh stop` 有序 `/rt/disable` 并退出，最终 EtherCAT
  Idle/Inactive、16 从站 PREOP、can0/can1 ERROR-ACTIVE 且 CAN error counters 为 0。Limit：启动阶段仍出现
  transient SDO read-only PDO-mapping warning 和 `0x001B Sync manager watchdog` 恢复消息，之后进入稳定 OP；
  因此该 smoke 只证明单次 enable/disable 可用，不关闭 BQ-120 的启动瞬态、物理链路和长时间导航负载根因。

## BQ-122 — 完整栈退出时 CANopen 清理先于 EtherCAT deactivate [RESOLVED 2026-07-27]

- Evidence：应用 BQ-119 的 CANopen callback 排空补丁后，首轮完整栈退出不再 SIGSEGV：spin thread 先退出、
  两履带 driver 正常 shutdown、`ros2_control_node` 与容器均 exit 0。但是 controller_manager 停止全局 250 Hz
  read/update/write 后，其 `ResourceManager::shutdown_components()` 按内部 `unordered_map` 实际先清理
  `canopen_mobile_axes`；约 240 ms 后才调用 `ecat_arms` deactivate。其间 EtherCAT PDO 已停止，14 个运动从站在主站
  deactivate 前全部报告一次 `0x001B Sync manager watchdog`。因此“CANopen UAF 已修复”不能等同于“完整栈干净退出”。
- Root cause：ros2_control 的硬件 shutdown 次序不由 Xacro 中 `<ros2_control>` 的文本顺序保证；交换 include 不能形成
  可验证契约。ICube 的 `ecrt_master_deactivate()` 本身有效，历史 EtherCAT-only 运行和本次显式调用都能在 PDO 尚持续时
  干净进入 PREOP；问题是它在默认全局 shutdown 中被排在耗时 CANopen 清理之后。
- Decision：不修改 BQ-033 已冻结的从站 SyncManager watchdog，不用更长硬件 watchdog 掩盖退出顺序。扩展 BQ-054
  已有的 one-shot 退出客户端，在同一个冻结的 30 s 总 deadline 内依次：调用 `/rt/disable`；严格停用
  `enable_manager` 与 `joint_state_broadcaster`（JTC 已由 disable 保证 inactive）；通过 controller_manager 标准
  `set_hardware_component_state` 把 `ecat_arms` 确认降为 inactive；最后才由既有 wrapper 向 launch 进程组转发 SIGINT，
  让 CANopen 完成 callback 排空、driver shutdown 与 NMT Stop。若 `/rt/disable` 因已有 Fault 返回失败，仍 best-effort
  执行后两步以避免再制造总线 watchdog，但 helper 保持非零退出并记录 `UNCLEAN_SHUTDOWN`，不得伪报干净停机。
- Benefit：在周期 PDO 仍存在时完成 EtherCAT OP→PREOP，保留驱动本地断链保护、IgH/ros2_control 标准生命周期和
  已有 100 s Compose grace；不新增常驻节点、新包、主站私有状态接口或硬件 watchdog 数值。
- Drawback（重点）：内部 `rt_disable_once` 现在同时依赖 controller-manager switch/hardware-state 两个服务，退出路径比
  单次 disable 调用更复杂；`joint_state_broadcaster` 会在进程最终退出前提前停止。三步共享 30 s deadline，前一步异常
  占满预算时后续步骤会 fail closed 并回退到不保证干净的普通进程 teardown，仍需下一次启动 sanitize。
- Pre-implementation hardware proof：在同一镜像上人工逐项执行该顺序后，内核只记录 EtherCAT master thread 切换到
  IDLE、全部从站 PREOP，随后进程退出时才 `Releasing master`/`Released`；窗口内 `0x001B=0`，容器 1.8 s exit 0。
  对照默认顺序同机同镜像稳定产生 14 条 `0x001B`。该 A/B 结果证明修复层级正确，但集成 helper 的新镜像仍必须重复
  启停后才可把本项和 BQ-119 改为 RESOLVED。
- Integrated hardware verification：新镜像 `4fc8414f67b63bf3a1c4fb4c34eb27fe8caafc9d` 的三轮退出均按
  `disable -> controllers quiesced -> ecat_arms inactive -> SIGINT -> canopen_mobile_axes deactivate` 执行；完整 OP 的第三轮
  也在约 3.0 s 内停止并 exit 0。每轮内核退出窗口均为 EtherCAT OP thread 先回 IDLE、从站回 PREOP、再 release master，
  `0x001B=0`，没有 SyncManager watchdog 或 datagram timeout。最终 master 为 Idle/Inactive、16 个从站全 PREOP。
- Resolution boundary：已关闭当前生产 wrapper 的确定性退出次序缺陷。绕过 `rt_control_start` 直接启动 launch、helper
  在 30 s 总预算内失败、进程被 SIGKILL 或主机掉电均不受此保证；这些路径仍按不干净停机处理。

## BQ-121 — BMS CANable 缺席会阻塞 rt-control CAN unit [OPEN/DEPLOYMENT 2026-07-27]

- Evidence：目标机只有 rt-control CANable serial `004D00675230500720333159`；BMS serial
  `003000265230500720333159` 缺席。`rt-control-can-names.service` 因要求两只设备同时存在而失败，连带
  `can0.service` 无法正常 start。T-019 在逐字核对现有 can0 serial 后仅本次使用
  `--job-mode=ignore-dependencies` 启动原 500 kbit/s/txqueuelen 128 unit，测试后恢复 DOWN；未修改 unit。
- Question：生产策略是否继续要求 BMS 适配器在场作为全机部署门禁，还是拆分为两个独立命名 unit，使 rt-control CAN
  只依赖自己的 serial？
- Trade-off：保持严格联锁能阻止设备缺失/误命名后启动，代价是与 rt-control 无关的 BMS 维护会阻塞履带；拆分 unit
  提高域独立性，代价是必须重新设计跨 unit 命名冲突、启动顺序和 host verification。
- Blocked scope：不阻塞离线开发；在裁决并修改前，`ignore-dependencies` 只能作为逐次记录、逐次授权的 commissioning
  例外，不得写入生产脚本或自动启动流程。
- 2026-07-27 observation：本次 corrective image 验证时两只批准序列号的适配器均在场，`can0`/`can1` 均为
  500 kbit/s、ERROR-ACTIVE 且 RX/TX/error/drop 计数无错误，因此没有再次使用缺席例外。此现场状态不回答“以后允许
  BMS 适配器缺席吗”，生产 unit 拆分与否仍保持 OPEN，不能把一次两设备在场写成架构裁决。

## BQ-123 — 导航与视觉共用的本体 TF 根链 [RESOLVED 2026-07-28]

- Evidence：冻结 `REQ-IF-003` 和旧控制器配置要求 diff-drive 发布 `odom -> base_link`，但迁入的权威模型同时包含
  固定 `world -> base_link`；RSP 与 diff-drive 同时运行时会让 `base_link` 出现两个父 frame。模型中该固定变换为
  精确 `xyz="0 0 0.202094" rpy="0 0 0"`，可在不引入新标定数值的前提下表达底盘投影平面到本体基座的高度。
- User decision：采用导航常用的 `odom -> base_footprint -> base_link`；由 rt-control Docker 同时运行
  diff-drive 和 `robot_state_publisher`，向导航与视觉提供 `/tf`、`/tf_static`，两者类型均为
  `tf2_msgs/msg/TFMessage`。本次授权同时覆盖实际 launch、URDF 加载、TF 发布和 Docker 配置检查/修改。
- Decision：从权威活动模型删除虚拟 `world` link 和 `world_to_base` joint，原值迁为固定
  `base_footprint_to_base`；diff-drive 以 50 Hz 发布唯一动态边 `odom -> base_footprint`；RSP 以同一展开模型发布
  活动关节 `/tf`，并在 `/tf_static` 发布 `base_footprint -> base_link`、固定 Pitch 及传感器/工具固定边。
  RSP 动态发布上限与已批准的 50 Hz `/joint_states` 对齐。`map -> odom` 仍由 perception/定位负责。
- Compatibility：`/diff_drive_controller/odom.header.frame_id` 保持 `odom`，但 `child_frame_id` 从
  `base_link` 改为 `base_footprint`。普通 TF 消费者继续可以查询组合后的 `odom -> base_link`；直接断言旧边或旧
  `Odometry.child_frame_id` 的消费者必须同步更新。仿真若需要 `world`，应由仿真/定位层建立外部世界关系，不能让
  本体 URDF 再次占有 `base_link` 的第二父节点。
- Benefit：消除 TF 多父冲突，得到适合二维导航的地面投影 frame，同时让视觉、雷达和机械臂共享同一模型版本的完整
  本体树；250 Hz 硬实时环不消费 TF，新增 RSP 参数不进入实时控制路径。
- Drawback：这是公共 frame 契约变更；依赖 `world`、直接 `odom -> base_link` 边或旧 odometry child frame 的消费方
  需要迁移。RSP 与 TF 可用性继续跟随 rt-control 容器生命周期，目标机必须重建并部署新镜像后才会生效。
- Verification boundary：源码合同测试、Xacro 展开、`check_urdf`、受影响包构建以及复用生产基座镜像的无设备 Mock
  已验证两话题类型、三段查询和运行消息边集合，且不存在 `world`。这不是目标机生产镜像部署或带电实机验证。
- 2026-07-30 supersession：BQ-127 仅取代本条中“由 rt-control/diff-drive 发布
  `odom -> base_footprint`”的所有权裁决；`base_footprint` 根、固定 `base_footprint -> base_link` 和无 `world`
  的本体模型裁决继续有效。

## BQ-124 — PLC/BMS 最小接口、写所有权与临时 CPU 放置 [RESOLVED 2026-07-28]

- Evidence：同事分支原实现同时开放批量输出 topic、三路 Bool topic 和三路 `SetBool` service，且 BMS 发布多组重复
  数值/JSON 话题并依赖未形成跨进程仲裁的 `can_bus_guard`。PLC 对接资料给出 `%MW200/201/210/211/212`，现场映射由
  用户确认：输出 bit0/bit1/bit2 分别为左电磁阀、右电磁阀、共用真空泵；输入 bit0/bit1 分别为左右真空已建立，1 为真。
- User decision：BMS 只要电压和 SOC，单话题、5 秒一次；删除 `can_bus_guard`。PLC 只留三路 `SetBool`，每次只改目标
  bit 并保持当前其他状态；启动、重连和退出不清输出。`%MW201 bit0` 必须始终为 1，连接发现为 0 时自动修复。
  为近期联调先让 PLC/BMS 与控制栈同容器、同实时 CPU，后续再评估迁移到 housekeeping CPU。
- Decision：BMS 只收 `0x3FC` 并发布 `/bms/battery_state`；PLC 只发布强类型 `/plc/io_state`，写服务需同时验证
  `%MW200` 命令位与 `%MW211` 实际位。`%MW201` 使用读改写，只置 bit0，并要求回读全字与目标值一致，避免漏检
  非目标 bit 变化。Compose 仍为单服务/PID 1
  `rt_control_start`，以环境变量启用两个节点，新增最小 `NET_RAW`，不授予 `NET_ADMIN`，不在容器内配置 `can1`。
- Realtime/safety boundary：同 CPU 放置是有意接受的短期联调风险，不关闭调度干扰、WCET 或生产实时性验收；后续 CPU
  拆分不得改变 ROS 接口或寄存器语义。PLC/BMS 节点不进入 EtherCAT/CANopen 闭环，但当前 cpuset 会让其系统调用和
  Python 调度与控制进程竞争。
- Verification boundary：受影响四包构建通过，BMS/PLC 47 项测试、Compose 展开、launch 参数解析及仓库 26 项质量门禁
  通过；没有访问 PLC、CAN 或输出。当前一键脚本仍锁旧镜像，必须以批准提交重建镜像并更新 release/SHA/镜像 ID 锁，
  再完成目标机 start→READY→stop 与 IO 点对点核对，才算进入明日联调运行版本。
- 2026-07-28 integrated verification：关停修复源码 `d415c0c2c75917a9545a4a2f87487718de8622a2`、发布锁
  `7bc8f16e903ef7174f60ebe9613f6eeed3a96185` 和镜像 ID
  `sha256:01bd550b068fccb9158b007067e55c30eed7d7d7253ef9179dfdf6d9be9a11c2` 已在目标机完成 PLC/BMS 读取、三路
  输出逐点 ON/OFF、无运动 start→READY→hold→stop。每路 `%MW200/%MW211` 只出现 `0x0001/0x0002/0x0004` 的目标
  bit，末态 MW200/MW210/MW211/MW212 为 0、MW201 为 1；容器 exit 0，EtherCAT Idle/Inactive、16 从站 PREOP，
  CAN0/CAN1 错误计数为 0。独立 BMS HMI 对照及左右实体/共用泵气路效果没有由 SSH 证据确认，仍是现场人工观察项；
  该边界不改变本裁决的 CPU14 临时风险接受。

## BQ-125 — 现场感知传感器坐标系与活动控制模型不一致 [RESOLVED/HIGH-RISK 2026-07-29]

- Evidence：用户指定的现场参考 URDF（SHA-256
  `db1b04c9b5aba34a2a9671a0f343d59b9d9dcdad9c9427dae2de2e3881ce8dff`）定义了
  `lidar_front_mount -> lidar_front -> {livox_fused_frame, top_sensor}`、
  `lidar_rear_mount -> lidar_rear` 和
  `cam_front -> camera_top_color_optical_frame`。活动 `robot_description` 只有直接挂在 `turn` 下的雷达 CAD link，
  缺少上述原生/融合/工作/光学 frame，且前相机安装位姿仍是旧值。
- Conflict：参考文件仍使用 `leftjoint1/rightjoint1` 等旧控制关节名，把 Updown 上限写成 `0.92 m`，并把多个运动关节
  `effort/velocity` 写为 0；整文件覆盖会破坏已经冻结、构建并实机验证的下划线命名、14 轴 ros2_control 接口和
  Updown `0–0.8 m` 限位。参考文件在现场仓库中还是未跟踪资产，因此本次用用户指定的内容哈希固定取值，不能把其余
  未审查字段静默提升为整机模型权威事实。
- User decision：rt-control 容器发布的 URDF 应按照该现场参考补齐用户给出的感知 frame；允许直接在目标工控机完成后续
  镜像更新，但不能因为同步感知 TF 覆盖既有控制关节裁决。
- Decision：只合并参考文件的完整感知传感器固定子树和精确数值：雷达网格保留在新增 mount link，
  `lidar_front/lidar_rear` 改为安装 MID360 的原生坐标系；增加 `livox_fused_frame`、`top_sensor` 和
  `camera_top_color_optical_frame`；前/后相机固定 joint 使用参考位姿。保留现有基座、Pitch、Turn、Updown、双臂、工具
  link/joint 名称、轴向和限位，不导入参考文件的旧控制字段或包名。
- Benefit：`robot_state_publisher` 能从同一公共模型稳定发布感知管线实际使用的 `/tf_static`，融合点云、FAST-LIO、相机
  和导航不再依赖各自私有静态 TF；CAD 网格与传感器原生轴分离，避免为了修正数据坐标而旋转实体可视/碰撞几何。
- Drawback（重点）：这是公共 TF 契约变化；`lidar_front` 和 `lidar_rear` 相对 `turn` 的语义从 CAD 安装 frame 改成传感器
  原生轴，`cam_front` 的安装平移/俯仰也变化。使用旧 frame 数值、重复发布同名静态 TF 或缓存旧模型的 perception、
  motion、RViz 配置必须同步升级；错误的现场标定仍会一致地传播给所有消费者。
- Verification/release boundary：合同测试必须逐项锁定上述父子关系和精确变换，同时防止旧控制关节名和 `0.92 m` 限位
  回流；Xacro、`check_urdf`、共享包构建、仓库门禁和无设备运行 TF 查询通过后才能构建镜像。当前带电容器不会被原地
  改写；生产切换仍需单独执行有序失能/停止、更新锁定镜像并验证 `/tf_static`，不能热改安装空间。

## BQ-126 — 主接触器掉电恢复计划的 literal 0x0040 与 Ti5 冻结例外冲突 [RESOLVED 2026-07-30]

- Evidence：已批准的恢复计划原写为 reset 后“14 轴均为 `0x0040`”，但 BQ-115 已根据协议手册、PDO/SDO
  和实机结果冻结：`right_joint2/right_joint3/left_joint2/left_joint3` 四个 Ti5 在标准 controlword
  `0x0000` 下保持 `Ready To Switch On (0x0021)`，并被限定接受为非激磁终态；其余轴严格要求 `0x0040`。
- Conflict：照字面要求 14 个 `0x0040` 会让现场已知正常的四个 Ti5 永远无法通过恢复入口；反过来把 `0x0021`
  放宽到所有轴会推翻 REQ-SAFE-002/BQ-115 的轴级边界。
- Decision：恢复入口必须沿用 BQ-115：其余 10 轴以 `statusword & 0x004F == 0x0040` 为唯一通过条件；仅四个
  指定 Ti5 额外接受 `statusword & 0x006F == 0x0021`。enable 后不设例外，14 轴都必须满足
  `statusword & 0x006F == 0x0027`。状态来自现有 `/dynamic_joint_states`，不增加 ROS 包或常驻节点。
- Recovery policy：专用 `recover-power-loss` 先最佳努力 disable 并销毁旧控制会话，确认 EtherCAT master
  Idle/Inactive 后要求现场复电授权；新会话稳定后只调用一次 group reset 和一次 enable。普通 start 不 reset，任一步
  失败不循环请求，并清理本次新会话。
- Benefit：不让旧 JTC/PDO/预装载跨越硬件掉电，同时以逐轴真实状态而不是 service `ok` 单独作为准入证据；严格保留
  Ti5 例外的轴级范围。
- Drawback（重点）：这是人工恢复而非急停检测/STO；掉电期间无法确认 drive 收到软件 disable。四个 Ti5 的
  `0x0021` 仍是 BQ-115 的高风险硬件兼容例外，脚本不会消除 master release 后 `0x7500` 的驱动侧原因。
- Verification boundary：源码合同和离线检查不授权主接触器操作。首次真实掉电—复电必须另行 L3 授权并记录旧
  disable 结果、容器/master 末态、reset 前后 14 轴状态、enable 结果和失败清理路径。

## BQ-127 — 原始轮速里程计名称与 odom TF 跨域所有权 [RESOLVED 2026-07-30]

- Evidence：修改前 rt-control 的 diff-drive 唯一发布 `/diff_drive_controller/odom`，消息内部为
  `odom/base_footprint`，并以 50 Hz 发布动态 `odom -> base_footprint`。目标机并不存在 `/odom` topic。导航域计划
  发布融合后的最终 `/odom`，若两域同时发布同名动态 TF 边会产生多权威、跳变和不可确定查询。
- User decision：rt-control 原始轮速 topic 改为 `/wheel/odom`；同时从 rt-control TF 树移除
  `odom -> base_footprint`，该边和最终 `/odom` 由导航域发布。
- Decision：diff-drive 保留消息内 `header.frame_id=odom`、`child_frame_id=base_footprint` 和 50 Hz 语义，但发布端
  显式 remap 为唯一 `/wheel/odom`，不保留 `/diff_drive_controller/odom` 或 `/odom` 别名；设置
  `enable_odom_tf=false`。RSP 继续唯一发布 `base_footprint -> base_link -> 本体/传感器`，不得把 topic 名
  `wheel/odom` 建成 TF frame。该裁决取代 BQ-123 的 diff-drive 动态 TF 所有权部分，不改变其本体根链。
- Benefit：原始轮速与导航融合结果在名称和发布者上明确分层，消除两个域争用同一动态 TF 边的风险。
- Drawback（重点）：只启动 rt-control 时 TF 树有意缺少 `odom -> base_footprint`，因此不能查询组合后的
  `odom -> base_link`；旧 topic 消费者必须迁移，导航域未启动时不能由 rt-control 自动回退补边。
- Verification：测试先行先复现旧 `enable_odom_tf=true`；修正后源码合同通过。复用已验收生产基座、只读挂载新
  Launch/config 的无设备 Domain 143 Mock 实扫：`enable_odom_tf=False`；仅 `/wheel/odom` 一个 diff-drive publisher，
  约 50 Hz，frame 为 `odom/base_footprint`；旧两 topic 不存在；6 秒内无任何 `odom` 父 TF；静态
  `base_footprint -> base_link` 保持 `z=0.202094 m`。本结果不是新生产镜像或导航联合验证。

## BQ-128 — 原生开发运行与同机 ROS 2 Domain/DDS 策略 [RESOLVED/HIGH-RISK 2026-07-31]

- Evidence：目标机导航、雷达和感知终端没有永久设置 `ROS_DOMAIN_ID`，因此使用 ROS 2 默认 Domain 0；已部署的
  rt-control 旧锁显式使用 Domain 42。把导航终端临时切到 Domain 42 后才能发现旧容器接口，但同时让原先不可见的
  ROS 数据源进入同一 graph。另一次实测证明宿主使用 Fast DDS 默认传输即可发现 host-network/host-IPC 容器，
  不依赖 UDP-only XML；该 XML 会关闭 shared-memory transport，不适合同时承载 FAST-LIO 点云等高带宽同机链路。
- User decision：rt-control 原生运行、下一 Docker 镜像以及导航/雷达/感知的正常运行全部使用 Domain 0；开发阶段
  允许不封镜像，直接在目标机独立文件夹中增量构建和运行。当前提交只交付非 Docker 原生版本，Docker 配置迁移另行提交。
- Decision：原生包装器显式设置 `ROS_DOMAIN_ID=0`、`ROS_LOCALHOST_ONLY=0`、
  `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`，并清除继承的 DDS XML 环境变量，保留 Fast DDS 默认 UDP+共享内存传输。
  其他域只 source 自己的 workspace，不 source rt-control overlay。原生运行与生产容器互斥，普通 `start` 不 reset、
  不 enable，只有显式 `enable` 或 `start-and-enable` 才能使能。下一 Docker 镜像统一 Domain 0 的决定继续有效，但不属于
  本次原生提交的实现范围。
- Native layout：目标机固定 `/home/ar/rt-control-dev/robot` 工作树，vendor/build/install/log 位于其上级工作区，
  由 `deps.repos` 完整 SHA 和既有补丁生成。准备流程不得 reset/checkout/clean 已存在的 vendor；依赖 HEAD 或补丁状态
  不符合冻结事实时 fail closed。Docker 继续作为里程碑发布、回归和同事交付载体。
- Benefit：同机跨域无需反复给每个终端追加 rt-control DDS 环境，原生 `--symlink-install` 支持包级增量开发；默认
  shared memory 不再被 rt-control 的低带宽 profile 全局关闭。
- Drawback（重点）：Domain 0 会让同网段其他 Domain 0 participant 进入发现范围，错误的重复 publisher、TF 所有权
  或旧 topic 消费者也更容易真正互相影响；原生运行直接依赖宿主 ROS/apt/IgH 状态，隔离、复现和回滚能力弱于锁定镜像。
  Fast DDS 默认传输在多进程/多容器点云与控制联合负载下的共享内存权限、QoS、吞吐和长稳尚未验收。
- Verification boundary：原生脚本合同、shell 静态检查和离线质量门禁必须先通过。本裁决不授权目标机安装、构建、
  启动、总线访问、故障复位或使能；旧目标镜像在独立 Docker 变更完成受控重建/切换前仍是 Domain 42。

## BQ-129 — CANopen/Lely master loop 与 CPU14 隔离策略 [RESOLVED/HIGH-RISK 2026-08-01]

- Evidence：现场复盘认为 CANopen heartbeat 长时间缺失和 EtherCAT 同步抖动可能同源，都是高负载下关键通信调度没有
  足够隔离。前一版 BQ-120 将整个 `ros2_control_node` 外层放在 housekeeping，再只把唯一 FIFO80 update 线程迁移到
  CPU14，因此 CANopen/Lely master event loop 仍可能受 housekeeping CPU 满载影响。运行态扫描显示该进程内普通线程默认都
  叫 `ros2_control_no`，无法靠 TID 顺序或 CPU 占用稳定识别 CANopen heartbeat 线程；源码确认 ros2_canopen 的 Lely
  master loop 来自 `NodeCanopenMaster::activate()` 创建的 `spinner_` 线程。
- User decision：把 CANopen/Lely master heartbeat/event-loop 也放到 CPU14，不再保持其运行在 housekeeping CPU。
- Decision：新增窄 ros2_canopen 补丁，仅给 Lely master loop 线程设置 Linux comm 名称 `rtcan-master`，不改变
  CANopen 协议、heartbeat/NMT/EMCY 参数或设备状态机。native affinity 门禁在启动后和每次 `/rt/enable` 前要求
  `ros2_control_node` 中恰好存在一个 `rtcan-master`，并将它与唯一 `SCHED_FIFO/80` update 线程一起 pin 到 CPU14；
  DDS、service、diagnostics、BMS/PLC 及其余普通线程继续留在 housekeeping。
- Benefit：CANopen master heartbeat/event-loop 不再被导航、FAST-LIO、RViz、rosbag 或桌面远控把 housekeeping CPU 打满
  时直接饿住，同时保持 DDS/service 不挤占 CPU14。
- Drawback（重点）：CPU14 现在承载 EtherCAT-OP、FIFO80 update 和一个 SCHED_OTHER 的 Lely master loop；如果控制线程
  或 EtherCAT 长期占满 CPU14，Lely loop 仍可能受 RT throttling/调度延迟影响。该方案依赖线程命名补丁和
  `taskset`/`sched_setaffinity` 语义，升级 ros2_canopen 时必须重核补丁；若未来增加第二个 CANopen master，当前
  “恰好一个 `rtcan-master`”门禁会 fail-fast，需要重新裁决命名和绑定策略。
- Verification boundary：本裁决只解决目标主站线程归属，不替代 CAN 总线物理层、USB-CAN 适配器、tx/rx 队列或驱动器
  heartbeat consumer 的长稳验证；CAN error counters、EMCY/heartbeat 故障注入和导航联调压力仍需单独记录。

## BQ-130 — ELECTRI-102 滚动关节控制器、稳态模式接管与既有安全路径 [RESOLVED/HIGH-RISK 2026-08-18]

- Evidence：ELECTRI-102 要求 Motion 在运动中持续替换短未来完整关节目标，由 rt-control 本地以 250 Hz 或更高频率
  采样、插值、限幅并驱动；迟到/乱序/历史目标不得追赶，输入中断必须有界降级。当前 `dual_arm_jtc` 只负责完整 14 轴
  FollowJointTrajectory，冻结补丁只接入 Action goal admission；`enable_manager` 的 disable、group fault 和意外掉
  Operation Enabled 路径又只硬编码停用 JTC。现有硬件暴露 14 个 position command/state，不暴露 velocity command/state。
- Conflict：把 rolling session/buffer/watchdog 加进 JTC 会扩大 BQ-019 的窄补丁并重写上游通用控制器行为；另建
  controller 却不泛化 `enable_manager`，会在 rolling ACTIVE 时错误停用已 INACTIVE 的 JTC、遗漏真正的运动控制器并把
  switch 状态误锁为不确定。若在 FJT 仍运动时直接切到 rolling 并从 actual position hold，会产生未经过受控减速的
  position-command 阶跃；订阅 Action status 也不能作为无竞态切换门。
- User decision：批准 ELECTRI-102 Gate 0 默认方案，在功能分支实现到 Mock Checkpoint F1；生产动态限制继续保持 TBD，
  不授权目标机或实机动作。
- Architecture decision：新增独立 `rolling_trajectory_controller` ros2_control 插件，保留 `dual_arm_jtc` 的普通完整
  14 轴 FJT 和 BQ-019 补丁边界。两者声明相同的 14 个 position command interfaces，任何时刻至多一个 ACTIVE；
  `enable_manager` 是唯一 controller-manager STRICT switch owner，Motion 不得绕过它直接切 controller。
- Normal switch decision：V1 不向 Motion 暴露运动中 force switch。进入 rolling 前，Motion 先 cancel 自己持有的 FJT
  并等待 Action result；rt-control 再要求完整 14 轴 position state 有限、连续 N 周期稳态，并要求切换前持久 position
  command 有限且与 actual 的误差在经批准接管阈值内。rolling 首周期保持该已校验的 source command，以维持 command
  C0，而不是直接跳到 actual。N、稳态阈值、接管误差均为证据参数；未冻结前只能使用明确 test-only 值，不能进入生产。
- FJT result decision：mode service 不声称能查询 active goal。成功停用 JTC 时，若仍有在途 goal，pinned JTC 原生
  `on_deactivate()` 负责 abort，FJT 客户端收到的 Action result 是 goal 终态权威；mode response 只回报 source
  controller 是否停用、target 是否激活。若发现 goal accepted/deactivate 竞态会让旧 goal 在重激活后复活，必须停下并
  新开裁决，不能静默扩大 JTC 补丁。
- Session decision：rolling controller 独占 boot/session/generation/sequence、固定容量未来缓冲、authoritative suffix、
  连续性/限值/可停车性校验、输入更新年龄、低水位、有界停止和 public state。timeout、queue exhaustion、producer
  restart、controller restart 或 fault 后不自动恢复；terminal hold 后必须 close，再 fresh open。活动 session 中请求切回
  FJT 一律拒绝；先 close 到 terminal hold，再做同样的稳态接管。
- Safety/lifecycle decision：BQ-041/BQ-042/BQ-044/BQ-059 中“停用 JTC”的实现载体扩展为“停用当前 active motion
  controller”，但 quick stop、control-word、downward、restart-only 和 disable 优先级保持原义。disable/group fault/
  unexpected Operation Enabled loss 可从任意 rolling 状态抢占；与在途 mode switch 竞争时沿用 BQ-058/BQ-080，结果
  不确定即 `restart_required`。该扩展必须先建立现有 `enable_manager` characterization tests，再分成无行为重构和新语义
  接入两步。
- Feedback-age decision：rolling open 可复用 BQ-020/BQ-021 的
  `ethercat_domain/process_data_age_ms <= 500 ms` admission 证据，并声明该共享 state interface；运行中 process-data age/WC
  继续遵守 BQ-045 的 WARN-only 策略，不新增基于反馈 age 的自动停车。rolling 自己的 update-age/low-water 只约束本 session
  的输入数据，不恢复 BQ-006 禁止的独立 `rt_watchdog` 或 motion/autonomy heartbeat。
- Joint-state decision：ELECTRI-102 不修改 BQ-068；`/joint_states` 保持 50 Hz。rolling 的 250 Hz command loop 直接读取同进程
  ros2_control state interfaces，不能等待 `/joint_states` DDS 回环。
- Benefit：rolling 的实时所有权、替换和停止不变量可在纯 C++/fake hardware 中独立验证，同时保留普通 FJT、硬件
  quick-stop 和既有使能语义；稳态接管避免把模式切换变成无界位置阶跃。
- Drawback（重点）：新增一个安全关键 controller 和 `enable_manager` 泛化，测试面明显扩大。position-only hardware 无法直接
  观测实际 qdot，稳态只能用 250 Hz position 有限差分和跟踪误差证据；C1 停车不保证加速度连续，生产限制、阈值、QoS、
  horizon、timeout 和 guard 在证据冻结前都不是 production-ready。
- Verification boundary：先完成协议/动态包络文档、enable_manager 回归基线、固定容量纯核心、controller/fake CiA402、
  Motion Mock 和异常矩阵。接口包与 `docs/cross-domain-interfaces.md` 必须原子冻结。任何目标机非运动压测、controller
  activation、`/rt/enable`、SDO write 或真实运动仍需各自单独授权；本裁决本身不授权这些动作。

## BQ-131 — ELECTRI-102 模式切换前的 source-command 证据与返回 JTC 的 C0 语义 [OPEN/HIGH-RISK 2026-08-18]

- Evidence：T4-01 已把 `enable_manager` 重构为 motion-controller registry，并在 controller-manager 明确成功后记录当前
  active controller；旧 FJT enable/disable/fault characterization 仍全绿。进入 T4-02 后，代码级检查确认
  `enable_manager` 当前只能声明/读取 CiA402 `status_word`，即使按计划新增 14 个 position state，也只能看到 actual。
  JTC 与 rolling controller 互斥独占同一组 14 个 position command interfaces，`enable_manager` 不能在 source ACTIVE
  时再 claim 它们。rolling controller 只有在 STRICT switch 已开始、source 已释放资源后，才能在 `on_activate()` 读取
  精确的持久 command 并做最终 residual 检查。
- Evidence：pinned JTC 的 controller-state `output.positions` 是其周期发布时读取的 command interface 值，可作为带年龄的
  非 RT 快照，但默认仅 50 Hz，且 DDS 快照与随后 controller-manager switch 不原子。另一方面，BQ-044 冻结
  `set_last_command_interface_value_as_state_on_activation: false`；JTC 从 rolling 返回时会从 actual seed hold，而不是保持
  source command。因此只要允许非零 takeover tolerance，返回 JTC 的 command 变化只能被有界，不能宣称严格 C0。
- Conflict：当前协议要求 mode server 在调用 controller-manager 之前确定性区分 `SourceMoving` 与 `TakeoverMismatch`，执行计划
  又写了“command 不阶跃”和单次 STRICT source→target switch。现有接口只能同时满足其中一部分。直接在 Mock 中把
  source command 当作 actual，会绕过唯一需要验证的接管残差；把 target activation 的通用 `ok=false` 冒充为事前
  `TakeoverMismatch`，则违反“此前失败不得调用 controller-manager”的协议向量 V-33。
- Option A（推荐用于继续 Mock F1）：显式冻结一个双层证据模型。`enable_manager` 以带接收年龄的
  `/dual_arm_jtc/controller_state.output.positions` 和 rolling public-state `desired_positions` 做事前 admission，并继续用
  250 Hz 直接 position state 做 N 周期有限差分；FJT→rolling 再由 rolling `on_activate()` 对 switch 边界上的持久 command
  做最终精确 residual 检查。rolling→JTC 保留 BQ-044，从 actual seed hold；把合同修正为“切换步长不超过已验证 takeover
  tolerance”，不再声称该方向严格 C0。source 快照年龄、N、速度阈值和 residual 阈值全部保持 test-only/production TBD。
- Option A benefit/cost：不扩大 JTC 或硬件 driver patch，能够继续纯 fake/mock；但 source command admission 是有年龄的快照，
  不是与 switch 原子的读数。目标 rolling 的二次校验可 fail closed，返回 JTC 则依赖 BQ-044 actual hold 和生产阈值证据。
- Option B（更强但扩大范围）：由 hardware abstraction 导出 14 个只读 last-position-command mirror state interfaces，并为
  JTC 增加只在受信 mode handoff 时从该 command seed hold 的窄机制；`enable_manager` 在同一 ros2_control 周期读取 actual 与
  command mirror。这样可接近双向 C0 和准确事前分类，但会新增 EtherCAT/mock 接口并重新打开 BQ-019/BQ-044 的 JTC
  patch 边界，超出已批准 T4-02 文件/评审范围。
- Decision needed：批准 Option A 并相应收窄 rolling→JTC 的 C0 声明，或明确扩大到 Option B。未决前 T4-02 不实现会伪造
  source-command 权威性的 mode service；T3-05/T4-01 及普通 FJT 路径不受影响。
- Verification boundary：本问题只涉及本机 source-command 证据、mock switch 和合同文字；无论选择哪项，都不授权目标机、
  controller activation、总线、使能或运动验证。
