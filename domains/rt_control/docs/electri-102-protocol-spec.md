# ELECTRI-102 Rolling Joint Control Protocol Specification

> Status: approved Gate 0 design contract; production numeric parameters remain TBD.
> Scope: rt-control/Motion rolling joint mode only; no target-machine or hardware authorization.
> Decision authority: [BQ-130](../BLOCKED-questions.md), [implementation plan](electri-102-implementation-plan.md).

## 1. Purpose

This document freezes the externally observable V1 behavior before implementation. It defines mode and session ownership, logical time, sequence/generation rules, authoritative suffix replacement, state/reject semantics, close/recovery behavior, and executable protocol vectors.

The contract intentionally does not freeze measured values for QoS, horizon, timeout, guard, splice tolerance, takeover tolerance, or dynamic limits. Tests may use values explicitly named `test_only_*`; production activation remains blocked until the corresponding evidence is approved.

## 2. Fixed Axis Contract

Every point contains exactly 14 positions and 14 velocities in this order:

| Index | Joint | Position unit | Velocity unit |
|---:|---|---|---|
| 0 | `right_joint1` | rad | rad/s |
| 1 | `right_joint2` | rad | rad/s |
| 2 | `right_joint3` | rad | rad/s |
| 3 | `right_joint4` | rad | rad/s |
| 4 | `right_joint5` | rad | rad/s |
| 5 | `right_joint6` | rad | rad/s |
| 6 | `left_joint1` | rad | rad/s |
| 7 | `left_joint2` | rad | rad/s |
| 8 | `left_joint3` | rad | rad/s |
| 9 | `left_joint4` | rad | rad/s |
| 10 | `left_joint5` | rad | rad/s |
| 11 | `left_joint6` | rad | rad/s |
| 12 | `turn` | rad | rad/s |
| 13 | `updown` | m | m/s |

The V1 transport uses fixed arrays. It does not accept partial joints, reordered joints, omitted velocities, NaN, Inf, or implicit unit conversion. The server publishes a stable 32-byte SHA-256 `axis_set_hash` for the exact ordered names and units; an open request with a mismatched hash is rejected as `AxisSetMismatch`.

The hash input is this exact ASCII/UTF-8 byte sequence, including the final LF and no BOM:

```text
v1
0,right_joint1,rad,rad/s
1,right_joint2,rad,rad/s
2,right_joint3,rad,rad/s
3,right_joint4,rad,rad/s
4,right_joint5,rad,rad/s
5,right_joint6,rad,rad/s
6,left_joint1,rad,rad/s
7,left_joint2,rad,rad/s
8,left_joint3,rad,rad/s
9,left_joint4,rad,rad/s
10,left_joint5,rad,rad/s
11,left_joint6,rad,rad/s
12,turn,rad,rad/s
13,updown,m,m/s
```

Its lowercase hexadecimal digest is `25c6e82bf505ca9eb99db1c645ab75d7ecde0153faaf6a7492c6210c4d362526`. IDL carries the 32 digest bytes in their displayed order, not the hexadecimal text.

## 3. Ownership and Identifiers

| Value | Owner | Lifetime | Rule |
|---|---|---|---|
| `protocol_major/minor` | interface contract | schema release | V1 is `1.0`; major must match exactly; minor compatibility must be explicitly advertised |
| `controller_boot_id` | rolling controller | process/configure epoch | `unique_identifier_msgs/UUID`; changes after controller/process restart; all older traffic is rejected |
| `client_instance_id` | Motion producer | producer process | `unique_identifier_msgs/UUID`; bound at open and carried by every update/close request |
| `session_id` | rolling controller | one open/close lifecycle | `unique_identifier_msgs/UUID`; unique within a boot; never reused |
| `sequence` | producer | one session | Starts at 1 and increases strictly; gaps are allowed under the rules below |
| `generation` | rolling controller | one accepted buffer image | Starts at 0; increments only when an update is accepted |
| `request_id` | service caller | client-defined | `unique_identifier_msgs/UUID`; makes mode/open/close retries idempotent within their documented cache lifetime |

`generation` is server evidence, not producer authority. V1 updates do not request a generation rollback. A producer may include the last observed generation for diagnostics, but acceptance depends on session, sequence, replacement boundary, continuity, capacity, limits, and stopping viability.

## 4. Two-Level State Model

### 4.1 Control mode

| Mode | Active motion controller | Meaning |
|---|---|---|
| `Disabled` | none | Existing rt-control disabled/fault lifecycle owns the outcome |
| `FjtReady` | `dual_arm_jtc` | Existing complete 14-axis FJT mode |
| `RollingReady` | rolling controller | Controller active and holding; no rolling session exists |
| `RestartRequired` | unknown/none | Switch result is ambiguous; no further motion request is accepted |

Normal `FjtReady <-> RollingReady` transitions require the source-quiescent gate from BQ-130. Motion cancels and awaits its FJT result before requesting rolling mode. The server validates stable actual position and the source-command/actual takeover error. V1 exposes no motion-time force switch.

A request for the already active mode is idempotent and does not restart the controller or change a session. `Disable` or group fault wins against an in-flight mode switch. Any ambiguous controller-manager result enters `RestartRequired` under BQ-080.

### 4.2 Rolling session state

| State | Session exists | Updates accepted | Meaning |
|---|---:|---:|---|
| `None` | no | no | `RollingReady` hold, available for open or switch back to FJT |
| `Priming` | yes | yes | Logical time remains 0 until a valid initial horizon is committed |
| `Running` | yes | yes | Logical time advances and the active future is sampled |
| `Stopping` | yes | no | Stop is latched; late updates cannot cancel it |
| `Holding` | yes | no | Stop completed at zero desired velocity; explicit close is required |
| `Terminated` | no/invalid | no | Disable, fault, controller deactivation, boot change, or fatal invariant |

Allowed session transitions:

```text
None -> Priming -> Running -> Stopping -> Holding -> None
          |           |
          +---------->Stopping

Any state -- disable/fault/deactivate/fatal --> Terminated
```

There is no `Holding -> Priming` edge. Re-entry requires close to `None`, then a fresh open with a new `session_id`.

`set_mode(FJT)` is rejected while the session state is `Priming`, `Running`, `Stopping`, or `Holding`. Motion first requests close, waits for `Holding`, then sends a finalize close with a new `request_id` to destroy the session and reach `None` before requesting FJT mode.

## 5. Service Semantics

### 5.1 Set mode

The mode request contains protocol version, `request_id`, expected source mode, and target mode. The expected source is a compare-and-set guard.

Success means the source controller was deactivated and the target controller was activated with a non-ambiguous STRICT switch. It does not mean the server detected an active FJT goal. If JTC still owns a goal, its Action result is the goal-terminal authority.

Stable mode/open/close service errors are:

```text
None, WrongProtocol, WrongRequest, WrongMode, NotEnabled,
SessionBusy, SessionExists, WrongBoot, WrongSession, WrongClient,
AxisSetMismatch, SourceMoving, TakeoverMismatch, UnsafeHold, FeedbackStale,
LimitsUnavailable, SwitchRejected, SwitchTimeout, RestartRequired,
NotReady
```

Mode failures include wrong expected mode, not globally Enabled, rolling session still exists, source not quiescent, unsafe source hold, takeover mismatch, switch rejected, switch timeout, and restart required. A switch timeout or partial/ambiguous result is not retried automatically. If multiple conditions apply, identity/version checks precede state checks, safety-preemption state precedes motion admission, and no controller-manager call is made after an earlier failure.

ROS services have no server-side cancellation contract in V1. If a caller times out locally, it must not infer failure or send a new request ID; it queries the idempotent outcome by retrying the same client/request ID. A controller-manager switch timeout remains ambiguous and enters `RestartRequired` even if a late response subsequently arrives.

### 5.2 Open session

Open is accepted only in `RollingReady/None` when:

- protocol and `axis_set_hash` match;
- no other session exists;
- all 14 actual positions and persisted source commands are finite;
- source-quiescent/takeover evidence remains valid;
- the advertised hold command is inside every shrunken safe position bound;
- `ethercat_domain/process_data_age_ms` exists, is finite, and is at most the BQ-130/BQ-020 500 ms admission limit;
- the controller has a complete approved or explicitly test-only limit set.

Open returns `controller_boot_id`, a new `session_id`, current hold `q`, initial `qdot=0`, `axis_set_hash`, `limits_version`, capability bits, and configured capacity. It enters `Priming`; it does not start logical time.

`limits_version` is a 32-byte SHA-256 digest of the complete, ordered numeric envelope artifact. A test-only envelope hashes its exact test artifact and also sets `test_only_limits=true`; the flag, not a special digest value, distinguishes it. A production controller with no approved envelope fails configuration and therefore cannot publish a fabricated or all-zero production version.

Repeating the same open `request_id` from the same client returns the original response. A different request while a session exists returns `SessionBusy` and never replaces the owner.

### 5.3 Close session

Close is graceful, asynchronous, and two-phase:

- In `Priming` or `Running`, the first close `request_id` latches `Stopping`; the response means accepted, not stopped.
- In `Stopping`, retrying that same request returns its cached response. A previously unseen close `request_id` for the same owner/session is also accepted and cached, but it does not regenerate/restart the stop.
- In `Holding`, retrying the original request still returns its cached response and does not destroy the session. Motion sends a finalize close with a new `request_id`; that request destroys the session and returns to `None` without moving.
- Retrying the successful finalize request returns its cached success even though the session no longer exists.
- Wrong boot/session IDs are rejected and cannot affect the current session.

The V1 idempotency storage is deliberately bounded. At most eight distinct non-finalizing close request IDs are cached for one active session. Once those slots are full, another previously unseen close request returns `WrongRequest`; retries of any cached request remain deterministic. The successful finalize response has one separate cache slot and remains replayable after session destruction, including while a later session exists, until a newer finalize replaces it or the controller boot epoch ends. Motion normally uses one stop request and one finalize request, so reaching this bound indicates a caller retry bug rather than flow control.

V1 has no public abrupt rolling abort. Disable/fault paths are separate higher-priority lifecycle operations.

## 6. Logical Time

All trajectory times are unsigned nanoseconds from the current session start. They are not ROS wall-clock timestamps.

1. Open creates a session with `execution_time_ns=0` in `Priming`.
2. The prime batch must start at exactly `t=0` and meet the minimum horizon/viability contract.
3. At an RT cycle boundary, the prime generation becomes active and state changes to `Running` with time still 0.
4. Each subsequent 250 Hz update adds a validated positive controller `period` to `execution_time_ns`.
5. A non-positive or over-limit period latches `ClockAnomaly` stopping. The stop trajectory advances by one nominal 4 ms step per subsequent update call; an abnormal large delta is never used to skip ahead.
6. Wall-clock/ROS timestamps are diagnostic only. They never order or sample trajectory points.

Update freshness uses a separate monotonic steady timestamp captured after full non-RT validation. The accepted generation and its arrival timestamp are published atomically. Rejects do not reset update age.

Before any batch is accepted, public state reports `has_accepted_update=false`; accepted-update age has no numeric meaning and is encoded as zero. A separate monotonic `prime_age` starts when open succeeds. If no valid prime is committed before the configured prime timeout, the controller latches `PrimeTimeout`, creates the zero-velocity stop from the hold state, and reaches `Holding`. Once the prime is accepted, `has_accepted_update=true`, `prime_age` stops being authoritative, and normal accepted-update timeout begins.

No software controller can guarantee wall-time stopping while the entire controller-manager loop is not being scheduled. Existing hard safety/drive behavior remains authoritative for that failure class.

## 7. Point and Batch Semantics

Each point contains:

- `time_from_session_start_ns`, strictly increasing within a batch;
- exactly 14 finite positions;
- exactly 14 finite velocities.

Each update contains:

- exact protocol/boot/session identity;
- the exact `client_instance_id` bound by open;
- producer `sequence`;
- `replace_from_ns`;
- a bounded non-empty list of points whose first time equals `replace_from_ns`.

The controller uses piecewise cubic Hermite interpolation. Position and velocity are boundary conditions; only position commands are written to hardware. Accepted candidates must satisfy C0 position and C1 velocity continuity, per-axis segment extrema, and the stopping-viability rules in [electri-102-dynamic-envelope.md](electri-102-dynamic-envelope.md).

`Priming` is the only no-candidate case. Its update must have `replace_from_ns=0`, its first point must be exactly `t=0`, and it is validated without sampling an old splice state. Any other replacement or first-point time is `TimeGap`. After the first generation is accepted, every update follows the normal splice rules.

V1 has no special terminal marker or implicit terminal hold. The last point is an ordinary q/qdot boundary whose time defines `buffered_until_ns`; it may have zero or nonzero velocity. Session stopping/holding is entered only through the explicit stop triggers, and low-water must trigger before a nonzero terminal velocity can be exhausted.

## 8. Sequence Rules

The session tracks both `last_seen_sequence` and `last_accepted_sequence`.

1. A message with wrong protocol major, boot ID, session ID, or bound client instance ID does not address the current session and does not consume its sequence.
2. A correctly addressed message with `sequence <= last_seen_sequence` is stale/duplicate and is rejected without changing either counter.
3. On the first correctly addressed message with `sequence > last_seen_sequence`, the server atomically sets `last_seen_sequence=sequence` before payload/continuity/limit validation.
4. Therefore, a correctly addressed higher sequence is consumed even when the batch is rejected. Motion must fix it with a newer sequence; it must not replay the same number.
5. Sequence gaps are allowed. The higher message must still be self-contained and produce a complete valid candidate future.
6. Sequence never wraps. Near `uint64` exhaustion, close the session and open a fresh one.
7. `last_accepted_sequence` and server `generation` advance only after all validation succeeds and the complete pending image is published.

This policy prevents a delayed invalid/rejected message from becoming valid later after the active trajectory changes.

## 9. Authoritative Suffix Replacement

At each RT cycle the controller publishes a snapshot containing:

- `execution_time_ns`;
- `replaceable_from_ns`, the earliest boundary a newly received candidate may modify;
- `buffered_until_ns`;
- active `generation`, latest accepted validation-base `generation`, optional pending `generation`, and session identity.

The latest accepted complete candidate is the single authoritative validation head. It is the pending candidate when one exists, otherwise the active candidate. Every suffix is sampled and built against exactly the generation named as `validation_base_generation` in its coherent snapshot; “old candidate” below always means that validation head, never an implementation-selected active/pending alternative.

For an update with replacement time `R`:

1. Require `R >= replaceable_from_ns` from the validation snapshot.
2. Require the validation-head trajectory to cover the old prefix continuously through `R`.
3. Sample the immutable old candidate at `R` to obtain `q_old(R), v_old(R)`.
4. Require the new first point time to equal `R` exactly.
5. Require its q/qdot to match the old sample within separately approved splice tolerances.
6. Retain the old candidate strictly before `R`; replace everything at and after `R` with the new suffix.
7. Validate the complete candidate: time coverage, capacity, horizon, q/v/a extrema, position bounds, and stopping viability. There is no separate terminal representation check.
8. On any failure, keep active and pending generations unchanged.
9. On success, publish one complete immutable generation. RT switches only at a cycle boundary.

If a pending generation exists when another update arrives, V1 uses a preallocated latest-valid exchange. The newer valid candidate is built against that pending validation head, carries forward its complete immutable prefix and earliest activation boundary, and may supersede it before RT consumption. If the validation-head generation changes before publication, validation restarts against the new head or rejects; it never publishes against a mixed base. The RT side may consume the newest complete generation directly, including the inherited pending prefix, but never a partial image.

`replaceable_from_ns` includes the approved non-RT validation and RT-visibility lead. Immediately before publication the controller rechecks the current coherent snapshot: the base generation must still be the validation head and the candidate's earliest changed boundary must not be earlier than the current `replaceable_from_ns`. This prevents a suffix validated in the future from becoming a command jump after its replacement time has already passed.

## 10. Running and Stop Rules

`last_accepted_update_age` resets only when a candidate is accepted. The controller latches `Stopping` when any approved trigger occurs, including:

- accepted update age exceeds the configured update timeout;
- available horizon reaches the stopping low-water boundary;
- graceful close is accepted;
- a sampled command or internal invariant is non-finite/invalid;
- an execution-period anomaly occurs.

On transition to `Stopping`, the controller creates one fixed-storage synchronous C1 stop from the current desired q/qdot using the approved directional deceleration envelope. It rejects all later updates, samples that stop, then holds the terminal position with desired qdot zero.

Queue exhaustion while desired velocity is nonzero is an invariant failure: normal acceptance/low-water rules must have started stopping earlier. The controller never linearly extrapolates the last velocity and never jumps the execution cursor to catch history.

Disable, group fault, and unexpected loss of Operation Enabled preempt these graceful rules and follow BQ-041/BQ-042/BQ-044/BQ-059/BQ-130.

## 11. Public State and Error Separation

State must let Motion decide what to do without parsing logs. It contains at least:

- protocol/controller boot/session/client identity;
- control mode and session state, plus `has_accepted_update`;
- active, validation-base and pending generation, plus last seen/accepted sequence;
- execution, replaceable, and buffered-until times;
- available horizon, prime age, and accepted-update age;
- desired q/qdot snapshot;
- last batch `RejectCode` and rejected sequence;
- latched session `StopReason`;
- controller switch/restart-required result;
- counters for accepted, rejected, superseded-pending, timeout, and invariant failures.

`RejectCode` describes one update and does not imply session termination. `StopReason` describes a latched session transition. At minimum they distinguish:

```text
RejectCode:
  None, WrongProtocol, WrongBoot, WrongSession, WrongClient, StaleSequence,
  InvalidShape, NonFinite, NonMonotonicTime, LateReplace,
  TimeGap, CapacityExceeded, InsufficientHorizon,
  PositionDiscontinuity, VelocityDiscontinuity,
  PositionLimit, VelocityLimit, AccelerationLimit, NotStoppingViable,
  SessionNotAccepting

StopReason:
  None, GracefulClose, PrimeTimeout, UpdateTimeout, LowWater,
  ClockAnomaly, InternalInvariant, ControllerDeactivated,
  Disable, GroupFault, ControllerRestart
```

Mode/open/close service errors remain separate from batch `RejectCode`. For a correctly addressed new sequence, validation and reject precedence are exactly the order in the dynamic-envelope runtime validation section. In `Stopping` or `Holding`, a correctly addressed higher sequence is consumed and then rejected as `SessionNotAccepting`; a stale sequence remains `StaleSequence`. In `None` there is no session counter to consume.

## 12. Endpoints, QoS and Version Boundary

V1 freezes these public endpoints:

| Endpoint | ROS form/type | Owner |
|---|---|---|
| `/rt/joint_control/set_mode` | Service / `robot_interfaces/srv/SetJointControlMode` | `enable_manager` |
| `/rt/rolling_joint_control/open` | Service / `robot_interfaces/srv/OpenRollingJointSession` | rolling controller |
| `/rt/rolling_joint_control/update` | Topic / `robot_interfaces/msg/RollingJointTargetBatch` | Motion publishes; rolling controller consumes |
| `/rt/rolling_joint_control/state` | Topic / `robot_interfaces/msg/RollingJointControlState` | rolling controller publishes; Motion consumes |
| `/rt/rolling_joint_control/close` | Service / `robot_interfaces/srv/CloseRollingJointSession` | rolling controller |

Services use standard ROS 2 service QoS. Update and state topics use named profiles `Q_ROLLING_COMMAND` and `Q_ROLLING_STATE`; per-endpoint ad hoc QoS is forbidden.

`RollingJointTargetBatch.points` has a schema transport ceiling of 256. This ceiling prevents unbounded deserialization and is not the runtime capacity, expected batch size, horizon, or update rate. Open returns the configured `buffer_capacity`, which must be no greater than 256; a batch must be non-empty and no larger than that advertised capacity.

Reliability, history depth, deadline, lifespan, expected update rate, maximum batch size, timeout, and horizon remain prototype parameters until local and target evidence closes them. A buildable interface is not by itself production authorization.

## 13. Executable Protocol Vectors

The symbolic fixtures are:

- current session is `(boot=B1, session=S1, client=C1)`;
- `last_seen_sequence=10`, `last_accepted_sequence=10`, active `generation=4` unless stated otherwise;
- `E` is execution time, `G` is the published replaceable boundary, `U` is buffered-until;
- `Q(t),V(t)` are samples of the immutable old trajectory;
- `eps_q/eps_v`, `H_min`, `capacity`, and limits are explicit test-only values.

| ID | Input / condition | Expected result |
|---|---|---|
| V-01 | Open while mode is `FjtReady` | Reject `WrongMode`; no session |
| V-02 | Open in `RollingReady/None`, valid admission | Accept; create S1; enter `Priming`; time remains 0 |
| V-03 | Repeat same open request ID/client | Return original S1; do not create a second session |
| V-04 | Different client opens while S1 exists | Reject `SessionBusy`; S1 unchanged |
| V-05 | Prime `seq=1`, first time 0, valid horizon | Accept generation 1; next RT boundary enters `Running` at time 0 |
| V-06 | Prime has `replace_from=1 ns`, first point time 1 ns | Reject `TimeGap`; consume seq 1; remain `Priming` |
| V-07 | Update uses boot B0 | Reject `WrongBoot`; current sequence counters unchanged |
| V-08 | Update uses session S0 | Reject `WrongSession`; current sequence counters unchanged |
| V-09 | Correct IDs, `seq=10` | Reject `StaleSequence`; counters/generation unchanged |
| V-10 | Correct IDs, `seq=11`, NaN payload | Reject `NonFinite`; `last_seen=11`, accepted/generation unchanged |
| V-11 | Corrected payload reuses `seq=11` | Reject `StaleSequence`; no mutation |
| V-12 | Corrected self-contained payload uses `seq=13` | Gap allowed; accept if all candidate checks pass; `last_seen=last_accepted=13`, generation+1 |
| V-13 | `replace_from=G-1 ns` | Reject `LateReplace`; consume sequence; candidate unchanged |
| V-14 | First point time differs from `replace_from` by 1 ns | Reject `TimeGap`; consume sequence |
| V-15 | `replace_from=G`; first q/v equals old sample exactly | Boundary continuity passes |
| V-16 | New first q differs by exactly `eps_q` | Accept continuity boundary, subject to remaining checks |
| V-17 | New first q differs by more than `eps_q` | Reject `PositionDiscontinuity`; no partial suffix mutation |
| V-18 | New first v differs by more than `eps_v` | Reject `VelocityDiscontinuity`; no partial suffix mutation |
| V-19 | Validation head does not cover the requested replacement point `R` | Reject `TimeGap`; no splice sample is invented |
| V-20 | Candidate contains `capacity+1` points | Reject `CapacityExceeded` before copy beyond fixed storage |
| V-21 | Cubic endpoints legal but internal velocity extremum exceeds limit | Reject `VelocityLimit` |
| V-22 | Cubic extrema legal but conservative stop envelope crosses a soft bound | Reject `NotStoppingViable` |
| V-23 | Accepted update arrives while generation 5 is pending and unconsumed | Build against generation 5, carry its prefix/earliest boundary, publish complete generation 6; RT may consume generation 6 directly |
| V-24 | Update timeout with a long old buffer | Latch `Stopping`; do not continue the entire old future; later updates reject `SessionNotAccepting` |
| V-25 | Horizon falls to low-water at boundary -1 cycle | Stop is already latched before moving queue exhaustion |
| V-26 | Queue exhausts at nonzero desired velocity | Latch `InternalInvariant`; never extrapolate velocity |
| V-27 | Close in `Running` | Accept request; enter `Stopping`; completion observed through state |
| V-28 | Retry the original close request ID in `Stopping` or `Holding` | Return cached accepted response; do not regenerate stop or destroy S1 |
| V-29 | In `Holding`, close S1 with a new finalize request ID | Destroy S1; return to `RollingReady/None` without motion; retries return cached success |
| V-30 | `set_mode(FJT)` in `Holding` before close | Reject `SessionExists`; controller remains rolling |
| V-31 | `set_mode(ROLLING)` while already rolling with S1 | Idempotent mode result; S1/generation unchanged |
| V-32 | Mode request while source position is not stable | Reject `SourceMoving`; do not call controller-manager switch |
| V-33 | Stable source but command/actual error exceeds takeover tolerance | Reject `TakeoverMismatch`; do not switch |
| V-34 | Valid stable switch with an in-flight FJT | Switch response reports controller transition only; FJT terminal status comes from Action result |
| V-35 | Disable races with mode switch | Disable wins; if switch result is ambiguous, `RestartRequired`; no retry |
| V-36 | Controller boot changes to B2, then delayed S1 update arrives | Reject `WrongBoot`; no new session is inferred |
| V-37 | Producer restarts as C2 and sends an update for C1-owned S1 | Reject `WrongClient`; do not consume C1 sequence; C2 must open a new session after S1 closes |
| V-38 | All desired velocities zero when stopping triggers | `T_stop=0`; enter terminal hold without division by zero |
| V-39 | Period is negative or above approved maximum in `Running` | Latch `ClockAnomaly`; advance stop with nominal step, not bad delta |
| V-40 | Rejected high-sequence batch arrives repeatedly | Only first occurrence advances `last_seen`; update age never resets |
| V-41 | Priming receives no accepted batch before prime timeout | Latch `PrimeTimeout`; zero-velocity hold reaches `Holding`; no normal update-age claim |
| V-42 | Prime uses `replace_from=0`, first point `t=0` | Validate without an old candidate; accept only if all other checks pass |
| V-43 | Final point has nonzero qdot but sufficient horizon remains | No implicit terminal rejection/hold; normal low-water rules remain authoritative |
| V-44 | New suffix validates against pending generation 5 but head changes before publish | Restart validation or reject; never publish a mixed-base generation |
| V-45 | Replacement time passes before final publication recheck | Reject `LateReplace`; active/pending images remain coherent |
| V-46 | Source hold is outside a shrunken safe bound | Reject mode/open with `UnsafeHold`; do not switch or create a session |
| V-47 | Correctly addressed new sequence arrives in `Stopping` | Consume sequence, reject `SessionNotAccepting`, do not alter stop |
| V-48 | A new close request ID arrives for S1 while `Stopping` and fewer than eight distinct close IDs are cached | Return/caches accepted; do not regenerate stop; its later retry in `Holding` remains cached and a further new ID is required to finalize |
| V-49 | A ninth distinct non-finalizing close request ID arrives for S1 | Reject `WrongRequest`; keep all eight cached outcomes and the latched stop unchanged |

These vectors are normative. Phase 2 tests may add cases but may not change an expected outcome without updating BQ-130 and this specification first.

## 14. Recovery Rules

- A stopped/holding session never resumes from a new update.
- A producer restart never inherits a session merely because its node name is unchanged.
- A controller restart changes boot ID and invalidates every delayed DDS sample and service retry from the old boot.
- A switch ambiguity never triggers automatic rollback or retry; BQ-080 restart-only handling applies.
- Disable/fault recovery follows existing rt-control lifecycle decisions and creates no rolling session automatically.

## 15. Verification Gate

Before production code depends on this contract:

- independent reviewers must reach the same result for all vectors;
- generated IDL names/types must map each required field without implicit defaults;
- pure C++ tests must ingest V-01 through V-48 where applicable;
- cross-domain documentation and interface schema must be changed atomically;
- no production numeric parameter may be copied from a test fixture.
