# ELECTRI-102 Dynamic Envelope and Stopping Contract

> Status: approved Gate 0 algorithm contract; production numeric envelope is `BLOCKING_TBD`.
> Scope: offline implementation and mock verification only. This document does not authorize controller activation or hardware motion.
> Decision authority: [BQ-130](../BLOCKED-questions.md), [protocol specification](electri-102-protocol-spec.md), [implementation plan](electri-102-implementation-plan.md).

## 1. Critical Safety Question

The critical question is not whether Motion can publish points quickly enough. It is:

> After any accepted point and any subsequent producer interruption, can all 14 axes still stop continuously, within directional dynamic limits and shrunken position bounds, before the committed future is exhausted?

An update is accepted only when the answer is yes for every segment of the complete candidate future. Runtime low-water detection then preserves enough committed time to construct the same bounded stop after the last accepted update.

## 2. Units and Axis Order

All vectors use the fixed protocol order:

```text
right_joint1..6, left_joint1..6, turn, updown
```

The first 13 axes use radians, radians/second, and radians/second squared. `updown` uses metres, metres/second, and metres/second squared. The mathematics is unit-independent, but values from different axis units are never compared or converted implicitly.

For every axis `i`, a complete envelope contains these independently named values:

```text
q_lower_i, q_upper_i
v_limit_i_positive, v_limit_i_negative
a_limit_i_positive, a_limit_i_negative
a_stop_i_positive, a_stop_i_negative
position_margin_i_lower, position_margin_i_upper
```

All magnitudes are finite and strictly positive except position margins, which are finite and non-negative. Position bounds must satisfy:

```text
q_safe_lower_i = q_lower_i + position_margin_i_lower
q_safe_upper_i = q_upper_i - position_margin_i_upper
q_safe_lower_i < q_safe_upper_i
```

`positive` and `negative` describe the velocity direction. A symmetric source may populate both directions with the same approved magnitude, but the implementation must not silently infer symmetry from a single unsigned field.

## 3. Numeric Authority Audit

### 3.1 Current sources

The repository currently contains two conflicting numeric descriptions:

- `src/rt_control/rt_control_bringup/config/joint_limits.yaml` is the single intended bringup-owned limits file under BQ-067. Its metadata says `plc_xml_extracted_requires_low_speed_validation`; the rotary direction and velocity scale still require supervised low-speed validation.
- `src/description/robot_description/urdf/robot.urdf.xacro` contains the robot-description position and velocity bounds currently expanded into `robot_description`.
- The current launch passes `controllers.yaml` and `robot_description`, but does not load `joint_limits.yaml` as a parameter file. Therefore the file's existence is not evidence that a controller enforces it.
- BQ-118 separately freezes the Updown mechanics and target limits, but it does not resolve the conflicting rotary limits or approve a 14-axis rolling stopping envelope.

The observed values are recorded below only to expose conflicts. They are not rolling-controller defaults.

| Axes | `joint_limits.yaml` position | URDF position | `joint_limits.yaml` max velocity | URDF velocity | Acceleration/deceleration evidence | Rolling status |
|---|---|---|---:|---:|---|---|
| right/left J1 | `[-1.57079632679, 1.57079632679] rad` | `[-2.35619449, 2.35619449] rad` | `87.2664626 rad/s` | `1.2 rad/s` | PLC extract has `174.532925 rad/s^2` acceleration only; no approved directional stop values | `BLOCKING_TBD` |
| right/left J2 | `[-1.57079632679, 1.57079632679] rad` | `[-1.57079633, 1.57079633] rad` | `87.2664626 rad/s` | `1.2 rad/s` | Same incomplete PLC evidence | `BLOCKING_TBD` |
| right/left J3 | `[-2.44346095279, 2.44346095279] rad` | `[-3.14159265, 3.14159265] rad` | `87.2664626 rad/s` | `1.2 rad/s` | Same incomplete PLC evidence | `BLOCKING_TBD` |
| right/left J4 | `[-3.14159265359, 3.14159265359] rad` | `[-3.14159265, 3.14159265] rad` | `87.2664626 rad/s` | `1.5 rad/s` | Same incomplete PLC evidence | `BLOCKING_TBD` |
| right/left J5 | `[-2.18166156499, 2.18166156499] rad` | `[-3.14159265, 3.14159265] rad` | `87.2664626 rad/s` | `1.5 rad/s` | Same incomplete PLC evidence | `BLOCKING_TBD` |
| right/left J6 | `[-3.12413936107, 3.12413936107] rad` | `[-3.14159265, 3.14159265] rad` | `87.2664626 rad/s` | `1.5 rad/s` | Same incomplete PLC evidence | `BLOCKING_TBD` |
| `turn` | `[-3.12413936107, 3.12413936107] rad` | same | `87.2664626 rad/s` | same | PLC extract has symmetric acceleration only; direction/velocity scale still unvalidated | `BLOCKING_TBD` |
| `updown` | `[0.0, 0.8] m` | same | `0.3 m/s` | same | BQ-118 records `0.5 m/s^2` maximum acceleration/deceleration; directional margins and rolling stop evidence remain unset | `BLOCKING_TBD` for rolling production |

The very high rotary PLC values and the much lower URDF velocities must not be reconciled by choosing the smaller value without approval. That would still guess the intended source, would not create directional stop limits, and would not prove the configured drives can track the resulting 4 ms command stream.

### 3.2 Required production evidence

A production envelope requires one versioned source that identifies, for every axis:

- the applicable mechanical/soft position bounds in ROS coordinates;
- positive and negative command velocity limits;
- positive and negative trajectory acceleration limits;
- positive and negative normal-stop deceleration limits;
- lower/upper tracking, sampling, quantization, and soft-limit margins;
- evidence revision, units, measurement procedure, approver, and applicable hardware/firmware revision.

The production configuration must carry a non-test `limits_version` derived from that source. Missing, duplicated, non-finite, non-positive, or unresolved values make controller configuration fail. There is no permissive fallback to URDF, PLC extract, FJT tolerance, or a compiled constant.

The existing FJT first-point tolerances (`1 degree` for rotary axes and `0.05 m` for Updown) are admission tolerances. They are not splice tolerances, tracking margins, mode-takeover tolerances, or stopping margins.

### 3.3 Test-only isolation

Pure C++ and mock tests may use a complete synthetic envelope only when all of the following hold:

- the artifact and parameter names contain `test_only`;
- `limits_source=test_only` and a test-only capability flag are visible in public state;
- production bringup contains no `allow_test_only_limits=true` value;
- the controller refuses test-only limits unless an explicit mock/test launch opts in;
- tests assert that the same configuration is rejected when the opt-in is absent;
- no test value is copied into `controllers.yaml` as a production default.

Test-only values prove algorithms and interfaces, not hardware feasibility.

## 4. Cubic Hermite Segment

For a segment from `(q0, v0)` at `t0` to `(q1, v1)` at `t1`, let:

```text
h = t1 - t0, h > 0
s = (t - t0) / h, 0 <= s <= 1

h00(s) =  2s^3 - 3s^2 + 1
h10(s) =    s^3 - 2s^2 + s
h01(s) = -2s^3 + 3s^2
h11(s) =    s^3 -   s^2

q(s) = h00*q0 + h10*h*v0 + h01*q1 + h11*h*v1
```

For implementation and review, the equivalent power coefficients are:

```text
c0 = q0
c1 = h*v0
c2 = -3*q0 - 2*h*v0 + 3*q1 - h*v1
c3 =  2*q0 +   h*v0 - 2*q1 + h*v1

q(s) = c0 + c1*s + c2*s^2 + c3*s^3
v(s) = (c1 + 2*c2*s + 3*c3*s^2) / h
a(s) = (2*c2 + 6*c3*s) / h^2
```

Endpoint evaluation must reproduce `q0/v0` and `q1/v1` within a documented numerical tolerance that is much smaller than the test envelope margins.

### 4.1 Analytic extrema

For each axis and segment:

1. Position candidates are `s=0`, `s=1`, and every real root in `(0,1)` of `c1 + 2*c2*s + 3*c3*s^2 = 0`.
2. Velocity candidates are `s=0`, `s=1`, and `s=-c2/(3*c3)` when `c3` is numerically non-zero and the root lies in `(0,1)`.
3. Acceleration is linear, so its extrema occur at `s=0` or `s=1`.
4. Every evaluated candidate must be finite.

Degenerate linear/quadratic cases use explicit branches; the implementation must not divide by a near-zero coefficient. Quadratic roots use a cancellation-resistant formulation, clamp only roots that are within the documented floating-point root tolerance of `[0,1]`, and then evaluate the original polynomial. A root tolerance must never become a physical limit tolerance.

The resulting values are:

```text
q_segment_min_i, q_segment_max_i
v_segment_positive_i = max(max(v(s), 0))
v_segment_negative_i = max(max(-v(s), 0))
a_segment_positive_i = max(max(a(s), 0))
a_segment_negative_i = max(max(-a(s), 0))
```

The segment passes its direct limits only if all position extrema lie within safe bounds and all directional velocity/acceleration extrema lie at or below their corresponding approved limits.

### 4.2 Splice continuity

At `replace_from_ns=R`, the immutable old candidate is sampled to obtain `q_old(R), v_old(R)`. The first point of the replacement suffix must be at exactly `R` and satisfy, independently per axis:

```text
abs(q_new(R) - q_old(R)) <= splice_position_tolerance_i
abs(v_new(R) - v_old(R)) <= splice_velocity_tolerance_i
```

The tolerances require independent evidence and remain `BLOCKING_TBD` for production. No implicit resampling, blending, clamping, or insertion of an actual-position anchor is allowed. Accepted suffixes are C0/C1 at the protocol boundary; acceleration continuity is not required.

## 5. Synchronous C1 Stop

At a stop trigger, use the current desired state, not potentially noisy finite-difference actual velocity. Define:

```text
v_i_positive = max(v_i, 0)
v_i_negative = max(-v_i, 0)

T_stop = max_i(
  v_i_positive / a_stop_i_positive,
  v_i_negative / a_stop_i_negative)
```

If every velocity is zero, `T_stop=0`; the controller enters `Holding` at the current desired position without division. Otherwise, for local stop time `tau`:

```text
0 <= tau <= T_stop
a_i = -v_i / T_stop
v_i(tau) = v_i + a_i*tau
q_i(tau) = q_i + v_i*tau + 0.5*a_i*tau^2
q_stop_i = q_i + 0.5*v_i*T_stop
```

After `T_stop`, command `q_stop_i` and report desired velocity zero. Because `T_stop` is the maximum across all axes, each non-limiting axis decelerates below its directional maximum and all axes reach zero velocity together.

The stop is C1: position and velocity are continuous. Acceleration can jump at stop entry and exit, so jerk is unbounded in the ideal model. V1 does not claim C2 or jerk-limited stopping. Changing that property requires a new protocol/dynamic-envelope decision because it changes the required horizon.

## 6. Conservative Segment Stopping Viability

Exact coupled 14-axis stopping extrema would correlate every axis velocity at one common sample time. V1 intentionally uses a conservative analytic bound that may reject a viable segment near a limit but cannot accept one merely because extrema occur at different times.

For each cubic segment, first compute the analytic extrema from section 4. Then define:

```text
T_stop_segment_max = max_j(
  v_segment_positive_j / a_stop_j_positive,
  v_segment_negative_j / a_stop_j_negative)

upper_stop_i = q_segment_max_i
             + 0.5 * v_segment_positive_i * T_stop_segment_max

lower_stop_i = q_segment_min_i
             - 0.5 * v_segment_negative_i * T_stop_segment_max
```

The segment is stopping-viable only if, for every axis:

```text
lower_stop_i >= q_safe_lower_i
upper_stop_i <= q_safe_upper_i
```

This combines position and velocity extrema that may occur at different times and uses the worst synchronous stopping time of any axis. That is the deliberate source of conservatism. Replacing it with time-grid sampling is forbidden unless the grid error, controller-period jitter, and between-sample extrema are separately bounded and approved.

Every accepted segment, including the terminal segment, must pass this test. Acceptance may not rely on an unvalidated final zero-velocity point to make earlier samples safe.

## 7. Horizon and Low-Water Guard

At execution time `E`:

```text
H_available = buffered_until - E
```

Running is permitted only while:

```text
H_available > T_stop(E) + scheduling_guard(E)
```

Equality is unsafe and latches `LowWater`; the strict inequality leaves the guard intact. The guard is a time quantity whose approved bound includes:

```text
scheduling_guard =
  one_cycle_detection_bound
  + stop_time_growth_during_detection
  + accepted_non_rt_to_rt_visibility_bound
  + controller_period_and_sampling_quantization_bound
```

- `one_cycle_detection_bound` covers the worst time until the 250 Hz update loop observes timeout/low-water.
- `stop_time_growth_during_detection` bounds any increase in required synchronous stop time while the old trajectory advances through that detection interval. It is derived from the candidate segment extrema, not guessed as zero.
- `accepted_non_rt_to_rt_visibility_bound` covers measured jitter between complete non-RT validation/publication and RT consumption when a just-in-time valid generation is considered available.
- `controller_period_and_sampling_quantization_bound` covers execution cursor and stop completion rounding to update cycles.

The implementation may combine these into one configured duration, but public state/evidence must retain the components and their source. All four are `BLOCKING_TBD` for production. Tests use explicit `test_only_*` components and exercise each boundary independently.

Update timeout and low-water are separate triggers. A long old buffer does not authorize continued motion after the last accepted update exceeds its timeout. Conversely, recent traffic does not authorize motion when committed horizon is too short to stop.

## 8. Mode-Switch Stability and Takeover

The rolling protocol does not estimate actual velocity during normal execution. Finite differences are used only as a pre-switch quiescence gate.

For `N` consecutive complete 14-axis state samples at validated controller periods `dt_k`:

```text
actual_velocity_estimate_i,k =
  abs(actual_position_i,k - actual_position_i,k-1) / dt_k

actual_velocity_estimate_i,k <= stable_velocity_threshold_i
abs(source_command_i,k - actual_position_i,k) <= takeover_error_tolerance_i
```

All actual positions, source commands, periods, estimates, and thresholds must be finite. `N >= 2`. The gate resets on a missing/non-finite state, invalid period, controller transition, disable/fault request, or threshold violation.

`N`, `stable_velocity_threshold_i`, and `takeover_error_tolerance_i` are independent `BLOCKING_TBD` production evidence. They are not derived from `/joint_states`, the FJT first-point admission tolerance, or splice tolerances.

After a successful strict switch, the rolling controller's first update cycle holds the validated persisted source position command. It must not substitute the latest actual position. This preserves command C0 across controller ownership; the subsequent rolling prime must splice from the advertised hold state.

## 9. Runtime Validation Order

To make rejection deterministic and avoid partial mutation, a correctly addressed new sequence is validated in this order after its sequence is consumed:

1. shape, count, capacity, finite values, and strictly increasing time;
2. replacement boundary and immutable-prefix coverage;
3. C0/C1 splice continuity;
4. per-segment analytic position/velocity/acceleration extrema;
5. conservative stopping viability for every segment;
6. committed horizon and configured guard;
7. publish one complete pending generation.

The first failure determines `RejectCode`. Identity/ownership and stale-sequence checks from the protocol run before this payload order. No validation step may clamp a point into compliance. A rejection leaves active and pending trajectory images unchanged and does not reset accepted-update age.

## 10. Verification Vectors

The following cases are normative for Phase 2 tests. Every numeric fixture must identify itself as test-only.

| ID | Case | Required assertion |
|---|---|---|
| D-01 | Constant-position cubic | q is constant; v/a are zero; all extrema are endpoints |
| D-02 | Constant-velocity cubic | q/v match analytic values; acceleration is zero |
| D-03 | Interior position extremum | derivative roots are found and position violation is rejected even when endpoints pass |
| D-04 | Interior velocity extremum | acceleration root is found and velocity violation is rejected even when endpoint velocities pass |
| D-05 | Positive/negative acceleration limits differ | the correct directional limit selects the outcome |
| D-06 | Degenerate near-linear polynomial | no division by zero/NaN; reference sample agreement remains bounded |
| D-07 | Exact splice tolerance | equality passes; one representable step beyond it fails |
| D-08 | All 14 velocities zero | `T_stop=0`, no division, immediate zero-velocity hold |
| D-09 | One limiting positive axis | synchronous duration and every axis deceleration obey directional limits |
| D-10 | One limiting negative axis | negative directional stop limit determines duration |
| D-11 | Mixed 14-axis velocities | all axes stop at the same time and terminal positions match the formula |
| D-12 | Stop exactly at safe bound | equality passes; one representable step outside fails |
| D-13 | Segment q/v extrema at different times | conservative bound may reject but never underestimates the sampled exact stop envelope |
| D-14 | Horizon exactly `T_stop+guard` | low-water latches because the running inequality is strict |
| D-15 | Stop-time growth during detection | guard covers the higher next-cycle stop requirement |
| D-16 | Timeout with long horizon | stopping latches from timeout; old future is not consumed to its end |
| D-17 | Recent update with insufficient horizon | stopping latches from low-water |
| D-18 | Test-only limits without explicit opt-in | controller configuration fails |
| D-19 | Production config with any TBD/missing axis field | controller configuration fails and claims no command interface |
| D-20 | Stable actual but takeover error too large | mode switch is rejected before controller-manager switch |
| D-21 | Source command matches but actual finite difference is moving | mode switch is rejected and stability history resets |
| D-22 | Successful takeover | first rolling command equals the validated source command exactly |

Analytic extrema tests compare against dense long-double reference sampling plus hand-derived boundary cases. Dense sampling is a test oracle aid only; production acceptance uses the analytic rules above.

## 11. Production Unblock Checklist

Mock Checkpoint F1 may pass while every item below remains open. Production or hardware work remains blocked until all are closed:

- one approved 14-axis numeric authority supersedes or reconciles the current URDF/PLC conflict;
- directional velocity, acceleration, and normal-stop deceleration are complete for every axis;
- position margins, splice tolerances, stability/takeover thresholds, horizon, timeout, and all guard components have measured evidence;
- `joint_limits.yaml` or its approved successor is actually wired into the controller configuration with a version identity;
- target scheduling/jitter measurements prove the guard and update-rate assumptions;
- mock/fake results, sanitizers, allocation/locking audit, and normal FJT regression are green;
- separate authorization is granted for target-machine non-motion tests, controller activation, and each hardware-motion stage.

Until then, public state must identify the running envelope as test-only and no document may describe ELECTRI-102 as production-ready.
