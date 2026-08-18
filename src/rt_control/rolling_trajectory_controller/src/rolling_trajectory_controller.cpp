#include "rolling_trajectory_controller/rolling_trajectory_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription_options.hpp"

namespace rolling_trajectory_controller
{
namespace
{

constexpr std::array<std::uint8_t, 32> kAxisSetHash = {
  0x25U, 0xc6U, 0xe8U, 0x2bU, 0xf5U, 0x05U, 0xcaU, 0x9eU,
  0xb9U, 0x9dU, 0xb1U, 0xc6U, 0x45U, 0xabU, 0x75U, 0xd7U,
  0xecU, 0xdeU, 0x01U, 0x53U, 0xfaU, 0xafU, 0x6aU, 0x74U,
  0x92U, 0xc6U, 0x21U, 0x0cU, 0x4dU, 0x36U, 0x25U, 0x26U};

constexpr std::array<std::uint8_t, 32> kTestOnlyLimitsVersion = {
  0xa1U, 0xc3U, 0x34U, 0x03U, 0xa1U, 0x37U, 0x0eU, 0xf8U,
  0x21U, 0x05U, 0xc8U, 0x48U, 0x8bU, 0x58U, 0x68U, 0x74U,
  0xacU, 0xa7U, 0x7cU, 0xa3U, 0x29U, 0x6eU, 0xeaU, 0xceU,
  0xbeU, 0x53U, 0xd2U, 0x41U, 0x3bU, 0xc1U, 0x12U, 0x3bU};

std::uint64_t splitMix64(std::uint64_t & state) noexcept
{
  state += 0x9e3779b97f4a7c15ULL;
  std::uint64_t value = state;
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

class BlockingFlagGuard
{
public:
  explicit BlockingFlagGuard(std::atomic_flag & flag) noexcept
  : flag_(flag)
  {
    while (flag_.test_and_set(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
  }

  ~BlockingFlagGuard() noexcept
  {
    flag_.clear(std::memory_order_release);
  }

  BlockingFlagGuard(const BlockingFlagGuard &) = delete;
  BlockingFlagGuard & operator=(const BlockingFlagGuard &) = delete;

private:
  std::atomic_flag & flag_;
};

class OptionalRtFlagGuard
{
public:
  OptionalRtFlagGuard(std::atomic_flag & flag, bool required) noexcept
  : flag_(flag), required_(required), owns_(!required || !flag_.test_and_set(
        std::memory_order_acquire))
  {
  }

  ~OptionalRtFlagGuard() noexcept
  {
    if (required_ && owns_) {
      flag_.clear(std::memory_order_release);
    }
  }

  OptionalRtFlagGuard(const OptionalRtFlagGuard &) = delete;
  OptionalRtFlagGuard & operator=(const OptionalRtFlagGuard &) = delete;

  [[nodiscard]] bool mayProceed() const noexcept
  {
    return owns_;
  }

private:
  std::atomic_flag & flag_;
  bool required_{false};
  bool owns_{false};
};

class RtStateVersionGuard
{
public:
  explicit RtStateVersionGuard(std::atomic<std::uint64_t> & version) noexcept
  : version_(version)
  {
    version_.fetch_add(1U, std::memory_order_acq_rel);
  }

  ~RtStateVersionGuard() noexcept
  {
    version_.fetch_add(1U, std::memory_order_release);
  }

  RtStateVersionGuard(const RtStateVersionGuard &) = delete;
  RtStateVersionGuard & operator=(const RtStateVersionGuard &) = delete;

private:
  std::atomic<std::uint64_t> & version_;
};

DynamicEnvelope makeTestOnlyEnvelope() noexcept
{
  DynamicEnvelope envelope;
  envelope.source = LimitsSource::kTestOnly;
  envelope.limits_version = kTestOnlyLimitsVersion;
  for (AxisEnvelope & axis : envelope.axes) {
    axis.position_lower = -10.0;
    axis.position_upper = 10.0;
    axis.velocity_positive = 10.0;
    axis.velocity_negative = 10.0;
    axis.acceleration_positive = 10.0;
    axis.acceleration_negative = 10.0;
    axis.stop_acceleration_positive = 10.0;
    axis.stop_acceleration_negative = 10.0;
    axis.position_margin_lower = 0.01;
    axis.position_margin_upper = 0.01;
  }
  return envelope;
}

rclcpp::QoS makePrototypeQRollingCommand()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

rclcpp::QoS makePrototypeQRollingState()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

}  // namespace

controller_interface::CallbackReturn RollingTrajectoryController::on_init()
{
  auto_declare<std::string>("configuration_source", "unconfigured");
  auto_declare<bool>("allow_test_only_configuration", false);
  auto_declare<std::vector<double>>("test_only_takeover_tolerances", {});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
RollingTrajectoryController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  configuration.names.reserve(kAxisCount);
  for (const char * joint : kJointNames) {
    configuration.names.emplace_back(std::string(joint) + "/position");
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
RollingTrajectoryController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  configuration.names.reserve(kAxisCount + 1U);
  for (const char * joint : kJointNames) {
    configuration.names.emplace_back(std::string(joint) + "/position");
  }
  configuration.names.emplace_back(kFeedbackAgeInterface);
  return configuration;
}

controller_interface::CallbackReturn RollingTrajectoryController::on_configure(
  const rclcpp_lifecycle::State &)
{
  configured_ = false;
  active_.store(false, std::memory_order_release);
  const std::string source = get_node()->get_parameter("configuration_source").as_string();
  const bool allow_test_only =
    get_node()->get_parameter("allow_test_only_configuration").as_bool();
  const std::vector<double> tolerances =
    get_node()->get_parameter("test_only_takeover_tolerances").as_double_array();
  if (source != "test_only" || !allow_test_only || tolerances.size() != kAxisCount) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Rolling controller requires an explicitly allowed complete test-only configuration");
    return controller_interface::CallbackReturn::ERROR;
  }
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    if (!std::isfinite(tolerances[axis]) || tolerances[axis] < 0.0) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Invalid test-only takeover tolerance at axis %zu", axis);
      return controller_interface::CallbackReturn::ERROR;
    }
    takeover_tolerances_[axis] = tolerances[axis];
  }

  BufferConfiguration buffer_configuration;
  buffer_configuration.capacity = kTestOnlyBufferCapacity;
  buffer_configuration.required_initial_horizon_ns = kTestOnlyRequiredInitialHorizonNs;
  buffer_configuration.splice_position_tolerance.fill(1.0e-9);
  buffer_configuration.splice_velocity_tolerance.fill(1.0e-9);
  envelope_ = makeTestOnlyEnvelope();
  if (
    !buffer_.configure(buffer_configuration) ||
    !buffer_.configureLimits(envelope_, true))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure rolling buffer test envelope");
    return controller_interface::CallbackReturn::ERROR;
  }
  rt_stop_trajectory_ = StopTrajectory{};
  if (!rt_stop_trajectory_.configure(envelope_, true)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure rolling stop trajectory");
    return controller_interface::CallbackReturn::ERROR;
  }

  callback_group_ =
    get_node()->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  state_publisher_ = get_node()->create_publisher<StateMessage>(
    "/rt/rolling_joint_control/state", makePrototypeQRollingState());
  state_timer_ = get_node()->create_wall_timer(
    std::chrono::nanoseconds(kTestOnlyStatePublishPeriodNs),
    [this]() {(void)publishPublicState();}, callback_group_);
  state_timer_->cancel();
  open_service_ = get_node()->create_service<OpenService>(
    "/rt/rolling_joint_control/open",
    std::bind(
      &RollingTrajectoryController::handleOpen, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, callback_group_);
  close_service_ = get_node()->create_service<CloseService>(
    "/rt/rolling_joint_control/close",
    std::bind(
      &RollingTrajectoryController::handleClose, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, callback_group_);
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group_;
  update_subscription_ = get_node()->create_subscription<BatchMessage>(
    "/rt/rolling_joint_control/update", makePrototypeQRollingCommand(),
    std::bind(&RollingTrajectoryController::handleUpdate, this, std::placeholders::_1),
    subscription_options);

  hold_positions_.fill(0.0);
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    observed_position_bits_[axis].store(encodeDouble(0.0), std::memory_order_relaxed);
    desired_position_bits_[axis].store(encodeDouble(0.0), std::memory_order_relaxed);
    desired_velocity_bits_[axis].store(encodeDouble(0.0), std::memory_order_relaxed);
  }
  feedback_age_bits_.store(encodeDouble(0.0), std::memory_order_relaxed);
  rt_session_state_.store(
    static_cast<std::uint8_t>(SessionState::kNone), std::memory_order_relaxed);
  stop_reason_.store(
    static_cast<std::uint8_t>(StopReason::kNone), std::memory_order_relaxed);
  rt_invariant_stage_.store(
    static_cast<std::uint8_t>(RtInvariantStage::kNone), std::memory_order_relaxed);
  execution_time_ns_.store(0U, std::memory_order_relaxed);
  active_generation_.store(0U, std::memory_order_relaxed);
  buffered_until_ns_.store(0U, std::memory_order_relaxed);
  replaceable_from_ns_.store(0U, std::memory_order_relaxed);
  prime_start_time_ns_.store(0U, std::memory_order_relaxed);
  session_epoch_.store(0U, std::memory_order_relaxed);
  session_publication_floor_.store(1U, std::memory_order_relaxed);
  rt_state_version_.store(0U, std::memory_order_relaxed);
  rt_state_session_epoch_.store(0U, std::memory_order_relaxed);
  rt_accepted_arrival_time_ns_.store(0U, std::memory_order_relaxed);
  timeout_count_.store(0U, std::memory_order_relaxed);
  invariant_failure_count_.store(0U, std::memory_order_relaxed);
  has_accepted_update_.store(false, std::memory_order_relaxed);
  initial_handoff_gate_.clear(std::memory_order_relaxed);
  configured_ = true;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RollingTrajectoryController::on_activate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false, std::memory_order_release);
  if (
    !configured_ || command_interfaces_.size() != kAxisCount ||
    state_interfaces_.size() != kAxisCount + 1U)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Rolling controller requires 14 position command/state pairs and feedback age");
    return controller_interface::CallbackReturn::ERROR;
  }

  std::array<double, kAxisCount> source_commands{};
  std::array<double, kAxisCount> actual_positions{};
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    const std::string expected_name = std::string(kJointNames[axis]) + "/position";
    if (
      command_interfaces_[axis].get_name() != expected_name ||
      state_interfaces_[axis].get_name() != expected_name)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unexpected position interface order at axis %zu", axis);
      return controller_interface::CallbackReturn::ERROR;
    }
    const double source_command = command_interfaces_[axis].get_value();
    const double actual_position = state_interfaces_[axis].get_value();
    if (
      !std::isfinite(source_command) || !std::isfinite(actual_position) ||
      std::abs(source_command - actual_position) > takeover_tolerances_[axis])
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unsafe rolling-controller takeover at axis %zu", axis);
      return controller_interface::CallbackReturn::ERROR;
    }
    source_commands[axis] = source_command;
    actual_positions[axis] = actual_position;
  }
  if (state_interfaces_[kAxisCount].get_name() != kFeedbackAgeInterface) {
    RCLCPP_ERROR(get_node()->get_logger(), "Missing shared feedback-age state interface");
    return controller_interface::CallbackReturn::ERROR;
  }
  const double feedback_age_ms = state_interfaces_[kAxisCount].get_value();

  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (
      buffer_.sessionState() == SessionState::kTerminated &&
      !buffer_.resetTerminated())
    {
      return controller_interface::CallbackReturn::ERROR;
    }
    if (buffer_.sessionState() != SessionState::kNone) {
      RCLCPP_ERROR(get_node()->get_logger(), "Rolling session still exists during activation");
      return controller_interface::CallbackReturn::ERROR;
    }
    controller_boot_id_ = generateIdentifier();
    open_cache_ = OpenCacheEntry{};
    clearActiveCloseCache();
    finalize_cache_ = CloseCacheEntry{};
    last_reject_code_ = RejectCode::kNone;
    last_rejected_sequence_ = 0U;
    accepted_count_ = 0U;
    rejected_count_ = 0U;
    superseded_pending_count_ = 0U;
    timeout_count_.store(0U, std::memory_order_release);
    invariant_failure_count_.store(0U, std::memory_order_release);
    last_service_error_ = OpenService::Response::ERROR_NONE;
    stop_requested_.store(false, std::memory_order_release);
  }

  hold_positions_ = source_commands;
  rt_active_trajectory_ = TrajectoryImage{};
  rt_identity_ = SessionIdentity{};
  rt_desired_ = JointPoint{};
  rt_desired_.positions = source_commands;
  rt_desired_.velocities.fill(0.0);
  rt_stop_trajectory_ = StopTrajectory{};
  if (!rt_stop_trajectory_.configure(envelope_, true)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to reset rolling stop trajectory");
    return controller_interface::CallbackReturn::ERROR;
  }
  rt_observed_session_epoch_ = 0U;
  rt_publication_floor_ = 1U;
  rt_consumed_publication_sequence_ = 0U;
  rt_last_accepted_arrival_ns_ = 0U;
  rt_accepted_arrival_time_ns_.store(0U, std::memory_order_release);
  rt_stop_elapsed_ns_ = 0U;
  rt_session_state_.store(
    static_cast<std::uint8_t>(SessionState::kNone), std::memory_order_release);
  stop_reason_.store(
    static_cast<std::uint8_t>(StopReason::kNone), std::memory_order_release);
  rt_invariant_stage_.store(
    static_cast<std::uint8_t>(RtInvariantStage::kNone), std::memory_order_release);
  execution_time_ns_.store(0U, std::memory_order_release);
  active_generation_.store(0U, std::memory_order_release);
  buffered_until_ns_.store(0U, std::memory_order_release);
  replaceable_from_ns_.store(0U, std::memory_order_release);
  prime_start_time_ns_.store(0U, std::memory_order_release);
  session_epoch_.store(0U, std::memory_order_release);
  session_publication_floor_.store(1U, std::memory_order_release);
  rt_state_version_.store(0U, std::memory_order_release);
  rt_state_session_epoch_.store(0U, std::memory_order_release);
  rt_accepted_arrival_time_ns_.store(0U, std::memory_order_release);
  has_accepted_update_.store(false, std::memory_order_release);
  initial_handoff_gate_.clear(std::memory_order_release);
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    observed_position_bits_[axis].store(
      encodeDouble(actual_positions[axis]), std::memory_order_release);
    desired_position_bits_[axis].store(
      encodeDouble(source_commands[axis]), std::memory_order_release);
    desired_velocity_bits_[axis].store(encodeDouble(0.0), std::memory_order_release);
  }
  feedback_age_bits_.store(encodeDouble(feedback_age_ms), std::memory_order_release);
  active_.store(true, std::memory_order_release);
  if (state_timer_) {
    state_timer_->reset();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RollingTrajectoryController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    active_.store(false, std::memory_order_release);
    buffer_.terminateSession();
    controller_boot_id_ = Identifier{};
    open_cache_ = OpenCacheEntry{};
    clearActiveCloseCache();
    finalize_cache_ = CloseCacheEntry{};
    stop_requested_.store(false, std::memory_order_release);
    stop_reason_.store(
      static_cast<std::uint8_t>(StopReason::kControllerDeactivated),
      std::memory_order_release);
    rt_session_state_.store(
      static_cast<std::uint8_t>(SessionState::kTerminated),
      std::memory_order_release);
    has_accepted_update_.store(false, std::memory_order_release);
    const std::uint64_t new_epoch = session_epoch_.fetch_add(
      1U, std::memory_order_acq_rel) + 1U;
    rt_state_session_epoch_.store(new_epoch, std::memory_order_release);
    rt_accepted_arrival_time_ns_.store(0U, std::memory_order_release);
    const SessionIdentity invalid_identity{};
    const TrajectoryImage invalid_trajectory{};
    (void)snapshot_exchange_.publish(invalid_identity, invalid_trajectory, steadyNowNs());
  }
  (void)publishPublicState();
  if (state_timer_) {
    state_timer_->cancel();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RollingTrajectoryController::update(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!active_.load(std::memory_order_acquire)) {
    return controller_interface::return_type::OK;
  }
  RtStateVersionGuard state_version(rt_state_version_);

  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    observed_position_bits_[axis].store(
      encodeDouble(state_interfaces_[axis].get_value()), std::memory_order_release);
  }
  feedback_age_bits_.store(
    encodeDouble(state_interfaces_[kAxisCount].get_value()), std::memory_order_release);

  const std::int64_t period_ns = period.nanoseconds();
  SessionState state = SessionState::kNone;
  bool coherent_session = false;
  constexpr std::size_t kMaximumEpochReadAttempts = 3U;
  for (std::size_t attempt = 0U; attempt < kMaximumEpochReadAttempts; ++attempt) {
    const std::uint64_t epoch_before = session_epoch_.load(std::memory_order_acquire);
    if (epoch_before != rt_observed_session_epoch_ && !resetRtEpoch(epoch_before)) {
      return controller_interface::return_type::ERROR;
    }
    state = static_cast<SessionState>(
      rt_session_state_.load(std::memory_order_acquire));
    const std::uint64_t epoch_after = session_epoch_.load(std::memory_order_acquire);
    if (epoch_before == epoch_after) {
      coherent_session = true;
      break;
    }
  }
  if (!coherent_session) {
    return writeRtDesired() ?
           controller_interface::return_type::OK :
           controller_interface::return_type::ERROR;
  }

  bool update_succeeded = true;
  if (state == SessionState::kPriming) {
    if (stop_requested_.load(std::memory_order_acquire)) {
      StopReason reason = static_cast<StopReason>(
        stop_reason_.load(std::memory_order_acquire));
      if (reason == StopReason::kNone) {
        reason = StopReason::kGracefulClose;
      }
      update_succeeded = beginRtStop(reason);
    } else if (
      elapsedExceeds(
        steadyNowNs(), prime_start_time_ns_.load(std::memory_order_acquire),
        kTestOnlyPrimeTimeoutNs))
    {
      update_succeeded = beginRtStop(StopReason::kPrimeTimeout);
    } else {
      OptionalRtFlagGuard handoff(initial_handoff_gate_, true);
      if (handoff.mayProceed()) {
        const SnapshotConsumeResult consumed = consumeLatestRtTrajectory(true);
        if (consumed == SnapshotConsumeResult::kInvalid) {
          update_succeeded = beginRtStop(StopReason::kInternalInvariant);
        }
      }
    }
  } else if (state == SessionState::kRunning) {
    if (stop_requested_.load(std::memory_order_acquire)) {
      StopReason reason = static_cast<StopReason>(
        stop_reason_.load(std::memory_order_acquire));
      if (reason == StopReason::kNone) {
        reason = StopReason::kGracefulClose;
      }
      update_succeeded = beginRtStop(reason);
    } else {
      const bool initial_handoff =
        execution_time_ns_.load(std::memory_order_relaxed) == 0U;
      OptionalRtFlagGuard handoff(initial_handoff_gate_, initial_handoff);
      if (!handoff.mayProceed()) {
        update_succeeded = true;
      } else {
        const SnapshotConsumeResult consumed = consumeLatestRtTrajectory(false);
        if (consumed == SnapshotConsumeResult::kInvalid) {
          update_succeeded = beginRtStop(StopReason::kInternalInvariant);
        } else if (!validPeriod(period_ns)) {
          update_succeeded = beginRtStop(StopReason::kClockAnomaly);
        } else if (
          elapsedExceeds(
            steadyNowNs(), rt_last_accepted_arrival_ns_, kTestOnlyUpdateTimeoutNs))
        {
          update_succeeded = beginRtStop(StopReason::kUpdateTimeout);
        } else {
          std::uint64_t stop_duration_ns = 0U;
          const std::uint64_t execution_ns =
            execution_time_ns_.load(std::memory_order_relaxed);
          const std::uint64_t buffered_until_ns =
            buffered_until_ns_.load(std::memory_order_relaxed);
          constexpr SchedulingGuard guard{
            kTestOnlyNominalPeriodNs, kTestOnlyNominalPeriodNs,
            kTestOnlyNominalPeriodNs, kTestOnlyNominalPeriodNs};
          if (!calculateStopDurationNs(rt_desired_, stop_duration_ns)) {
            rt_invariant_stage_.store(
              static_cast<std::uint8_t>(RtInvariantStage::kStopDuration),
              std::memory_order_release);
            update_succeeded = beginRtStop(StopReason::kInternalInvariant);
          } else if (!hasSufficientStoppingHorizon(
              execution_ns, buffered_until_ns, stop_duration_ns, guard))
          {
            update_succeeded = beginRtStop(StopReason::kLowWater);
          } else {
            std::uint64_t next_execution_ns = 0U;
            JointPoint candidate;
            const std::uint64_t increment_ns = static_cast<std::uint64_t>(period_ns);
            if (
              !addTime(execution_ns, increment_ns, next_execution_ns) ||
              !sampleTrajectoryImage(
                rt_active_trajectory_, next_execution_ns, candidate) ||
              !desiredIsValid(candidate))
            {
              rt_invariant_stage_.store(
                static_cast<std::uint8_t>(RtInvariantStage::kTrajectorySample),
                std::memory_order_release);
              update_succeeded = beginRtStop(StopReason::kInternalInvariant);
            } else {
              rt_desired_ = candidate;
              execution_time_ns_.store(next_execution_ns, std::memory_order_release);
              update_succeeded = updateRtReplaceableBoundary();
              if (!update_succeeded) {
                rt_invariant_stage_.store(
                  static_cast<std::uint8_t>(RtInvariantStage::kReplaceableBoundary),
                  std::memory_order_release);
                update_succeeded = beginRtStop(StopReason::kInternalInvariant);
              }
            }
          }
        }
      }
    }
  } else if (state == SessionState::kStopping) {
    update_succeeded = advanceRtStop(period_ns);
  } else if (state == SessionState::kHolding || state == SessionState::kNone) {
    rt_desired_.velocities.fill(0.0);
  }

  if (!update_succeeded || !writeRtDesired()) {
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

Identifier RollingTrajectoryController::generateIdentifier() noexcept
{
  static std::atomic<std::uint64_t> counter{0U};
  const auto system_ticks = std::chrono::system_clock::now().time_since_epoch().count();
  const auto steady_ticks = std::chrono::steady_clock::now().time_since_epoch().count();
  std::uint64_t state =
    static_cast<std::uint64_t>(system_ticks) ^
    (static_cast<std::uint64_t>(steady_ticks) << 1U) ^
    counter.fetch_add(1U, std::memory_order_relaxed);
  Identifier identifier{};
  for (std::size_t word_index = 0U; word_index < 2U; ++word_index) {
    const std::uint64_t word = splitMix64(state);
    for (std::size_t byte = 0U; byte < sizeof(word); ++byte) {
      identifier[word_index * sizeof(word) + byte] =
        static_cast<std::uint8_t>(word >> (byte * 8U));
    }
  }
  identifier[6] = static_cast<std::uint8_t>((identifier[6] & 0x0fU) | 0x40U);
  identifier[8] = static_cast<std::uint8_t>((identifier[8] & 0x3fU) | 0x80U);
  return identifier;
}

std::uint64_t RollingTrajectoryController::steadyNowNs() noexcept
{
  const auto elapsed = std::chrono::steady_clock::now().time_since_epoch();
  const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count();
  return nanoseconds < 0 ? 0U : static_cast<std::uint64_t>(nanoseconds);
}

std::uint64_t RollingTrajectoryController::encodeDouble(double value) noexcept
{
  std::uint64_t bits = 0U;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

double RollingTrajectoryController::decodeDouble(std::uint64_t bits) noexcept
{
  double value = 0.0;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

bool RollingTrajectoryController::isZeroIdentifier(const Identifier & identifier) noexcept
{
  return std::all_of(
    identifier.begin(), identifier.end(), [](std::uint8_t value) {return value == 0U;});
}

bool RollingTrajectoryController::sameOpenRequest(
  const OpenService::Request & lhs, const OpenService::Request & rhs) noexcept
{
  return lhs.protocol_major == rhs.protocol_major && lhs.protocol_minor == rhs.protocol_minor &&
         lhs.request_id.uuid == rhs.request_id.uuid &&
         lhs.expected_controller_boot_id.uuid == rhs.expected_controller_boot_id.uuid &&
         lhs.client_instance_id.uuid == rhs.client_instance_id.uuid &&
         lhs.axis_set_hash == rhs.axis_set_hash;
}

bool RollingTrajectoryController::sameCloseRequest(
  const CloseService::Request & lhs, const CloseService::Request & rhs) noexcept
{
  return lhs.protocol_major == rhs.protocol_major && lhs.protocol_minor == rhs.protocol_minor &&
         lhs.request_id.uuid == rhs.request_id.uuid &&
         lhs.controller_boot_id.uuid == rhs.controller_boot_id.uuid &&
         lhs.session_id.uuid == rhs.session_id.uuid &&
         lhs.client_instance_id.uuid == rhs.client_instance_id.uuid;
}

bool RollingTrajectoryController::elapsedExceeds(
  std::uint64_t now_ns, std::uint64_t start_ns, std::uint64_t limit_ns) noexcept
{
  return now_ns < start_ns || now_ns - start_ns > limit_ns;
}

bool RollingTrajectoryController::validPeriod(std::int64_t period_ns) noexcept
{
  return period_ns > 0 &&
         static_cast<std::uint64_t>(period_ns) <= kTestOnlyMaximumPeriodNs;
}

bool RollingTrajectoryController::addTime(
  std::uint64_t value_ns, std::uint64_t increment_ns, std::uint64_t & sum_ns) noexcept
{
  if (increment_ns > std::numeric_limits<std::uint64_t>::max() - value_ns) {
    return false;
  }
  sum_ns = value_ns + increment_ns;
  return true;
}

bool RollingTrajectoryController::synchronizeBufferStateFromRt() noexcept
{
  const SessionState rt_state = static_cast<SessionState>(
    rt_session_state_.load(std::memory_order_acquire));
  SessionState buffer_state = buffer_.sessionState();

  if (
    rt_state == SessionState::kRunning &&
    (buffer_state == SessionState::kPriming || buffer_state == SessionState::kRunning))
  {
    const std::uint64_t active_generation =
      active_generation_.load(std::memory_order_acquire);
    if (!buffer_.acknowledgeActiveGeneration(active_generation)) {
      return false;
    }
    buffer_state = buffer_.sessionState();
  }
  if (
    (rt_state == SessionState::kStopping || rt_state == SessionState::kHolding) &&
    (buffer_state == SessionState::kPriming || buffer_state == SessionState::kRunning))
  {
    if (!buffer_.requestStop()) {
      return false;
    }
    buffer_state = buffer_.sessionState();
  }
  if (rt_state == SessionState::kHolding && buffer_state == SessionState::kStopping) {
    if (!buffer_.markHolding()) {
      return false;
    }
    buffer_state = buffer_.sessionState();
  }
  if (rt_state == SessionState::kTerminated) {
    if (buffer_state != SessionState::kTerminated) {
      buffer_.terminateSession();
    }
    return true;
  }

  if (rt_state == SessionState::kRunning) {
    return buffer_state == SessionState::kRunning;
  }
  if (rt_state == SessionState::kStopping) {
    return buffer_state == SessionState::kStopping;
  }
  if (rt_state == SessionState::kHolding) {
    return buffer_state == SessionState::kHolding;
  }
  return true;
}

bool RollingTrajectoryController::readRtStateView(RtStateView & view) const noexcept
{
  constexpr std::size_t kMaximumReadAttempts = 8U;
  for (std::size_t attempt = 0U; attempt < kMaximumReadAttempts; ++attempt) {
    const std::uint64_t version_before =
      rt_state_version_.load(std::memory_order_acquire);
    if ((version_before & 1U) != 0U) {
      continue;
    }

    RtStateView candidate;
    candidate.session_epoch =
      rt_state_session_epoch_.load(std::memory_order_acquire);
    candidate.session_state = static_cast<SessionState>(
      rt_session_state_.load(std::memory_order_acquire));
    candidate.stop_reason = static_cast<StopReason>(
      stop_reason_.load(std::memory_order_acquire));
    candidate.active_generation =
      active_generation_.load(std::memory_order_acquire);
    candidate.execution_time_ns =
      execution_time_ns_.load(std::memory_order_acquire);
    candidate.replaceable_from_ns =
      replaceable_from_ns_.load(std::memory_order_acquire);
    candidate.buffered_until_ns =
      buffered_until_ns_.load(std::memory_order_acquire);
    candidate.prime_start_time_ns =
      prime_start_time_ns_.load(std::memory_order_acquire);
    candidate.accepted_arrival_time_ns =
      rt_accepted_arrival_time_ns_.load(std::memory_order_acquire);
    candidate.timeout_count = timeout_count_.load(std::memory_order_acquire);
    candidate.invariant_failure_count =
      invariant_failure_count_.load(std::memory_order_acquire);
    candidate.has_accepted_update =
      has_accepted_update_.load(std::memory_order_acquire);
    candidate.desired.time_ns = candidate.execution_time_ns;
    for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
      candidate.desired.positions[axis] = decodeDouble(
        desired_position_bits_[axis].load(std::memory_order_acquire));
      candidate.desired.velocities[axis] = decodeDouble(
        desired_velocity_bits_[axis].load(std::memory_order_acquire));
    }

    const std::uint64_t version_after =
      rt_state_version_.load(std::memory_order_acquire);
    if (
      version_before == version_after && (version_after & 1U) == 0U &&
      candidate.session_state <= SessionState::kTerminated &&
      candidate.stop_reason <= StopReason::kControllerRestart &&
      desiredIsValid(candidate.desired))
    {
      view = candidate;
      return true;
    }
  }
  return false;
}

bool RollingTrajectoryController::buildPublicState(StateMessage & state)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  RtStateView view;
  bool coherent = false;
  constexpr std::size_t kMaximumStateBuildAttempts = 8U;
  for (std::size_t attempt = 0U; attempt < kMaximumStateBuildAttempts; ++attempt) {
    if (!synchronizeBufferStateFromRt() || !readRtStateView(view)) {
      continue;
    }
    const std::uint64_t session_epoch = session_epoch_.load(std::memory_order_acquire);
    if (view.session_epoch != session_epoch) {
      continue;
    }

    const SessionState buffer_state = buffer_.sessionState();
    const bool state_aligned =
      (view.session_state == SessionState::kRunning &&
      buffer_state == SessionState::kRunning) ||
      (view.session_state == SessionState::kStopping &&
      buffer_state == SessionState::kStopping) ||
      (view.session_state == SessionState::kHolding &&
      buffer_state == SessionState::kHolding) ||
      (view.session_state == SessionState::kTerminated &&
      buffer_state == SessionState::kTerminated) ||
      (view.session_state == SessionState::kPriming &&
      buffer_state == SessionState::kPriming) ||
      (view.session_state == SessionState::kNone &&
      buffer_state == SessionState::kNone);
    if (!state_aligned) {
      continue;
    }
    if (
      view.session_state == SessionState::kRunning &&
      buffer_.hasPending() &&
      buffer_.pendingImage().generation <= view.active_generation)
    {
      continue;
    }
    coherent = true;
    break;
  }
  if (!coherent) {
    return false;
  }

  const std::uint64_t now_ns = steadyNowNs();
  const SessionState buffer_state = buffer_.sessionState();
  const bool has_session =
    buffer_state != SessionState::kNone && buffer_state != SessionState::kTerminated;
  const bool pending_valid = buffer_.hasPending();
  const TrajectoryImage & validation_head =
    pending_valid ? buffer_.pendingImage() : buffer_.image();

  state = StateMessage{};
  state.stamp = get_node()->now();
  state.protocol_major = kProtocolMajor;
  state.protocol_minor = kProtocolMinor;
  state.controller_boot_id.uuid = controller_boot_id_;
  if (has_session) {
    state.session_id.uuid = buffer_.identity().session_id;
    state.client_instance_id.uuid = buffer_.identity().client_instance_id;
  }
  state.control_mode = active_.load(std::memory_order_acquire) ?
    StateMessage::MODE_ROLLING_READY : StateMessage::MODE_DISABLED;
  state.session_state = static_cast<std::uint8_t>(view.session_state);
  state.has_session = has_session;
  state.has_accepted_update = view.has_accepted_update;
  state.pending_generation_valid = pending_valid;
  state.test_only_limits = envelope_.source == LimitsSource::kTestOnly;
  state.axis_set_hash = kAxisSetHash;
  state.limits_version = envelope_.limits_version;
  state.active_generation = view.active_generation;
  state.validation_base_generation = buffer_.validationBaseGeneration();
  state.pending_generation = pending_valid ? validation_head.generation : 0U;
  state.last_seen_sequence = buffer_.lastSeenSequence();
  state.last_accepted_sequence = buffer_.lastAcceptedSequence();
  state.last_rejected_sequence = last_rejected_sequence_;
  state.execution_time_ns = view.execution_time_ns;
  state.replaceable_from_ns = view.replaceable_from_ns;
  state.buffered_until_ns = view.buffered_until_ns;
  state.available_horizon_ns = view.buffered_until_ns >= view.execution_time_ns ?
    view.buffered_until_ns - view.execution_time_ns : 0U;
  state.prime_age_ns =
    view.session_state == SessionState::kPriming &&
    now_ns >= view.prime_start_time_ns ?
    now_ns - view.prime_start_time_ns : 0U;
  state.accepted_update_age_ns =
    view.has_accepted_update && now_ns >= view.accepted_arrival_time_ns ?
    now_ns - view.accepted_arrival_time_ns : 0U;
  state.buffer_point_count = static_cast<std::uint32_t>(validation_head.point_count);
  state.buffer_capacity = static_cast<std::uint32_t>(buffer_.capacity());
  state.desired_positions = view.desired.positions;
  state.desired_velocities = view.desired.velocities;
  state.last_reject_code = static_cast<std::uint8_t>(last_reject_code_);
  state.stop_reason = static_cast<std::uint8_t>(view.stop_reason);
  state.last_mode_request_id.uuid.fill(0U);
  state.last_service_error = last_service_error_;
  state.source_controller_deactivated =
    view.stop_reason == StopReason::kControllerDeactivated;
  state.target_controller_activated = false;
  state.restart_required = false;
  state.accepted_count = accepted_count_;
  state.rejected_count = rejected_count_;
  state.superseded_pending_count = superseded_pending_count_;
  state.timeout_count = view.timeout_count;
  state.invariant_failure_count = view.invariant_failure_count;
  return true;
}

bool RollingTrajectoryController::publishPublicState()
{
  std::lock_guard<std::mutex> publish_lock(state_publish_mutex_);
  if (!state_publisher_ || !state_publisher_->is_activated()) {
    return false;
  }
  StateMessage state;
  if (!buildPublicState(state)) {
    return false;
  }
  state_publisher_->publish(state);
  return true;
}

bool RollingTrajectoryController::resetRtEpoch(std::uint64_t epoch) noexcept
{
  const std::uint64_t publication_floor =
    session_publication_floor_.load(std::memory_order_acquire);
  if (publication_floor == 0U) {
    return false;
  }

  JointPoint hold;
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    hold.positions[axis] = decodeDouble(
      desired_position_bits_[axis].load(std::memory_order_acquire));
    hold.velocities[axis] = 0.0;
  }
  if (!desiredIsValid(hold)) {
    return false;
  }

  StopTrajectory stop_trajectory;
  if (!stop_trajectory.configure(envelope_, true)) {
    return false;
  }

  rt_active_trajectory_ = TrajectoryImage{};
  rt_identity_ = SessionIdentity{};
  rt_desired_ = hold;
  rt_stop_trajectory_ = stop_trajectory;
  rt_publication_floor_ = publication_floor;
  rt_consumed_publication_sequence_ = publication_floor - 1U;
  rt_last_accepted_arrival_ns_ = 0U;
  rt_accepted_arrival_time_ns_.store(0U, std::memory_order_release);
  rt_stop_elapsed_ns_ = 0U;
  rt_invariant_stage_.store(
    static_cast<std::uint8_t>(RtInvariantStage::kNone), std::memory_order_release);
  execution_time_ns_.store(0U, std::memory_order_release);
  active_generation_.store(0U, std::memory_order_release);
  buffered_until_ns_.store(0U, std::memory_order_release);
  replaceable_from_ns_.store(0U, std::memory_order_release);
  has_accepted_update_.store(false, std::memory_order_release);
  rt_observed_session_epoch_ = epoch;
  return true;
}

RollingTrajectoryController::SnapshotConsumeResult
RollingTrajectoryController::consumeLatestRtTrajectory(bool priming) noexcept
{
  SnapshotLease lease;
  if (!snapshot_exchange_.acquire(lease)) {
    return SnapshotConsumeResult::kNoChange;
  }
  const RollingSnapshot & snapshot = *lease;
  if (
    snapshot.publication_sequence < rt_publication_floor_ ||
    snapshot.publication_sequence <= rt_consumed_publication_sequence_)
  {
    return SnapshotConsumeResult::kNoChange;
  }

  const TrajectoryImage & trajectory = snapshot.trajectory;
  const std::uint64_t active_generation =
    active_generation_.load(std::memory_order_relaxed);
  if (
    trajectory.point_count == 0U || trajectory.point_count > kTransportMaxPoints ||
    trajectory.generation == 0U || trajectory.generation <= active_generation ||
    trajectory.points[0].time_ns != 0U)
  {
    rt_invariant_stage_.store(
      static_cast<std::uint8_t>(RtInvariantStage::kSnapshotShapeOrGeneration),
      std::memory_order_release);
    return SnapshotConsumeResult::kInvalid;
  }

  const std::uint64_t execution_ns = execution_time_ns_.load(std::memory_order_relaxed);
  if (priming) {
    if (execution_ns != 0U) {
      rt_invariant_stage_.store(
        static_cast<std::uint8_t>(RtInvariantStage::kSnapshotPrimingTime),
        std::memory_order_release);
      return SnapshotConsumeResult::kInvalid;
    }
  } else {
    const std::uint64_t replaceable_from_ns =
      replaceable_from_ns_.load(std::memory_order_acquire);
    const bool safe_zero_time_handoff =
      execution_ns == 0U && trajectory.earliest_changed_ns == 0U;
    if (
      snapshot.identity.controller_boot_id != rt_identity_.controller_boot_id ||
      snapshot.identity.session_id != rt_identity_.session_id ||
      snapshot.identity.client_instance_id != rt_identity_.client_instance_id)
    {
      rt_invariant_stage_.store(
        static_cast<std::uint8_t>(RtInvariantStage::kSnapshotIdentity),
        std::memory_order_release);
      return SnapshotConsumeResult::kInvalid;
    }
    if (trajectory.earliest_changed_ns < replaceable_from_ns && !safe_zero_time_handoff) {
      rt_invariant_stage_.store(
        static_cast<std::uint8_t>(RtInvariantStage::kSnapshotLateBoundary),
        std::memory_order_release);
      return SnapshotConsumeResult::kInvalid;
    }
  }

  JointPoint candidate;
  std::uint64_t replacement_boundary_ns = 0U;
  if (
    !sampleTrajectoryImage(trajectory, execution_ns, candidate) ||
    !desiredIsValid(candidate) ||
    !addTime(execution_ns, kTestOnlyReplaceLeadNs, replacement_boundary_ns))
  {
    rt_invariant_stage_.store(
      static_cast<std::uint8_t>(RtInvariantStage::kSnapshotSample),
      std::memory_order_release);
    return SnapshotConsumeResult::kInvalid;
  }

  rt_active_trajectory_ = trajectory;
  rt_identity_ = snapshot.identity;
  rt_desired_ = candidate;
  rt_consumed_publication_sequence_ = snapshot.publication_sequence;
  rt_last_accepted_arrival_ns_ = snapshot.arrival_time_ns;
  rt_accepted_arrival_time_ns_.store(snapshot.arrival_time_ns, std::memory_order_release);
  active_generation_.store(trajectory.generation, std::memory_order_release);
  buffered_until_ns_.store(
    trajectory.points[trajectory.point_count - 1U].time_ns,
    std::memory_order_release);
  replaceable_from_ns_.store(replacement_boundary_ns, std::memory_order_release);
  has_accepted_update_.store(true, std::memory_order_release);
  if (priming) {
    rt_session_state_.store(
      static_cast<std::uint8_t>(SessionState::kRunning),
      std::memory_order_release);
  }
  return SnapshotConsumeResult::kAccepted;
}

bool RollingTrajectoryController::beginRtStop(StopReason reason) noexcept
{
  const SessionState state = static_cast<SessionState>(
    rt_session_state_.load(std::memory_order_relaxed));
  if (state == SessionState::kStopping || state == SessionState::kHolding) {
    return true;
  }
  if (state != SessionState::kPriming && state != SessionState::kRunning) {
    return false;
  }

  std::uint8_t expected_reason = static_cast<std::uint8_t>(StopReason::kNone);
  const bool reason_latched = stop_reason_.compare_exchange_strong(
    expected_reason, static_cast<std::uint8_t>(reason),
    std::memory_order_acq_rel, std::memory_order_acquire);
  if (reason_latched) {
    if (reason == StopReason::kPrimeTimeout || reason == StopReason::kUpdateTimeout) {
      timeout_count_.fetch_add(1U, std::memory_order_relaxed);
    } else if (reason == StopReason::kInternalInvariant) {
      invariant_failure_count_.fetch_add(1U, std::memory_order_relaxed);
    }
  }
  if (!rt_stop_trajectory_.begin(rt_desired_)) {
    return false;
  }
  rt_stop_elapsed_ns_ = 0U;
  stop_requested_.store(false, std::memory_order_release);

  if (rt_stop_trajectory_.state() == StopTrajectoryState::kHolding) {
    JointPoint terminal;
    if (!rt_stop_trajectory_.sample(0U, terminal) || !desiredIsValid(terminal)) {
      return false;
    }
    rt_desired_ = terminal;
    rt_session_state_.store(
      static_cast<std::uint8_t>(SessionState::kHolding),
      std::memory_order_release);
  } else {
    rt_session_state_.store(
      static_cast<std::uint8_t>(SessionState::kStopping),
      std::memory_order_release);
  }
  return true;
}

bool RollingTrajectoryController::advanceRtStop(std::int64_t period_ns) noexcept
{
  const StopReason reason = static_cast<StopReason>(
    stop_reason_.load(std::memory_order_acquire));
  std::uint64_t increment_ns = kTestOnlyNominalPeriodNs;
  if (reason != StopReason::kClockAnomaly && validPeriod(period_ns)) {
    increment_ns = static_cast<std::uint64_t>(period_ns);
  }

  std::uint64_t next_elapsed_ns = 0U;
  std::uint64_t next_execution_ns = 0U;
  if (
    !addTime(rt_stop_elapsed_ns_, increment_ns, next_elapsed_ns) ||
    !addTime(
      execution_time_ns_.load(std::memory_order_relaxed), increment_ns,
      next_execution_ns))
  {
    return false;
  }

  JointPoint candidate;
  if (
    !rt_stop_trajectory_.sample(next_elapsed_ns, candidate) ||
    !desiredIsValid(candidate))
  {
    return false;
  }
  rt_desired_ = candidate;
  rt_stop_elapsed_ns_ = next_elapsed_ns;
  execution_time_ns_.store(next_execution_ns, std::memory_order_release);
  if (rt_stop_trajectory_.state() == StopTrajectoryState::kHolding) {
    rt_session_state_.store(
      static_cast<std::uint8_t>(SessionState::kHolding),
      std::memory_order_release);
  }
  return true;
}

bool RollingTrajectoryController::updateRtReplaceableBoundary() noexcept
{
  std::uint64_t boundary_ns = 0U;
  if (!addTime(
      execution_time_ns_.load(std::memory_order_relaxed),
      kTestOnlyReplaceLeadNs, boundary_ns))
  {
    return false;
  }
  replaceable_from_ns_.store(boundary_ns, std::memory_order_release);
  return true;
}

bool RollingTrajectoryController::writeRtDesired() noexcept
{
  if (!desiredIsValid(rt_desired_)) {
    return false;
  }
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    command_interfaces_[axis].set_value(rt_desired_.positions[axis]);
    desired_position_bits_[axis].store(
      encodeDouble(rt_desired_.positions[axis]), std::memory_order_release);
    desired_velocity_bits_[axis].store(
      encodeDouble(rt_desired_.velocities[axis]), std::memory_order_release);
  }
  rt_state_session_epoch_.store(rt_observed_session_epoch_, std::memory_order_release);
  rt_accepted_arrival_time_ns_.store(
    rt_last_accepted_arrival_ns_, std::memory_order_release);
  return true;
}

bool RollingTrajectoryController::desiredIsValid(const JointPoint & desired) const noexcept
{
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    const AxisEnvelope & limits = envelope_.axes[axis];
    const double position = desired.positions[axis];
    const double velocity = desired.velocities[axis];
    const double safe_lower = limits.position_lower + limits.position_margin_lower;
    const double safe_upper = limits.position_upper - limits.position_margin_upper;
    if (
      !std::isfinite(position) || !std::isfinite(velocity) ||
      position < safe_lower || position > safe_upper ||
      velocity > limits.velocity_positive || velocity < -limits.velocity_negative)
    {
      return false;
    }
  }
  return true;
}

bool RollingTrajectoryController::calculateStopDurationNs(
  const JointPoint & desired, std::uint64_t & duration_ns) const noexcept
{
  if (!desiredIsValid(desired)) {
    return false;
  }
  long double duration_seconds = 0.0L;
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    const double velocity = desired.velocities[axis];
    const double limit = velocity >= 0.0 ?
      envelope_.axes[axis].stop_acceleration_positive :
      envelope_.axes[axis].stop_acceleration_negative;
    const long double axis_duration =
      std::abs(static_cast<long double>(velocity)) /
      static_cast<long double>(limit);
    duration_seconds = std::max(duration_seconds, axis_duration);
  }
  const long double nanoseconds = std::ceil(duration_seconds * 1.0e9L);
  if (
    !std::isfinite(nanoseconds) || nanoseconds < 0.0L ||
    nanoseconds > static_cast<long double>(std::numeric_limits<std::uint64_t>::max()))
  {
    return false;
  }
  duration_ns = static_cast<std::uint64_t>(nanoseconds);
  return true;
}

void RollingTrajectoryController::handleOpen(
  const std::shared_ptr<OpenService::Request> request,
  std::shared_ptr<OpenService::Response> response)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  response->request_id = request->request_id;
  response->protocol_major = kProtocolMajor;
  response->protocol_minor = kProtocolMinor;
  response->controller_boot_id.uuid = controller_boot_id_;

  if (request->protocol_major != kProtocolMajor || request->protocol_minor != kProtocolMinor) {
    setOpenError(*response, OpenService::Response::ERROR_WRONG_PROTOCOL);
    return;
  }
  if (isZeroIdentifier(request->request_id.uuid)) {
    setOpenError(*response, OpenService::Response::ERROR_WRONG_REQUEST);
    return;
  }
  if (request->expected_controller_boot_id.uuid != controller_boot_id_) {
    setOpenError(*response, OpenService::Response::ERROR_WRONG_BOOT);
    return;
  }
  if (isZeroIdentifier(request->client_instance_id.uuid)) {
    setOpenError(*response, OpenService::Response::ERROR_WRONG_CLIENT);
    return;
  }
  if (request->axis_set_hash != kAxisSetHash) {
    setOpenError(*response, OpenService::Response::ERROR_AXIS_SET_MISMATCH);
    return;
  }
  if (
    open_cache_.valid &&
    open_cache_.request.request_id.uuid == request->request_id.uuid &&
    open_cache_.request.client_instance_id.uuid == request->client_instance_id.uuid)
  {
    if (!sameOpenRequest(open_cache_.request, *request)) {
      setOpenError(*response, OpenService::Response::ERROR_WRONG_REQUEST);
      return;
    }
    *response = open_cache_.response;
    last_service_error_ = response->error_code;
    return;
  }
  if (!active_.load(std::memory_order_acquire)) {
    setOpenError(*response, OpenService::Response::ERROR_NOT_READY);
    return;
  }
  if (!synchronizeBufferStateFromRt()) {
    setOpenError(*response, OpenService::Response::ERROR_NOT_READY);
    return;
  }
  if (buffer_.sessionState() != SessionState::kNone) {
    setOpenError(*response, OpenService::Response::ERROR_SESSION_BUSY);
    return;
  }

  const double feedback_age_ms =
    decodeDouble(feedback_age_bits_.load(std::memory_order_acquire));
  if (
    !std::isfinite(feedback_age_ms) || feedback_age_ms < 0.0 ||
    feedback_age_ms > kOpenFeedbackAgeLimitMs)
  {
    setOpenError(*response, OpenService::Response::ERROR_FEEDBACK_STALE);
    return;
  }

  std::array<double, kAxisCount> hold_positions{};
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    const double hold =
      decodeDouble(desired_position_bits_[axis].load(std::memory_order_acquire));
    const double actual =
      decodeDouble(observed_position_bits_[axis].load(std::memory_order_acquire));
    const AxisEnvelope & limits = envelope_.axes[axis];
    const double safe_lower = limits.position_lower + limits.position_margin_lower;
    const double safe_upper = limits.position_upper - limits.position_margin_upper;
    if (!std::isfinite(hold) || hold < safe_lower || hold > safe_upper) {
      setOpenError(*response, OpenService::Response::ERROR_UNSAFE_HOLD);
      return;
    }
    if (
      !std::isfinite(actual) ||
      std::abs(hold - actual) > takeover_tolerances_[axis])
    {
      setOpenError(*response, OpenService::Response::ERROR_TAKEOVER_MISMATCH);
      return;
    }
    hold_positions[axis] = hold;
  }

  SessionIdentity identity;
  identity.controller_boot_id = controller_boot_id_;
  identity.session_id = generateIdentifier();
  identity.client_instance_id = request->client_instance_id.uuid;
  const std::uint64_t current_epoch = session_epoch_.load(std::memory_order_acquire);
  const std::uint64_t latest_publication =
    snapshot_exchange_.latestPublicationSequence();
  if (
    current_epoch == std::numeric_limits<std::uint64_t>::max() ||
    latest_publication == std::numeric_limits<std::uint64_t>::max())
  {
    setOpenError(*response, OpenService::Response::ERROR_RESTART_REQUIRED);
    return;
  }
  if (!buffer_.beginSession(identity)) {
    setOpenError(*response, OpenService::Response::ERROR_LIMITS_UNAVAILABLE);
    return;
  }

  response->success = true;
  response->error_code = OpenService::Response::ERROR_NONE;
  response->client_instance_id = request->client_instance_id;
  response->session_id.uuid = identity.session_id;
  response->session_state = OpenService::Response::SESSION_PRIMING;
  response->axis_set_hash = kAxisSetHash;
  response->limits_version = envelope_.limits_version;
  response->capability_bits = kCapabilityBits;
  response->buffer_capacity = static_cast<std::uint32_t>(buffer_.capacity());
  response->max_horizon_ns = kTestOnlyMaxHorizonNs;
  response->required_initial_horizon_ns = kTestOnlyRequiredInitialHorizonNs;
  response->initial_replaceable_from_ns = 0U;
  response->test_only_limits = true;
  response->hold_positions = hold_positions;
  response->hold_velocities.fill(0.0);
  open_cache_.valid = true;
  open_cache_.request = *request;
  open_cache_.response = *response;
  clearActiveCloseCache();
  execution_time_ns_.store(0U, std::memory_order_release);
  active_generation_.store(0U, std::memory_order_release);
  buffered_until_ns_.store(0U, std::memory_order_release);
  replaceable_from_ns_.store(0U, std::memory_order_release);
  has_accepted_update_.store(false, std::memory_order_release);
  prime_start_time_ns_.store(steadyNowNs(), std::memory_order_release);
  session_publication_floor_.store(latest_publication + 1U, std::memory_order_release);
  stop_reason_.store(
    static_cast<std::uint8_t>(StopReason::kNone), std::memory_order_release);
  stop_requested_.store(false, std::memory_order_release);
  session_epoch_.store(current_epoch + 1U, std::memory_order_release);
  rt_session_state_.store(
    static_cast<std::uint8_t>(SessionState::kPriming),
    std::memory_order_release);
  last_service_error_ = OpenService::Response::ERROR_NONE;
}

void RollingTrajectoryController::handleClose(
  const std::shared_ptr<CloseService::Request> request,
  std::shared_ptr<CloseService::Response> response)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  response->request_id = request->request_id;
  if (request->protocol_major != kProtocolMajor || request->protocol_minor != kProtocolMinor) {
    setCloseError(*response, CloseService::Response::ERROR_WRONG_PROTOCOL);
    return;
  }
  if (isZeroIdentifier(request->request_id.uuid)) {
    setCloseError(*response, CloseService::Response::ERROR_WRONG_REQUEST);
    return;
  }
  if (request->controller_boot_id.uuid != controller_boot_id_) {
    setCloseError(*response, CloseService::Response::ERROR_WRONG_BOOT);
    return;
  }
  if (
    finalize_cache_.valid &&
    finalize_cache_.request.request_id.uuid == request->request_id.uuid)
  {
    if (!sameCloseRequest(finalize_cache_.request, *request)) {
      setCloseError(*response, CloseService::Response::ERROR_WRONG_REQUEST);
      return;
    }
    *response = finalize_cache_.response;
    last_service_error_ = response->error_code;
    return;
  }
  if (!active_.load(std::memory_order_acquire)) {
    setCloseError(*response, CloseService::Response::ERROR_NOT_READY);
    return;
  }
  if (!synchronizeBufferStateFromRt()) {
    setCloseError(*response, CloseService::Response::ERROR_NOT_READY);
    return;
  }

  const SessionState state = buffer_.sessionState();
  if (state == SessionState::kNone || state == SessionState::kTerminated) {
    setCloseError(*response, CloseService::Response::ERROR_WRONG_SESSION);
    return;
  }
  const SessionIdentity & identity = buffer_.identity();
  if (request->session_id.uuid != identity.session_id) {
    setCloseError(*response, CloseService::Response::ERROR_WRONG_SESSION);
    return;
  }
  if (request->client_instance_id.uuid != identity.client_instance_id) {
    setCloseError(*response, CloseService::Response::ERROR_WRONG_CLIENT);
    return;
  }

  if (CloseCacheEntry * cached = findActiveCloseCache(request->request_id.uuid)) {
    if (!sameCloseRequest(cached->request, *request)) {
      setCloseError(*response, CloseService::Response::ERROR_WRONG_REQUEST);
      return;
    }
    *response = cached->response;
    last_service_error_ = response->error_code;
    return;
  }

  if (state == SessionState::kHolding) {
    response->success = true;
    response->error_code = CloseService::Response::ERROR_NONE;
    response->session_state = CloseService::Response::SESSION_NONE;
    response->stop_reason = CloseService::Response::STOP_GRACEFUL_CLOSE;
    response->stop_accepted = false;
    response->session_destroyed = true;
    response->restart_required = false;
    if (!buffer_.finishSession()) {
      setCloseError(*response, CloseService::Response::ERROR_NOT_READY);
      return;
    }
    finalize_cache_.valid = true;
    finalize_cache_.request = *request;
    finalize_cache_.response = *response;
    clearActiveCloseCache();
    stop_requested_.store(false, std::memory_order_release);
    rt_session_state_.store(
      static_cast<std::uint8_t>(SessionState::kNone),
      std::memory_order_release);
    stop_reason_.store(
      static_cast<std::uint8_t>(StopReason::kNone),
      std::memory_order_release);
    has_accepted_update_.store(false, std::memory_order_release);
    session_epoch_.fetch_add(1U, std::memory_order_acq_rel);
    last_service_error_ = CloseService::Response::ERROR_NONE;
    return;
  }

  CloseCacheEntry * cache = allocateActiveCloseCache();
  if (cache == nullptr) {
    setCloseError(*response, CloseService::Response::ERROR_WRONG_REQUEST);
    return;
  }
  if (state == SessionState::kPriming || state == SessionState::kRunning) {
    if (!buffer_.requestStop()) {
      setCloseError(*response, CloseService::Response::ERROR_NOT_READY);
      return;
    }
    std::uint8_t expected_reason = static_cast<std::uint8_t>(StopReason::kNone);
    (void)stop_reason_.compare_exchange_strong(
      expected_reason, static_cast<std::uint8_t>(StopReason::kGracefulClose),
      std::memory_order_acq_rel, std::memory_order_acquire);
    stop_requested_.store(true, std::memory_order_release);
  } else if (state != SessionState::kStopping) {
    setCloseError(*response, CloseService::Response::ERROR_NOT_READY);
    return;
  }

  response->success = true;
  response->error_code = CloseService::Response::ERROR_NONE;
  response->session_state = CloseService::Response::SESSION_STOPPING;
  response->stop_reason = CloseService::Response::STOP_GRACEFUL_CLOSE;
  response->stop_accepted = true;
  response->session_destroyed = false;
  response->restart_required = false;
  cache->valid = true;
  cache->request = *request;
  cache->response = *response;
  last_service_error_ = CloseService::Response::ERROR_NONE;
}

void RollingTrajectoryController::handleUpdate(const BatchMessage::SharedPtr message)
{
  if (!message) {
    return;
  }
  BlockingFlagGuard initial_handoff(initial_handoff_gate_);
  std::array<PointView, kTransportMaxPoints> point_views{};
  const std::size_t point_count = message->points.size();
  if (point_count <= point_views.size()) {
    for (std::size_t index = 0U; index < point_count; ++index) {
      const auto & point = message->points[index];
      point_views[index] = PointView{
        point.time_from_session_start_ns,
        point.positions.data(), point.positions.size(),
        point.velocities.data(), point.velocities.size()};
    }
  }

  std::lock_guard<std::mutex> lock(callback_mutex_);
  RejectCode result = RejectCode::kNone;
  PreparedSubmission submission;
  bool supersedes_pending = false;
  if (!synchronizeBufferStateFromRt()) {
    result = RejectCode::kSessionNotAccepting;
  } else if (
    message->protocol_major != kProtocolMajor || message->protocol_minor != kProtocolMinor)
  {
    result = RejectCode::kWrongProtocol;
  } else if (message->controller_boot_id.uuid != controller_boot_id_) {
    result = RejectCode::kWrongBoot;
  } else {
    BatchView batch;
    batch.protocol_major = message->protocol_major;
    batch.protocol_minor = message->protocol_minor;
    batch.controller_boot_id = message->controller_boot_id.uuid;
    batch.session_id = message->session_id.uuid;
    batch.client_instance_id = message->client_instance_id.uuid;
    batch.sequence = message->sequence;
    batch.replace_from_ns = message->replace_from_ns;
    batch.points = point_views.data();
    batch.point_count = point_count;
    buffer_.setReplaceableFromNs(
      replaceable_from_ns_.load(std::memory_order_acquire));
    supersedes_pending = buffer_.hasPending();
    result = buffer_.prepare(batch, submission);
    if (result == RejectCode::kNone) {
      if (!synchronizeBufferStateFromRt()) {
        result = RejectCode::kSessionNotAccepting;
      } else {
        result = buffer_.commit(
          submission, replaceable_from_ns_.load(std::memory_order_acquire));
      }
    }
  }

  if (result == RejectCode::kNone) {
    if (!snapshot_exchange_.publish(
        buffer_.identity(), buffer_.pendingImage(), steadyNowNs()))
    {
      buffer_.terminateSession();
      std::uint8_t expected_reason = static_cast<std::uint8_t>(StopReason::kNone);
      (void)stop_reason_.compare_exchange_strong(
        expected_reason, static_cast<std::uint8_t>(StopReason::kInternalInvariant),
        std::memory_order_acq_rel, std::memory_order_acquire);
      rt_invariant_stage_.store(
        static_cast<std::uint8_t>(RtInvariantStage::kSnapshotPublication),
        std::memory_order_release);
      stop_requested_.store(true, std::memory_order_release);
      invariant_failure_count_.fetch_add(1U, std::memory_order_relaxed);
      result = RejectCode::kSessionNotAccepting;
    } else {
      ++accepted_count_;
      if (supersedes_pending) {
        ++superseded_pending_count_;
      }
    }
  }
  last_reject_code_ = result;
  if (result != RejectCode::kNone) {
    last_rejected_sequence_ = message->sequence;
    ++rejected_count_;
  }
}

void RollingTrajectoryController::clearActiveCloseCache() noexcept
{
  for (CloseCacheEntry & entry : close_cache_) {
    entry = CloseCacheEntry{};
  }
}

RollingTrajectoryController::CloseCacheEntry *
RollingTrajectoryController::findActiveCloseCache(const Identifier & request_id) noexcept
{
  for (CloseCacheEntry & entry : close_cache_) {
    if (entry.valid && entry.request.request_id.uuid == request_id) {
      return &entry;
    }
  }
  return nullptr;
}

RollingTrajectoryController::CloseCacheEntry *
RollingTrajectoryController::allocateActiveCloseCache() noexcept
{
  for (CloseCacheEntry & entry : close_cache_) {
    if (!entry.valid) {
      return &entry;
    }
  }
  return nullptr;
}

void RollingTrajectoryController::setOpenError(
  OpenService::Response & response, std::uint8_t error_code) noexcept
{
  response.success = false;
  response.error_code = error_code;
  last_service_error_ = error_code;
}

void RollingTrajectoryController::setCloseError(
  CloseService::Response & response, std::uint8_t error_code) noexcept
{
  response.success = false;
  response.error_code = error_code;
  response.session_state = static_cast<std::uint8_t>(buffer_.sessionState());
  response.stop_reason = buffer_.sessionState() == SessionState::kStopping ||
    buffer_.sessionState() == SessionState::kHolding ?
    CloseService::Response::STOP_GRACEFUL_CLOSE : CloseService::Response::STOP_NONE;
  response.stop_accepted = false;
  response.session_destroyed = false;
  response.restart_required = false;
  last_service_error_ = error_code;
}

}  // namespace rolling_trajectory_controller

PLUGINLIB_EXPORT_CLASS(
  rolling_trajectory_controller::RollingTrajectoryController,
  controller_interface::ControllerInterface)
