#include "enable_manager/enable_manager_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <thread>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/qos.hpp"

namespace enable_manager
{

const std::array<const char *, EnableManagerController::kAxisCount>
EnableManagerController::kJointNames = {
  "right_joint1", "right_joint2", "right_joint3", "right_joint4", "right_joint5",
  "right_joint6", "left_joint1", "left_joint2", "left_joint3", "left_joint4",
  "left_joint5", "left_joint6", "turn", "updown"};

const std::array<std::array<std::int8_t, 3>, EnableManagerController::kBatchCount>
EnableManagerController::kEnableBatches = {{{0, 1, 2}, {6, 7, 8}, {3, 4, 5}, {9, 10, 11},
  {12, 13, -1}}};

const std::array<std::uint8_t, EnableManagerController::kBatchCount>
EnableManagerController::kBatchSizes = {3U, 3U, 3U, 3U, 2U};

controller_interface::CallbackReturn EnableManagerController::on_init()
{
  auto_declare<double>("batch_timeout", 4.0);
  auto_declare<double>("disable_stage_timeout", 4.0);
  auto_declare<double>("inter_batch_delay", 0.2);
  auto_declare<double>("fault_reset_timeout", 4.0);
  auto_declare<double>("controller_switch_timeout", 4.0);
  auto_declare<int>("service_result_timeout_ms", 30000);
  auto_declare<std::string>("jtc_name", "dual_arm_jtc");
  auto_declare<std::vector<std::string>>(
    "motion_controller_names", std::vector<std::string>{});
  auto_declare<std::string>("default_motion_controller_name", "");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
EnableManagerController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  configuration.names.reserve(kAxisCount);
  for (const char * joint : kJointNames) {
    configuration.names.emplace_back(std::string(joint) + "/control_word");
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
EnableManagerController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  configuration.names.reserve(kAxisCount);
  for (const char * joint : kJointNames) {
    configuration.names.emplace_back(std::string(joint) + "/status_word");
  }
  return configuration;
}

controller_interface::CallbackReturn EnableManagerController::on_configure(
  const rclcpp_lifecycle::State &)
{
  batch_timeout_seconds_ = get_node()->get_parameter("batch_timeout").as_double();
  disable_stage_timeout_seconds_ =
    get_node()->get_parameter("disable_stage_timeout").as_double();
  inter_batch_delay_seconds_ = get_node()->get_parameter("inter_batch_delay").as_double();
  fault_reset_timeout_seconds_ = get_node()->get_parameter("fault_reset_timeout").as_double();
  controller_switch_timeout_seconds_ =
    get_node()->get_parameter("controller_switch_timeout").as_double();
  const auto service_timeout = get_node()->get_parameter("service_result_timeout_ms").as_int();
  const std::string legacy_jtc_name = get_node()->get_parameter("jtc_name").as_string();
  motion_controller_names_ =
    get_node()->get_parameter("motion_controller_names").as_string_array();
  std::string default_motion_controller_name =
    get_node()->get_parameter("default_motion_controller_name").as_string();

  if (motion_controller_names_.empty() && !legacy_jtc_name.empty()) {
    motion_controller_names_.push_back(legacy_jtc_name);
  }
  if (default_motion_controller_name.empty()) {
    default_motion_controller_name = legacy_jtc_name;
  }
  bool valid_motion_controller_registry = !motion_controller_names_.empty();
  for (auto controller = motion_controller_names_.begin();
    controller != motion_controller_names_.end(); ++controller)
  {
    valid_motion_controller_registry =
      valid_motion_controller_registry && !controller->empty() &&
      std::find(motion_controller_names_.begin(), controller, *controller) == controller;
  }
  const auto default_controller = std::find(
    motion_controller_names_.begin(), motion_controller_names_.end(),
    default_motion_controller_name);
  valid_motion_controller_registry =
    valid_motion_controller_registry && default_controller != motion_controller_names_.end();

  if (
    batch_timeout_seconds_ <= 0.0 || disable_stage_timeout_seconds_ <= 0.0 ||
    inter_batch_delay_seconds_ < 0.0 || fault_reset_timeout_seconds_ <= 0.0 ||
    controller_switch_timeout_seconds_ <= 0.0 || service_timeout <= 0 ||
    !valid_motion_controller_registry)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid enable-manager timing or controller parameter");
    return controller_interface::CallbackReturn::ERROR;
  }
  default_motion_controller_index_ = static_cast<std::size_t>(
    std::distance(motion_controller_names_.begin(), default_controller));
  active_motion_controller_index_.store(kNoMotionController, std::memory_order_release);
  service_result_timeout_ = std::chrono::milliseconds(service_timeout);

  enable_callback_group_ =
    get_node()->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  disable_callback_group_ =
    get_node()->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  reset_callback_group_ =
    get_node()->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  worker_callback_group_ =
    get_node()->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  enable_service_ = get_node()->create_service<robot_interfaces::srv::RtEnable>(
    "/rt/enable", std::bind(
      &EnableManagerController::handleEnable, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, enable_callback_group_);
  disable_service_ = get_node()->create_service<robot_interfaces::srv::RtEnable>(
    "/rt/disable", std::bind(
      &EnableManagerController::handleDisable, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, disable_callback_group_);
  reset_service_ = get_node()->create_service<robot_interfaces::srv::RtEnable>(
    "/rt/reset_fault", std::bind(
      &EnableManagerController::handleResetFault, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, reset_callback_group_);
  switch_client_ = get_node()->create_client<controller_manager_msgs::srv::SwitchController>(
    "/controller_manager/switch_controller", rmw_qos_profile_services_default,
    worker_callback_group_);

  diagnostics_publisher_ = get_node()->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::QoS(10));
  worker_timer_ = get_node()->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&EnableManagerController::handleNonRtFaultStop, this), worker_callback_group_);
  diagnostics_timer_ = get_node()->create_wall_timer(
    std::chrono::seconds(1), std::bind(&EnableManagerController::publishDiagnostics, this),
    worker_callback_group_);

  active_.store(false, std::memory_order_release);
  phase_.store(Phase::kInactive, std::memory_order_release);
  owner_.store(Owner::kInternal, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EnableManagerController::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.size() != kAxisCount || state_interfaces_.size() != kAxisCount) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected 14 control_word and 14 status_word interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }

  setAllControlWords(0x0000U);
  enable_request_.store(false, std::memory_order_release);
  disable_request_.store(false, std::memory_order_release);
  reset_request_.store(false, std::memory_order_release);
  enable_hardware_ready_.store(false, std::memory_order_release);
  jtc_activate_failed_request_.store(false, std::memory_order_release);
  emergency_jtc_deactivate_request_.store(false, std::memory_order_release);
  restart_required_.store(false, std::memory_order_release);
  clearFailure();
  current_batch_ = 0U;
  downward_stage_ = 0U;
  stage_deadline_ns_ = 0;
  enable_preempt_requested_ = false;
  jtc_deactivate_failed_.store(false, std::memory_order_release);
  primary_failure_stage_ = Stage::kSuccess;
  primary_failed_batch_ = -1;
  primary_failed_joint_ = -1;
  primary_failed_status_word_ = 0U;
  owner_.store(Owner::kInternal, std::memory_order_release);
  phase_.store(Phase::kStartupSanitizing, std::memory_order_release);
  active_.store(true, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EnableManagerController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false, std::memory_order_release);
  setAllControlWords(0x0000U);
  phase_.store(Phase::kInactive, std::memory_order_release);
  owner_.store(Owner::kInternal, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type EnableManagerController::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (!active_.load(std::memory_order_acquire)) {
    return controller_interface::return_type::OK;
  }

  refreshStatusWords();
  const std::int64_t now_ns = time.nanoseconds();
  Phase phase = phase_.load(std::memory_order_acquire);

  if (phase == Phase::kEnabled || phase == Phase::kJtcActivating ||
    phase == Phase::kJtcDeactivating)
  {
    for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
      const DriveState state = decodeState(status_words_[axis]);
      if (isFaultState(state)) {
        startEmergency(
          Stage::kFaultDetected, static_cast<std::int8_t>(axis), status_words_[axis], now_ns);
        phase = Phase::kEmergencyQuickStop;
        break;
      }
      if (phase == Phase::kEnabled && state != DriveState::kOperationEnabled) {
        startEmergency(
          Stage::kUnexpectedDriveState, static_cast<std::int8_t>(axis), status_words_[axis],
          now_ns);
        phase = Phase::kEmergencyQuickStop;
        break;
      }
    }
  }

  switch (phase) {
    case Phase::kStartupSanitizing:
      if (stage_deadline_ns_ == 0) {
        startDownward(Phase::kStartupSanitizing, now_ns);
      }
      updateDownward(now_ns);
      break;
    case Phase::kIdle:
      setAllControlWords(0x0000U);
      if (enable_request_.exchange(false, std::memory_order_acq_rel)) {
        current_batch_ = 0U;
        enable_preempt_requested_ = false;
        primary_failure_stage_ = Stage::kSuccess;
        primary_failed_batch_ = -1;
        primary_failed_joint_ = -1;
        primary_failed_status_word_ = 0U;
        enable_hardware_ready_.store(false, std::memory_order_release);
        stage_deadline_ns_ = now_ns + static_cast<std::int64_t>(batch_timeout_seconds_ * 1e9);
        phase_.store(Phase::kEnabling, std::memory_order_release);
      } else if (reset_request_.exchange(false, std::memory_order_acq_rel)) {
        reset_targets_.fill(false);
        bool has_fault = false;
        for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
          reset_targets_[axis] = isFaultState(decodeState(status_words_[axis]));
          has_fault = has_fault || reset_targets_[axis];
        }
        if (!has_fault) {
          clearFailure();
          publishResult(reset_result_, true, Stage::kAlreadyClear);
          owner_.store(Owner::kNone, std::memory_order_release);
        } else {
          setAllControlWords(0x0000U);
          phase_.store(Phase::kResetLow, std::memory_order_release);
        }
      } else if (disable_request_.exchange(false, std::memory_order_acq_rel)) {
        clearFailure();
        publishResult(disable_result_, true, Stage::kAlreadyDisabled);
        owner_.store(Owner::kNone, std::memory_order_release);
      }
      break;
    case Phase::kEnabling:
    case Phase::kInterBatchDelay:
      updateEnable(now_ns);
      break;
    case Phase::kJtcActivating:
      if (jtc_activate_failed_request_.exchange(false, std::memory_order_acq_rel)) {
        primary_failure_stage_ = Stage::kJtcActivateFailed;
        primary_failed_batch_ = -1;
        primary_failed_joint_ = -1;
        primary_failed_status_word_ = 0U;
        recordFailure(Stage::kJtcActivateFailed, -1, -1, 0U);
        startDownward(Phase::kRollback, now_ns);
        break;
      }
      setAllControlWords(0x000FU);
      break;
    case Phase::kJtcDeactivating:
    case Phase::kEnabled:
      setAllControlWords(0x000FU);
      if (disable_request_.exchange(false, std::memory_order_acq_rel)) {
        startDownward(Phase::kDisabling, now_ns);
      }
      break;
    case Phase::kDisabling:
    case Phase::kRollback:
    case Phase::kEmergencyDisable:
      if (stage_deadline_ns_ == 0) {
        startDownward(phase, now_ns);
      }
      updateDownward(now_ns);
      break;
    case Phase::kResetLow:
    case Phase::kResetHigh:
      updateReset(now_ns);
      break;
    case Phase::kEmergencyQuickStop:
      updateEmergencyQuickStop(now_ns);
      break;
    case Phase::kFailed:
      setAllControlWords(0x0000U);
      if (disable_request_.exchange(false, std::memory_order_acq_rel)) {
        startDownward(Phase::kDisabling, now_ns);
      } else if (reset_request_.exchange(false, std::memory_order_acq_rel)) {
        reset_targets_.fill(false);
        bool has_fault = false;
        for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
          reset_targets_[axis] = isFaultState(decodeState(status_words_[axis]));
          has_fault = has_fault || reset_targets_[axis];
        }
        if (!has_fault) {
          clearFailure();
          publishResult(reset_result_, true, Stage::kAlreadyClear);
          phase_.store(Phase::kIdle, std::memory_order_release);
          owner_.store(Owner::kNone, std::memory_order_release);
        } else {
          phase_.store(Phase::kResetLow, std::memory_order_release);
        }
      }
      break;
    case Phase::kInactive:
      break;
  }

  return controller_interface::return_type::OK;
}

void EnableManagerController::handleEnable(
  const std::shared_ptr<robot_interfaces::srv::RtEnable::Request>,
  std::shared_ptr<robot_interfaces::srv::RtEnable::Response> response)
{
  bool expected_callback = false;
  if (!enable_callback_active_.compare_exchange_strong(expected_callback, true)) {
    fillImmediateResponse(*response, false, Stage::kOperationInProgress);
    return;
  }
  const auto release_callback = [this]() {
      enable_callback_active_.store(false, std::memory_order_release);
    };

  if (!active_.load(std::memory_order_acquire)) {
    fillImmediateResponse(*response, false, Stage::kControllerInactive);
    release_callback();
    return;
  }
  if (restart_required_.load(std::memory_order_acquire)) {
    fillImmediateResponse(*response, false, Stage::kRestartRequired);
    release_callback();
    return;
  }
  if (phase_.load(std::memory_order_acquire) == Phase::kEnabled) {
    fillImmediateResponse(*response, true, Stage::kAlreadyEnabled);
    release_callback();
    return;
  }

  Owner expected_owner = Owner::kNone;
  if (
    phase_.load(std::memory_order_acquire) != Phase::kIdle ||
    !owner_.compare_exchange_strong(expected_owner, Owner::kEnable))
  {
    fillImmediateResponse(*response, false, Stage::kOperationInProgress);
    release_callback();
    return;
  }

  enable_result_.ready.store(false, std::memory_order_release);
  enable_hardware_ready_.store(false, std::memory_order_release);
  enable_request_.store(true, std::memory_order_release);
  const auto deadline = std::chrono::steady_clock::now() + service_result_timeout_;
  while (
    active_.load(std::memory_order_acquire) &&
    !enable_hardware_ready_.load(std::memory_order_acquire) &&
    !enable_result_.ready.load(std::memory_order_acquire) &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  if (enable_result_.ready.load(std::memory_order_acquire)) {
    fillResponse(enable_result_, *response);
    release_callback();
    return;
  }
  if (!enable_hardware_ready_.load(std::memory_order_acquire)) {
    fillImmediateResponse(*response, false, Stage::kControllerUpdateTimeout);
    release_callback();
    return;
  }

  const SwitchResult activation_result =
    switchMotionController(default_motion_controller_index_, true);
  if (activation_result != SwitchResult::kSuccess) {
    if (activation_result == SwitchResult::kAmbiguous) {
      restart_required_.store(true, std::memory_order_release);
      emergency_jtc_deactivate_request_.store(true, std::memory_order_release);
    }
    if (phase_.load(std::memory_order_acquire) == Phase::kJtcActivating) {
      jtc_activate_failed_request_.store(true, std::memory_order_release);
    }
    if (waitForResult(enable_result_)) {
      fillResponse(enable_result_, *response);
    } else {
      fillImmediateResponse(*response, false, Stage::kControllerUpdateTimeout);
    }
    release_callback();
    return;
  }

  if (phase_.load(std::memory_order_acquire) == Phase::kJtcActivating) {
    clearFailure();
    phase_.store(Phase::kEnabled, std::memory_order_release);
    publishResult(enable_result_, true, Stage::kSuccess);
    owner_.store(Owner::kNone, std::memory_order_release);
  }
  if (waitForResult(enable_result_)) {
    fillResponse(enable_result_, *response);
  } else {
    fillImmediateResponse(*response, false, Stage::kControllerUpdateTimeout);
  }
  release_callback();
}

void EnableManagerController::handleDisable(
  const std::shared_ptr<robot_interfaces::srv::RtEnable::Request>,
  std::shared_ptr<robot_interfaces::srv::RtEnable::Response> response)
{
  bool expected_callback = false;
  if (!disable_callback_active_.compare_exchange_strong(expected_callback, true)) {
    fillImmediateResponse(*response, false, Stage::kOperationInProgress);
    return;
  }
  const auto release_callback = [this]() {
      disable_callback_active_.store(false, std::memory_order_release);
    };
  if (!active_.load(std::memory_order_acquire)) {
    fillImmediateResponse(*response, false, Stage::kControllerInactive);
    release_callback();
    return;
  }

  Phase phase = phase_.load(std::memory_order_acquire);
  if (phase == Phase::kIdle) {
    fillImmediateResponse(*response, true, Stage::kAlreadyDisabled);
    release_callback();
    return;
  }
  if (
    phase == Phase::kStartupSanitizing || phase == Phase::kEmergencyQuickStop ||
    phase == Phase::kEmergencyDisable || phase == Phase::kJtcActivating ||
    phase == Phase::kDisabling || phase == Phase::kRollback)
  {
    fillImmediateResponse(*response, false, Stage::kOperationInProgress);
    release_callback();
    return;
  }

  const Owner current_owner = owner_.load(std::memory_order_acquire);
  const bool preempt_enable =
    current_owner == Owner::kEnable &&
    (phase == Phase::kEnabling || phase == Phase::kInterBatchDelay);
  const bool preempt_reset =
    current_owner == Owner::kReset &&
    (phase == Phase::kResetLow || phase == Phase::kResetHigh);
  if (!preempt_enable && !preempt_reset) {
    Owner expected_owner = Owner::kNone;
    if (!owner_.compare_exchange_strong(expected_owner, Owner::kDisable)) {
      fillImmediateResponse(*response, false, Stage::kOperationInProgress);
      release_callback();
      return;
    }
  }

  disable_result_.ready.store(false, std::memory_order_release);
  jtc_deactivate_failed_.store(false, std::memory_order_release);
  if (phase == Phase::kEnabled) {
    phase_.store(Phase::kJtcDeactivating, std::memory_order_release);
    if (switchMotionController(controllerIndexForDeactivation(), false) !=
      SwitchResult::kSuccess)
    {
      jtc_deactivate_failed_.store(true, std::memory_order_release);
      restart_required_.store(true, std::memory_order_release);
      recordFailure(Stage::kJtcDeactivateFailed, -1, -1, 0U);
    }
  }
  disable_request_.store(true, std::memory_order_release);
  if (waitForResult(disable_result_)) {
    fillResponse(disable_result_, *response);
  } else {
    fillImmediateResponse(*response, false, Stage::kControllerUpdateTimeout);
  }
  release_callback();
}

void EnableManagerController::handleResetFault(
  const std::shared_ptr<robot_interfaces::srv::RtEnable::Request>,
  std::shared_ptr<robot_interfaces::srv::RtEnable::Response> response)
{
  bool expected_callback = false;
  if (!reset_callback_active_.compare_exchange_strong(expected_callback, true)) {
    fillImmediateResponse(*response, false, Stage::kOperationInProgress);
    return;
  }
  const auto release_callback = [this]() {
      reset_callback_active_.store(false, std::memory_order_release);
    };
  if (!active_.load(std::memory_order_acquire)) {
    fillImmediateResponse(*response, false, Stage::kControllerInactive);
    release_callback();
    return;
  }
  if (restart_required_.load(std::memory_order_acquire)) {
    fillImmediateResponse(*response, false, Stage::kRestartRequired);
    release_callback();
    return;
  }

  const Phase phase = phase_.load(std::memory_order_acquire);
  if (phase != Phase::kIdle && phase != Phase::kFailed) {
    fillImmediateResponse(*response, false, Stage::kOperationInProgress);
    release_callback();
    return;
  }
  Owner expected_owner = Owner::kNone;
  if (!owner_.compare_exchange_strong(expected_owner, Owner::kReset)) {
    fillImmediateResponse(*response, false, Stage::kOperationInProgress);
    release_callback();
    return;
  }

  reset_result_.ready.store(false, std::memory_order_release);
  reset_request_.store(true, std::memory_order_release);
  if (waitForResult(reset_result_)) {
    fillResponse(reset_result_, *response);
  } else {
    fillImmediateResponse(*response, false, Stage::kControllerUpdateTimeout);
  }
  release_callback();
}

bool EnableManagerController::waitForResult(ResultSlot & slot)
{
  const auto deadline = std::chrono::steady_clock::now() + service_result_timeout_;
  while (
    active_.load(std::memory_order_acquire) &&
    !slot.ready.load(std::memory_order_acquire) &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return slot.ready.load(std::memory_order_acquire);
}

void EnableManagerController::fillResponse(
  const ResultSlot & slot, robot_interfaces::srv::RtEnable::Response & response) const
{
  response.ok = slot.ok.load(std::memory_order_acquire);
  response.failed_batch = slot.failed_batch.load(std::memory_order_acquire);
  const std::int8_t failed_joint = slot.failed_joint.load(std::memory_order_acquire);
  response.failed_joint = failed_joint >= 0 ? kJointNames[failed_joint] : "";
  response.status_word = slot.status_word.load(std::memory_order_acquire);
  response.stage = stageName(slot.stage.load(std::memory_order_acquire));
}

void EnableManagerController::fillImmediateResponse(
  robot_interfaces::srv::RtEnable::Response & response, bool ok, Stage stage) const
{
  response.ok = ok;
  response.failed_batch = -1;
  response.failed_joint.clear();
  response.status_word = 0U;
  response.stage = stageName(stage);
}

EnableManagerController::SwitchResult EnableManagerController::switchMotionController(
  std::size_t controller_index, bool activate)
{
  if (controller_index >= motion_controller_names_.size()) {
    return SwitchResult::kFailed;
  }
  bool expected = false;
  if (!switch_in_progress_.compare_exchange_strong(expected, true)) {
    return SwitchResult::kAmbiguous;
  }
  const auto clear_in_progress = [this]() {
      switch_in_progress_.store(false, std::memory_order_release);
    };
  if (!switch_client_->wait_for_service(std::chrono::milliseconds(0))) {
    clear_in_progress();
    return SwitchResult::kAmbiguous;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  if (activate) {
    request->activate_controllers.push_back(motion_controller_names_[controller_index]);
  } else {
    request->deactivate_controllers.push_back(motion_controller_names_[controller_index]);
  }
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  request->activate_asap = true;
  const std::int64_t switch_timeout_ns =
    rclcpp::Duration::from_seconds(controller_switch_timeout_seconds_).nanoseconds();
  request->timeout.sec = static_cast<std::int32_t>(switch_timeout_ns / 1000000000LL);
  request->timeout.nanosec =
    static_cast<std::uint32_t>(switch_timeout_ns % 1000000000LL);
  auto future = switch_client_->async_send_request(request);
  const auto wait_status = future.wait_for(
    std::chrono::duration<double>(controller_switch_timeout_seconds_));
  SwitchResult result = SwitchResult::kAmbiguous;
  if (wait_status == std::future_status::ready) {
    const auto switch_response = future.get();
    result = switch_response != nullptr && switch_response->ok ?
      SwitchResult::kSuccess : SwitchResult::kFailed;
  }
  if (result == SwitchResult::kSuccess) {
    if (activate) {
      active_motion_controller_index_.store(controller_index, std::memory_order_release);
    } else {
      std::size_t active_controller = controller_index;
      (void)active_motion_controller_index_.compare_exchange_strong(
        active_controller, kNoMotionController, std::memory_order_acq_rel);
    }
  }
  clear_in_progress();
  return result;
}

std::size_t EnableManagerController::controllerIndexForDeactivation() const
{
  const std::size_t active_controller =
    active_motion_controller_index_.load(std::memory_order_acquire);
  return active_controller < motion_controller_names_.size() ?
         active_controller : default_motion_controller_index_;
}

void EnableManagerController::handleNonRtFaultStop()
{
  if (!emergency_jtc_deactivate_request_.exchange(false, std::memory_order_acq_rel)) {
    return;
  }
  if (switchMotionController(controllerIndexForDeactivation(), false) !=
    SwitchResult::kSuccess)
  {
    restart_required_.store(true, std::memory_order_release);
  }
}

void EnableManagerController::publishDiagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = get_node()->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "/robot/rt_control/enable_manager";
  status.hardware_id = "robot-001";
  const Phase phase = phase_.load(std::memory_order_acquire);
  if (phase == Phase::kFailed || restart_required_.load(std::memory_order_acquire)) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "failed";
  } else if (
    phase != Phase::kIdle && phase != Phase::kEnabled && phase != Phase::kInactive)
  {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "operation in progress";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "operational";
  }
  const std::int8_t failed_joint = last_failed_joint_.load(std::memory_order_acquire);
  const auto add_value = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = value;
      status.values.push_back(std::move(item));
    };
  add_value("state", phaseName(phase));
  add_value(
    "failed_batch", std::to_string(last_failed_batch_.load(std::memory_order_acquire)));
  add_value("failed_joint", failed_joint >= 0 ? kJointNames[failed_joint] : "");
  add_value(
    "failed_status_word",
    std::to_string(last_failed_status_word_.load(std::memory_order_acquire)));
  add_value("stage", stageName(last_failure_stage_.load(std::memory_order_acquire)));
  array.status.push_back(std::move(status));
  diagnostics_publisher_->publish(array);
}

void EnableManagerController::publishResult(
  ResultSlot & slot, bool ok, Stage stage, std::int8_t failed_batch,
  std::int8_t failed_joint, std::uint16_t status_word)
{
  slot.ok.store(ok, std::memory_order_relaxed);
  slot.failed_batch.store(failed_batch, std::memory_order_relaxed);
  slot.failed_joint.store(failed_joint, std::memory_order_relaxed);
  slot.status_word.store(status_word, std::memory_order_relaxed);
  slot.stage.store(stage, std::memory_order_relaxed);
  slot.ready.store(true, std::memory_order_release);
}

void EnableManagerController::recordFailure(
  Stage stage, std::int8_t failed_batch, std::int8_t failed_joint,
  std::uint16_t status_word)
{
  last_failure_stage_.store(stage, std::memory_order_relaxed);
  last_failed_batch_.store(failed_batch, std::memory_order_relaxed);
  last_failed_joint_.store(failed_joint, std::memory_order_relaxed);
  last_failed_status_word_.store(status_word, std::memory_order_release);
}

void EnableManagerController::clearFailure()
{
  last_failure_stage_.store(Stage::kSuccess, std::memory_order_relaxed);
  last_failed_batch_.store(-1, std::memory_order_relaxed);
  last_failed_joint_.store(-1, std::memory_order_relaxed);
  last_failed_status_word_.store(0U, std::memory_order_release);
}

void EnableManagerController::startDownward(Phase phase, std::int64_t now_ns)
{
  downward_stage_ = 0U;
  stage_deadline_ns_ =
    now_ns + static_cast<std::int64_t>(disable_stage_timeout_seconds_ * 1e9);
  phase_.store(phase, std::memory_order_release);
}

void EnableManagerController::updateDownward(std::int64_t now_ns)
{
  bool any_operation_enabled = false;
  bool any_switched_on = false;
  bool all_nonfault_terminal = true;
  bool any_fault = false;
  std::int8_t first_nonterminal = -1;

  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    const DriveState state = decodeState(status_words_[axis]);
    std::uint16_t command = 0x0000U;
    switch (state) {
      case DriveState::kOperationEnabled:
        command = 0x0007U;
        any_operation_enabled = true;
        all_nonfault_terminal = false;
        break;
      case DriveState::kSwitchedOn:
        command = 0x0006U;
        any_switched_on = true;
        all_nonfault_terminal = false;
        break;
      case DriveState::kReadyToSwitchOn:
        command = 0x0000U;
        all_nonfault_terminal =
          all_nonfault_terminal && isConfirmedDisableTerminal(axis, state);
        break;
      case DriveState::kQuickStopActive:
      case DriveState::kNotReady:
      case DriveState::kUnknown:
        command = 0x0000U;
        all_nonfault_terminal = false;
        break;
      case DriveState::kFaultReactionActive:
      case DriveState::kFault:
        any_fault = true;
        command = 0x0000U;
        break;
      case DriveState::kSwitchOnDisabled:
        command = 0x0000U;
        break;
    }
    if (!isConfirmedDisableTerminal(axis, state) && !isFaultState(state) &&
      first_nonterminal < 0)
    {
      first_nonterminal = static_cast<std::int8_t>(axis);
    }
    command_interfaces_[axis].set_value(command);
  }

  bool advance_stage = false;
  if (downward_stage_ == 0U) {
    advance_stage = !any_operation_enabled;
  } else if (downward_stage_ == 1U) {
    advance_stage = !any_operation_enabled && !any_switched_on;
  } else if (all_nonfault_terminal) {
    finishDownward();
    return;
  }

  if (advance_stage) {
    ++downward_stage_;
    stage_deadline_ns_ =
      now_ns + static_cast<std::int64_t>(disable_stage_timeout_seconds_ * 1e9);
    if (downward_stage_ > 2U && all_nonfault_terminal) {
      finishDownward();
    }
    return;
  }

  if (now_ns >= stage_deadline_ns_) {
    if (first_nonterminal < 0 && any_fault) {
      for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
        if (isFaultState(decodeState(status_words_[axis]))) {
          first_nonterminal = static_cast<std::int8_t>(axis);
          break;
        }
      }
    }
    if (primary_failure_stage_ == Stage::kSuccess) {
      primary_failure_stage_ = any_fault ? Stage::kFaultRequiresReset : Stage::kDisableTimeout;
      primary_failed_joint_ = first_nonterminal;
      primary_failed_status_word_ =
        first_nonterminal >= 0 ? status_words_[first_nonterminal] : 0U;
      recordFailure(
        primary_failure_stage_, -1, primary_failed_joint_, primary_failed_status_word_);
    }
    if (phase_.load(std::memory_order_acquire) != Phase::kEmergencyDisable) {
      interrupted_owner_ = owner_.load(std::memory_order_acquire);
      stage_deadline_ns_ =
        now_ns + static_cast<std::int64_t>(disable_stage_timeout_seconds_ * 1e9);
      emergency_jtc_deactivate_request_.store(true, std::memory_order_release);
      phase_.store(Phase::kEmergencyQuickStop, std::memory_order_release);
    } else {
      finishDownward();
    }
  }
}

void EnableManagerController::startEmergency(
  Stage stage, std::int8_t failed_joint, std::uint16_t status_word,
  std::int64_t now_ns)
{
  interrupted_owner_ = owner_.load(std::memory_order_acquire);
  primary_failure_stage_ = stage;
  primary_failed_batch_ =
    interrupted_owner_ == Owner::kEnable ? static_cast<std::int8_t>(current_batch_) : -1;
  primary_failed_joint_ = failed_joint;
  primary_failed_status_word_ = status_word;
  recordFailure(stage, primary_failed_batch_, failed_joint, status_word);
  owner_.store(Owner::kInternal, std::memory_order_release);
  stage_deadline_ns_ =
    now_ns + static_cast<std::int64_t>(disable_stage_timeout_seconds_ * 1e9);
  emergency_jtc_deactivate_request_.store(true, std::memory_order_release);
  phase_.store(Phase::kEmergencyQuickStop, std::memory_order_release);
}

void EnableManagerController::updateEmergencyQuickStop(std::int64_t now_ns)
{
  bool any_operation_enabled = false;
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    const DriveState state = decodeState(status_words_[axis]);
    std::uint16_t command = 0x0000U;
    if (state == DriveState::kOperationEnabled || state == DriveState::kQuickStopActive) {
      command = 0x0002U;
    } else if (state == DriveState::kSwitchedOn) {
      command = 0x0006U;
    }
    any_operation_enabled = any_operation_enabled || state == DriveState::kOperationEnabled;
    command_interfaces_[axis].set_value(command);
  }
  if (!any_operation_enabled || now_ns >= stage_deadline_ns_) {
    startDownward(Phase::kEmergencyDisable, now_ns);
  }
}

void EnableManagerController::finishDownward()
{
  const Phase completed_phase = phase_.load(std::memory_order_acquire);
  bool any_fault = false;
  bool all_terminal = true;
  std::int8_t first_fault = -1;
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    const DriveState state = decodeState(status_words_[axis]);
    if (isFaultState(state)) {
      any_fault = true;
      if (first_fault < 0) {
        first_fault = static_cast<std::int8_t>(axis);
      }
    } else if (!isConfirmedDisableTerminal(axis, state)) {
      all_terminal = false;
    }
  }

  if (completed_phase == Phase::kStartupSanitizing) {
    if (primary_failure_stage_ != Stage::kSuccess) {
      phase_.store(Phase::kFailed, std::memory_order_release);
    } else if (any_fault) {
      recordFailure(Stage::kFaultRequiresReset, -1, first_fault, status_words_[first_fault]);
      phase_.store(Phase::kFailed, std::memory_order_release);
    } else {
      phase_.store(Phase::kIdle, std::memory_order_release);
    }
    owner_.store(Owner::kNone, std::memory_order_release);
    return;
  }

  if (completed_phase == Phase::kDisabling) {
    if (any_fault) {
      publishResult(
        disable_result_, false, Stage::kFaultRequiresReset, -1, first_fault,
        status_words_[first_fault]);
      recordFailure(Stage::kFaultRequiresReset, -1, first_fault, status_words_[first_fault]);
      phase_.store(Phase::kFailed, std::memory_order_release);
    } else if (jtc_deactivate_failed_.load(std::memory_order_acquire)) {
      publishResult(disable_result_, false, Stage::kJtcDeactivateFailed);
      phase_.store(Phase::kFailed, std::memory_order_release);
    } else if (primary_failure_stage_ != Stage::kSuccess) {
      publishResult(
        disable_result_, false, primary_failure_stage_, primary_failed_batch_,
        primary_failed_joint_, primary_failed_status_word_);
      phase_.store(Phase::kFailed, std::memory_order_release);
    } else {
      clearFailure();
      publishResult(disable_result_, true, Stage::kSuccess);
      phase_.store(Phase::kIdle, std::memory_order_release);
    }
    owner_.store(Owner::kNone, std::memory_order_release);
    return;
  }

  if (completed_phase == Phase::kRollback) {
    publishResult(
      enable_result_, false, primary_failure_stage_, primary_failed_batch_,
      primary_failed_joint_, primary_failed_status_word_);
    const bool restart = restart_required_.load(std::memory_order_acquire);
    phase_.store(any_fault || restart ? Phase::kFailed : Phase::kIdle, std::memory_order_release);
    owner_.store(Owner::kNone, std::memory_order_release);
    return;
  }

  if (completed_phase == Phase::kEmergencyDisable) {
    if (interrupted_owner_ == Owner::kEnable) {
      publishResult(
        enable_result_, false, primary_failure_stage_, primary_failed_batch_,
        primary_failed_joint_, primary_failed_status_word_);
    } else if (interrupted_owner_ == Owner::kDisable) {
      publishResult(
        disable_result_, false, primary_failure_stage_, primary_failed_batch_,
        primary_failed_joint_, primary_failed_status_word_);
    }
    const bool restart = restart_required_.load(std::memory_order_acquire);
    phase_.store(
      any_fault || !all_terminal || restart ? Phase::kFailed : Phase::kIdle,
      std::memory_order_release);
    owner_.store(Owner::kNone, std::memory_order_release);
  }
}

void EnableManagerController::updateEnable(std::int64_t now_ns)
{
  const Phase phase = phase_.load(std::memory_order_acquire);
  if (disable_request_.load(std::memory_order_acquire)) {
    enable_preempt_requested_ = true;
  }

  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    if (isFaultState(decodeState(status_words_[axis]))) {
      disable_request_.store(false, std::memory_order_release);
      startEmergency(
        Stage::kFaultDetected, static_cast<std::int8_t>(axis), status_words_[axis], now_ns);
      return;
    }
  }

  if (phase == Phase::kInterBatchDelay) {
    for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
      command_interfaces_[axis].set_value(
        decodeState(status_words_[axis]) == DriveState::kOperationEnabled ? 0x000FU : 0x0000U);
    }
    if (enable_preempt_requested_) {
      disable_request_.store(false, std::memory_order_release);
      publishResult(enable_result_, false, Stage::kPreemptedByDisable);
      owner_.store(Owner::kDisable, std::memory_order_release);
      primary_failure_stage_ = Stage::kSuccess;
      startDownward(Phase::kDisabling, now_ns);
    } else if (now_ns >= stage_deadline_ns_) {
      ++current_batch_;
      stage_deadline_ns_ = now_ns + static_cast<std::int64_t>(batch_timeout_seconds_ * 1e9);
      phase_.store(Phase::kEnabling, std::memory_order_release);
    }
    return;
  }

  bool batch_complete = true;
  std::int8_t invalid_axis = -1;
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    bool is_current = false;
    bool is_previous = false;
    for (std::size_t batch = 0; batch <= current_batch_; ++batch) {
      for (std::size_t item = 0; item < kBatchSizes[batch]; ++item) {
        if (static_cast<std::size_t>(kEnableBatches[batch][item]) == axis) {
          is_current = batch == current_batch_;
          is_previous = batch < current_batch_;
        }
      }
    }

    const DriveState state = decodeState(status_words_[axis]);
    std::uint16_t command = 0x0000U;
    if (is_previous) {
      command = 0x000FU;
      if (state != DriveState::kOperationEnabled && invalid_axis < 0) {
        invalid_axis = static_cast<std::int8_t>(axis);
      }
    } else if (is_current) {
      switch (state) {
        case DriveState::kSwitchOnDisabled:
          command = 0x0006U;
          batch_complete = false;
          break;
        case DriveState::kReadyToSwitchOn:
          command = 0x0007U;
          batch_complete = false;
          break;
        case DriveState::kSwitchedOn:
          command = 0x000FU;
          batch_complete = false;
          break;
        case DriveState::kOperationEnabled:
          command = 0x000FU;
          break;
        default:
          command = 0x0000U;
          batch_complete = false;
          if (invalid_axis < 0) {
            invalid_axis = static_cast<std::int8_t>(axis);
          }
          break;
      }
    }
    command_interfaces_[axis].set_value(command);
  }

  if (invalid_axis >= 0) {
    primary_failure_stage_ = Stage::kEnableInvalidState;
    primary_failed_batch_ = static_cast<std::int8_t>(current_batch_);
    primary_failed_joint_ = invalid_axis;
    primary_failed_status_word_ = status_words_[invalid_axis];
    recordFailure(
      primary_failure_stage_, primary_failed_batch_, invalid_axis,
      primary_failed_status_word_);
    if (enable_preempt_requested_) {
      publishResult(
        enable_result_, false, primary_failure_stage_, primary_failed_batch_,
        primary_failed_joint_, primary_failed_status_word_);
      owner_.store(Owner::kDisable, std::memory_order_release);
      disable_request_.store(false, std::memory_order_release);
      startDownward(Phase::kDisabling, now_ns);
    } else {
      startDownward(Phase::kRollback, now_ns);
    }
    return;
  }

  if (batch_complete) {
    if (enable_preempt_requested_) {
      disable_request_.store(false, std::memory_order_release);
      publishResult(enable_result_, false, Stage::kPreemptedByDisable);
      owner_.store(Owner::kDisable, std::memory_order_release);
      primary_failure_stage_ = Stage::kSuccess;
      startDownward(Phase::kDisabling, now_ns);
    } else if (current_batch_ + 1U == kBatchCount) {
      enable_hardware_ready_.store(true, std::memory_order_release);
      phase_.store(Phase::kJtcActivating, std::memory_order_release);
    } else {
      stage_deadline_ns_ =
        now_ns + static_cast<std::int64_t>(inter_batch_delay_seconds_ * 1e9);
      phase_.store(Phase::kInterBatchDelay, std::memory_order_release);
    }
    return;
  }

  if (now_ns >= stage_deadline_ns_) {
    std::int8_t failed_axis = kEnableBatches[current_batch_][0];
    for (std::size_t item = 0; item < kBatchSizes[current_batch_]; ++item) {
      const auto axis = static_cast<std::size_t>(kEnableBatches[current_batch_][item]);
      if (decodeState(status_words_[axis]) != DriveState::kOperationEnabled) {
        failed_axis = static_cast<std::int8_t>(axis);
        break;
      }
    }
    primary_failure_stage_ = Stage::kEnableBatchTimeout;
    primary_failed_batch_ = static_cast<std::int8_t>(current_batch_);
    primary_failed_joint_ = failed_axis;
    primary_failed_status_word_ = status_words_[failed_axis];
    recordFailure(
      primary_failure_stage_, primary_failed_batch_, failed_axis,
      primary_failed_status_word_);
    if (enable_preempt_requested_) {
      publishResult(
        enable_result_, false, primary_failure_stage_, primary_failed_batch_,
        primary_failed_joint_, primary_failed_status_word_);
      owner_.store(Owner::kDisable, std::memory_order_release);
      disable_request_.store(false, std::memory_order_release);
      primary_failure_stage_ = Stage::kSuccess;
      startDownward(Phase::kDisabling, now_ns);
    } else {
      startDownward(Phase::kRollback, now_ns);
    }
  }
}

void EnableManagerController::updateReset(std::int64_t now_ns)
{
  const Phase phase = phase_.load(std::memory_order_acquire);
  if (disable_request_.exchange(false, std::memory_order_acq_rel)) {
    setAllControlWords(0x0000U);
    publishResult(reset_result_, false, Stage::kPreemptedByDisable);
    owner_.store(Owner::kDisable, std::memory_order_release);
    primary_failure_stage_ = Stage::kSuccess;
    startDownward(Phase::kDisabling, now_ns);
    return;
  }

  if (phase == Phase::kResetLow) {
    setAllControlWords(0x0000U);
    stage_deadline_ns_ =
      now_ns + static_cast<std::int64_t>(fault_reset_timeout_seconds_ * 1e9);
    phase_.store(Phase::kResetHigh, std::memory_order_release);
    return;
  }

  bool all_reset = true;
  std::int8_t first_pending = -1;
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    const DriveState state = decodeState(status_words_[axis]);
    if (reset_targets_[axis] && state == DriveState::kFault) {
      command_interfaces_[axis].set_value(0x0080U);
      all_reset = false;
      if (first_pending < 0) {
        first_pending = static_cast<std::int8_t>(axis);
      }
    } else {
      command_interfaces_[axis].set_value(0x0000U);
      if (!isConfirmedDisableTerminal(axis, state)) {
        all_reset = false;
        if (first_pending < 0) {
          first_pending = static_cast<std::int8_t>(axis);
        }
      }
    }
  }

  if (all_reset) {
    clearFailure();
    publishResult(reset_result_, true, Stage::kSuccess);
    phase_.store(Phase::kIdle, std::memory_order_release);
    owner_.store(Owner::kNone, std::memory_order_release);
  } else if (now_ns >= stage_deadline_ns_) {
    const std::uint16_t status = first_pending >= 0 ? status_words_[first_pending] : 0U;
    recordFailure(Stage::kFaultResetTimeout, -1, first_pending, status);
    publishResult(reset_result_, false, Stage::kFaultResetTimeout, -1, first_pending, status);
    setAllControlWords(0x0000U);
    phase_.store(Phase::kFailed, std::memory_order_release);
    owner_.store(Owner::kNone, std::memory_order_release);
  }
}

void EnableManagerController::setAllControlWords(std::uint16_t control_word)
{
  for (auto & interface : command_interfaces_) {
    interface.set_value(control_word);
  }
}

bool EnableManagerController::allAxesInState(DriveState state) const
{
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    if (decodeState(status_words_[axis]) != state) {
      return false;
    }
  }
  return true;
}

std::int8_t EnableManagerController::firstAxisNotInState(DriveState state) const
{
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    if (decodeState(status_words_[axis]) != state) {
      return static_cast<std::int8_t>(axis);
    }
  }
  return -1;
}

void EnableManagerController::refreshStatusWords()
{
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    const double value = state_interfaces_[axis].get_value();
    status_words_[axis] =
      std::isfinite(value) ? static_cast<std::uint16_t>(value) : 0xFFFFU;
  }
}

EnableManagerController::DriveState EnableManagerController::decodeState(
  std::uint16_t status_word)
{
  if ((status_word & 0x004FU) == 0x0000U) {
    return DriveState::kNotReady;
  }
  if ((status_word & 0x004FU) == 0x0040U) {
    return DriveState::kSwitchOnDisabled;
  }
  if ((status_word & 0x006FU) == 0x0021U) {
    return DriveState::kReadyToSwitchOn;
  }
  if ((status_word & 0x006FU) == 0x0023U) {
    return DriveState::kSwitchedOn;
  }
  if ((status_word & 0x006FU) == 0x0027U) {
    return DriveState::kOperationEnabled;
  }
  if ((status_word & 0x006FU) == 0x0007U) {
    return DriveState::kQuickStopActive;
  }
  if ((status_word & 0x004FU) == 0x000FU) {
    return DriveState::kFaultReactionActive;
  }
  if ((status_word & 0x004FU) == 0x0008U) {
    return DriveState::kFault;
  }
  return DriveState::kUnknown;
}

bool EnableManagerController::isFaultState(DriveState state)
{
  return state == DriveState::kFault || state == DriveState::kFaultReactionActive;
}

bool EnableManagerController::isConfirmedDisableTerminal(std::size_t axis, DriveState state)
{
  if (state == DriveState::kSwitchOnDisabled) {
    return true;
  }
  const bool is_ti5_axis = axis == 1U || axis == 2U || axis == 7U || axis == 8U;
  return is_ti5_axis && state == DriveState::kReadyToSwitchOn;
}

const char * EnableManagerController::stageName(Stage stage)
{
  switch (stage) {
    case Stage::kSuccess: return "success";
    case Stage::kAlreadyEnabled: return "already_enabled";
    case Stage::kAlreadyDisabled: return "already_disabled";
    case Stage::kAlreadyClear: return "already_clear";
    case Stage::kOperationInProgress: return "operation_in_progress";
    case Stage::kControllerInactive: return "controller_inactive";
    case Stage::kPreemptedByDisable: return "preempted_by_disable";
    case Stage::kEnableBatchTimeout: return "enable_batch_timeout";
    case Stage::kEnableInvalidState: return "enable_invalid_state";
    case Stage::kFaultDetected: return "fault_detected";
    case Stage::kUnexpectedDriveState: return "unexpected_drive_state";
    case Stage::kDisableTimeout: return "disable_timeout";
    case Stage::kFaultRequiresReset: return "fault_requires_reset";
    case Stage::kFaultResetTimeout: return "fault_reset_timeout";
    case Stage::kJtcActivateFailed: return "jtc_activate_failed";
    case Stage::kJtcDeactivateFailed: return "jtc_deactivate_failed";
    case Stage::kControllerUpdateTimeout: return "controller_update_timeout";
    case Stage::kRestartRequired: return "restart_required";
  }
  return "unknown";
}

const char * EnableManagerController::phaseName(Phase phase)
{
  switch (phase) {
    case Phase::kInactive: return "INACTIVE";
    case Phase::kStartupSanitizing: return "STARTUP_SANITIZING";
    case Phase::kIdle: return "IDLE";
    case Phase::kEnabling: return "ENABLING";
    case Phase::kInterBatchDelay: return "INTER_BATCH_DELAY";
    case Phase::kJtcActivating: return "JTC_ACTIVATING";
    case Phase::kEnabled: return "ENABLED";
    case Phase::kJtcDeactivating: return "JTC_DEACTIVATING";
    case Phase::kDisabling: return "DISABLING";
    case Phase::kRollback: return "ROLLBACK";
    case Phase::kResetLow: return "RESET_LOW";
    case Phase::kResetHigh: return "RESET_HIGH";
    case Phase::kEmergencyQuickStop: return "EMERGENCY_QUICK_STOP";
    case Phase::kEmergencyDisable: return "EMERGENCY_DISABLE";
    case Phase::kFailed: return "FAILED";
  }
  return "UNKNOWN";
}

}  // namespace enable_manager

PLUGINLIB_EXPORT_CLASS(
  enable_manager::EnableManagerController, controller_interface::ControllerInterface)
