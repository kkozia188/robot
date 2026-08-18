#ifndef ENABLE_MANAGER__ENABLE_MANAGER_CONTROLLER_HPP_
#define ENABLE_MANAGER__ENABLE_MANAGER_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/timer.hpp"
#include "robot_interfaces/srv/rt_enable.hpp"

namespace enable_manager
{

class EnableManagerTestPeer;

class EnableManagerController final : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  friend class EnableManagerTestPeer;

  static constexpr std::size_t kAxisCount = 14U;
  static constexpr std::size_t kBatchCount = 5U;
  static constexpr std::size_t kNoMotionController =
    std::numeric_limits<std::size_t>::max();

  enum class Phase : std::uint8_t
  {
    kInactive,
    kStartupSanitizing,
    kIdle,
    kEnabling,
    kInterBatchDelay,
    kJtcActivating,
    kEnabled,
    kJtcDeactivating,
    kDisabling,
    kRollback,
    kResetLow,
    kResetHigh,
    kEmergencyQuickStop,
    kEmergencyDisable,
    kFailed
  };

  enum class Owner : std::uint8_t {kNone, kEnable, kDisable, kReset, kInternal};
  enum class SwitchResult : std::uint8_t {kSuccess, kFailed, kAmbiguous};
  enum class DriveState : std::uint8_t
  {
    kNotReady,
    kSwitchOnDisabled,
    kReadyToSwitchOn,
    kSwitchedOn,
    kOperationEnabled,
    kQuickStopActive,
    kFaultReactionActive,
    kFault,
    kUnknown
  };

  enum class Stage : std::uint8_t
  {
    kSuccess,
    kAlreadyEnabled,
    kAlreadyDisabled,
    kAlreadyClear,
    kOperationInProgress,
    kControllerInactive,
    kPreemptedByDisable,
    kEnableBatchTimeout,
    kEnableInvalidState,
    kFaultDetected,
    kUnexpectedDriveState,
    kDisableTimeout,
    kFaultRequiresReset,
    kFaultResetTimeout,
    kJtcActivateFailed,
    kJtcDeactivateFailed,
    kControllerUpdateTimeout,
    kRestartRequired
  };

  struct ResultSlot
  {
    std::atomic_bool ready{false};
    std::atomic_bool ok{false};
    std::atomic<std::int8_t> failed_batch{-1};
    std::atomic<std::int8_t> failed_joint{-1};
    std::atomic<std::uint16_t> status_word{0U};
    std::atomic<Stage> stage{Stage::kOperationInProgress};
  };

  static DriveState decodeState(std::uint16_t status_word);
  static const char * stageName(Stage stage);
  static const char * phaseName(Phase phase);
  static bool isFaultState(DriveState state);
  static bool isConfirmedDisableTerminal(std::size_t axis, DriveState state);

  void handleEnable(
    const std::shared_ptr<robot_interfaces::srv::RtEnable::Request> request,
    std::shared_ptr<robot_interfaces::srv::RtEnable::Response> response);
  void handleDisable(
    const std::shared_ptr<robot_interfaces::srv::RtEnable::Request> request,
    std::shared_ptr<robot_interfaces::srv::RtEnable::Response> response);
  void handleResetFault(
    const std::shared_ptr<robot_interfaces::srv::RtEnable::Request> request,
    std::shared_ptr<robot_interfaces::srv::RtEnable::Response> response);
  bool waitForResult(ResultSlot & slot);
  void fillResponse(
    const ResultSlot & slot, robot_interfaces::srv::RtEnable::Response & response) const;
  void fillImmediateResponse(
    robot_interfaces::srv::RtEnable::Response & response, bool ok, Stage stage) const;
  SwitchResult switchMotionController(std::size_t controller_index, bool activate);
  std::size_t controllerIndexForDeactivation() const;
  void handleNonRtFaultStop();
  void publishDiagnostics();

  void publishResult(
    ResultSlot & slot, bool ok, Stage stage, std::int8_t failed_batch = -1,
    std::int8_t failed_joint = -1, std::uint16_t status_word = 0U);
  void recordFailure(
    Stage stage, std::int8_t failed_batch, std::int8_t failed_joint,
    std::uint16_t status_word);
  void clearFailure();
  void startDownward(Phase phase, std::int64_t now_ns);
  void updateDownward(std::int64_t now_ns);
  void startEmergency(
    Stage stage, std::int8_t failed_joint, std::uint16_t status_word,
    std::int64_t now_ns);
  void updateEmergencyQuickStop(std::int64_t now_ns);
  void finishDownward();
  void updateEnable(std::int64_t now_ns);
  void updateReset(std::int64_t now_ns);
  void setAllControlWords(std::uint16_t control_word);
  bool allAxesInState(DriveState state) const;
  std::int8_t firstAxisNotInState(DriveState state) const;
  void refreshStatusWords();

  static const std::array<const char *, kAxisCount> kJointNames;
  static const std::array<std::array<std::int8_t, 3>, kBatchCount> kEnableBatches;
  static const std::array<std::uint8_t, kBatchCount> kBatchSizes;

  std::atomic_bool active_{false};
  std::atomic<Phase> phase_{Phase::kInactive};
  std::atomic<Owner> owner_{Owner::kInternal};
  std::atomic_bool enable_request_{false};
  std::atomic_bool disable_request_{false};
  std::atomic_bool reset_request_{false};
  std::atomic_bool enable_hardware_ready_{false};
  std::atomic_bool jtc_activate_failed_request_{false};
  std::atomic_bool emergency_jtc_deactivate_request_{false};
  std::atomic_bool restart_required_{false};
  std::atomic_bool switch_in_progress_{false};
  std::atomic_bool enable_callback_active_{false};
  std::atomic_bool disable_callback_active_{false};
  std::atomic_bool reset_callback_active_{false};

  ResultSlot enable_result_;
  ResultSlot disable_result_;
  ResultSlot reset_result_;
  std::atomic<Stage> last_failure_stage_{Stage::kSuccess};
  std::atomic<std::int8_t> last_failed_batch_{-1};
  std::atomic<std::int8_t> last_failed_joint_{-1};
  std::atomic<std::uint16_t> last_failed_status_word_{0U};

  std::array<std::uint16_t, kAxisCount> status_words_{};
  std::array<bool, kAxisCount> reset_targets_{};
  std::size_t current_batch_{0U};
  std::uint8_t downward_stage_{0U};
  std::int64_t stage_deadline_ns_{0};
  bool enable_preempt_requested_{false};
  Owner interrupted_owner_{Owner::kNone};
  std::atomic_bool jtc_deactivate_failed_{false};
  Stage primary_failure_stage_{Stage::kSuccess};
  std::int8_t primary_failed_batch_{-1};
  std::int8_t primary_failed_joint_{-1};
  std::uint16_t primary_failed_status_word_{0U};

  double batch_timeout_seconds_{4.0};
  double disable_stage_timeout_seconds_{4.0};
  double inter_batch_delay_seconds_{0.2};
  double fault_reset_timeout_seconds_{4.0};
  double controller_switch_timeout_seconds_{4.0};
  std::chrono::milliseconds service_result_timeout_{30000};
  std::vector<std::string> motion_controller_names_{"dual_arm_jtc"};
  std::size_t default_motion_controller_index_{0U};
  std::atomic_size_t active_motion_controller_index_{kNoMotionController};

  rclcpp::CallbackGroup::SharedPtr enable_callback_group_;
  rclcpp::CallbackGroup::SharedPtr disable_callback_group_;
  rclcpp::CallbackGroup::SharedPtr reset_callback_group_;
  rclcpp::CallbackGroup::SharedPtr worker_callback_group_;
  rclcpp::Service<robot_interfaces::srv::RtEnable>::SharedPtr enable_service_;
  rclcpp::Service<robot_interfaces::srv::RtEnable>::SharedPtr disable_service_;
  rclcpp::Service<robot_interfaces::srv::RtEnable>::SharedPtr reset_service_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr worker_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace enable_manager

#endif  // ENABLE_MANAGER__ENABLE_MANAGER_CONTROLLER_HPP_
