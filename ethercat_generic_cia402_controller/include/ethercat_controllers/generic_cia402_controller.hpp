#ifndef ETHERCAT_CONTROLLERS__GENERIC_CIA402_CONTROLLER_HPP_
#define ETHERCAT_CONTROLLERS__GENERIC_CIA402_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <atomic>

#include "controller_interface/controller_interface.hpp"
#include "ethercat_controllers/visibility_control.h"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "ethercat_controller_msgs/msg/cia402_drive_states.hpp"
#include "ethercat_controller_msgs/srv/switch_drive_mode_of_operation.hpp"
#include "ethercat_controller_msgs/srv/reset_drive_fault.hpp"
#include "ethercat_controller_msgs/srv/set_drive_states.hpp"
#include "ethercat_controller_msgs/action/set_modes_of_operation_action.hpp"
#include "ethercat_controllers/transition_utilities.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace ethercat_controllers
{
using DriveStateMsgType = ethercat_controller_msgs::msg::Cia402DriveStates;
using SwitchMOOSrv = ethercat_controller_msgs::srv::SwitchDriveModeOfOperation;
using ResetFaultSrv = ethercat_controller_msgs::srv::ResetDriveFault;
using SetDriveStatesSrv = ethercat_controller_msgs::srv::SetDriveStates;
using SetModesOfOperationAction = ethercat_controller_msgs::action::SetModesOfOperationAction;
using GoalHandleSetModesOfOperationAction = rclcpp_action::ServerGoalHandle<SetModesOfOperationAction>;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CiA402Controller : public controller_interface::ControllerInterface
{
public:
  CIA402_CONTROLLER_PUBLIC
  CiA402Controller();

  // CIA402_CONTROLLER_PUBLIC
  // ~CiA402Controller();

  CIA402_CONTROLLER_PUBLIC
  CallbackReturn on_init() override;

  CIA402_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  CIA402_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CIA402_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CIA402_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CIA402_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CIA402_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

protected:

  std::vector<std::unique_ptr<std::atomic<int8_t>>> current_mode_ops_;

  std::vector<std::string> dof_names_;
  std::vector<int> mode_ops_;
  std::vector<double> control_words_;
  std::vector<int16_t> status_words_;
  std::vector<bool> reset_faults_;

  bool forward_state(std::string& state);
  bool backward_state(std::string& state);

  using DriveStatePublisher = realtime_tools::RealtimePublisher<DriveStateMsgType>;
  rclcpp::Publisher<DriveStateMsgType>::SharedPtr drive_state_publisher_;
  std::unique_ptr<DriveStatePublisher> rt_drive_state_publisher_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<SwitchMOOSrv::Request>> rt_moo_srv_ptr_;
  rclcpp::Service<SwitchMOOSrv>::SharedPtr moo_srv_ptr_;

  // realtime_tools::RealtimeBuffer<std::shared_ptr<ResetFaultSrv::Request>> rt_reset_fault_srv_ptr_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_fault_srv_ptr_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<SetDriveStatesSrv::Request>> rt_sds_srv_ptr_;
  rclcpp::Service<SetDriveStatesSrv>::SharedPtr sds_srv_ptr_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr turn_on_ptr_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr turn_off_ptr_;

  realtime_tools::RealtimeBuffer<std::atomic<uint16_t>> actual_state_ptr_;
  std::vector<std::atomic<uint16_t>> actual_state_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<std_srvs::srv::Trigger::Request>> rt_homing_srv_ptr_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr homing_srv_ptr_;


  std::string logger_name_;

  std::string device_state_str(uint16_t status_word);

  std::string mode_of_operation_str(double mode_of_operation);

  int8_t mode_of_operation_int(const std::string&);

  uint16_t state_str_to_int(const std::string& state);
  uint16_t calc_controlword(uint8_t act_state, uint8_t next_state, uint16_t control_word);
  std::string state_int_to_str(const uint16_t& state);



  bool try_to_turn_off();
  bool try_turn_on();

  void switch_moo_callback(
    const std::shared_ptr<SwitchMOOSrv::Request> request,
    std::shared_ptr<SwitchMOOSrv::Response> response
  );

  void reset_fault_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response
  );

  void set_drive_states_callback(
    const std::shared_ptr<SetDriveStatesSrv::Request> request,
    std::shared_ptr<SetDriveStatesSrv::Response> response
  );

  void try_turn_on_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void try_turn_off_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void perform_homing_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp_action::Server<SetModesOfOperationAction>::SharedPtr set_moo_action_server_;

  rclcpp_action::GoalResponse set_moo_action_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SetModesOfOperationAction::Goal> goal);

  rclcpp_action::CancelResponse set_moo_action_handle_cancel(const std::shared_ptr<GoalHandleSetModesOfOperationAction>& goal_handle);
  
  void set_moo_action_handle_accepted(const std::shared_ptr<GoalHandleSetModesOfOperationAction>& goal_handle);

  void set_moo_action_execute(const std::shared_ptr<GoalHandleSetModesOfOperationAction>& goal_handle);
};




}  // namespace ethercat_controllers

#endif  // ETHERCAT_CONTROLLERS__GENERIC_CIA402_CONTROLLER_HPP_
