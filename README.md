# ethercat_controller

The package has been dervied from the folder [ethercat_generic_cia402_controller](https://github.com/ICube-Robotics/ethercat_driver_ros2/tree/7fa84771fbca82406b88eb6a1b609b699960823d/ethercat_controllers/ethercat_generic_cia402_controller) from the branch [mcbed-ec-cia402-controller](https://github.com/ICube-Robotics/ethercat_driver_ros2/tree/mcbed-ec-cia402-controller) of the project [ethercat_driver_ros2 ICube-Robotics](https://github.com/ICube-Robotics/ethercat_driver_ros2) that has never been merged into the main.

The modification from the original folder are many.

## ROS 2 Dependencies
beyond the depndencies of  [ethercat_driver_ros2](https://github.com/ICube-Robotics/ethercat_driver_ros2)
The package needs the message definition stored in the package [ethercat_controller_msgs](https://github.com/CNR-STIIMA-IRAS/ethercat_controller_msgs)

## ROS 2 interfaces

The generic CIA402 controller exposes the following ROS 2 interfaces under the controller namespace (the `~` prefix resolves to the controller name that is configured when the controller is loaded).

### Topics
- `~/drive_states` (publisher, `ethercat_controller_msgs::msg::Cia402DriveStates`): publishes the mode of operation and status word for each configured axis so that other nodes can monitor the controller's view of the drives.

### Services
These services were present in the original implementation at [mcbed-ec-cia402-controller](https://github.com/ICube-Robotics/ethercat_driver_ros2/tree/mcbed-ec-cia402-controller)
- `~/switch_mode_of_operation` (`ethercat_controller_msgs::srv::SwitchDriveModeOfOperation`): request a mode change for one or more drives .
- `~/reset_fault` (`std_srvs::srv::Trigger`): attempt to clear any reported drive faults.

These services are **NEW**
- `~/set_drive_state` (`ethercat_controller_msgs::srv::SetDriveStates`): request specific control-word/status-word pairs for the configured drives.
- `~/try_turn_on` (`std_srvs::srv::Trigger`): try to transition the drives toward the “Operation Enabled” state.
- `~/try_turn_off` (`std_srvs::srv::Trigger`): request the controller to move the drives toward a non-operational state.
- `~/perform_homing` (`std_srvs::srv::Trigger`): invoke the configured homing routine for all drives.

### Actions
- `~/async_set_modes_of_operation` (`ethercat_controller_msgs::action::SetModesOfOperationAction`): asynchronously change the mode of operation for a group of drives with feedback.
 
