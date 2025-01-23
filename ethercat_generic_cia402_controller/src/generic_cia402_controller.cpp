
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "ethercat_controllers/generic_cia402_controller.hpp"

namespace ethercat_controllers
{
using hardware_interface::LoanedCommandInterface;

CiA402Controller::CiA402Controller()
: controller_interface::ControllerInterface(),
  rt_drive_state_publisher_(nullptr),
  actual_state_(3)
{

    for (auto& item : actual_state_) {
        item.store(ethercat_controller_msgs::msg::Cia402DriveStates::STATE_UNDEFINED);
    }
}

CallbackReturn CiA402Controller::on_init()
{
  try {
    // definition of the parameters that need to be queried from the
    // controller configuration file with default values
    auto_declare<std::vector<std::string>>("dofs", std::vector<std::string>());
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }    

  return CallbackReturn::SUCCESS;
}

CallbackReturn CiA402Controller::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // getting the names of the dofs to be controlled
  dof_names_ = get_node()->get_parameter("dofs").as_string_array();

  if (dof_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'dofs' parameter was empty");
    return CallbackReturn::FAILURE;
  }

  mode_ops_.resize(dof_names_.size(), std::numeric_limits<int>::quiet_NaN());
  control_words_.resize(dof_names_.size(), 0);
  reset_faults_.resize(dof_names_.size(), false);

  try {
    // register data publisher
    drive_state_publisher_ = get_node()->create_publisher<DriveStateMsgType>(
      "~/drive_states", rclcpp::SystemDefaultsQoS());
    rt_drive_state_publisher_ = std::make_unique<DriveStatePublisher>(drive_state_publisher_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Exception thrown during publisher creation at configure stage"
      "with message : %s \n",
      e.what()
    );
    return CallbackReturn::ERROR;
  }

  using namespace std::placeholders;
  moo_srv_ptr_ = get_node()->create_service<SwitchMOOSrv>(
    "~/switch_mode_of_operation", std::bind(&CiA402Controller::switch_moo_callback, this, _1, _2));
  // reset_fault_srv_ptr_ = get_node()->create_service<ResetFaultSrv>(
  reset_fault_srv_ptr_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/reset_fault", std::bind(&CiA402Controller::reset_fault_callback, this, _1, _2));

  sds_srv_ptr_ = get_node()->create_service<SetDriveStatesSrv>(
    "~/set_drive_state", std::bind(&CiA402Controller::set_drive_states_callback, this, _1, _2));

  turn_on_ptr_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/try_turn_on", std::bind(&CiA402Controller::try_turn_on_callback, this, _1, _2));

  turn_off_ptr_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/try_turn_off", std::bind(&CiA402Controller::try_turn_off_callback, this, _1, _2));

  homing_srv_ptr_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/perform_homing", std::bind(&CiA402Controller::perform_homing_callback, this, _1, _2));


  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CiA402Controller::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(dof_names_.size() * 3);
  for (const auto & dof_name : dof_names_) {
    conf.names.push_back(dof_name + "/" + "control_word");
    conf.names.push_back(dof_name + "/" + "mode_of_operation");
    conf.names.push_back(dof_name + "/" + "reset_fault");
  }
  return conf;
}

controller_interface::InterfaceConfiguration
CiA402Controller::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(dof_names_.size() * 2);
  for (const auto & dof_name : dof_names_) {
    conf.names.push_back(dof_name + "/" + "mode_of_operation");
    conf.names.push_back(dof_name + "/" + "status_word");
  }
  return conf;
}

CallbackReturn CiA402Controller::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn CiA402Controller::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type CiA402Controller::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{

  std::shared_ptr<SetDriveStatesSrv::Request> rt_state_req;
  if (rt_drive_state_publisher_ && rt_drive_state_publisher_->trylock()) {
    auto & msg = rt_drive_state_publisher_->msg_;
    msg.header.stamp = get_node()->now();
    msg.dof_names.resize(dof_names_.size());
    msg.modes_of_operation.resize(dof_names_.size());
    msg.status_words.resize(dof_names_.size());
    msg.drive_states.resize(dof_names_.size());
    status_words_.resize(dof_names_.size());

    bool fault_present = false;
    for (auto i = 0ul; i < dof_names_.size(); i++) {
      msg.dof_names[i] = dof_names_[i];
      msg.modes_of_operation[i] = mode_of_operation_str(state_interfaces_[2 * i].get_value());
      msg.status_words[i] = state_interfaces_[2 * i + 1].get_value();
      status_words_[i] = state_interfaces_[2 * i + 1].get_value();
      msg.drive_states[i] = device_state_str(state_interfaces_[2 * i + 1].get_value());
      actual_state_[i].store(state_str_to_int(device_state_str(state_interfaces_[2 * i + 1].get_value())));
      if (state_str_to_int(device_state_str(state_interfaces_[2 * i + 1].get_value())) == ethercat_controller_msgs::msg::Cia402DriveStates::STATE_FAULT)
      {
        fault_present = true;
      }
    }
    if (fault_present)
    {
      std::vector<std::string> req_names;
      for (auto i = 0ul; i < dof_names_.size(); i++) {
        if(actual_state_[i] == ethercat_controller_msgs::msg::Cia402DriveStates::STATE_OPERATION_ENABLED)
        {
          req_names.push_back(dof_names_[i]);
        }
      }
      if(req_names.size()>0)
      {
        rt_state_req = std::make_shared<SetDriveStatesSrv::Request>();
        rt_state_req->dof_names = req_names;
        rt_state_req->drive_states.resize(req_names.size(), state_int_to_str(ethercat_controller_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON_DISABLED));
      }


    }

    rt_drive_state_publisher_->unlockAndPublish();
  }
  
  
  // getting the data from services using the rt pipe
  auto moo_request = rt_moo_srv_ptr_.readFromRT();
  auto homing_request = rt_homing_srv_ptr_.readFromRT();
  // auto reset_fault_request = rt_reset_fault_srv_ptr_.readFromRT();
  auto sds_request = rt_sds_srv_ptr_.readFromRT();

  for (auto i = 0ul; i < dof_names_.size(); i++) {
    if (!moo_request || !(*moo_request)) {
      mode_ops_[i] = state_interfaces_[2 * i].get_value();
    } else {
      if (dof_names_[i] == (*moo_request)->dof_name) {
        mode_ops_[i] = (*moo_request)->mode_of_operation;
      }
    }

    // if (reset_fault_request && (*reset_fault_request)) {
    //   if (dof_names_[i] == (*reset_fault_request)->dof_name) {
    //     reset_faults_[i] = true;
    //     rt_reset_fault_srv_ptr_.reset();
    //   }
    // }

    if (homing_request && (*homing_request)) {
      int dof_idx = 0;
      for (auto i = 0ul; i < dof_names_.size(); i++) {
        dof_idx ++;
        RCLCPP_INFO(get_node()->get_logger(),"Inside homing for loop, setting control world for dof number %d", dof_idx);
        uint16_t cw = static_cast<uint16_t>(control_words_[i]);
        cw |= (1 << 4);
        std::cout << cw << " " << control_words_[i] << std::endl;
        control_words_[i] = static_cast<double>(cw);
        RCLCPP_INFO(get_node()->get_logger(),"Set control world: %f", control_words_[i]);
      }
      dof_idx = 0;
      rt_homing_srv_ptr_.reset();
    }
    else
    {
      // for (auto i = 0ul; i < dof_names_.size(); i++) {
      //   uint16_t cw = static_cast<uint16_t>(control_words_[i]);
      //   cw &= ~(1 << 4);
      //   control_words_[i] = static_cast<double>(cw);
      // }
    }

    if (sds_request && (*sds_request)) {
      for (auto jj = 0ul; jj < (*sds_request)->dof_names.size(); jj++) {
        if (dof_names_[i] == (*sds_request)->dof_names[jj]) {
          uint16_t st = state_str_to_int((*sds_request)->drive_states[jj]);
          auto actual_state = state_str_to_int(device_state_str(status_words_[i]));
          control_words_[i] = calc_controlword(
            actual_state,
            st,
            command_interfaces_[3 * i].get_value());

        }
      }
    }

    if (rt_state_req) {
      for (auto jj = 0ul; jj < rt_state_req->dof_names.size(); jj++) {
        if (dof_names_[i] == rt_state_req->dof_names[jj]) {
          uint16_t st = state_str_to_int(rt_state_req->drive_states[jj]);
          auto actual_state = state_str_to_int(device_state_str(status_words_[i]));
          control_words_[i] = calc_controlword(
            actual_state,
            st,
            command_interfaces_[3 * i].get_value());

        }
      }
    }
    command_interfaces_[3 * i].set_value(control_words_[i]);  // control_word
    command_interfaces_[3 * i + 1].set_value(mode_ops_[i]);  // mode_of_operation
    command_interfaces_[3 * i + 2].set_value(reset_faults_[i]);  // reset_fault
    reset_faults_[i] = false;
  }
  rt_sds_srv_ptr_.reset();


  return controller_interface::return_type::OK;
}




uint16_t CiA402Controller::calc_controlword(uint8_t act_state, uint8_t next_state, uint16_t control_word)
{
    uint16_t new_controlword;
    new_controlword = control_word;
    if ( act_state==next_state )
        return control_word;
    
    int ret = 0;
    std::vector<std::pair<TransitionID,uint8_t> >   possible_states_from_act  = STATE_TRANSITIONS_MAP.at(act_state);
    std::vector<std::pair<TransitionID,uint8_t> >   possible_states_from_next = STATE_TRANSITIONS_MAP.at(next_state);

    std::vector<TransitionID>                       possible_transitions;

    for ( std::vector<std::pair<TransitionID,uint8_t> >::const_iterator ita = possible_states_from_act.begin(); ita != possible_states_from_act.end(); ita++)
    {
        // for ( std::vector<std::pair<TransitionID,uint8_t> >::const_iterator itn = possible_states_from_next.begin(); itn != possible_states_from_next.end(); itn++)
        // {
          if( ita->second == next_state )
          {
            if(TRANSITION_TYPE_MAP.at( ita->first ) != TRANSITION_FOO)
            {
              possible_transitions.push_back( ita->first );
            }
          }
        // }
    }

    if(possible_transitions.size() == 0 || possible_transitions.size() > 1 )
    {
        // RCLCPP_ERROR(get_node()->get_logger(), "ERROR ******** \n" );
        // RCLCPP_ERROR(get_node()->get_logger(), "       Act state (IN ): %s \n ", STATEID_STRINGS.at( act_state ).c_str() );
        // RCLCPP_ERROR(get_node()->get_logger(), "ERROR ******** \n" );
        // RCLCPP_ERROR(get_node()->get_logger(), "       Nex state (IN ): %s \n ", STATEID_STRINGS.at( next_state ).c_str() );
        RCLCPP_ERROR(get_node()->get_logger(), "TRANSITIONS FOUND %zu: ", possible_transitions.size() );
        for(size_t i=0;i<possible_transitions.size(); i++)
        {
            RCLCPP_ERROR (get_node()->get_logger(), "\t  FOUND TRANSITIONS : %s", TRANSITIONID_STRINGS.at(  possible_transitions.at(i) ).c_str() );
        }
        ret = control_word;
    } 
    else if(possible_transitions.size() == 1 )
    {
    //     new_controlword = ( *controlword_addr );
        new_controlword &= ( ~COMMAND_MASK.at( to_commandid(possible_transitions[0] ) ).at("MASK") );
        new_controlword |= COMMAND_MASK.at( to_commandid( possible_transitions[0] ) ).at("VAL");
    
    //     std::memcpy ( controlword_addr, &new_controlword, sizeof ( uint16_t ) );
    //     ret = 0;
    }
    return new_controlword;
}

std::string CiA402Controller::state_int_to_str(const uint16_t& state)
{
  if (state == ethercat_controller_msgs::msg::Cia402DriveStates::STATE_START) {
    return "STATE_START";
  } else if (state == ethercat_controller_msgs::msg::Cia402DriveStates::STATE_NOT_READY_TO_SWITCH_ON) {
    return "STATE_NOT_READY_TO_SWITCH_ON";
  } else if (state == ethercat_controller_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON_DISABLED) {
    return "STATE_SWITCH_ON_DISABLED";
  } else if (state == ethercat_controller_msgs::msg::Cia402DriveStates::STATE_READY_TO_SWITCH_ON) {
    return "STATE_READY_TO_SWITCH_ON";
  } else if (state == ethercat_controller_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON) {
    return "STATE_SWITCH_ON";
  } else if (state == ethercat_controller_msgs::msg::Cia402DriveStates::STATE_OPERATION_ENABLED) {
    return "STATE_OPERATION_ENABLED";
  } else if (state == ethercat_controller_msgs::msg::Cia402DriveStates::STATE_QUICK_STOP_ACTIVE) {
    return "STATE_QUICK_STOP_ACTIVE";
  } else if (state == ethercat_controller_msgs::msg::Cia402DriveStates::STATE_FAULT_REACTION_ACTIVE) {
    return "STATE_FAULT_REACTION_ACTIVE";
  } else if (state == ethercat_controller_msgs::msg::Cia402DriveStates::STATE_FAULT) {
    return "STATE_FAULT";
  }
  return "STATE_UNDEFINED";

}

uint16_t CiA402Controller::state_str_to_int(const std::string& state)
{
  if (state == "STATE_START") {
    return ethercat_controller_msgs::msg::Cia402DriveStates::STATE_START;
  } else if (state == "STATE_NOT_READY_TO_SWITCH_ON") {
    return ethercat_controller_msgs::msg::Cia402DriveStates::STATE_NOT_READY_TO_SWITCH_ON;
  } else if (state == "STATE_SWITCH_ON_DISABLED") {
    return ethercat_controller_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON_DISABLED;
  } else if (state == "STATE_READY_TO_SWITCH_ON") {
    return ethercat_controller_msgs::msg::Cia402DriveStates::STATE_READY_TO_SWITCH_ON;
  } else if (state == "STATE_SWITCH_ON") {
    return ethercat_controller_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON;
  } else if (state == "STATE_OPERATION_ENABLED") {
    return ethercat_controller_msgs::msg::Cia402DriveStates::STATE_OPERATION_ENABLED;
  } else if (state == "STATE_QUICK_STOP_ACTIVE") {
    return ethercat_controller_msgs::msg::Cia402DriveStates::STATE_QUICK_STOP_ACTIVE;
  } else if (state == "STATE_FAULT_REACTION_ACTIVE") {
    return ethercat_controller_msgs::msg::Cia402DriveStates::STATE_FAULT_REACTION_ACTIVE;
  } else if (state == "STATE_FAULT") {
    return ethercat_controller_msgs::msg::Cia402DriveStates::STATE_FAULT;
  }
  return ethercat_controller_msgs::msg::Cia402DriveStates::STATE_UNDEFINED;
}


/** returns device state based upon the status_word */
std::string CiA402Controller::device_state_str(uint16_t status_word)
{
  if ((status_word & 0b01001111) == 0b00000000) {
    return "STATE_NOT_READY_TO_SWITCH_ON";
  } else if ((status_word & 0b01001111) == 0b01000000) {
    return "STATE_SWITCH_ON_DISABLED";
  } else if ((status_word & 0b01101111) == 0b00100001) {
    return "STATE_READY_TO_SWITCH_ON";
  } else if ((status_word & 0b01101111) == 0b00100011) {
    return "STATE_SWITCH_ON";
  } else if ((status_word & 0b01101111) == 0b00100111) {
    return "STATE_OPERATION_ENABLED";
  } else if ((status_word & 0b01101111) == 0b00000111) {
    return "STATE_QUICK_STOP_ACTIVE";
  } else if ((status_word & 0b01001111) == 0b00001111) {
    return "STATE_FAULT_REACTION_ACTIVE";
  } else if ((status_word & 0b01001111) == 0b00001000) {
    return "STATE_FAULT";
  }
  return "STATE_UNDEFINED";
}

/** returns mode str based upon the mode_of_operation value */
std::string CiA402Controller::mode_of_operation_str(double mode_of_operation)
{
  if (mode_of_operation == 0) {
    return "MODE_NO_MODE";
  } else if (mode_of_operation == 1) {
    return "MODE_PROFILED_POSITION";
  } else if (mode_of_operation == 3) {
    return "MODE_PROFILED_VELOCITY";
  } else if (mode_of_operation == 4) {
    return "MODE_PROFILED_TORQUE";
  } else if (mode_of_operation == 6) {
    return "MODE_HOMING";
  } else if (mode_of_operation == 7) {
    return "MODE_INTERPOLATED_POSITION";
  } else if (mode_of_operation == 8) {
    return "MODE_CYCLIC_SYNC_POSITION";
  } else if (mode_of_operation == 9) {
    return "MODE_CYCLIC_SYNC_VELOCITY";
  } else if (mode_of_operation == 10) {
    return "MODE_CYCLIC_SYNC_TORQUE";
  }
  return "MODE_UNDEFINED";
}

void CiA402Controller::switch_moo_callback(
  const std::shared_ptr<SwitchMOOSrv::Request> request,
  std::shared_ptr<SwitchMOOSrv::Response> response
)
{
  if (find(dof_names_.begin(), dof_names_.end(), request->dof_name) != dof_names_.end()) {
    rt_moo_srv_ptr_.writeFromNonRT(request);
    response->return_message = "Request transmitted to drive at dof:" + request->dof_name;
  } else {
    response->return_message = "Abort. DoF " + request->dof_name + " not configured.";
  }
}

// void CiA402Controller::reset_fault_callback(
//   const std::shared_ptr<ResetFaultSrv::Request> request,
//   std::shared_ptr<ResetFaultSrv::Response> response
// )
void CiA402Controller::reset_fault_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response
)
{
  // if (find(dof_names_.begin(), dof_names_.end(), request->dof_name) != dof_names_.end()) {
  //   rt_reset_fault_srv_ptr_.writeFromNonRT(request);
  //   response->return_message = "Request transmitted to drive at dof:" + request->dof_name;
  // } else {
  //   response->return_message = "Abort. DoF " + request->dof_name + " not configured.";
  // }
  
  //guarda quale dof ha bisogno di reset
  std::vector<std::string> req_names;
  for (auto i = 0ul; i < dof_names_.size(); i++) {
    if (actual_state_[i] == ethercat_controller_msgs::msg::Cia402DriveStates::STATE_FAULT) {
      req_names.push_back(dof_names_[i]);
    }
  }
  
  std::string message = "";
  if (req_names.size() == 0) {
    response->success = true;
    response->message = "No drives in fault state";
    return;
  } 

  std::shared_ptr<SetDriveStatesSrv::Request> state_req = std::make_shared<SetDriveStatesSrv::Request>();
  state_req->dof_names = req_names;
  state_req->drive_states.resize(req_names.size(), state_int_to_str(ethercat_controller_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON_DISABLED));

  rt_sds_srv_ptr_.writeFromNonRT(state_req);
  // // wait for the actual state equal to the requested state with a timeout
  rclcpp::Time start_time = get_node()->now();
  rclcpp::Duration timeout = rclcpp::Duration::from_seconds(3.0);
  rclcpp::Time current_time = get_node()->now();
  bool all_states_reached = true;

  while (current_time - start_time < timeout) {
    all_states_reached = true;
    for (auto i = 0ul; i < req_names.size(); i++) {
      if (actual_state_[i] != state_str_to_int(state_req->drive_states[i])) {
        all_states_reached = false;
        message = "Fault reset failed";
        break;
      }
    }
    if (all_states_reached) {
      break;
    }
    // sleep a millesecond
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    current_time = get_node()->now();
  }
  response->success = all_states_reached;
  response->message = message;


}

bool CiA402Controller::backward_state(std::string& message)
{

  std::vector<uint16_t> desired_states;
  desired_states.resize(dof_names_.size(), ethercat_controller_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON_DISABLED);

  std::shared_ptr<SetDriveStatesSrv::Request> state_req = std::make_shared<SetDriveStatesSrv::Request>();
  state_req->dof_names = dof_names_;
  state_req->drive_states.resize(dof_names_.size());

  // mappa che asegna al dof richiesto un index

  for (auto i = 0ul; i < dof_names_.size(); i++) {
    if (actual_state_[i] == ethercat_controller_msgs::msg::Cia402DriveStates::STATE_FAULT) {
      message = "Fault needs reset";
      return false;
    }
    else if (actual_state_[i] > ethercat_controller_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON_DISABLED &&
                     actual_state_[i] <= ethercat_controller_msgs::msg::Cia402DriveStates::STATE_OPERATION_ENABLED) {
      desired_states[i] = actual_state_[i];
      desired_states[i]--;
      state_req->drive_states[i] = state_int_to_str(desired_states[i]);
    }
    else if (actual_state_[i] == ethercat_controller_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON_DISABLED)
    {
      desired_states[i] = ethercat_controller_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON_DISABLED;
      state_req->drive_states[i] = state_int_to_str(ethercat_controller_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON_DISABLED);
    }
    else
    {
      message = "Unrecognized actual state state setting";
      return false;
    }
  }
  rt_sds_srv_ptr_.writeFromNonRT(state_req);
  // // wait for the actual state equal to the requested state with a timeout
  rclcpp::Time start_time = get_node()->now();
  rclcpp::Duration timeout = rclcpp::Duration::from_seconds(3.0);
  rclcpp::Time current_time = get_node()->now();
  bool all_states_reached = true;

  while (current_time - start_time < timeout) {
    all_states_reached = true;
    for (auto i = 0ul; i < dof_names_.size(); i++) {
      if (actual_state_[i] != state_str_to_int(state_req->drive_states[i])) {
        all_states_reached = false;
        break;
      }
    }
    if (all_states_reached) {
      break;
    }
    // sleep a millesecond
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    current_time = get_node()->now();
  }
  return all_states_reached;
}

bool CiA402Controller::forward_state(std::string& message)
{

  std::vector<uint16_t> desired_states;
  desired_states.resize(dof_names_.size(), ethercat_controller_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON_DISABLED);

  std::shared_ptr<SetDriveStatesSrv::Request> state_req = std::make_shared<SetDriveStatesSrv::Request>();
  state_req->dof_names = dof_names_;
  state_req->drive_states.resize(dof_names_.size());

  // mappa che asegna al dof richiesto un index

  for (auto i = 0ul; i < dof_names_.size(); i++) {
    if (actual_state_[i] == ethercat_controller_msgs::msg::Cia402DriveStates::STATE_FAULT) {
      message = "Fault needs reset";
      return false;
    }
    else if (actual_state_[i] >= ethercat_controller_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON_DISABLED &&
                     actual_state_[i] < ethercat_controller_msgs::msg::Cia402DriveStates::STATE_OPERATION_ENABLED) {
      desired_states[i] = actual_state_[i];
      desired_states[i]++;
      state_req->drive_states[i] = state_int_to_str(desired_states[i]);
    }
    else if (actual_state_[i] == ethercat_controller_msgs::msg::Cia402DriveStates::STATE_OPERATION_ENABLED)
    {
      desired_states[i] = ethercat_controller_msgs::msg::Cia402DriveStates::STATE_OPERATION_ENABLED;
      state_req->drive_states[i] = state_int_to_str(ethercat_controller_msgs::msg::Cia402DriveStates::STATE_OPERATION_ENABLED);
    }
    else
    {
      message = "Unrecognized actual state state setting";
      return false;
    }
  }
  rt_sds_srv_ptr_.writeFromNonRT(state_req);
  // // wait for the actual state equal to the requested state with a timeout
  rclcpp::Time start_time = get_node()->now();
  rclcpp::Duration timeout = rclcpp::Duration::from_seconds(3.0);
  rclcpp::Time current_time = get_node()->now();
  bool all_states_reached = true;

  while (current_time - start_time < timeout) {
    all_states_reached = true;
    for (auto i = 0ul; i < dof_names_.size(); i++) {
      if (actual_state_[i] != state_str_to_int(state_req->drive_states[i])) {
        all_states_reached = false;
        break;
      }
    }
    if (all_states_reached) {
      break;
    }
    // sleep a millesecond
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    current_time = get_node()->now();
  }
  return all_states_reached;
}

bool CiA402Controller::try_turn_on()
{
  rclcpp::Time start_time = get_node()->now();
  rclcpp::Duration timeout = rclcpp::Duration::from_seconds(3.0);
  rclcpp::Time current_time = get_node()->now();
  bool all_states_reached = false;

  while (current_time - start_time < timeout && !all_states_reached) {
    all_states_reached = true;
    for (auto i = 0ul; i < dof_names_.size(); i++) {
      if (actual_state_[i] != ethercat_controller_msgs::msg::Cia402DriveStates::STATE_OPERATION_ENABLED) {
        all_states_reached = false;
      }
    }
    bool res = false;
    std::string message;
    if (!all_states_reached) {
      auto res = forward_state(message);
      if(!res)
      {
        // response->success = false;
        // response->message = message;
        return res;
      }
    }
    // sleep a millesecond
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    current_time = get_node()->now();
  }
  return true;

}

void CiA402Controller::try_turn_on_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response
)
{

    bool res = false;
    std::string message;
    res = try_turn_on();
    if(res)
    {
      response->success = true;
      response->message = "All drives are in operation enabled state";
    }
    else
    {
      response->success = true;
      response->message = "failed on turn on";

    }


  // rclcpp::Time start_time = get_node()->now();
  // rclcpp::Duration timeout = rclcpp::Duration::from_seconds(3.0);
  // rclcpp::Time current_time = get_node()->now();
  // bool all_states_reached = false;

  // while (current_time - start_time < timeout && !all_states_reached) {
  //   all_states_reached = true;
  //   for (auto i = 0ul; i < dof_names_.size(); i++) {
  //     if (actual_state_[i] != ethercat_controller_msgs::msg::Cia402DriveStates::STATE_OPERATION_ENABLED) {
  //       all_states_reached = false;
  //     }
  //   }
  //   bool res = false;
  //   std::string message;
  //   if (!all_states_reached) {
  //     auto res = forward_state(message);
  //     if(!res)
  //     {
  //       response->success = false;
  //       response->message = message;
  //       return;
  //     }
  //   }
  //   // sleep a millesecond
  //   std::this_thread::sleep_for(std::chrono::milliseconds(1));
  //   current_time = get_node()->now();
  // }

  // std::string message;
  // response->success = true;
  // response->message = "All drives are in operation enabled state";
}

void CiA402Controller::try_turn_off_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response
)
{
  rclcpp::Time start_time = get_node()->now();
  rclcpp::Duration timeout = rclcpp::Duration::from_seconds(3.0);
  rclcpp::Time current_time = get_node()->now();
  bool all_states_reached = false;

  while (current_time - start_time < timeout && !all_states_reached) {
    all_states_reached = true;
    for (auto i = 0ul; i < dof_names_.size(); i++) {
      if (actual_state_[i] != ethercat_controller_msgs::msg::Cia402DriveStates::STATE_SWITCH_ON_DISABLED) {
        all_states_reached = false;
      }
    }
    bool res = false;
    std::string message;
    if (!all_states_reached) {
      auto res = backward_state(message);
      if(!res)
      {
        response->success = false;
        response->message = message;
        return;
      }
    }
    // sleep a millesecond
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    current_time = get_node()->now();
  }

  std::string message;
  response->success = true;
  response->message = "All drives are in operation disabled state";
}

void CiA402Controller::perform_homing_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response
)
{

  rt_homing_srv_ptr_.writeFromNonRT(request);
  RCLCPP_INFO(get_node()->get_logger(), "Received homing service request");

  response->success = true;
  response->message = "All drives are in operation disabled state";
}



void CiA402Controller::set_drive_states_callback(
  const std::shared_ptr<SetDriveStatesSrv::Request> request,
  std::shared_ptr<SetDriveStatesSrv::Response> response
)
{
  response->success = true;
  response->message = "Request transmitted to drive";
  // check if every dof is present
  for (const auto & dof_name : request->dof_names) {
    if (find(dof_names_.begin(), dof_names_.end(), dof_name) == dof_names_.end()) {
      response->success = false;
      response->message = "Abort state setting";
      return;
    }
  }

  rt_sds_srv_ptr_.writeFromNonRT(request);

  // // wait for the actual state equal to the requested state with a timeout
  rclcpp::Time start_time = get_node()->now();
  rclcpp::Duration timeout = rclcpp::Duration::from_seconds(3.0);
  rclcpp::Time current_time = get_node()->now();
  bool all_states_reached = true;

  while (current_time - start_time < timeout) {
    all_states_reached = true;

    for (auto i = 0ul; i < dof_names_.size(); i++) {
      if (actual_state_[i] != state_str_to_int(request->drive_states[i])) {
        all_states_reached = false;
        break;
      }
    }
    if (all_states_reached) {
      break;
    }
    // sleep a millesecond
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    current_time = get_node()->now();
  }
  response->success = all_states_reached;
}


}  // namespace ethercat_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ethercat_controllers::CiA402Controller, controller_interface::ControllerInterface)
