// Copyright (c) 2022 PickNik, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <dh_gripper_driver/hardware_interface.h>

#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <rclcpp/rclcpp.hpp>

const auto kLogger = rclcpp::get_logger("DHGripperHardwareInterface");

constexpr int kGripperMinPos = 0;
constexpr int kGripperMaxPos = 1000;
constexpr int kGripperMinSpeed = 1;    
constexpr int kGripperMaxSpeed = 100;  
constexpr int kGripperMinforce = 20;
constexpr int kGripperMaxforce = 100;  
constexpr int kGripperRangePos = kGripperMaxPos - kGripperMinPos;
constexpr int kGripperRangeSpeed = kGripperMaxSpeed - kGripperMinSpeed;
constexpr int kGripperRangeForce = kGripperMaxforce - kGripperMinforce;

constexpr auto kGripperCommsLoopPeriod = std::chrono::milliseconds{ 10 };

namespace dh_gripper_driver
{
DHGripperHardwareInterface::DHGripperHardwareInterface()
{
  driver_factory_ = std::make_unique<DH_Gripper_Factory>();
}

DHGripperHardwareInterface::~DHGripperHardwareInterface()
{
  communication_thread_is_running_.store(false);
  if (communication_thread_.joinable())
  {
    communication_thread_.join();
  }

  on_cleanup(rclcpp_lifecycle::State());
}

hardware_interface::CallbackReturn DHGripperHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  RCLCPP_DEBUG(kLogger, "on_init");

  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Read parameters.
  gripper_closed_pos_ = stod(info_.hardware_parameters["gripper_closed_position"]);

  gripper_position_ = std::numeric_limits<double>::quiet_NaN();
  gripper_velocity_ = std::numeric_limits<double>::quiet_NaN();
  gripper_position_command_ = std::numeric_limits<double>::quiet_NaN();
  reactivate_gripper_cmd_ = NO_NEW_CMD_;
  reactivate_gripper_async_cmd_.store(false);

  const hardware_interface::ComponentInfo& joint = info_.joints[0];

  // There is one command interface: position.
  if (joint.command_interfaces.size() != 1)
  {
    RCLCPP_FATAL(kLogger, "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                 joint.command_interfaces.size());
    return CallbackReturn::ERROR;
  }

  if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_FATAL(kLogger, "Joint '%s' has %s command interfaces found. '%s' expected.", joint.name.c_str(),
                 joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
    return CallbackReturn::ERROR;
  }

  // There are two state interfaces: position and velocity.
  if (joint.state_interfaces.size() != 2)
  {
    RCLCPP_FATAL(kLogger, "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                 joint.state_interfaces.size());
    return CallbackReturn::ERROR;
  }

  for (int i = 0; i < 2; ++i)
  {
    if (!(joint.state_interfaces[i].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[i].name == hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_FATAL(kLogger, "Joint '%s' has %s state interface. Expected %s or %s.", joint.name.c_str(),
                   joint.state_interfaces[i].name.c_str(), hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  std::string gripper_id = info_.hardware_parameters["gripper_id"];
  std::string gripper_model = info_.hardware_parameters["gripper_model"];
  std::string gripper_connect_port = info_.hardware_parameters["gripper_connect_port"];
  std::string gripper_baudrate = info_.hardware_parameters["gripper_baudrate"];

  try
  {
    driver_factory_->Set_Parameter(atoi(gripper_id.c_str()), gripper_connect_port, atoi(gripper_baudrate.c_str()));
    driver_ = std::unique_ptr<DH_Gripper>(driver_factory_->CreateGripper(gripper_model));
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(kLogger, "Failed to create a driver: %s", e.what());
    return CallbackReturn::ERROR;
  }

  if (!driver_)
  {
    RCLCPP_FATAL(kLogger, "Invalid gripper model specified: %s", gripper_model.c_str());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DHGripperHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(kLogger, "on_configure");
  try
  {
    if (hardware_interface::SystemInterface::on_configure(previous_state) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // Open the serial port and handshake.
    int connected = driver_->open();
    if (connected < 0)
    {
      RCLCPP_ERROR(kLogger, "Unable to open connect port to %s", info_.hardware_parameters["gripper_connect_port"].c_str());
      return CallbackReturn::ERROR;
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(kLogger, "Cannot configure the DH gripper: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DHGripperHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(kLogger, "on_cleanup");
  try
  {
    driver_->close();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(kLogger, "Cannot close the DH gripper: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DHGripperHardwareInterface::export_state_interfaces()
{
  RCLCPP_DEBUG(kLogger, "export_state_interfaces");

  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &gripper_position_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &gripper_velocity_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DHGripperHardwareInterface::export_command_interfaces()
{
  RCLCPP_DEBUG(kLogger, "export_command_interfaces");

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_POSITION, &gripper_position_command_));

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(info_.joints[0].name, "set_gripper_max_velocity", &gripper_speed_));
  gripper_speed_ = info_.hardware_parameters.count("gripper_speed_multiplier") ?
                       info_.hardware_parameters.count("gripper_speed_multiplier") :
                       1.0;

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(info_.joints[0].name, "set_gripper_max_effort", &gripper_force_));
  gripper_force_ = info_.hardware_parameters.count("gripper_force_multiplier") ?
                       info_.hardware_parameters.count("gripper_force_multiplier") :
                       1.0;

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface("reactivate_gripper", "reactivate_gripper_cmd", &reactivate_gripper_cmd_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "reactivate_gripper", "reactivate_gripper_response", &reactivate_gripper_response_));

  return command_interfaces;
}

hardware_interface::CallbackReturn
DHGripperHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_DEBUG(kLogger, "on_activate");

  // set some default values for joints
  if (std::isnan(gripper_position_))
  {
    gripper_position_ = 0;
    gripper_velocity_ = 0;
    gripper_position_command_ = 0;
  }

  // Activate the gripper.
  try
  {
    bool success = init_gripper();
    if (!success)
    {
      RCLCPP_ERROR(kLogger, "Failed to initialize the DH gripper.");
      return CallbackReturn::ERROR;
    }

    communication_thread_is_running_.store(true);
    communication_thread_ = std::thread([this] { this->background_task(); });
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(kLogger, "Failed to communicate with the DH gripper: %s", e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(kLogger, "DH Gripper successfully activated!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DHGripperHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_DEBUG(kLogger, "on_deactivate");

  communication_thread_is_running_.store(false);
  communication_thread_.join();
  if (communication_thread_.joinable())
  {
    communication_thread_.join();
  }

  // try
  // {
  //   driver_->close();
  // }
  // catch (const std::exception& e)
  // {
  //   RCLCPP_ERROR(kLogger, "Failed to deactivate the DH gripper: %s", e.what());
  //   return CallbackReturn::ERROR;
  // }
  RCLCPP_INFO(kLogger, "DH Gripper successfully deactivated!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DHGripperHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                                      const rclcpp::Duration& period)
{
  auto old_gripper_position = gripper_position_;
  gripper_position_ = gripper_closed_pos_ * (1 - (static_cast<double>(gripper_current_state_.load()) - static_cast<double>(kGripperMinPos)) / static_cast<double>(kGripperRangePos));
  gripper_velocity_ = (gripper_position_ - old_gripper_position) / (period.seconds() + 1e-9);

  if (!std::isnan(reactivate_gripper_cmd_))
  {
    RCLCPP_INFO(kLogger, "Sending gripper reactivation request.");
    reactivate_gripper_async_cmd_.store(true);
    reactivate_gripper_cmd_ = NO_NEW_CMD_;
  }

  if (reactivate_gripper_async_response_.load().has_value())
  {
    reactivate_gripper_response_ = reactivate_gripper_async_response_.load().value();
    reactivate_gripper_async_response_.store(std::nullopt);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DHGripperHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                                       const rclcpp::Duration& /*period*/)
{
  int gripper_pos = (1-(gripper_position_command_ / gripper_closed_pos_)) * kGripperRangePos + kGripperMinPos;
  write_command_.store(gripper_pos);
  int gripper_speed = (gripper_speed_ * kGripperRangeSpeed) + kGripperMinSpeed;
  write_speed_.store(gripper_speed);
  int gripper_force = (gripper_force_ * kGripperRangeForce) + kGripperMinforce;
  write_force_.store(gripper_force);

  return hardware_interface::return_type::OK;
}

void DHGripperHardwareInterface::background_task()
{
  while (communication_thread_is_running_.load())
  {
    try
    {
      // Re-activate the gripper
      // (this can be used, for example, to re-run the auto-calibration).
      if (reactivate_gripper_async_cmd_.load())
      {
        bool success = init_gripper();
        if (!success)
        {
          throw std::runtime_error("Failed to re-activate the DH gripper.");
        }
        reactivate_gripper_async_cmd_.store(false);
        reactivate_gripper_async_response_.store(true);
      }

      // Write the latest command to the gripper.
      this->driver_->SetTargetPosition(write_command_.load());
      this->driver_->SetTargetSpeed(write_speed_.load());
      this->driver_->SetTargetForce(write_force_.load());

      // Read the state of the gripper.
      int tmp_pos = 0;
      this->driver_->GetCurrentPosition(tmp_pos);
      gripper_current_state_.store(tmp_pos);
    }
    catch (std::exception& e)
    {
      RCLCPP_ERROR(kLogger, "Error: %s", e.what());
    }

    std::this_thread::sleep_for(kGripperCommsLoopPeriod);
  }
}

bool DHGripperHardwareInterface::init_gripper()
{
  int initstate = 0;
  driver_->GetInitState(initstate);
  const std::chrono::seconds timeoutDuration(5); // Set your desired timeout duration

  if (initstate != DH_Gripper::S_INIT_FINISHED) {
      driver_->Initialization();
      auto startTime = std::chrono::steady_clock::now();
      do {
          driver_->GetInitState(initstate);
          std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Adjust sleep duration as needed
      } while (initstate != DH_Gripper::S_INIT_FINISHED &&
                std::chrono::steady_clock::now() - startTime < timeoutDuration);
  }

  if (initstate != DH_Gripper::S_INIT_FINISHED) {
      RCLCPP_FATAL(kLogger, "Initialization of the Robotiq gripper timed out.");
      return false;
  }

  return true;
}

}  // namespace dh_gripper_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dh_gripper_driver::DHGripperHardwareInterface, hardware_interface::SystemInterface)