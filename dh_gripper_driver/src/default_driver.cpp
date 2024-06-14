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

#include <serial/serial.h>

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "dh_gripper_driver/data_utils.hpp"
#include "dh_gripper_driver/default_driver.hpp"
#include <dh_gripper_driver/crc_utils.hpp>
#include <dh_gripper_driver/driver_exception.hpp>

#include <rclcpp/rclcpp.hpp>

namespace dh_gripper_driver
{
const auto kLogger = rclcpp::get_logger("DefaultDriver");

constexpr uint8_t kReadFunctionCode = 0x03;
constexpr uint16_t kFirstOutputRegister = 0x0200;
constexpr int kReadResponseSize = 7;

constexpr uint8_t kWriteFunctionCode = 0x06;
constexpr uint16_t kActionRequestRegister = 0x0100;
constexpr int kWriteResponseSize = 8;

// If the gripper connection is not stable we may want to try sending the command again.
constexpr auto kMaxRetries = 5;

DefaultDriver::DefaultDriver(std::unique_ptr<Serial> serial)
  : serial_{ std::move(serial) }, commanded_gripper_speed_(0x80), commanded_gripper_force_(0x80)
{
}

std::vector<uint8_t> DefaultDriver::send(const std::vector<uint8_t>& request, size_t response_size) const
{
  std::vector<uint8_t> response;
  response.reserve(response_size);

  int retry_count = 0;
  while (retry_count < kMaxRetries)
  {
    try
    {
      serial_->write(request);
      response = serial_->read(response_size);
      break;
    }
    catch (const serial::IOException& e)
    {
      RCLCPP_WARN(kLogger, "Resending the command because the previous attempt (%d of %d) failed: %s", retry_count + 1,
                  kMaxRetries, e.what());
      retry_count++;
    }
  }

  if (retry_count == kMaxRetries)
  {
    RCLCPP_ERROR(kLogger, "Reached maximum retries. Operation failed.");
    return {};
  }

  return response;
}

bool DefaultDriver::connect()
{
  serial_->open();
  return serial_->is_open();
}

void DefaultDriver::disconnect()
{
  serial_->close();
}

void DefaultDriver::set_slave_address(uint8_t slave_address)
{
  slave_address_ = slave_address;
}

void DefaultDriver::activate()
{
  RCLCPP_INFO(kLogger, "Activate...");

  // set rACT to 1, clear all other registers.
  const auto request = create_write_command(kActionRequestRegister, { 0x0001});
  auto response = send(request, kWriteResponseSize);
  if (response.empty())
  {
    throw DriverException{ "Failed to activate the gripper." };
  }

  update_status();
  if (activation_status_ == ActivationStatus::ACTIVE)
  {
    return;
  }
  while (activation_status_ != ActivationStatus::ACTIVE)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    update_status();
  }
}

void DefaultDriver::deactivate()
{
  // RCLCPP_INFO(kLogger, "Deactivate...");

  // const auto request = create_write_command(kActionRequestRegister, { 0x0000, 0x0000, 0x0000, 0x0000 });
  // auto response = send(request, kWriteResponseSize);
  // if (response.empty())
  // {
  //   throw DriverException{ "Failed to deactivate the gripper." };
  // }
}

void DefaultDriver::set_gripper_position(uint16_t pos)
{
  const auto request1 =
      create_write_command(kActionRequestRegister+1, {uint16_t(100)}); // force

  auto response1 = send(request1, kWriteResponseSize);
  if (response1.empty())
  {
    throw DriverException{ "Failed to set gripper force." };
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  const auto request2 =
      create_write_command(kActionRequestRegister+3, {pos});

  auto response2 = send(request2, kWriteResponseSize);
  if (response2.empty())
  {
    throw DriverException{ "Failed to set gripper position." };
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(5));


  
}

uint16_t DefaultDriver::get_gripper_position()
{
  update_status();
  // std::cout  << "grip pos driver = " << std::dec << unsigned(gripper_position_) << std::endl;
  return gripper_position_;
}

bool DefaultDriver::gripper_is_moving()
{
  update_status();
  return object_detection_status_ == ObjectDetectionStatus::MOVING;
}

void DefaultDriver::set_speed(uint8_t speed)
{
  commanded_gripper_speed_ = speed;
}

void DefaultDriver::set_force(uint8_t force)
{
  commanded_gripper_force_ = force;
}

std::vector<uint8_t> DefaultDriver::create_read_command(uint16_t address, uint8_t num_registers)
{


  std::vector<uint8_t> request = { slave_address_,
                                   kReadFunctionCode,
                                   data_utils::get_msb(address),
                                   data_utils::get_lsb(address),
                                   0x00,
                                   0x01};
  auto crc = crc_utils::compute_crc(request);
  request.push_back(data_utils::get_msb(crc));
  request.push_back(data_utils::get_lsb(crc));

  return request;
}

std::vector<uint8_t> DefaultDriver::create_write_command(uint16_t address, const std::vector<uint16_t>& data)
{
  uint16_t num_registers = data.size();
  uint8_t num_bytes = 2 * num_registers;

  std::vector<uint8_t> request = { slave_address_,
                                   kWriteFunctionCode,
                                   data_utils::get_msb(address),
                                   data_utils::get_lsb(address),
                                   data_utils::get_msb(data[0]),
                                   data_utils::get_lsb(data[0])};

  auto crc = crc_utils::compute_crc(request);
  request.push_back(data_utils::get_msb(crc));
  request.push_back(data_utils::get_lsb(crc));


  return request;
}

void DefaultDriver::update_status()
{

  const auto request1 =
      create_read_command(kFirstOutputRegister, 0); // position

  auto response1 = send(request1, kReadResponseSize);
  if (response1.empty())
  {
    throw DriverException{ "Failed to read gripper position." };
  }
  if ( response1[4] == 1)
  {
    activation_status_ = ActivationStatus::ACTIVE;
  }
  else
  {
    activation_status_ = ActivationStatus::RESET;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  const auto request2 =
      create_read_command(kFirstOutputRegister+2, 0); // position

  auto response2 = send(request2, kReadResponseSize);
  if (response2.empty())
  {
    throw DriverException{ "Failed to read gripper position." };
  }


  gripper_position_ = (response2[3] << 8) | response2[4];

  // log gripper position
  // RCLCPP_INFO(kLogger, "Response2: %s", data_utils::to_hex(response2).c_str());

  std::this_thread::sleep_for(std::chrono::milliseconds(5));


}
}  // namespace dh_gripper_driver
