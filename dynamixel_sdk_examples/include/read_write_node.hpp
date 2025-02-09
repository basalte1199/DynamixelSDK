// Copyright 2021 ROBOTIS CO., LTD.
//
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

#ifndef READ_WRITE_NODE_HPP_
#define READ_WRITE_NODE_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_position_six_motor.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_position_five_motor.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_position_four_motor.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_position_two_motor.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"


class ReadWriteNode : public rclcpp::Node
{
public:
  using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
  using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;
  using SetPositionSixMotor = dynamixel_sdk_custom_interfaces::msg::SetPositionSixMotor;
  using SetPositionFiveMotor = dynamixel_sdk_custom_interfaces::msg::SetPositionFiveMotor;
  using SetPositionFourMotor = dynamixel_sdk_custom_interfaces::msg::SetPositionFourMotor;
  using SetPositionTwoMotor = dynamixel_sdk_custom_interfaces::msg::SetPositionTwoMotor;

  ReadWriteNode();
  virtual ~ReadWriteNode();

private:
  void publishData();
  void publishCurrentData();
  rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
  rclcpp::Service<GetPosition>::SharedPtr get_position_server_;
  rclcpp::Publisher<SetPosition>::SharedPtr publisher_;
  rclcpp::Publisher<SetPositionFiveMotor>::SharedPtr publisher_five_motor_;
  rclcpp::Publisher<SetPositionFourMotor>::SharedPtr publisher_four_motor_;
  rclcpp::Publisher<SetPositionFiveMotor>::SharedPtr publisher_five_motor_present_position_;
  rclcpp::Publisher<SetPositionSixMotor>::SharedPtr publisher_six_motor_present_current_;
  rclcpp::Publisher<SetPositionSixMotor>::SharedPtr publisher_six_motor_present_position_;
  rclcpp::Publisher<SetPositionFiveMotor>::SharedPtr publisher_five_motor_present_current_;
  rclcpp::Publisher<SetPositionSixMotor>::SharedPtr publisher_six_motor_;
  rclcpp::Subscription<SetPositionFourMotor>::SharedPtr set_position_four_motor_subscriber_;
  rclcpp::Subscription<SetPositionSixMotor>::SharedPtr set_position_six_motor_subscriber_;
  rclcpp::Publisher<SetPositionTwoMotor>::SharedPtr publisher_two_motor_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_position_;
  int present_position;
  int present_position_1;
  int present_position_2;
  int present_position_3;
  int present_position_4;
  int present_position_5;
  int present_position_6;
};

#endif  // READ_WRITE_NODE_HPP_
