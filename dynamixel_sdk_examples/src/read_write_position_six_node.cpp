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

/*******************************************************************************
// This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
// For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
// To test this example, please follow the commands below.
//
// Open terminal #1
// $ ros2 run dynamixel_sdk_examples read_write_node
//
// Open terminal #2 (run one of below commands at a time)
// $ ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/SetPosition "{id: 1, position: 1000}"
// $ ros2 service call /get_position dynamixel_sdk_custom_interfaces/srv/GetPosition "id: 1"
//
// Author: Will Son
*******************************************************************************/

/******************************************************************************/
/* include                                                                    */
/******************************************************************************/
#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_position_four_motor.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_position_six_motor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "read_write_node.hpp"


/******************************************************************************/
/* define                                                                     */
/******************************************************************************/
/* Control table address for X series                                         */
#define ADDR_OPERATING_MODE       11
#define ADDR_TORQUE_ENABLE				64
#define ADDR_GOAL_CURRENT		  		102	 // Does NOT exist in Rot motors
#define ADDR_GOAL_VELOCITY				104
#define ADDR_GOAL_POSITION				116
#define ADDR_PRESENT_CURRENT			126	 // Represents "Present Load" in Rot motors
#define ADDR_PRESENT_VELOCITY			128
#define ADDR_PRESENT_POSITION			132

/* Motors ID */
#define DXL1_ID                         1
#define DXL2_ID                         2
#define DXL3_ID                         3
#define DXL4_ID                         4

#define DXL11_ID 11
#define DXL12_ID 12
#define DXL13_ID 13
#define DXL14_ID 14
#define DXL15_ID 15
#define DXL16_ID 16


/* TORQUE ENABLE/DISABLE */
#define TORQUE_ENABLE                   1	 // Value for enabling the torque
#define TORQUE_DISABLE                  0	 // Value for disabling the torque

/* OPERATING_MODE */
#define CURRENT_BASED_POSITION_CONTROL 5
#define POSITION_CONTROL				       3
#define VELOCITY_CONTROL				       1
#define TORQUE_CONTROL					       0


#define ADDR_DRIVE_MODE           10
#define TIME_BASED_PROFILE        4

#define PROFILE_VELOCITY          112

/* Protocol version */ 
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

/* Default setting */
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB1"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;

/******************************************************************************/
/* Constructor                                                                */
/******************************************************************************/


ReadWriteNode::ReadWriteNode()
: Node("read_write_six_node")
{
  RCLCPP_INFO(this->get_logger(), "Run read write node");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();
    
  set_position_six_motor_subscriber_ =
    this->create_subscription<SetPositionSixMotor>(
    "six_motor_presentposition",
    QOS_RKL10V,
    [this](const SetPositionSixMotor::SharedPtr msg) -> void
    {
      uint8_t dxl_error = 0;

      // Position Value of X series is 4 byte data.
      // For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
      uint32_t goal_position_1 = (unsigned int)msg->position_1;  // Convert int32 -> uint32

      // Write Goal Position (length : 4 bytes)
      // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) msg->id_1,
        ADDR_GOAL_POSITION,
        goal_position_1,
        &dxl_error
      );

      uint32_t goal_position_2 = (unsigned int)msg->position_2;

      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) msg->id_2,
        ADDR_GOAL_POSITION,
        goal_position_2,
        &dxl_error
      );

      uint32_t goal_position_3 = (unsigned int)msg->position_3;

      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) msg->id_3,
        ADDR_GOAL_POSITION,
        goal_position_3,
        &dxl_error
      );


      uint32_t goal_position_4 = (unsigned int)msg->position_4;

      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) msg->id_4,
        ADDR_GOAL_POSITION,
        goal_position_4,
        &dxl_error
      );


      uint32_t goal_position_5 = (unsigned int)msg->position_5;

      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) msg->id_5,
        ADDR_GOAL_POSITION,
        goal_position_5,
        &dxl_error
      );


      uint32_t goal_position_6 = (unsigned int)msg->position_6;

      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) msg->id_6,
        ADDR_GOAL_POSITION,
        goal_position_6,
        &dxl_error
      );
    }
    );

  
}

ReadWriteNode::~ReadWriteNode()
{
}

/******************************************************************************/
/* Function                                                                   */
/******************************************************************************/



void setupDynamixel(uint8_t dxl_id)
{
  /* Use Position Control Mode                */
  /* #define CURRENT_BASED_POSITION_CONTROL 5 */
  /* #define POSITION_CONTROL				        3 */
  /* #define VELOCITY_CONTROL				        1 */
  /* #define TORQUE_CONTROL					        0 */
  uint32_t arm_profile = 500;
  uint32_t gripper_profile = 300;
  
  dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_OPERATING_MODE,
        CURRENT_BASED_POSITION_CONTROL,  
        &dxl_error
  );
  
  dxl_comm_result = packetHandler->write4ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_GOAL_CURRENT,
        400,
        &dxl_error
  );
  
  
  
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to enable drive mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to enable drive mode.");
  }
  

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    TORQUE_ENABLE,  /* Torque ON */
    &dxl_error
  );
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to enable torque.");
  }

    //unsigned char* id[] = { &DXL11_ID, &DXL12_ID, &DXL13_ID ,&DXL14_ID, &DXL15_ID };
    uint32_t target_current = 200;

    uint32_t torque_enable = TORQUE_ENABLE;
    /*
    for (int i = 11; i <= 15; i++){
      
        dxl_comm_result = packetHandler->read4ByteTxRx(
            portHandler,
            i,
            ADDR_TORQUE_ENABLE,
            &torque_enable,
            &dxl_error
        );
        
        auto logger = rclcpp::get_logger("logger_name");
        RCLCPP_INFO(logger, "Torque enable: %d", i);
        if (i == 15){
            dxl_comm_result = packetHandler->write1ByteTxRx(
                portHandler,
                i,
                ADDR_OPERATING_MODE,
                CURRENT_BASED_POSITION_CONTROL,  
                &dxl_error
            );
            packetHandler->write2ByteTxRx(
              portHandler,
              i,
              ADDR_GOAL_CURRENT,
              target_current,
              &dxl_error
            );
            RCLCPP_INFO(logger, "Torque enable: %d", i);
            dxl_comm_result = packetHandler->write1ByteTxRx(
            portHandler,
            i,
            ADDR_DRIVE_MODE,
            TIME_BASED_PROFILE,  
            &dxl_error
            );
            dxl_comm_result = packetHandler->write1ByteTxRx(
            portHandler,
            i,
            PROFILE_VELOCITY,
            gripper_profile,  
            &dxl_error
            );
        }
        dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        i,
        ADDR_OPERATING_MODE,
        POSITION_CONTROL,  
        &dxl_error
        );
        RCLCPP_INFO(logger, "Torque enable: %d", i);
        dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        i,
        ADDR_DRIVE_MODE,
        TIME_BASED_PROFILE,  
        &dxl_error
        );
        dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        i,
        PROFILE_VELOCITY,
        arm_profile,  
        &dxl_error
        );
      

    }
  */


  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set Position Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set Position Control Mode.");
  }

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to enable torque.");
  }

}

int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set the baudrate.");
  }
  
  setupDynamixel(BROADCAST_ID);
  
  rclcpp::init(argc, argv);

  auto readwritenode = std::make_shared<ReadWriteNode>();
  rclcpp::spin(readwritenode);

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    TORQUE_DISABLE,
    &dxl_error
  );
  portHandler->closePort();
	
	rclcpp::shutdown();

  return 0;
}