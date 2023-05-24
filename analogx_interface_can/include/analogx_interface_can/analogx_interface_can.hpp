// Copyright (c) 2020 New Eagle, All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// * Neither the name of the {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
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

/** \brief This file defines the AnalogxInterfaceCAN class.
 * \copyright Copyright 2021 New Eagle LLC
 * \file analogx_interface_can.hpp
 */

#ifndef ANALOGX_INTERFACE_CAN__ANALOGX_INTERFACE_CAN_HPP_
#define ANALOGX_INTERFACE_CAN__ANALOGX_INTERFACE_CAN_HPP_

#include <cmath>
#include <array>
#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

// ROS messages
#include "can_msgs/msg/frame.hpp"
#include "analogx_interface_msgs/msg/analogx1.hpp"
#include "analogx_interface_msgs/msg/analogx2.hpp"
#include "analogx_interface_msgs/msg/front_right_wheel_encoder.hpp"
#include "analogx_interface_msgs/msg/front_left_wheel_encoder.hpp"
#include "analogx_interface_msgs/msg/rear_axle_wheel_encoder.hpp"
#include "analogx_interface_msgs/msg/brake_temp.hpp"
#include "analogx_interface_msgs/msg/front_left_external_tire_temp.hpp"
#include "analogx_interface_msgs/msg/front_right_external_tire_temp.hpp"
#include "analogx_interface_msgs/msg/rear_left_external_tire_temp.hpp"
#include "analogx_interface_msgs/msg/rear_right_external_tire_temp.hpp"
#include "analogx_interface_msgs/msg/brake_sensor_body_temp.hpp"

#include "can_dbc_parser/DbcMessage.hpp"
#include "can_dbc_parser/DbcSignal.hpp"
#include "can_dbc_parser/Dbc.hpp"
#include "can_dbc_parser/DbcBuilder.hpp"

#include "analogx_interface_can/dispatch.hpp"

using can_msgs::msg::Frame;
using NewEagle::DbcMessage;

using analogx_interface_msgs::msg::Analogx1;
using analogx_interface_msgs::msg::Analogx2;
using analogx_interface_msgs::msg::FrontRightWheelEncoder;
using analogx_interface_msgs::msg::FrontLeftWheelEncoder;
using analogx_interface_msgs::msg::RearAxleWheelEncoder;
using analogx_interface_msgs::msg::BrakeTemp;
using analogx_interface_msgs::msg::FrontLeftExternalTireTemp;
using analogx_interface_msgs::msg::FrontRightExternalTireTemp;
using analogx_interface_msgs::msg::RearLeftExternalTireTemp;
using analogx_interface_msgs::msg::RearRightExternalTireTemp;
using analogx_interface_msgs::msg::BrakeSensorBodyTemp;

namespace analogx_interface_can
{
class AnalogxInterfaceCAN : public rclcpp::Node
{
public:
/** \brief Default constructor.
 * \param[in] options The options for this node.
 */
  explicit AnalogxInterfaceCAN(const rclcpp::NodeOptions & options);

private:
/** \brief Convert reports received over CAN into ROS messages.
 * \param[in] msg The message received over CAN.
 */
  void recvCAN(const Frame::SharedPtr msg);

  void recvAnalogx1(const Frame::SharedPtr msg, DbcMessage * message);
  void recvAnalogx2(const Frame::SharedPtr msg, DbcMessage * message);
  void recvFrontRightWheelEncoder(const Frame::SharedPtr msg, DbcMessage * message);
  void recvFrontLeftWheelEncoder(const Frame::SharedPtr msg, DbcMessage * message);
  void recvRearAxleWheelEncoder(const Frame::SharedPtr msg, DbcMessage * message);
  void recvBrakeTemp(const Frame::SharedPtr msg, DbcMessage * message);
  void recvFrontLeftExternalTireTemp(const Frame::SharedPtr msg, DbcMessage * message);
  void recvFrontRightExternalTireTemp(const Frame::SharedPtr msg, DbcMessage * message);
  void recvRearLeftExternalTireTemp(const Frame::SharedPtr msg, DbcMessage * message);
  void recvRearRightExternalTireTemp(const Frame::SharedPtr msg, DbcMessage * message);
  void recvBrakeSensorBodyTemp(const Frame::SharedPtr msg, DbcMessage * message);


  std::uint8_t vehicle_number_;

  // Parameters from launch
  std::string dbc_file_;
  float max_steer_angle_;
  bool publish_my_laps_;

  rclcpp::Subscription<Analogx1>::SharedPtr subAnalogx1_;
  rclcpp::Subscription<Analogx2>::SharedPtr subAnalogx2_;
  rclcpp::Subscription<FrontRightWheelEncoder>::SharedPtr subFrontRightWheelEncoder_;
  rclcpp::Subscription<FrontLeftWheelEncoder>::SharedPtr subFrontLeftWheelEncoder_;
  rclcpp::Subscription<RearAxleWheelEncoder>::SharedPtr subRearAxleWheelEncoder_;
  rclcpp::Subscription<BrakeTemp>::SharedPtr subBrakeTemp_;
  rclcpp::Subscription<FrontLeftExternalTireTemp>::SharedPtr subFrontLeftExternalTireTemp_;
  rclcpp::Subscription<FrontRightExternalTireTemp>::SharedPtr subFrontRightExternalTireTemp_;
  rclcpp::Subscription<RearLeftExternalTireTemp>::SharedPtr subRearLeftExternalTireTemp_;
  rclcpp::Subscription<RearRightExternalTireTemp>::SharedPtr subRearRightExternalTireTemp_;
  rclcpp::Subscription<BrakeSensorBodyTemp>::SharedPtr subBrakeSensorBodyTemp_;
  rclcpp::Subscription<Frame>::SharedPtr sub_can_;

  rclcpp::Publisher<Analogx1>::SharedPtr pubAnalogx1_;
  rclcpp::Publisher<Analogx2>::SharedPtr pubAnalogx2_;
  rclcpp::Publisher<FrontRightWheelEncoder>::SharedPtr pubFrontRightWheelEncoder_;
  rclcpp::Publisher<FrontLeftWheelEncoder>::SharedPtr pubFrontLeftWheelEncoder_;
  rclcpp::Publisher<RearAxleWheelEncoder>::SharedPtr pubRearAxleWheelEncoder_;
  rclcpp::Publisher<BrakeTemp>::SharedPtr pubBrakeTemp_;
  rclcpp::Publisher<FrontLeftExternalTireTemp>::SharedPtr pubFrontLeftExternalTireTemp_;
  rclcpp::Publisher<FrontRightExternalTireTemp>::SharedPtr pubFrontRightExternalTireTemp_;
  rclcpp::Publisher<RearLeftExternalTireTemp>::SharedPtr pubRearLeftExternalTireTemp_;
  rclcpp::Publisher<RearRightExternalTireTemp>::SharedPtr pubRearRightExternalTireTemp_;
  rclcpp::Publisher<BrakeSensorBodyTemp>::SharedPtr pubBrakeSensorBodyTemp_;
  rclcpp::Publisher<Frame>::SharedPtr pub_can_;

  NewEagle::Dbc dbc_;
};

}  // namespace analogx_interface_can

#endif  // ANALOGX_INTERFACE_CAN__ANALOGX_INTERFACE_CAN_HPP_
