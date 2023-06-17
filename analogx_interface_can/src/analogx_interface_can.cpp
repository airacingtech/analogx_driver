// Copyright (c) 2015-2018, Dataspeed Inc., 2018-2020 New Eagle, All rights reserved.
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

#include <cmath>
#include <algorithm>
#include <string>

#include "analogx_interface_can/analogx_interface_can.hpp"

using std::chrono::duration;

namespace analogx_interface_can
{

static constexpr uint64_t MS_IN_SEC = 1000;

AnalogxInterfaceCAN::AnalogxInterfaceCAN(const rclcpp::NodeOptions & options)
: Node("analogx_interface_can_node", options)
{

  dbc_file_ = declare_parameter<std::string>("dbc_file", "");

  pub_can_ = this->create_publisher<Frame>(
    "can_rx", 20
  );
  pubAnalogx1_ = this->create_publisher<Analogx1>("analogx1", rclcpp::SensorDataQoS());
  pubBrakeTemp_ = this->create_publisher<BrakeTemp>("brake_temp", rclcpp::SensorDataQoS());
  pubFrontLeftExternalTireTemp_ = this->create_publisher<FrontLeftExternalTireTemp>(
    "front_left_external_tire_temp", rclcpp::SensorDataQoS());
  pubFrontRightExternalTireTemp_ = this->create_publisher<FrontRightExternalTireTemp>(
    "front_right_external_tire_temp", rclcpp::SensorDataQoS());
  pubRearLeftExternalTireTemp_ = this->create_publisher<RearLeftExternalTireTemp>(
    "rear_left_external_tire_temp", rclcpp::SensorDataQoS());
  pubRearRightExternalTireTemp_ = this->create_publisher<RearRightExternalTireTemp>(
    "rear_right_external_tire_temp", rclcpp::SensorDataQoS());
  pubBrakeSensorBodyTemp_ = this->create_publisher<BrakeSensorBodyTemp>(
    "brake_sensor_body_temp",
    rclcpp::SensorDataQoS());


  sub_can_ = this->create_subscription<Frame>(
    "can_tx", 500,
    std::bind(&AnalogxInterfaceCAN::recvCAN, this, std::placeholders::_1)
  );

  dbc_ = NewEagle::DbcBuilder().NewDbc(dbc_file_);
}

#define RECV_DBC(handler) \
  message = dbc_.GetMessageById(id); \
  if (msg->dlc >= message->GetDlc()) {message->SetFrame(msg); handler(msg, message);}

void AnalogxInterfaceCAN::recvCAN(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = nullptr;
  if (!msg->is_rtr && !msg->is_error) {
    auto id = msg->id;
    switch (id) {
      case ID_ANALOGX1:
        RECV_DBC(recvAnalogx1);
        break;
      case ID_BRAKE_TEMP:
        RECV_DBC(recvBrakeTemp);
        break;
      case ID_FRONT_LEFT_EXTERNAL_TIRE_TEMP:
        RECV_DBC(recvFrontLeftExternalTireTemp);
        break;
      case ID_FRONT_RIGHT_EXTERNAL_TIRE_TEMP:
        RECV_DBC(recvFrontRightExternalTireTemp);
        break;
      case ID_REAR_LEFT_EXTERNAL_TIRE_TEMP:
        RECV_DBC(recvRearLeftExternalTireTemp);
        break;
      case ID_REAR_RIGHT_EXTERNAL_TIRE_TEMP:
        RECV_DBC(recvRearRightExternalTireTemp);
        break;
      case ID_BRAKE_SENSOR_BODY_TEMP:
        RECV_DBC(recvBrakeSensorBodyTemp);
        break;
      default:
        break;
    }
  }
}

void AnalogxInterfaceCAN::recvAnalogx1(const Frame::SharedPtr msg, DbcMessage * message)
{
  Analogx1 out;
  out.stamp = msg->header.stamp;

  out.brake_pressure = message->GetSignal("Brake_Pressure")->GetResult();
  out.brake_pedal_position = message->GetSignal("Brake_Pedal_Position")->GetResult();
  out.throttle_position = message->GetSignal("Throttle_Position")->GetResult();

  pubAnalogx1_->publish(out);
}

void AnalogxInterfaceCAN::recvBrakeTemp(const Frame::SharedPtr msg, DbcMessage * message)
{
  BrakeTemp out;
  out.stamp = msg->header.stamp;

  out.temp = message->GetSignal("Temp")->GetResult();

  pubBrakeTemp_->publish(out);
}

void AnalogxInterfaceCAN::recvFrontLeftExternalTireTemp(
  const Frame::SharedPtr msg,
  DbcMessage * message)
{
  FrontLeftExternalTireTemp out;
  out.stamp = msg->header.stamp;

  out.inside = message->GetSignal("Inside")->GetResult();
  out.inside_middle = message->GetSignal("Inside_Middle")->GetResult();
  out.outside = message->GetSignal("Outside")->GetResult();
  out.outside_middle = message->GetSignal("Outside_Middle")->GetResult();

  pubFrontLeftExternalTireTemp_->publish(out);
}

void AnalogxInterfaceCAN::recvFrontRightExternalTireTemp(
  const Frame::SharedPtr msg,
  DbcMessage * message)
{
  FrontRightExternalTireTemp out;
  out.stamp = msg->header.stamp;

  out.inside = message->GetSignal("Inside")->GetResult();
  out.inside_middle = message->GetSignal("Inside_Middle")->GetResult();
  out.outside = message->GetSignal("Outside")->GetResult();
  out.outside_middle = message->GetSignal("Outside_Middle")->GetResult();

  pubFrontRightExternalTireTemp_->publish(out);
}

void AnalogxInterfaceCAN::recvRearLeftExternalTireTemp(
  const Frame::SharedPtr msg,
  DbcMessage * message)
{
  RearLeftExternalTireTemp out;
  out.stamp = msg->header.stamp;

  out.inside = message->GetSignal("Inside")->GetResult();
  out.inside_middle = message->GetSignal("Inside_Middle")->GetResult();
  out.outside = message->GetSignal("Outside")->GetResult();
  out.outside_middle = message->GetSignal("Outside_Middle")->GetResult();

  pubRearLeftExternalTireTemp_->publish(out);
}

void AnalogxInterfaceCAN::recvRearRightExternalTireTemp(
  const Frame::SharedPtr msg,
  DbcMessage * message)
{
  RearRightExternalTireTemp out;
  out.stamp = msg->header.stamp;

  out.inside = message->GetSignal("Inside")->GetResult();
  out.inside_middle = message->GetSignal("Inside_Middle")->GetResult();
  out.outside = message->GetSignal("Outside")->GetResult();
  out.outside_middle = message->GetSignal("Outside_Middle")->GetResult();

  pubRearRightExternalTireTemp_->publish(out);
}

void AnalogxInterfaceCAN::recvBrakeSensorBodyTemp(const Frame::SharedPtr msg, DbcMessage * message)
{
  BrakeSensorBodyTemp out;
  out.stamp = msg->header.stamp;

  out.temperature = message->GetSignal("Temperature")->GetResult();

  pubBrakeSensorBodyTemp_->publish(out);
}


}  // namespace analogx_interface_can
