// Copyright (c) 2019 Emerson Knapp
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

#include "parking_rviz_plugins/spot_tool.hpp"

#include <memory>
#include <string>

#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"

namespace parking_rviz_plugins
{

ParkingSpotTool::ParkingSpotTool()
: rviz_default_plugins::tools::PoseTool()
{
  shortcut_key_ = 'p';
}

ParkingSpotTool::~ParkingSpotTool() = default;

void ParkingSpotTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Parking Spot");
  setIcon(rviz_common::loadPixmap("package://rviz_default_plugins/icons/classes/SetGoal.png"));
  add_client_ = context_->getRosNodeAbstraction().lock()->get_raw_node()->
    template create_client<robot_interfaces::srv::AddParkingSpot>("/parking/add_parking_spot");
}

void
ParkingSpotTool::onPoseSet(double x, double y, double theta)
{
  auto request = std::make_shared<robot_interfaces::srv::AddParkingSpot::Request>();
  request->pose.x = x;
  request->pose.y = y;
  request->pose.theta = theta;
  printf("Sending %f %f %f\n", x, y, theta);
  using ServiceResponseFuture =
    rclcpp::Client<robot_interfaces::srv::AddParkingSpot>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      auto result = future.get();
      printf("Result of adding: %d\n", result->success);
    };
  auto future_result = add_client_->async_send_request(request, response_received_callback);
}

}  // namespace parking_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(parking_rviz_plugins::ParkingSpotTool, rviz_common::Tool)
