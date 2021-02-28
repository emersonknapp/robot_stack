#include "robot_gazebo_plugins/dock_ir.hpp"

#include "kobuki_ros_interfaces/msg/dock_infra_red.hpp"

#include "gazebo_ros/node.hpp"
#include <gazebo_ros/utils.hpp>
#include "gazebo_ros/conversions/sensor_msgs.hpp"
#include "gazebo_ros/conversions/geometry_msgs.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "gazebo/rendering/Visual.hh"

namespace robot_gazebo_plugins
{

using namespace gazebo;
using kobuki_ros_interfaces::msg::DockInfraRed;

template <typename T>
using RosPub = typename rclcpp::Publisher<T>::SharedPtr;

class KobukiDockImpl {
public:
  KobukiDockImpl() {}
  virtual ~KobukiDockImpl() {}

  void Load(physics::WorldPtr world, sdf::ElementPtr sdf);
  void PublishDockStatus();
  void TimerCallback();

  std::string frame_name_;

  // ros resources
  gazebo_ros::Node::SharedPtr ros_node_;
  RosPub<DockInfraRed> ir_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // gazebo resources
  physics::WorldPtr world_;
  gazebo::transport::NodePtr gazebo_node_;
  physics::EntityPtr dock_link_;
  physics::EntityPtr ir_left_link_;
  physics::EntityPtr ir_center_link_;
  physics::EntityPtr ir_right_link_;
};

void KobukiDockImpl::Load(physics::WorldPtr world, sdf::ElementPtr sdf)
{
  world_ = world;
  ros_node_ = gazebo_ros::Node::Get(sdf);
  gazebo_node_ = boost::make_shared<gazebo::transport::Node>();

  ir_pub_ = ros_node_->create_publisher<kobuki_ros_interfaces::msg::DockInfraRed>(
    "dock_ir", rclcpp::SensorDataQoS());

  timer_ = ros_node_->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&KobukiDockImpl::TimerCallback, this));
}

uint8_t evaluatePose(const ignition::math::Pose3d & pose)
{
  const double near_dist = 0.5;
  const double far_dist = 2.0;
  const double c_width = 0.05;
  const double s_width = 0.5;
  const double s_offset = c_width / 2.0;
  const double s_max = s_width + s_offset;

  uint8_t out = 0;
  auto pos = pose.Pos();
  if (pos.Y() < 0) {
    return 0;
  }
  if (pos.X() > s_offset && pos.X() < s_max) {
    if (pos.Y() < near_dist) {
      out |= DockInfraRed::NEAR_RIGHT;
    }
    if (pos.Y() < far_dist) {
      out |= DockInfraRed::FAR_RIGHT;
    }
  }
  if (pos.X() < s_offset && pos.X() > -s_offset) {
    if (pos.Y() < near_dist) {
      out |= DockInfraRed::NEAR_CENTER;
    }
    if (pos.Y() < far_dist) {
      out |= DockInfraRed::FAR_CENTER;
    }
  }
  if (pos.X() < -s_offset && pos.X() > -s_max) {
    if (pos.Y() < near_dist) {
      out |= DockInfraRed::NEAR_LEFT;
    }
    if (pos.Y() < far_dist) {
      out |= DockInfraRed::FAR_LEFT;
    }
  }
  return out;
}

void KobukiDockImpl::PublishDockStatus()
{
  auto dock_pose = dock_link_->WorldPose();
  uint8_t left = 0;
  uint8_t center = 0;
  uint8_t right = 0;

  if (ir_left_link_) {
    left = evaluatePose(ir_left_link_->WorldPose() - dock_pose);
  }
  if (ir_center_link_) {
    center = evaluatePose(ir_center_link_->WorldPose() - dock_pose);
  }
  if (ir_right_link_) {
    right = evaluatePose(ir_right_link_->WorldPose() - dock_pose);
  }

  auto ir_msg = std::make_unique<kobuki_ros_interfaces::msg::DockInfraRed>();
  ir_msg->header.frame_id = "dock_ir_link";
  ir_msg->header.stamp = ros_node_->get_clock()->now();
  ir_msg->data.reserve(3);
  ir_msg->data.push_back(left);
  ir_msg->data.push_back(center);
  ir_msg->data.push_back(right);
  ir_pub_->publish(std::move(ir_msg));
}

void KobukiDockImpl::TimerCallback()
{
  auto msg = std::make_unique<std_msgs::msg::String>();

  if (!dock_link_) {
    dock_link_ = world_->EntityByName("dock_link");
  }
  if (!ir_left_link_) {
    ir_left_link_ = world_->EntityByName("ir_receiver_left_link");
  }
  if (!ir_center_link_) {
    ir_center_link_ = world_->EntityByName("ir_receiver_center_link");
  }
  if (!ir_right_link_) {
    ir_right_link_ = world_->EntityByName("ir_receiver_right_link");
  }

  if (dock_link_) {
    PublishDockStatus();
  }
}


KobukiDockPlugin::KobukiDockPlugin()
  : WorldPlugin()
  , impl_(std::make_unique<KobukiDockImpl>())
{
}

void KobukiDockPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf)
{
  impl_->Load(world, sdf);
}

GZ_REGISTER_WORLD_PLUGIN(KobukiDockPlugin)
}  // namespace robot_gazebo_plugins
