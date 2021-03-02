#include "dock_ir.hpp"

#include "gazebo/rendering/Visual.hh"
#include "gazebo_ros/conversions/geometry_msgs.hpp"
#include "gazebo_ros/node.hpp"
#include "gazebo_ros/utils.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "kobuki_ros_interfaces/msg/dock_infra_red.hpp"
#include "std_msgs/msg/string.hpp"

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
  RosPub<geometry_msgs::msg::PoseStamped> pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // gazebo resources
  physics::WorldPtr world_;
  physics::EntityPtr dock_link_;
  physics::EntityPtr ir_left_link_;
  physics::EntityPtr ir_center_link_;
  physics::EntityPtr ir_right_link_;
};

void KobukiDockImpl::Load(physics::WorldPtr world, sdf::ElementPtr sdf)
{
  world_ = world;
  ros_node_ = gazebo_ros::Node::Get(sdf);

  ir_pub_ = ros_node_->create_publisher<kobuki_ros_interfaces::msg::DockInfraRed>(
    "dock_ir", rclcpp::SensorDataQoS());
  pose_pub_ = ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "ir_pose", rclcpp::SensorDataQoS());

  timer_ = ros_node_->create_wall_timer(
    std::chrono::milliseconds(33),
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
  if (pos.X() < 0) {
    return 0;
  }
  if (pos.Y() > s_offset && pos.Y() < s_max) {
    if (pos.X() < near_dist) {
      out |= DockInfraRed::NEAR_LEFT;
    }
    if (pos.Y() < far_dist) {
      out |= DockInfraRed::FAR_LEFT;
    }
  }
  if (pos.Y() < s_offset && pos.Y() > -s_offset) {
    if (pos.X() < near_dist) {
      out |= DockInfraRed::NEAR_CENTER;
    }
    if (pos.X() < far_dist) {
      out |= DockInfraRed::FAR_CENTER;
    }
  }
  if (pos.Y() < -s_offset && pos.Y() > -s_max) {
    if (pos.X() < near_dist) {
      out |= DockInfraRed::NEAR_RIGHT;
    }
    if (pos.X() < far_dist) {
      out |= DockInfraRed::FAR_RIGHT;
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
    auto rel = ir_left_link_->WorldPose() - dock_pose;
    left = evaluatePose(rel);
    auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
    msg->pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(ir_left_link_->WorldPose());
    msg->header.frame_id = "ir_left";
    pose_pub_->publish(std::move(msg));
  }
  if (ir_center_link_) {
    center = evaluatePose(ir_center_link_->WorldPose() - dock_pose);
    auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
    msg->pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(ir_center_link_->WorldPose());
    msg->header.frame_id = "ir_center";
    pose_pub_->publish(std::move(msg));
  }
  if (ir_right_link_) {
    right = evaluatePose(ir_right_link_->WorldPose() - dock_pose);
    auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
    msg->pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(ir_right_link_->WorldPose());
    msg->header.frame_id = "ir_right";
    pose_pub_->publish(std::move(msg));
  }

  auto ir_msg = std::make_unique<kobuki_ros_interfaces::msg::DockInfraRed>();
  ir_msg->header.frame_id = "dock_ir_link";
  ir_msg->header.stamp = ros_node_->get_clock()->now();
  ir_msg->data.reserve(3);
  ir_msg->data.push_back(right);
  ir_msg->data.push_back(center);
  ir_msg->data.push_back(left);
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
