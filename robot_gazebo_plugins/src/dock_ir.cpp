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

template <typename T>
using RosPub = typename rclcpp::Publisher<T>::SharedPtr;

class KobukiDockImpl {
public:
  KobukiDockImpl() {}
  virtual ~KobukiDockImpl() {}

  void Load(physics::WorldPtr world, sdf::ElementPtr sdf);
  void DoThing()
  {
    auto ir_pose = ir_center_link_->WorldPose();
    auto dock_pose = dock_link_->WorldPose();

    auto diff = ir_pose - dock_pose;

    geometry_msgs::msg::Pose pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(diff);
    auto msg = std::make_unique<geometry_msgs::msg::Pose>(pose);

    pose_pub_->publish(std::move(msg));
  }

  void pub_msg();

  std::string frame_name_;

  // ros resources
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::DockInfraRed>::SharedPtr ir_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr str_pub_;
  RosPub<geometry_msgs::msg::Pose> pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // gazebo resources
  physics::WorldPtr world_;
  gazebo::transport::NodePtr gazebo_node_;
  physics::ModelPtr dock_model_;
  physics::ModelPtr robot_model_;
  physics::EntityPtr dock_link_;
  physics::EntityPtr ir_center_link_;
};

void KobukiDockImpl::Load(physics::WorldPtr world, sdf::ElementPtr sdf)
{
  world_ = world;
  ros_node_ = gazebo_ros::Node::Get(sdf);
  gazebo_node_ = boost::make_shared<gazebo::transport::Node>();

  ir_pub_ = ros_node_->create_publisher<kobuki_ros_interfaces::msg::DockInfraRed>(
    "dock_ir", rclcpp::SensorDataQoS());
  str_pub_ = ros_node_->create_publisher<std_msgs::msg::String>(
    "kobuki_str", rclcpp::SensorDataQoS());
  pose_pub_ = ros_node_->create_publisher<geometry_msgs::msg::Pose>(
    "ir_center_pose", rclcpp::SensorDataQoS());

  timer_ = ros_node_->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&KobukiDockImpl::pub_msg, this));
}




void KobukiDockImpl::pub_msg()
{
  auto msg = std::make_unique<std_msgs::msg::String>();

  if (!ir_center_link_) {
    ir_center_link_ = world_->EntityByName("ir_receiver_center_link");
  }
  if (!dock_link_) {
    dock_link_ = world_->EntityByName("dock_link");
  }

  if (ir_center_link_ && dock_link_) {
    msg->data = "Yes the ir_center is there";
    str_pub_->publish(std::move(msg));
    DoThing();
  } else {
    msg->data = "Haven't found all desired entities yet.";
    str_pub_->publish(std::move(msg));
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
