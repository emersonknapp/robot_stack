#include "robot_gazebo_plugins/dock_ir.hpp"

#include "rclcpp/rclcpp.hpp"
#include "gazebo_ros/node.hpp"
#include <gazebo_ros/utils.hpp>
#include "gazebo_ros/conversions/sensor_msgs.hpp"
#include "kobuki_ros_interfaces/msg/dock_infra_red.hpp"
#include "std_msgs/msg/string.hpp"

namespace robot_gazebo_plugins
{

using namespace gazebo;

class KobukiDockImpl {
public:
  KobukiDockImpl() {}
  virtual ~KobukiDockImpl() {}

  void Load(physics::WorldPtr world, sdf::ElementPtr sdf);
  void DoThing();
  void pub_msg();

  std::string frame_name_;

  // ros resources
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::DockInfraRed>::SharedPtr ir_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr str_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // gazebo resources
  physics::WorldPtr world_;
  gazebo::transport::NodePtr gazebo_node_;
  physics::ModelPtr dock_model_;
  physics::ModelPtr robot_model_;
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

  timer_ = ros_node_->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&KobukiDockImpl::pub_msg, this));
}

void KobukiDockImpl::DoThing()
{
  // auto dock_pose = DockPose();
  // auto sensor_pose = SensorPose();
}

void KobukiDockImpl::pub_msg()
{
  if (!dock_model_) {
    dock_model_ = world_->ModelByName("kobuki_dock");
  }
  if (!robot_model_) {
    robot_model_ = world_->ModelByName("homey");
  }

  auto msg = std::make_unique<std_msgs::msg::String>();
  if (dock_model_ && robot_model_) {
    msg->data = "Have both!";
  } else {
    msg->data = "Haven't found robot and dock yet.";
  }
  str_pub_->publish(std::move(msg));
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
