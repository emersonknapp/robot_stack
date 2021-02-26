#include "robot_gazebo_plugins/dock_ir.hpp"

#include "rclcpp/rclcpp.hpp"
#include "gazebo_ros/node.hpp"
#include <gazebo_ros/utils.hpp>
#include "gazebo_ros/conversions/sensor_msgs.hpp"
#include "kobuki_ros_interfaces/msg/dock_infra_red.hpp"

namespace robot_gazebo_plugins
{

using namespace gazebo;

class KobukiDockImpl {
public:
  KobukiDockImpl();
  virtual ~KobukiDockImpl();

  void Load(physics::WorldPtr world, sdf::ElementPtr sdf);

  gazebo_ros::Node::SharedPtr ros_node_;
  physics::WorldPtr world_;
  std::string frame_name_;
  gazebo::transport::NodePtr gazebo_node_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::DockInfraRed>::SharedPtr ir_pub_;
};

void KobukiDockImpl::Load(physics::WorldPtr world, sdf::ElementPtr sdf)
{
  ros_node_ = gazebo_ros::Node::Get(sdf);

  ir_pub_ = ros_node_->create_publisher<kobuki_ros_interfaces::msg::DockInfraRed>(
    "dock_ir", rclcpp::SensorDataQoS());

  gazebo_node_ = boost::make_shared<gazebo::transport::Node>();
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
