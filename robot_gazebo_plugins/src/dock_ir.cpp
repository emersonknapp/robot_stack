#include "robot_gazebo_plugins/dock_ir.hpp"

#include "rclcpp/rclcpp.hpp"
#include "gazebo_ros/node.hpp"
#include <gazebo_ros/utils.hpp>
#include "gazebo_ros/conversions/sensor_msgs.hpp"
#include "kobuki_ros_interfaces/msg/dock_infra_red.hpp"

namespace robot_gazebo_plugins
{

using namespace gazebo;

class DockInfraredPluginImpl {
public:
  DockInfraredPluginImpl();
  virtual ~DockInfraredPluginImpl();

  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf);

  gazebo_ros::Node::SharedPtr ros_node_;
  std::string frame_name_;
  gazebo::transport::NodePtr gazebo_node_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::DockInfraRed>::SharedPtr ir_pub_;
};

void DockInfraredPluginImpl::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  ros_node_ = gazebo_ros::Node::Get(sdf);
  frame_name_ = gazebo_ros::SensorFrameID(*sensor, *sdf);

  ir_pub_ = ros_node_->create_publisher<kobuki_ros_interfaces::msg::DockInfraRed>(
    "dock_ir", rclcpp::SensorDataQoS());

  gazebo_node_ = boost::make_shared<gazebo::transport::Node>();
  gazebo_node_->Init(sensor->WorldName());
}


DockInfraredPlugin::DockInfraredPlugin()
  : SensorPlugin()
  , impl_(std::make_unique<DockInfraredPluginImpl>())
{
}

void DockInfraredPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  impl_->Load(sensor, sdf);
}

GZ_REGISTER_SENSOR_PLUGIN(DockInfraredPlugin)
}  // namespace robot_gazebo_plugins
