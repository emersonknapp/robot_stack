#include "dock_vis.hpp"

#include "gazebo_ros/conversions/geometry_msgs.hpp"
#include "gazebo/gazebo.hh"
#include "gazebo/rendering/ArrowVisual.hh"
#include "gazebo_ros/node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "kobuki_ros_interfaces/msg/dock_infra_red.hpp"

using kobuki_ros_interfaces::msg::DockInfraRed;

namespace robot_gazebo_plugins
{

using namespace gazebo;

template <typename T>
using RosPub = typename rclcpp::Publisher<T>::SharedPtr;

template <typename T>
using RosSub = typename rclcpp::Subscription<T>::SharedPtr;

using kobuki_ros_interfaces::msg::DockInfraRed;


class DockVisImpl {
public:
  DockVisImpl() {}
  virtual ~DockVisImpl() {}

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::unique_lock<std::mutex> lock(vislock_);
    auto gpose = gazebo_ros::Convert<ignition::math::Pose3d>(msg->pose);
    if (msg->header.frame_id == "ir_left") {
      left_pose_ = gpose;
    } else if (msg->header.frame_id == "ir_center") {
      center_pose_ = gpose;
    } else if (msg->header.frame_id == "ir_right") {
      right_pose_ = gpose;
    }
  }

  void update_arrow(rendering::VisualPtr arrow, uint8_t data)
  {
    using ignition::math::Color;
    Color color{1, 0, 1, 1};

    Color left{1, 0, 0, 1};
    Color center{1, 1, 0, 1};
    Color right{0, 1, 0, 1};

    if (data & DockInfraRed::NEAR_LEFT) {
      color = left / 2.0;
    } else if (data & DockInfraRed::FAR_LEFT) {
      color = left;
    } else if (data & DockInfraRed::NEAR_CENTER) {
      color = center / 2.0;
    } else if (data & DockInfraRed::FAR_CENTER) {
      color = center;
    } else if (data & DockInfraRed::NEAR_RIGHT) {
      color = right / 2.0;
    } else if (data & DockInfraRed::FAR_RIGHT) {
      color = right;
    } else {
      arrow->SetVisible(false);
      return;
    }
    color.A(1.0);

    arrow->SetVisible(true);
    arrow->SetDiffuse(color);
    arrow->SetAmbient(color);
  }

  void ir_callback(const DockInfraRed::SharedPtr msg)
  {
    std::unique_lock<std::mutex> lock(vislock_);
    data_[0] = msg->data[0];
    data_[1] = msg->data[1];
    data_[2] = msg->data[2];
  }

  rendering::VisualPtr init_arrow(std::string name, rendering::VisualPtr parent)
  {
    rendering::VisualPtr arrow(new rendering::ArrowVisual(name, parent));
    arrow->Load();
    arrow->SetDiffuse({1, 0, 1, 1});
    arrow->SetAmbient({1, 0, 1, 1});
    arrow->SetScale({5, 5, 5});
    parent->AttachVisual(arrow);
    arrow->SetVisibilityFlags(GZ_VISIBILITY_GUI);
    return arrow;
  }

  void Update()
  {
    left_arrow_->SetWorldPose(left_pose_);
    center_arrow_->SetWorldPose(center_pose_);
    right_arrow_->SetWorldPose(right_pose_);
    update_arrow(left_arrow_, data_[2]);
    update_arrow(center_arrow_, data_[1]);
    update_arrow(right_arrow_, data_[0]);
  }

  void Load(gazebo::rendering::VisualPtr parent, sdf::ElementPtr sdf)
  {
    std::unique_lock<std::mutex> lock(vislock_);
    parent_ = parent;
    ros_node_ = gazebo_ros::Node::Get(sdf);

    pose_sub_ = ros_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "ir_pose",
      rclcpp::SensorDataQoS(),
      std::bind(&DockVisImpl::pose_callback, this, std::placeholders::_1));

    ir_sub_ = ros_node_->create_subscription<DockInfraRed>(
      "dock_ir", rclcpp::SensorDataQoS(),
      std::bind(&DockVisImpl::ir_callback, this, std::placeholders::_1));

    left_arrow_ = init_arrow("ir_left_vis", parent);
    center_arrow_ = init_arrow("ir_center_vis", parent);
    right_arrow_ = init_arrow("ir_right_vis", parent);
    update_connection_ = gazebo::event::Events::ConnectPreRender(
      std::bind(&DockVisImpl::Update, this));
  }

  gazebo_ros::Node::SharedPtr ros_node_;
  rendering::VisualPtr parent_;
  rendering::VisualPtr left_arrow_;
  rendering::VisualPtr center_arrow_;
  rendering::VisualPtr right_arrow_;

  ignition::math::Pose3d left_pose_;
  ignition::math::Pose3d center_pose_;
  ignition::math::Pose3d right_pose_;
  uint8_t data_[3];
  std::mutex vislock_;
  gazebo::event::ConnectionPtr update_connection_;

  RosSub<geometry_msgs::msg::PoseStamped> pose_sub_;
  RosSub<DockInfraRed> ir_sub_;
};

DockVisPlugin::DockVisPlugin()
  : VisualPlugin()
  , impl_(std::make_unique<DockVisImpl>())
{
}

void DockVisPlugin::Load(rendering::VisualPtr parent, sdf::ElementPtr sdf)
{
  impl_->Load(parent, sdf);
}

GZ_REGISTER_VISUAL_PLUGIN(DockVisPlugin)
}  // namespace robot_gazebo_plugins
