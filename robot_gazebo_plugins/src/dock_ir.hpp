#ifndef ROBOT_GAZEBO_PLUGINS__DOCK_IR__HPP_
#define ROBOT_GAZEBO_PLUGINS__DOCK_IR__HPP_

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace robot_gazebo_plugins
{

class KobukiDockImpl;


class KobukiDockPlugin : public gazebo::WorldPlugin
{
public:
  KobukiDockPlugin();
  virtual ~KobukiDockPlugin() = default;

protected:
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override;

private:
  std::unique_ptr<KobukiDockImpl> impl_;
};

}  // namespace robot_gazebo_plugins

#endif  // ROBOT_GAZEBO_PLUGINS__DOCK_IR__HPP_
