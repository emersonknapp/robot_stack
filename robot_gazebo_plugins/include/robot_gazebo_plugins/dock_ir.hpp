#ifndef ROBOT_GAZEBO_PLUGINS__DOCK_IR__HPP_
#define ROBOT_GAZEBO_PLUGINS__DOCK_IR__HPP_

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace robot_gazebo_plugins
{

class DockInfraredPluginImpl;


class DockInfraredPlugin : public gazebo::SensorPlugin
{
public:
  DockInfraredPlugin();
  virtual ~DockInfraredPlugin() = default;

  virtual void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf);

private:
  std::unique_ptr<DockInfraredPluginImpl> impl_;
};

}  // namespace robot_gazebo_plugins

#endif  // ROBOT_GAZEBO_PLUGINS__DOCK_IR__HPP_
