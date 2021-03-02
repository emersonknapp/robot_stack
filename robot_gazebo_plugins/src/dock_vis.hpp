#ifndef ROBOT_GAZEBO_PLUGINS__DOCK_VIS_HPP__
#define ROBOT_GAZEBO_PLUGINS__DOCK_VIS_HPP__

#include "gazebo/gazebo.hh"

namespace robot_gazebo_plugins
{

class DockVisImpl;

class DockVisPlugin : public gazebo::VisualPlugin
{
public:
  DockVisPlugin();
  virtual ~DockVisPlugin() = default;

protected:
  void Load(gazebo::rendering::VisualPtr parent, sdf::ElementPtr sdf) override;

private:
  std::unique_ptr<DockVisImpl> impl_;
};

}  // namespace robot_gazebo_plugins

#endif  // ROBOT_GAZEBO_PLUGINS__DOCK_VIS_HPP__
