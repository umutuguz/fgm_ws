#ifndef MOVE_BOX_PLUGIN_H
#define MOVE_BOX_PLUGIN_H

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

namespace gazebo
{
  class MoveBoxPlugin : public ModelPlugin
  {
  public:
    MoveBoxPlugin() : ModelPlugin() {}

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
    void OnUpdate(const common::UpdateInfo & _info);

  private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    ignition::math::Vector3d velocity;
    // Publishers for box's pose and velocity
    ros::Publisher pose_pub, velocity_pub;
  };
}

#endif
