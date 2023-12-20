#include "MoveBoxPlugin.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(MoveBoxPlugin)

void MoveBoxPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;

  // Read the velocity from the SDF
  if (_sdf->HasElement("velocity"))
    this->velocity = _sdf->Get<ignition::math::Vector3d>("velocity");
  else
    this->velocity = ignition::math::Vector3d(1, 0, 0); // Default velocity if not specified

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&MoveBoxPlugin::OnUpdate, this, std::placeholders::_1));

  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "move_box_plugin",
        ros::init_options::NoSigintHandler);
  }

  ros::NodeHandle nh("move_box_plugin");
  std::string model_name = _model->GetName();
  this->pose_pub = nh.advertise<geometry_msgs::PoseStamped>(model_name + "_pose", 10);
  this->velocity_pub = nh.advertise<geometry_msgs::TwistStamped>(model_name + "_velocity", 10);
}

void MoveBoxPlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  // Apply velocity to the box
  this->model->SetLinearVel(this->velocity);

  // Get pose and velocity
  ignition::math::Pose3d pose = this->model->WorldPose();
  ignition::math::Vector3d velocity = this->model->WorldLinearVel();

  // Create and publish PoseStamped message
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = "world";
  pose_msg.pose.position.x = pose.Pos().X();
  pose_msg.pose.position.y = pose.Pos().Y();
  pose_msg.pose.position.z = pose.Pos().Z();
  pose_msg.pose.orientation.x = pose.Rot().X();
  pose_msg.pose.orientation.y = pose.Rot().Y();
  pose_msg.pose.orientation.z = pose.Rot().Z();
  pose_msg.pose.orientation.w = pose.Rot().W();
  pose_pub.publish(pose_msg);

  // Create and publish TwistStamped message
  geometry_msgs::TwistStamped velocity_msg;
  velocity_msg.header.stamp = pose_msg.header.stamp;
  velocity_msg.header.frame_id = "world";
  velocity_msg.twist.linear.x = velocity.X();
  velocity_msg.twist.linear.y = velocity.Y();
  velocity_msg.twist.linear.z = velocity.Z();
  velocity_pub.publish(velocity_msg);
}
