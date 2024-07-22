#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class MEKFPoseToTF
{
public:
    MEKFPoseToTF() : loop_rate(50), alpha(0.1) // Set alpha for EMA filter
    {
        mekf_pose_sub_ = nh_.subscribe("/MEKF_pose", 10, &MEKFPoseToTF::mekfPoseCallback, this);
        tf_listener_ = new tf::TransformListener();
        initialized = false;
    }

    void spin()
    {
        while (ros::ok())
        {
            ros::spinOnce(); // Process callbacks
            loop_rate.sleep(); // Sleep to control the loop frequency
        }
    }

private:
    void mekfPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        ROS_INFO("Received MEKF pose: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

        // Get the transformation from map to base_link (from MEKF_pose)
        tf::Transform map_to_base;
        map_to_base.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
        tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        map_to_base.setRotation(q);

        // Get the transformation from odom to base_link (from TF listener)
        tf::StampedTransform odom_to_base;
        try
        {
            tf_listener_->lookupTransform("odom", "base_link", ros::Time(0), odom_to_base);
            ROS_INFO("Received odom to base_link transform");
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }

        // Compute the transformation from map to odom
        tf::Transform map_to_odom = map_to_base * odom_to_base.inverse();

        // Apply exponential moving average (EMA) filter
        if (initialized)
        {
            smooth_map_to_odom.setOrigin(smooth_map_to_odom.getOrigin() * (1.0 - alpha) + map_to_odom.getOrigin() * alpha);
            smooth_map_to_odom.setRotation(smooth_map_to_odom.getRotation().slerp(map_to_odom.getRotation(), alpha));
        }
        else
        {
            smooth_map_to_odom = map_to_odom;
            initialized = true;
        }

        // Publish the smoothed transformation from map to odom
        tf_broadcaster_.sendTransform(tf::StampedTransform(smooth_map_to_odom, ros::Time::now(), "map", "odom"));
        ROS_INFO("Published smoothed map to odom transform");
    }

    ros::NodeHandle nh_;
    ros::Subscriber mekf_pose_sub_;
    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener* tf_listener_;
    ros::Rate loop_rate;
    tf::Transform smooth_map_to_odom;
    bool initialized;
    double alpha; // Smoothing factor for EMA
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mekf_pose_to_tf");
    MEKFPoseToTF mekfPoseToTF;
    mekfPoseToTF.spin();
    return 0;
}
