#ifndef LOCAL_PLANNER_H_
#define LOCAL_PLANNER_H_

// abstract class from which our plugin inherits
#include <ros/ros.h>
#include <nav_core/base_local_planner.h>


#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <tf/tfMessage.h>

#include <boost/shared_ptr.hpp>
#include <base_local_planner/goal_functions.h>

#include <vector>
#include <deque>
#include <string>
#include <cmath>
#include <algorithm>
#include <exception>

using namespace std;

namespace local_planner{

class LocalPlanner : public nav_core::BaseLocalPlanner{
public:

    LocalPlanner();
    LocalPlanner(std::string name, tf2_ros::Buffer* tf,
                 costmap_2d::Costmap2DROS* costmapROS_);

    

    void initialize(std::string name, tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmapROS_);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    bool isGoalReached();

    void odomCallback(boost::shared_ptr<nav_msgs::Odometry const> msg);

    void scanCallback(boost::shared_ptr<sensor_msgs::LaserScan const> msg);
        
    void poseCallback(boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> msg);
    
    void cmdCallback(const std_msgs::Float64::ConstPtr& msg);

    double distanceToGlobalGoal();

    double LLCallback();

    // double headingController(double phi);

    // double PControl(double phi);

    void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

    // void publishDistToGoal(const ros::Publisher& pub, double dist);

    // void publishWRef(const ros::Publisher& pub, double wRef);

    ~LocalPlanner();

private:
    bool initialized_;
    costmap_2d::Costmap2D* costmap_;
    costmap_2d::Costmap2DROS* costmapROS_;
    tf2_ros::Buffer* tf_;
    ros::NodeHandle nh_;

    bool goalReached_;
    std::vector<geometry_msgs::PoseStamped> globalPlan_;
    geometry_msgs::Pose currentPose_;
    geometry_msgs::Pose currentGoalPose_;
    tf::Stamped<tf::tfMessage> currentPoseTF1_;
    geometry_msgs::PoseStamped currentPoseTF2_;
    
    // Sensor subscriptions
    ros::Subscriber odomSub_;
    ros::Subscriber scanSub_;
    ros::Subscriber poseSub_;
    ros::Subscriber cmdSub_;
    boost::shared_ptr<nav_msgs::Odometry const> odomPtr_;
    boost::shared_ptr<sensor_msgs::LaserScan const> scanPtr_;
    boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> posePtr_;
    double cmdPtr_;

    // Publishers
    ros::Publisher globalPlanPub_;
    ros::Publisher distToGoalPub_;
    ros::Publisher wRefPub_;

    double lookAheadDist_ = 1.1;
    double goalDistTolerance_;
    int currentGoalPoseIdx_;
    bool isGapExist_;
    double dmin;

    double lastCallbackTime_;

};
};

#endif
