#ifndef LOCAL_PLANNER_H_
#define LOCAL_PLANNER_H_

// abstract class from which our plugin inherits
#include <ros/ros.h>
#include <nav_core/base_local_planner.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <tf/tfMessage.h>
#include <gazebo_msgs/ContactsState.h>

#include <boost/shared_ptr.hpp>
#include <base_local_planner/goal_functions.h>

#include <vector>
#include <deque>
#include <string>
#include <cmath>
#include <algorithm>
#include <exception>
#include <deque>

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

    void collisionCallback(const gazebo_msgs::ContactsState::ConstPtr& msg);

    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& costmap_msg);

        // Callback functions
    void poseCallback1(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void velocityCallback1(const geometry_msgs::TwistStamped::ConstPtr& msg);

    void poseCallback2(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void velocityCallback2(const geometry_msgs::TwistStamped::ConstPtr& msg);

    void poseCallback3(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void velocityCallback3(const geometry_msgs::TwistStamped::ConstPtr& msg);

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
    ros::Subscriber multiScanSub_;
    ros::Subscriber scanSub_;
    ros::Subscriber poseSub_;
    ros::Subscriber cmdSub_;
    ros::Subscriber collisionSub_;
    ros::Subscriber costmapSub_;
    // Subscribers for each box's pose and velocity
    ros::Subscriber pose_sub_1, velocity_sub_1;
    ros::Subscriber pose_sub_2, velocity_sub_2;
    ros::Subscriber pose_sub_3, velocity_sub_3;
    boost::shared_ptr<nav_msgs::Odometry const> odomPtr_;
    boost::shared_ptr<sensor_msgs::LaserScan const> scanPtr_;
    boost::shared_ptr<sensor_msgs::LaserScan const> scanMultiPtr_;
    boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> posePtr_;
    boost::shared_ptr<nav_msgs::OccupancyGrid const> costmapPtr_;
    double cmdPtr_;
    // Data structures to store the received data
    geometry_msgs::PoseStamped pos_box1;
    geometry_msgs::TwistStamped vel_box1;

    geometry_msgs::PoseStamped pos_box2;
    geometry_msgs::TwistStamped vel_box2;

    geometry_msgs::PoseStamped pos_box3;
    geometry_msgs::TwistStamped vel_box3;

    // Publishers
    ros::Publisher globalPlanPub_;
    ros::Publisher distToGoalPub_;
    ros::Publisher wRefPub_;
    ros::Publisher marker_pub_;
    ros::Publisher virtual_lidar_pub_;

    double lookAheadDist_ = 1.1;
    int collision_counter = 0;
    double dist_travelled = 0.0;
    double goalDistTolerance_;
    int currentGoalPoseIdx_;
    bool isGapExist_;
    double dmin;
    double posX_box1, posY_box1, velX_box1, velY_box1;
    double posX_box2, posY_box2, velX_box2, velY_box2;
    double posX_box3, posY_box3, velX_box3, velY_box3;
    double prevPosX_box1 = 0.0; 
    double prevPosY_box1 = 0.0;
    double prevPosX_box2 = 0.0;
    double prevPosY_box2 = 0.0;
    double prevPosX_box3 = 0.0;
    double prevPosY_box3 = 0.0;
    double lastCallbackTime_;
    double lastCallbackTime_end;
    bool newPoseData;
    double prevPoseTime1_=0.0;
    double prevPoseTime2_=0.0;
    double prevPoseTime3_=0.0;
    double boxPoseTime1_=0.0;
    double boxPoseTime2_=0.0;
    double boxPoseTime3_=0.0;
    double phi_gap;

};
};

#endif
