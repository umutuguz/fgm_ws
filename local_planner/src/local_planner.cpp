#include <pluginlib/class_list_macros.h>
#include "local_planner/local_planner.h"


PLUGINLIB_EXPORT_CLASS(local_planner::LocalPlanner, nav_core::BaseLocalPlanner)

namespace local_planner
{
    LocalPlanner::LocalPlanner() : costmapROS_(NULL), tf_(NULL), initialized_(false) {}

    LocalPlanner::LocalPlanner(std::string name, tf2_ros::Buffer* tf,
                            costmap_2d::Costmap2DROS* costmapROS)
        : costmapROS_(NULL), tf_(NULL), initialized_(false), goalDistTolerance_(1)
        
    {
        ROS_INFO("The structure of the local planner was constructed.");
        initialize(name, tf, costmapROS);
    }

    LocalPlanner::~LocalPlanner() 
    
    {
        ROS_INFO("Local planner out.");
    }

    // Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
    void LocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf,
                                costmap_2d::Costmap2DROS* costmapROS)
    {
        if(!initialized_)
        {
            ros::NodeHandle nh("~/local_planner");

            nh_ = nh;
            tf_ = tf;
            costmapROS_ = costmapROS;
            costmapROS_->getRobotPose(currentPoseTF2_);
            costmap_ = costmapROS_->getCostmap();

            // Subscribers
            odomSub_ = nh_.subscribe("/odom", 100, &LocalPlanner::odomCallback, this);
            
            // scanSub_ = nh_.subscribe("/front_rp/rp_scan_filtered_front", 100, &LocalPlanner::odomCallback, this);
            scanSub_ = nh_.subscribe("/scan", 100, &LocalPlanner::scanCallback, this);
            
            poseSub_ = nh_.subscribe("/amcl_pose", 100, &LocalPlanner::poseCallback, this);

            nh_.getParam("/move_base/local_planner/look_ahead_dist", lookAheadDist_);

            // Publishers
            globalPlanPub_ = nh_.advertise<nav_msgs::Path>("global_plan", 1);
            
            distToGoalPub_ = nh_.advertise<std_msgs::Float32>("distance_to_goal", 1);
            
            wRefPub_ = nh_.advertise<std_msgs::Float32>("angular_vel_output", 1);

            ROS_INFO("Local planner has been initialized successfully.");
            initialized_ = true;
        }
        else
        {
            ROS_WARN("Local planner has been initialized successfully.");
        }
    }

    bool LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
    {
        if(!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }
        globalPlan_.clear();
        globalPlan_ = orig_global_plan;
        goalReached_ = false;
        ROS_INFO("Got new plan.");
        return true;
    }

    bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        if(!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }
        
        ROS_INFO_ONCE("Computing velocity commands...");

        // Publish global plan for visualization
        publishGlobalPlan(globalPlan_);

        // Find the local goal in the global plan considering look ahead distance
        for (unsigned int i = 0; i < globalPlan_.size(); i++)
        {
            currentPose_.position.x = posePtr_->pose.pose.position.x;
            currentPose_.position.y = posePtr_->pose.pose.position.y;
            currentPose_.position.z = posePtr_->pose.pose.position.z;

            double waypointX = globalPlan_[i].pose.position.x;
            double waypointY = globalPlan_[i].pose.position.y;

            double diffX = waypointX - currentPose_.position.x;
            double diffY = waypointY - currentPose_.position.y;

            if (hypot(diffX, diffY) > lookAheadDist_ && distanceToGlobalGoal() > goalDistTolerance_)
            {
                currentGoalPoseIdx_ = i;
                break;
            }
            else if (distanceToGlobalGoal() < goalDistTolerance_)
            {
                currentGoalPoseIdx_ = globalPlan_.size() - 1;
                break;
            }
            else
            {
                continue;
            }
        }

        ROS_INFO_STREAM("Current goal index: " << currentGoalPoseIdx_);
        
        currentGoalPose_ = globalPlan_.at(currentGoalPoseIdx_).pose;
        // ROS_WARN_STREAM("Goal: " << currentGoalPose_.position.x);

        double phiFinal = LLCallback(); // LL Algorithm
        // headingController_.setSampleTime(ros::Time::now().toSec() - lastCallbackTime_); /* headingController_ ? */
        // ROS_WARN_STREAM("Sample time: " << headingController_.getSampleTime());
        // double omega = headingController_.derivativeFilteredControlSignal(phiFinal);  /* headingController_ ? */

        // Print and publish the distance to global goal
        double distToGlobGoal = distanceToGlobalGoal();
        ROS_INFO_STREAM("Distance to global goal: " << distToGlobGoal);
        publishDistToGoal(distToGoalPub_, distToGlobGoal);

        double omega;
        double linearVel;

        // Publish reference omega for Fuzzy planner
        publishWRef(wRefPub_, omega);

        // Linear velocity value calculated by the Fuzzy velocity planner
        // double linearVel = scaledLinVelPtr_->data;  //scaledLinVelPtr_ ?

        // Send velocity commands to robot's base
        cmd_vel.linear.x = linearVel;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;

        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = omega;

        if (distanceToGlobalGoal() < goalDistTolerance_) 
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;

            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = 0.0;

            goalReached_ = true;
        }

        if (!isGapExist_)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;

            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = 0.0;
            if (!isGoalReached())
            {
                // Use the last refence cmd_vel command
                cmd_vel.linear.x = linearVel;
                cmd_vel.linear.y = 0.0;
                cmd_vel.linear.z = 0.0;

                cmd_vel.angular.x = 0.0;
                cmd_vel.angular.y = 0.0;
                cmd_vel.angular.z = omega;
            }
            else
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.linear.z = 0.0;
    
                cmd_vel.angular.x = 0.0;
                cmd_vel.angular.y = 0.0;
                cmd_vel.angular.z = 0.0;                
            }
        }

        return true;
    }

    bool LocalPlanner::isGoalReached()
    {
        if(!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }
        
        if (!costmapROS_->getRobotPose(currentPoseTF2_))
        {
            ROS_ERROR("Could not get robot pose");
            return false;
        }

        if (goalReached_)
        {
            ROS_INFO("Goal reached!");
            return true;
        }
        
        return false;
    }

    void LocalPlanner::odomCallback(boost::shared_ptr<nav_msgs::Odometry const> msg)
    {
        odomPtr_ = msg;
    } // end function odomCallback


    void LocalPlanner::scanCallback(boost::shared_ptr<sensor_msgs::LaserScan const> msg)
    {
        scanPtr_ = msg;
    } // end function laserScanCallback

    
    void LocalPlanner::poseCallback(boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> msg)
    {
        posePtr_ = msg;
    } // end function poseCallback

    double LocalPlanner::distanceToGlobalGoal()
    {
        currentPose_.position.x = posePtr_->pose.pose.position.x;
        currentPose_.position.y = posePtr_->pose.pose.position.y;
        currentPose_.position.z = posePtr_->pose.pose.position.z;

        geometry_msgs::PoseStamped globalGoal = globalPlan_.back(); 
        double xDist = globalGoal.pose.position.x - currentPose_.position.x;
        double yDist = globalGoal.pose.position.y - currentPose_.position.y;

        return hypot(xDist, yDist);
    }


    double LocalPlanner::LLCallback()
    {
        ROS_INFO_STREAM("Local Planner started");
        lastCallbackTime_ = ros::Time::now().toSec();

        // Get odometry informations
        // WARNING: These are not odometry information! Variable names remained
        // unchanged since the latest update. These are AMCL positions.
        double odomRX = posePtr_->pose.pose.position.x;
        double odomRY = posePtr_->pose.pose.position.y;
        currentPose_.orientation = posePtr_->pose.pose.orientation;
        double odomRYaw = tf::getYaw(currentPose_.orientation);

        double goalX = currentGoalPose_.position.x;
        double goalY = currentGoalPose_.position.y;

        // Calculate robot's goal angle
        double phiGoalConstraint = atan2(goalY - odomRY, goalX - odomRX);

        // Calculate goal phi
        double phiGoal;
        if (phiGoalConstraint - odomRYaw > M_PI)
            phiGoal = (phiGoalConstraint - odomRYaw) - 2 * M_PI;
        else if (phiGoalConstraint - odomRYaw < -M_PI)
            phiGoal = (phiGoalConstraint - odomRYaw) + 2 * M_PI;
        else
            phiGoal = phiGoalConstraint - odomRYaw;

        return phiGoal;

    }

        // end for

    void LocalPlanner::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path)
    {
        base_local_planner::publishPlan(path, globalPlanPub_);
    } // end function publishGlobalPlan


    void publishDistToGoal(const ros::Publisher& pub, double dist)
    {
        std_msgs::Float32 msg;
        msg.data = dist;

        pub.publish(msg);
    } // end function publishDistToGoal

    void publishWRef(const ros::Publisher& pub, double wRef)
    {
        std_msgs::Float32 msg;
        msg.data = wRef;

        pub.publish(msg);
    } // end function publishWRef
}
