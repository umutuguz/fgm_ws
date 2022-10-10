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
            
            // distToGoalPub_ = nh_.advertise<std_msgs::Float32>("distance_to_goal", 1);
            
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
        ROS_INFO("global plan size is: %lu", globalPlan_.size());
        for (int j=0; j<globalPlan_.size();j++)
        {
            //ROS_INFO("global plan is: %f and %f",globalPlan_[j].pose.position.x, globalPlan_[j].pose.position.y);
            //ROS_INFO("j is: %u",j);
        }


        // Find the local goal in the global plan considering look ahead distance
        for (unsigned int i = 0; i < globalPlan_.size(); i++)
        {
            currentPose_.position.x = posePtr_->pose.pose.position.x;
            currentPose_.position.y = posePtr_->pose.pose.position.y;
            currentPose_.position.z = posePtr_->pose.pose.position.z;
            ROS_INFO("currentpose_x is: %f",currentPose_.position.x);
            ROS_INFO("currentpose_y is: %f",currentPose_.position.y);

            double waypointX = globalPlan_[i].pose.position.x;
            double waypointY = globalPlan_[i].pose.position.y;

            double diffX = waypointX - currentPose_.position.x; 
            double diffY = waypointY - currentPose_.position.y;
            ROS_INFO("global plan waypoint index: %u", i);
            ROS_INFO("hypot is: %f", hypot(diffX,diffY));
            ROS_INFO("distance to global goal is: %f",distanceToGlobalGoal());
            ROS_INFO("goaldisttolerance is: %f", goalDistTolerance_);
            ROS_INFO("lookaheaddist is: %f", lookAheadDist_);

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
        ROS_INFO_STREAM("Current goal pose: "<< currentGoalPose_);


        double phiFinal = LLCallback(); // LL Algorithm
        // headingController_.setSampleTime(ros::Time::now().toSec() - lastCallbackTime_); /* headingController_ ? */
        // ROS_WARN_STREAM("Sample time: " << headingController_.getSampleTime());
        // double omega = headingController_.derivativeFilteredControlSignal(phiFinal);  /* headingController_ ? */

        // Print and publish the distance to global goal
        double distToGlobGoal = distanceToGlobalGoal();
        ROS_INFO_STREAM("Distance to global goal: " << distToGlobGoal);

        // publishDistToGoal(distToGoalPub_, distToGlobGoal);

        double omega;
        double linearVel;

        // Publish reference omega for Fuzzy planner
        // publishWRef(wRefPub_, 0.1*M_PI);

        // Linear velocity value calculated by the Fuzzy velocity planner
        // double linearVel = scaledLinVelPtr_->data;  //scaledLinVelPtr_ ?

        // Send velocity commands to robot's base
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;

        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.0;//phiFinal - M_PI/4;

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
                if (distanceToGlobalGoal() > 0.01)
                {
                    // Use the last refence cmd_vel command
                cmd_vel.linear.x = 0.5;
                cmd_vel.linear.y = 0.0;
                cmd_vel.linear.z = 0.0;

                cmd_vel.angular.x = 0.0;
                cmd_vel.angular.y = 0.0;
                cmd_vel.angular.z = 0.1 * M_PI;
                }
                
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
        ROS_INFO("odomRYaw is: %f", odomRYaw);

        double goalX = currentGoalPose_.position.x;
        double goalY = currentGoalPose_.position.y;

        // Calculate robot's goal angle
        double phiGoalConstraint = atan2(goalY - odomRY, goalX - odomRX);
        ROS_INFO("phiGoalConstraint is : %f", phiGoalConstraint);

        // Calculate goal phi
        double phiGoal;
        if (phiGoalConstraint - odomRYaw > M_PI)
            phiGoal = (phiGoalConstraint - odomRYaw) - 2 * M_PI;
        else if (phiGoalConstraint - odomRYaw < -M_PI)
            phiGoal = (phiGoalConstraint - odomRYaw) + 2 * M_PI;
        else
            phiGoal = phiGoalConstraint - odomRYaw;

        // Calculate surrounding gaps
        std::deque<double> obstacleAngles;
        std::deque<unsigned int> obstacleAnglesIdx;

        obstacleAngles.push_back(scanPtr_->angle_min);
        obstacleAnglesIdx.push_back(0);
        int obstacleCounter = 1;

        // Get laser ranges
        std::vector<double> laserRanges;
        std::vector<double> currRange;
        for (unsigned int i = 0; i < scanPtr_->ranges.size(); i++)
        {

            // if (isinff(scanPtr_->ranges[i]))
            //     laserRanges.push_back(scanPtr_->range_max + 99);
            // else
            //     laserRanges.push_back(scanPtr_->ranges[i]);
            
            //ROS_INFO_STREAM("scans: "<< scanPtr_->ranges[i] << "for index: " << i);

            if (fabs(scanPtr_->ranges[i] - 2.75) < 0.01)
            {
                laserRanges.push_back(scanPtr_->range_max + 99.);
                // bu indexteki laserRanges[i]'yi 102.5'a eşitliyor. Nedendir bilinmez.
            }
            else
            {
                laserRanges.push_back(scanPtr_->ranges[i]);
            }

            //ROS_INFO_STREAM("this is laserRanges vector, at index: "<< i << " range is: " << laserRanges[i]);

        }

        currRange = laserRanges;
        currRange.erase(currRange.begin()+91,currRange.begin()+271);

        reverse(currRange.begin(), currRange.begin()+91);
        reverse(currRange.begin()+91, currRange.end());

        for (unsigned int i = 0; i < currRange.size(); i++)
        {
            ROS_INFO_STREAM("currrange vector is: "<< currRange[i] << "for index : " << i);

        }

        std::vector<double> gap_starting_points;
        std::vector<double> gap_ending_points;
        int gap_number;
        int gap_validator;
    
    
        for (unsigned int i=1; i<currRange.size(); i++)
        {
            if (currRange[i+1]>currRange[i]+0.5)
            {
                gap_number = gap_number+1;
                gap_starting_points.push_back(i);
            }
            if (currRange[i]>currRange[i+1]+0.5)
            {
                gap_validator = gap_validator+1;
                gap_ending_points.push_back(i+1);
            }
        }
        for (int i = 0; i < gap_starting_points.size()-1;i++)
        {
            ROS_INFO_STREAM("gap starting points are: "<< gap_starting_points[i]);
        }
        for (int i = 0; i < gap_ending_points.size()-1;i++)
        {
            ROS_INFO_STREAM("gap ending points are: "<< gap_ending_points[i]);
        }


        

        /*for (unsigned int j = 90; j >= 0 ; j--)
        {
            currRange.push_back(laserRanges[j]);
            ROS_INFO_STREAM("CurrRange vector is: " << currRange[j] << " for index: " << j);
        }
        for (unsigned int k = 359; k>270; k--)
        {
            currRange.push_back(laserRanges[k]);
            ROS_INFO_STREAM("CurrRange vector is: " << currRange[k] << "for index: " << k);
        }*/

        
        //ROS_INFO_STREAM_ONCE("Laser range size:" << laserRanges.size());

        // Obstacle detection
        for (unsigned int i = 0; i < laserRanges.size() - 1; i++)
        {
            int temp;
            double tempAngle;
            if (fabs(laserRanges[i] - laserRanges[i+1]) > (scanPtr_->range_max) + 0.1)
            {
                if (i < floor(laserRanges.size() / 2))
                {
                    // Right side of laser readings
                    if (laserRanges[i] > laserRanges[i+1])
                        temp = i + 1;
                    else
                        temp = i;
                    
                    tempAngle = -(scanPtr_->angle_max - (scanPtr_->angle_increment * temp));
                }
                else
                {
                    if (i == floor(laserRanges.size() / 2))
                    {
                        // Front of the robot
                        temp = i;
                        tempAngle = 0; //sandalyede denerken buraya dikkat etmek lazım. i = 180 önü mü gösteriyor arkayı mı?
                    }
                    else
                    {
                        // Left side of laser readings
                        if (laserRanges[i] > laserRanges[i+1])
                            temp = i + 1;
                        else
                            temp = i;                       

                        tempAngle = ((scanPtr_->angle_increment * temp) + scanPtr_->angle_min);
                    }
                }

                obstacleAngles.push_back(tempAngle);
                obstacleAnglesIdx.push_back(temp);
                obstacleCounter++;
            }
        }

        obstacleAngles.push_back(scanPtr_->angle_max);
        obstacleAnglesIdx.push_back(laserRanges.size() - 1);

        // for (auto item : obstacleAnglesIdx)
        // {
        //     ROS_INFO_STREAM("Obstacle index " << item);
        //     ROS_INFO_STREAM("Angle " << obstacleAngles[0]);
        //     ROS_INFO_STREAM("Angle " << obstacleAngles[1]);            
        // }

        // Check borders of the gaps
        if (laserRanges[0] > 0 && (laserRanges[0] != 101.75))
        {
            obstacleAngles.pop_front();
            obstacleAnglesIdx.pop_front();
            obstacleCounter--;
        }

        // ROS_INFO_STREAM("first border check");
        // for (auto item : obstacleAnglesIdx)
        // {
        //     ROS_INFO_STREAM("Obstacle index " << item);
        // }

        // ROS_INFO_STREAM("second border check");
        if (laserRanges[laserRanges.size()-1] > 0 && (laserRanges[laserRanges.size()-1] != 101.75))
        {
            obstacleAngles.pop_back();
            obstacleAnglesIdx.pop_back();
            obstacleCounter--;
        }
       
        // for (auto item : obstacleAnglesIdx)
        // {
        //     ROS_INFO_STREAM("Obstacle index " << item);
            
        // }

        // Gap calculations
        std::vector<double> gap;
        double tempTheta;
        for (unsigned int i = 1; i < obstacleAngles.size(); i = i + 2)
        {
            tempTheta = fabs(obstacleAngles[i] - obstacleAngles[i-1]);
            gap.push_back(tempTheta);
        }

        int maxGapIndex = std::max_element(gap.begin(), gap.end()) - gap.begin();

        double theta;
        int angleIndex;
        double phiGapCenter;
        double phiFinal;
        double alpha = 1.4;
        auto dminIdxItr = std::min_element(gap.begin(), gap.end());
        int dminIdx = std::distance(gap.begin(), dminIdxItr);
        double dmin = laserRanges.at(dminIdx);
        int beta = 2; // 1

        if (!gap.empty())
        {
            isGapExist_ = true;
            angleIndex = maxGapIndex * 2;
            double d1 = laserRanges[obstacleAnglesIdx[angleIndex]];
            double d2 = laserRanges[obstacleAnglesIdx[angleIndex+1]];
            double phi1 = obstacleAngles[angleIndex];
            double phi2 = obstacleAngles[angleIndex+1];

            double numer = d1 + d2 * cos(phi1 - phi2);
            double denum = sqrt(pow(d1, 2) + pow(d2, 2) + 2 * d1 * d2 * cos(phi1 - phi2));

            phiGapCenter = acos(numer / denum) + phi1;
            phiFinal = (((alpha / dmin) * phiGapCenter) + (beta * phiGoal)) / (alpha / dmin + beta);

            double tempPhiFinal = fmod(phiFinal, 2 * M_PI);

            if (tempPhiFinal > M_PI)
                phiFinal = tempPhiFinal - 2 * M_PI;
            else if (tempPhiFinal < -M_PI)
                 phiFinal = tempPhiFinal + 2 * M_PI;
            else
                 phiFinal = tempPhiFinal;

            ROS_INFO_STREAM("Final phi: " << phiFinal);
            ROS_INFO_STREAM("Gap center phi: " << phiGapCenter);
            ROS_INFO_STREAM("Goal phi: " << phiGoal);
        }
        else
        {
            ROS_WARN_STREAM("No gap exists");
            isGapExist_ = false;
        }

        ROS_WARN_STREAM("Gap existance: " << isGapExist_);
        // ROS_WARN_STREAM("Phi final: " << phiFinal);
        //ROS_INFO("SELAM");
        return phiFinal;

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
