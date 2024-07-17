#include <pluginlib/class_list_macros.h>
#include "local_planner/local_planner.h"
#include <ros/console.h>
#include <fstream>

PLUGINLIB_EXPORT_CLASS(local_planner::LocalPlanner, nav_core::BaseLocalPlanner)

double x_buf[2] = {0.0, 0.0};
double y_buf[2] = {0.0, 0.0};
double xx_buf[2] = {0.0, 0.0};
double yy_buf[2] = {0.0, 0.0};
ros::Time startTime;
ros::Time endTime;
ros::Time totalTimeEnd;
ros::Duration totalExecutionTime;
std::vector<ros::Duration> executionTimes;
int dminIdx;
double averageExecTime;
ofstream myfile;
namespace local_planner
{
    LocalPlanner::LocalPlanner() : costmapROS_(NULL), tf_(NULL), initialized_(false) {}

    LocalPlanner::LocalPlanner(std::string name, tf2_ros::Buffer *tf,
                               costmap_2d::Costmap2DROS *costmapROS)
        : costmapROS_(NULL), tf_(NULL), initialized_(false), goalDistTolerance_(11)

    {
        ROS_INFO("The structure of the local planner was constructed.");
        initialize(name, tf, costmapROS);
    }

    LocalPlanner::~LocalPlanner()

    {
        ROS_INFO("Local planner out.");
    }

    // Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
    void LocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf,
                                  costmap_2d::Costmap2DROS *costmapROS)
    {
        if (!initialized_)
        {
            ros::NodeHandle nh("~/local_planner");

            nh_ = nh;
            tf_ = tf;
            costmapROS_ = costmapROS;
            costmapROS_->getRobotPose(currentPoseTF2_);
            costmap_ = costmapROS_->getCostmap();

            // Subscribers
            odomSub_ = nh_.subscribe("/odom", 100, &LocalPlanner::odomCallback, this);

            scanSub_ = nh_.subscribe("/scan", 100, &LocalPlanner::scanCallback, this); //scan move base içinde remap edildi buradaki scan scan_multi_filtered a yönlendiriliyor.

            poseSub_ = nh_.subscribe("/amcl_pose", 100, &LocalPlanner::poseCallback, this);
            
            cmdSub_ = nh_.subscribe("/cmd_vel_controller", 100, &LocalPlanner::cmdCallback, this);

            collisionSub_ = nh_.subscribe("/base_footprint_contact_sensor_state", 100, &LocalPlanner::collisionCallback, this);

            // Initialize subscribers
            pose_sub_1 = nh_.subscribe("/move_box_plugin/moving_box_1_pose", 10, &LocalPlanner::poseCallback1, this);
            // velocity_sub_1 = nh_.subscribe("/move_box_plugin/moving_box_1_velocity", 10, &LocalPlanner::velocityCallback1, this);

            pose_sub_2 = nh_.subscribe("/move_box_plugin/moving_box_2_pose", 10, &LocalPlanner::poseCallback2, this);
            // velocity_sub_2 = nh_.subscribe("/move_box_plugin/moving_box_2_velocity", 10, &LocalPlanner::velocityCallback2, this);

            pose_sub_3 = nh_.subscribe("/move_box_plugin/moving_box_3_pose", 10, &LocalPlanner::poseCallback3, this);
            // velocity_sub_3 = nh_.subscribe("/move_box_plugin/moving_box_3_velocity", 10, &LocalPlanner::velocityCallback3, this);

            // nh_.getParam("/move_base/local_planner/look_ahead_dist", lookAheadDist_);

            // Publishers
            marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("gap_markers", 10);

            globalPlanPub_ = nh_.advertise<nav_msgs::Path>("global_plan", 1);

            distToGoalPub_ = nh_.advertise<std_msgs::Float32>("distance_to_goal", 1);

            wRefPub_ = nh_.advertise<std_msgs::Float32>("angular_vel_output", 1);

            ROS_INFO("Local planner has been initialized successfully.");
            initialized_ = true;
            myfile.open("/home/otonom/fgm_ws/src/log/mylogs.txt", ios::out | ios::app);
            myfile << "Simulation Started!  ||  ";
            myfile.close();
        }
        else
        {
            ROS_WARN("Local planner has been initialized successfully.");
        }
    }

    bool LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        if (!initialized_)
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

    bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }

        ROS_INFO_ONCE("Computing velocity commands...");
        startTime = ros::Time::now();

        // Publish global plan for visualization
        publishGlobalPlan(globalPlan_);
        ROS_INFO("global plan size is: %lu", globalPlan_.size());
        for (int j = 0; j < globalPlan_.size(); j++)
        {
            // ROS_INFO("global plan is: %f and %f", globalPlan_[j].pose.position.x, globalPlan_[j].pose.position.y);
            // ROS_INFO("j is: %u", j);
        }

        // Find the local goal in the global plan considering look ahead distance
        for (unsigned int i = 0; i < globalPlan_.size(); i++)
        {
            currentPose_.position.x = posePtr_->pose.pose.position.x;
            currentPose_.position.y = posePtr_->pose.pose.position.y;
            currentPose_.position.z = posePtr_->pose.pose.position.z;
            // ROS_INFO("currentpose_x is: %f", currentPose_.position.x);
            // ROS_INFO("currentpose_y is: %f", currentPose_.position.y);

            double waypointX = globalPlan_[i].pose.position.x;
            double waypointY = globalPlan_[i].pose.position.y;

            double diffX = waypointX - currentPose_.position.x;
            double diffY = waypointY - currentPose_.position.y;

            double lookAheadDist_ = 200; // index //global plan size ına göre farklı haritalarda güncellenmesi gereklidir.
            goalDistTolerance_ = 0.55;

            // ROS_INFO("global plan waypoint index: %u", i);
            // ROS_INFO("hypot is: %f", hypot(diffX, diffY));
            // ROS_INFO("distance to global goal is: %f", distanceToGlobalGoal());
            // ROS_INFO("goaldisttolerance is: %f", goalDistTolerance_);
            // ROS_INFO("lookaheaddist is: %f", lookAheadDist_);

            if (globalPlan_.size()>lookAheadDist_)
            {
                currentGoalPoseIdx_ = lookAheadDist_;
            }
            else if (globalPlan_.size()<=lookAheadDist_ || distanceToGlobalGoal() < goalDistTolerance_)
            {
                currentGoalPoseIdx_ = globalPlan_.size()-1;
            }
            else
            {
                continue;
            }

            // if (hypot(diffX, diffY) > lookAheadDist_ && distanceToGlobalGoal() > goalDistTolerance_)
            // {
            //     currentGoalPoseIdx_ = i;
            //     break;
            // }
            // else if (distanceToGlobalGoal() < goalDistTolerance_)
            // {
            //     ROS_INFO("else if e girdik");
            //     currentGoalPoseIdx_ = globalPlan_.size() - 1;
            //     break;
            // }
            // else
            // {
            //     continue;
            // }
        }
        ROS_INFO_STREAM("Global plan size is:  " << globalPlan_.size());
        ROS_INFO_STREAM("Current goal index: " << currentGoalPoseIdx_);

        // ROS_INFO("patlamadi1");

        currentGoalPose_ = globalPlan_.at(currentGoalPoseIdx_).pose;
        // ROS_INFO("patlamadi2");
        // ROS_WARN_STREAM("Goal: " << currentGoalPose_.position.x);
        ROS_INFO_STREAM("Current goal pose: " << currentGoalPose_);

        double phiFinal = LLCallback(); // LL Algorithm

        // Print and publish the distance to global goal
        double distToGlobGoal = distanceToGlobalGoal();
        ROS_INFO_STREAM("Distance to global goal: " << distToGlobGoal);

        double angularVel;
        double linearVel;
        double dmin_temp;
        double phiFinal_abs;
        double coefVel;
        double linearVelocity;
        double alpha_buf;
        double beta_buf;
        // double a0, a1, a2, b0, b1, b2;

        // cut_off_freq = 5.0; 
        // sampling_rate = 10.0;
        // omega = cut_off_freq / sampling_rate;
        alpha_buf = 0.1535;
        beta_buf = 0.8;

        // b0 = (1 - cos(omega)) / 2;
        // b1 = 1 - cos(omega);
        // b2 = (1 - cos(omega)) / 2;
        // a0 = 1 + alpha;
        // a1 = -2 * cos(omega);
        // a2 = 1 - alpha;

        coefVel = 0.7;

        if (dmin > 8)
            dmin_temp = 8;
        else if (dmin <= 0.15)
            dmin_temp = 0.151;
        else
            dmin_temp = dmin;

        phiFinal_abs = abs(phiFinal);

        // linearVel = 0.3 * ((0.292 * log((10 * dmin_temp) + 1)) / (exp(0.883 * phiFinal_temp)) + (exp(1.57 - phiFinal_temp) / 8.01));
        // linearVel = (coefVel * ((0.7 * log((4 * (dmin_temp - 0.1)) + 0.0)) / (exp(0.883 * phiFinal_temp)) + (exp(1.57 - phiFinal_temp) / 5.0))) + 0.1;
        linearVel = (coefVel * ((0.4 * log((3.5 * (dmin_temp - 0.15)) + 0.0)) / (exp(0.883 * phiFinal_abs)) + (exp(1.57 - phiFinal_abs) / 6.5))) + 0.01;
        // angularVel = phiFinal * 0.5 * (exp(dmin_temp - 10) - exp(-4 * dmin_temp) + 1);
        // angularVel = phiFinal * coefVel * (exp(dmin_temp - 10) - exp(-1 * dmin_temp) + (0.1 / (dmin_temp + 0.1)) + 1);
        angularVel = 0.75 * phiFinal * coefVel * ((exp(-4 * dmin_temp) / 2.0) + 1);

        // linearVelocity = min(linearVel, cmdPtr_);
        // linearVelocity = min(10.0, cmdPtr_);
        linearVelocity = min(linearVel, 10.0);

        if (linearVelocity <= 0.0)
        {
            linearVelocity = 0.0;
        }

        x_buf[1] = x_buf[0];
        x_buf[0] = linearVelocity;
        y_buf[1] = y_buf[0];

        y_buf[0] = y_buf[1] * (1 - alpha_buf) + alpha_buf * x_buf[0];
        linearVelocity = y_buf[0]; //linear hız denklemden gelen alınır önce, sonra bir önceki cycledaki linear hız ile alpha, 1-alpha oranında birleştirilir.

        xx_buf[1] = xx_buf[0];
        xx_buf[0] = angularVel;
        yy_buf[1] = yy_buf[0];

        yy_buf[0] = yy_buf[1] * (1 - beta_buf) + beta_buf * xx_buf[0]; //aynısı angular hız için beta kullanılarak yapılır.
        angularVel = yy_buf[0];


        ROS_INFO_STREAM("Lineer velocity: " << linearVelocity);
        ROS_INFO_STREAM("Angular velocity: " << angularVel);


        ROS_INFO_STREAM("dmin: " << dmin_temp);
        ROS_WARN_STREAM("dminidx is: " <<dminIdx);

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

        else if (dmin < 1.4 && dminIdx > 137 && dminIdx < 207 && dmin >= 1.0)
        {
            ROS_WARN_STREAM("Hiyaa!");
            if(dminIdx < 172)
            {
                cmd_vel.linear.x = 0.2;
                // cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.linear.z = 0.0;

                cmd_vel.angular.x = 0.0;
                cmd_vel.angular.y = 0.0;
                // cmd_vel.angular.z = 0.0;
                cmd_vel.angular.z = 0.7;
            }
            else
            {
                cmd_vel.linear.x = 0.2;
                // cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.linear.z = 0.0;

                cmd_vel.angular.x = 0.0;
                cmd_vel.angular.y = 0.0;
                // cmd_vel.angular.z = 0.0;
                cmd_vel.angular.z = -0.7;
            }
        }
        else if (dmin < 1.0 && dmin > 0.75)
        {
            ROS_WARN_STREAM("We are in low dmin!");
            if(dminIdx > 127 && dminIdx <= 172)
            {
                ROS_ERROR_STREAM("here1!");
                // Send velocity commands to robot's base
                cmd_vel.linear.x = 0.2;
                // cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.linear.z = 0.0;

                cmd_vel.angular.x = 0.0;
                cmd_vel.angular.y = 0.0;
                // cmd_vel.angular.z = 0.0;
                cmd_vel.angular.z = 0.7;
            }
            else if(dminIdx > 172 && dminIdx < 217)
            {
                ROS_ERROR_STREAM("here2!");
                // Send velocity commands to robot's base
                cmd_vel.linear.x = 0.2;
                // cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.linear.z = 0.0;

                cmd_vel.angular.x = 0.0;
                cmd_vel.angular.y = 0.0;
                // cmd_vel.angular.z = 0.0;
                cmd_vel.angular.z = -0.7;
            }
            else
            {
                ROS_ERROR_STREAM("here3!");
                // Send velocity commands to robot's base
                cmd_vel.linear.x = linearVelocity*0.5;
                // cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.linear.z = 0.0;

                cmd_vel.angular.x = 0.0;
                cmd_vel.angular.y = 0.0;
                // cmd_vel.angular.z = 0.0;
                cmd_vel.angular.z = angularVel*1.5;
            }
        }
        else if (dmin <= 0.75)
        {
            ROS_WARN_STREAM("We are in lowest dmin!");
            if(dminIdx <= 172)
            {
                ROS_ERROR_STREAM("here4!");
                // Send velocity commands to robot's base
                cmd_vel.linear.x = 0.15;
                // cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.linear.z = 0.0;

                cmd_vel.angular.x = 0.0;
                cmd_vel.angular.y = 0.0;
                // cmd_vel.angular.z = 0.0;
                cmd_vel.angular.z = -0.8;
            }
            else if(dminIdx > 172)
            {
                ROS_ERROR_STREAM("here5!");
                // Send velocity commands to robot's base
                cmd_vel.linear.x = 0.15;
                // cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.linear.z = 0.0;

                cmd_vel.angular.x = 0.0;
                cmd_vel.angular.y = 0.0;
                // cmd_vel.angular.z = 0.0;
                cmd_vel.angular.z = 0.8;
            }
            else
            {
                ROS_ERROR_STREAM("here6!");
                // Send velocity commands to robot's base
                cmd_vel.linear.x = linearVelocity*0.5;
                // cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.linear.z = 0.0;

                cmd_vel.angular.x = 0.0;
                cmd_vel.angular.y = 0.0;
                // cmd_vel.angular.z = 0.0;
                cmd_vel.angular.z = angularVel*1.5;
            }
        }
        // else if (dmin < 1.4 && dminIdx > 110 && dminIdx < 234)
        // {
        //     cmd_vel.linear.x = linearVelocity*0.6;
        //     cmd_vel.linear.y = 0.0;
        //     cmd_vel.linear.z = 0.0;

        //     cmd_vel.angular.x = 0.0;
        //     cmd_vel.angular.y = 0.0;
        //     cmd_vel.angular.z = angularVel*2.2;
        // }
        // else if (dmin < 1.4 && (dminIdx <110 || dminIdx>234))
        // {
        //     cmd_vel.linear.x = linearVelocity;
        //     cmd_vel.linear.y = 0.0;
        //     cmd_vel.linear.z = 0.0;

        //     cmd_vel.angular.x = 0.0;
        //     cmd_vel.angular.y = 0.0;
        //     cmd_vel.angular.z = angularVel*1.8;
        // }
        else
        {
            // Send velocity commands to robot's base
            cmd_vel.linear.x = linearVelocity;
            // cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;

            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            // cmd_vel.angular.z = 0.0;
            cmd_vel.angular.z = angularVel;
        }

        if (distanceToGlobalGoal() < goalDistTolerance_ + 2.5)
        {
            // Send velocity commands to robot's base
            cmd_vel.linear.x = cmd_vel.linear.x;
            // cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;

            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            // cmd_vel.angular.z = 0.0;
            cmd_vel.angular.z = cmd_vel.angular.z;
        }

        ROS_ERROR_STREAM("cmd vel x is : " << cmd_vel.linear.x);
        ROS_ERROR_STREAM("cmd vel z is: " << cmd_vel.angular.z);
        // BURAYI NORMAL FGMDE KULLANMAK LAZIM OLACAK SILME

        // if (!isGapExist_)
        // {
        //     // ROS_INFO("Gap yok!");

        //     if (!isGoalReached())
        //     {
        //         if ((distanceToGlobalGoal() > goalDistTolerance_) && (dmin > 0.2))
        //         {
        //             // Use the last refence cmd_vel command
        //             cmd_vel.linear.x = linearVelocity;
        //             // cmd_vel.linear.x = 0.0;
        //             cmd_vel.linear.y = 0.0;
        //             cmd_vel.linear.z = 0.0;

        //             cmd_vel.angular.x = 0.0;
        //             cmd_vel.angular.y = 0.0;
        //             // cmd_vel.angular.z = 0.0;
        //             cmd_vel.angular.z = angularVel;
        //             ROS_WARN_STREAM("Güncel gap yok hafizadan gidiyor");
        //         }
        //         else
        //         {
        //         ROS_INFO("else 1 in icinde");
        //         cmd_vel.linear.x = 0.0;
        //         cmd_vel.linear.y = 0.0;
        //         cmd_vel.linear.z = 0.0;

        //         cmd_vel.angular.x = 0.0;
        //         cmd_vel.angular.y = 0.0;
        //         // cmd_vel.angular.z = 0.0; //tubitak raporu icin eklendi
        //         cmd_vel.angular.z =-0.5; //çözümsüz kaldığı durumlarda kendi etrafında dönsün diye 
        //         }
        //     }
            
        // }
        endTime = ros::Time::now();

        ros::Duration executionTime = endTime - startTime;

        ROS_INFO("Execution time: %f seconds", executionTime.toSec());
        executionTimes.push_back(executionTime);
        return true;
    }

    bool LocalPlanner::isGoalReached()
    {
        if (!initialized_)
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

            if(!executionTimes.empty())
            {
                for (const auto& time: executionTimes)
                {
                    totalExecutionTime += time;
                }
                averageExecTime = (totalExecutionTime.toSec() / executionTimes.size());
            }
            myfile.open("/home/otonom/fgm_ws/src/log/mylogs.txt", ios::out | ios::app);
            myfile << "Goal Reached!  Total distance traveled is: " << dist_travelled << " || " << "Avg execution time per cycle is: " << averageExecTime << "\n";
            myfile.close();


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
        newPoseData = true;
        lastCallbackTime_ = ros::Time::now().toSec();
    } // end function poseCallback
    
    void LocalPlanner::cmdCallback(const std_msgs::Float64::ConstPtr& msg)
    {
        cmdPtr_ = msg->data;
    } // end function cmdCallback

    void LocalPlanner::collisionCallback(const gazebo_msgs::ContactsState::ConstPtr& msg)
    {
        if (msg->states.empty())
        {
            // No Collision Occured
        }
        else
        {
            // Collision occurred
            if(!executionTimes.empty())
            {
                for (const auto& time: executionTimes)
                {
                    totalExecutionTime += time;
                }
                averageExecTime = (totalExecutionTime.toSec() / executionTimes.size());
            }

            ROS_ERROR_STREAM("Collision occurred!");
            if (collision_counter == 0)
            {
                myfile.open("/home/otonom/fgm_ws/src/log/mylogs.txt", ios::out | ios::app);
                myfile << "Collision occured!  ||  Avg execution time per cycle is: " << averageExecTime << "\n";
                myfile.close();
                collision_counter++;
            }

            


            
        }
    }
    void LocalPlanner::poseCallback1(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        double currentTime1 = ros::Time::now().toSec();
        double timeDiff1 = currentTime1 - prevPoseTime1_;

        if(timeDiff1 > 0)
        {
            velX_box1 = (msg->pose.position.x - prevPosX_box1) / timeDiff1;
            velY_box1 = (msg->pose.position.y - prevPosY_box1) / timeDiff1;
        }
        prevPosX_box1=msg->pose.position.x;
        prevPosY_box1=msg->pose.position.y;
        prevPoseTime1_=currentTime1;
        posX_box1 = msg->pose.position.x;
        posY_box1 = msg->pose.position.y;
        // Process pose data
    }

    // void LocalPlanner::velocityCallback1(const geometry_msgs::TwistStamped::ConstPtr& msg)
    // {
    //     velX_box1 = msg->twist.linear.x;
    //     velY_box1 = msg->twist.linear.y;
    //     // Process velocity data
    // }
    void LocalPlanner::poseCallback2(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        double currentTime2 = ros::Time::now().toSec();
        double timeDiff2 = currentTime2 - prevPoseTime2_;

        if(timeDiff2 > 0)
        {
            velX_box2 = (msg->pose.position.x - prevPosX_box2) / timeDiff2;
            velY_box2 = (msg->pose.position.y - prevPosY_box2) / timeDiff2;
        }
        prevPosX_box2=msg->pose.position.x;
        prevPosY_box2=msg->pose.position.y;
        prevPoseTime2_=currentTime2;
        posX_box2 = msg->pose.position.x;
        posY_box2 = msg->pose.position.y;
        // Process pose data
    }

    // void LocalPlanner::velocityCallback2(const geometry_msgs::TwistStamped::ConstPtr& msg)
    // {
    //     velX_box2 = msg->twist.linear.x;
    //     velY_box2 = msg->twist.linear.y;
    //     // Process velocity data
    // }
    void LocalPlanner::poseCallback3(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        double currentTime3 = ros::Time::now().toSec();
        double timeDiff3 = currentTime3 - prevPoseTime3_;

        if(timeDiff3 > 0)
        {
            velX_box3 = (msg->pose.position.x - prevPosX_box3) / timeDiff3;
            velY_box3 = (msg->pose.position.y - prevPosY_box3) / timeDiff3;
        }
        prevPosX_box3=msg->pose.position.x;
        prevPosY_box3=msg->pose.position.y;
        prevPoseTime3_=currentTime3;
        posX_box3 = msg->pose.position.x;
        posY_box3 = msg->pose.position.y;
        // Process pose data
    }

    // void LocalPlanner::velocityCallback3(const geometry_msgs::TwistStamped::ConstPtr& msg)
    // {
    //     velX_box3 = msg->twist.linear.x;
    //     velY_box3 = msg->twist.linear.y;
    //     // Process velocity data
    // }

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


    vector<vector<double>> midpoint_memory; //dış vektör koordinat olarak tutan
    double prev_odomRX = 0.0;
    double prev_odomRY = 0.0;
    

    double robot_vel_x;
    double robot_vel_y;

    double LocalPlanner::LLCallback()
    {
        ROS_INFO_STREAM("Local Planner started");
        
        ROS_INFO_STREAM("box1 pose: X: " << posX_box1 << " Y:"<< posY_box1 <<" velocity: X: " << velX_box1 << " Y: "<< velY_box1);
        ROS_INFO_STREAM("box1 pose: X: " << posX_box2 << " Y:"<< posY_box2 <<" velocity: X: " << velX_box2 << " Y: "<< velY_box2);
        ROS_INFO_STREAM("box1 pose: X: " << posX_box3 << " Y:"<< posY_box3 <<" velocity: X: " << velX_box3 << " Y: "<< velY_box3);
        // Get odometry informations
        // WARNING: These are not odometry information! Variable names remained
        // unchanged since the latest update. These are AMCL positions.
        double odomRX = posePtr_->pose.pose.position.x;
        double odomRY = posePtr_->pose.pose.position.y;

        dist_travelled += sqrt((odomRX - prev_odomRX)*(odomRX - prev_odomRX) +(odomRY - prev_odomRY)*(odomRY - prev_odomRY)) ;
        // ROS_INFO_STREAM("total distance traveled is: " << dist_travelled);
        
        if(newPoseData)
        {
            robot_vel_x = (odomRX - prev_odomRX)/(lastCallbackTime_-lastCallbackTime_end);
            robot_vel_y = (odomRY - prev_odomRY)/(lastCallbackTime_-lastCallbackTime_end);
            lastCallbackTime_end = ros::Time::now().toSec();
        }
        ROS_WARN_STREAM("robot vel X:" << robot_vel_x);
        ROS_WARN_STREAM("robot vel Y:" << robot_vel_y);
        ROS_WARN_STREAM("odomRX: " << odomRX);
        ROS_WARN_STREAM("odomRY: " << odomRY);
        ROS_WARN_STREAM("prevodomRX: " << prev_odomRX);
        ROS_WARN_STREAM("prevodomRX: " << prev_odomRX);
        ROS_WARN_STREAM("time passed: " << lastCallbackTime_ - lastCallbackTime_end);
        newPoseData = false;

        prev_odomRX = odomRX;
        prev_odomRY = odomRY;
        


        currentPose_.orientation = posePtr_->pose.pose.orientation;
        double robot_pose_theta_real = tf::getYaw(currentPose_.orientation);
        double robot_pose_theta_manipulated;
        robot_pose_theta_real = robot_pose_theta_real * 180 / M_PI;
        // ROS_INFO("robot_pose_theta real is: %f", robot_pose_theta_real);


        double goalX = currentGoalPose_.position.x;
        double goalY = currentGoalPose_.position.y;

        double closest_time_t1 = (-2*(posX_box1 - odomRX)*(velX_box1-robot_vel_x)-2*(posY_box1-odomRY)*(velY_box1-robot_vel_y))/(2*pow((velX_box1-robot_vel_x),2)+2*pow((velY_box1-robot_vel_y),2));
        double closest_time_t2 = (-2*(posX_box2 - odomRX)*(velX_box2-robot_vel_x)-2*(posY_box2-odomRY)*(velY_box2-robot_vel_y))/(2*pow((velX_box2-robot_vel_x),2)+2*pow((velY_box2-robot_vel_y),2));
        double closest_time_t3 = (-2*(posX_box3 - odomRX)*(velX_box3-robot_vel_x)-2*(posY_box3-odomRY)*(velY_box3-robot_vel_y))/(2*pow((velX_box3-robot_vel_x),2)+2*pow((velY_box3-robot_vel_y),2));
        ROS_INFO_STREAM("t1 is: " << closest_time_t1);
        ROS_INFO_STREAM("t2 is: " << closest_time_t2);
        ROS_INFO_STREAM("t3 is: " << closest_time_t3);
        // Get laser ranges
        std::vector<double> laserRanges;
        std::vector<double> currRange;
        // ROS_INFO_STREAM("Scan ptr size is: " << scanPtr_->ranges.size());

        for (unsigned int i = 0; i < scanPtr_->ranges.size(); i++)
        {
            laserRanges.push_back(scanPtr_->ranges[i]);
            // ROS_INFO_STREAM("this is laserRanges vector, at index: "<< i << " range is: " << laserRanges[i]);
        }

        currRange = laserRanges;
        // currRange.erase(currRange.begin(), currRange.begin() + 190); //scanmulti filtrelenmiş verisi sandalyenin arkası 0 olacak şekilde saat yönü tersinde geliyor
        // buradan arkadan ilk 90 ve son 90 derece kırpılarak field of view sadece öndeki 180 derece olacak şekilde ayarlanmıştır.
        // ROS_INFO_STREAM("currrange size is now: "<< currRange.size());
        // currRange.erase(currRange.begin() + 380, currRange.end());
        // ROS_INFO_STREAM("currrange size is now: "<< currRange.size());

        // ilk 90 veri soldan karşıya olan tarama, ikinci 90 veri
        // karşıdan sağa olan tarama olacak şekilde ayarlandı. Turtlebot3 lidarı dönüş yönünden ötürü böyle. Sandalyeye geçildiğinde tarama yönünün teyit edilmesi gerekli.

        reverse(currRange.begin(), currRange.end()); // kırpma işleminden sonra sağdan sola taranmış şekilde öndeki 180 derecelik veriler kalmıştı
        // ters çevrilerek ilk indeksli olan nokta sol 90derecede kalan yer olmaktadır buradan sağa doğru taranmış hale gelir.

        // her lazer ölçümünden 10cm çıkartıldı (obstacle inflation)


        // for (unsigned int i = 0; i < currRange.size() ; i++)
        // {
        //     currRange[i] -= 0.25;
        // }

        // auto dminIdxItr = std::min_element(currRange.begin(), currRange.end());
        auto dminIdxItr = std::min_element(currRange.begin(), currRange.end());
        // int dminIdx = std::distance(currRange.begin(), dminIdxItr);
        dminIdx = std::distance(currRange.begin(), dminIdxItr);
        ROS_ERROR_STREAM("dminidx is : " << dminIdx);

        // dmin = currRange.at(dminIdx);
        dmin = currRange.at(dminIdx);
        if (dmin <= 0.01)
        {
            dmin = 0.01;
        }
        // ROS_INFO_STREAM("curRangesize is " << currRange.size());
        // for (unsigned int i = 0; i < currRange.size(); i++)
        // {
        //     ROS_INFO_STREAM("currrange vector is: "<< currRange[i] << "for index : " << i);
        // }
        ROS_INFO_STREAM("dmin is : " << dmin);


        std::vector<int> gap_starting_points;
        std::vector<int> gap_ending_points;
        std::vector<int> common_angles;
        int gap_number;
        int gap_validator;

        for (unsigned int i = 0; i < currRange.size() - 1; i++)
        {
            if (currRange[i + 1] > currRange[i] + 1.0)
            {
                gap_number = gap_number + 1;
                gap_starting_points.push_back(i);
            }
            if (currRange[i] > currRange[i + 1] + 1.0)
            {
                gap_validator = gap_validator + 1;
                gap_ending_points.push_back(i + 1);
            }
        }
        double phiGoal;
        double phiFinal;
        double xDiff;
        double yDiff;
        
        xDiff = goalX - odomRX;
        yDiff = goalY - odomRY;
        // ROS_INFO_STREAM("goalX is : " << goalX << " goalY is : " << goalY);
        // ROS_INFO_STREAM("odomRX is : " << odomRX << " odomRY is : " << odomRY);

        if ((90 < robot_pose_theta_real) && (robot_pose_theta_real < 180))
        {
            robot_pose_theta_manipulated = 450 - robot_pose_theta_real;
        }
        else
        {
            robot_pose_theta_manipulated = 90 - robot_pose_theta_real;
        }

        double atan2_output = atan2(yDiff, xDiff);
        atan2_output = atan2_output * 180 / M_PI;
        // ROS_INFO_STREAM("atan2 output is : " << atan2_output);

        phiGoal = robot_pose_theta_real - atan2_output + 90;

        if (phiGoal < -90)
        {
            phiGoal = phiGoal + 360;
        }
        else if (phiGoal > 270)
        {
            phiGoal = phiGoal - 360;
        }
        vector<vector<double>> gaps_in_memory; //en son halinde gaplerin koordinatlarını tutacak olan vektör
        vector<vector<double>> gaps_in_memory_predicted; //predictive FGM için modifiye edilecek olan

        double phi_gap = 0.0;
        
        //gap olmadığı durum için phifinal ayarlaması sadece
    
        if ((gap_starting_points.size()== 0 || gap_ending_points.size()==0) || (distanceToGlobalGoal()<2.2) )
        {
            isGapExist_ = false;
            return (M_PI_2 - (M_PI*phiGoal)/180);
            // if (phiGoal > 270)
            //     phiFinal = (450 - phiGoal) * (M_PI / 180);
            // else
            //     phiFinal = (90 - phiGoal) * (M_PI / 180);

            // if (phiFinal > M_PI && phiFinal < 2*M_PI)
            // {
            //     phiFinal = phiFinal - 2*M_PI;
            // }
            // else if (phiFinal > 2*M_PI)
            // {ca
            //     phiFinal = M_PI_2 - (phiFinal - 2*M_PI);
            // }
            // // ROS_ERROR("No gap found, FGM failed.");
            // ROS_WARN("No current gaps found, moving from memory.");
            //return phiFinal; //tubitak raporunda gap gorulmediginde fail olacak sekilde ayarlandıgı icin kapatıldı
            //return 0;
        }
        else
        {
            isGapExist_ = true;
            // for (int i = 0; i < gap_starting_points.size(); i++)
            // {
            //     ROS_INFO_STREAM("gap starting points are: " << gap_starting_points[i]);
            // }
            // for (int i = 0; i < gap_ending_points.size(); i++)
            // {
            //     ROS_INFO_STREAM("gap ending points are: " << gap_ending_points[i]);
            // }

            // gap starting ve gap ending vektörlerinde ortak olan açı değerlerini bulma kısmı (MATLAB intersect fonksiyonunun, common kısmı)
            std::vector<int> intersector(gap_starting_points.size() + gap_ending_points.size());
            std::vector<int>::iterator it, end;

            end = set_intersection(gap_starting_points.begin(), gap_starting_points.end(), gap_ending_points.begin(), gap_ending_points.end(), intersector.begin());

            for (it = intersector.begin(); it != end; it++)
            {
                common_angles.push_back(*it);
            }

            // for (int i = 0; i < common_angles.size(); i++)
            // {
            //     ROS_INFO_STREAM("common angles are: " << common_angles[i]);
            // }

            // MATLAB intersect fonksiyonunun, her iki vektörde ortak olan elemanların indexlerini belirlediği kısım
            for (auto x : common_angles)
            {
                int key = x;
                vector<int>::iterator itr_start = std::find(gap_starting_points.begin(), gap_starting_points.end(), key);
                vector<int>::iterator itr_end = std::find(gap_ending_points.begin(), gap_ending_points.end(), key);

                if (itr_start != gap_starting_points.end())
                {
                    int index_of_common_angle = std::distance(gap_starting_points.begin(), itr_start);
                    // ROS_INFO_STREAM("starting points vector indices are: " << index_of_common_angle);
                    gap_starting_points.erase(gap_starting_points.begin() + index_of_common_angle);
                }

                if (itr_end != gap_ending_points.end())
                {
                    int index_of_common_angle = std::distance(gap_ending_points.begin(), itr_end);
                    // ROS_INFO_STREAM("ending points vector indices are: " << std::distance(gap_ending_points.begin(), itr_end));
                    gap_ending_points.erase(gap_ending_points.begin() + index_of_common_angle);
                }
            }

            // for (int i = 0; i < gap_starting_points.size(); i++)
            // {
            //     ROS_INFO_STREAM("gap starting points are: " << gap_starting_points[i]);
            // }
            // for (int i = 0; i < gap_ending_points.size(); i++)
            // {
            //     ROS_INFO_STREAM("gap ending points are: " << gap_ending_points[i]);
            // }

            int counter = 0;
            int j = 0;
            int k = 1;
            int l = 0;
            int m = 1;
            int n = 0;
            int o = 1;
            vector<int> indices;

            for (unsigned int i = 0; i < gap_starting_points.size(); i++)
            {
                if (i == 0)
                {
                    double temp = gap_starting_points[i];
                    // ROS_INFO_STREAM("temp is : " << temp);
                    for (j; j < gap_ending_points.size(); j++)
                    {
                        if (gap_ending_points[j] < temp)
                        {
                            counter++;
                            // ROS_INFO_STREAM("counter is : " << counter);
                            indices.push_back(j);
                            // ROS_INFO_STREAM("indices are : " << indices[j]);
                        }
                    }
                    if (counter > 1)
                    {
                        for (k; k < indices.size(); k++)
                        {
                            // ROS_INFO_STREAM(" k is " << k);
                            // ROS_INFO_STREAM("indices size is : " << indices.size());
                            gap_ending_points.erase(gap_ending_points.begin() + 1);
                            gap_ending_points.insert(gap_ending_points.begin() + 1, -99);
                        }
                    }
                    // ROS_INFO_STREAM("gap ending points are : ");
                    // for (unsigned int z = 0; z < gap_ending_points.size(); z++)
                    // {
                    //     ROS_INFO_STREAM(" " << gap_ending_points[z]);
                    // }

                    // ROS_INFO_STREAM("gap starting points are : ");
                    // for (unsigned int z = 0; z < gap_starting_points.size(); z++)
                    // {
                    //     ROS_INFO_STREAM(" " << gap_starting_points[z]);
                    // }
                }
                indices = {};
                counter = 0;

                if (i > 0)
                {
                    double temp_prev = gap_starting_points[i - 1];
                    double temp = gap_starting_points[i];
                    // ROS_INFO_STREAM("temp prev is : " << temp_prev);
                    // ROS_INFO_STREAM("temp is : " << temp);
                    for (l; l < gap_ending_points.size(); l++)
                    {
                        // ROS_INFO_STREAM("inside l = " << l);
                        if ((gap_ending_points[l] < temp) && (gap_ending_points[l] > temp_prev))
                        {
                            counter++;
                            // ROS_INFO_STREAM("counter is : " << counter);
                            indices.push_back(l);
                        }
                    }

                    if (counter > 1)
                    {
                        for (m; m < indices.size(); m++)
                        {
                            // ROS_INFO_STREAM(" m is " << m);
                            // ROS_INFO_STREAM("indices size is : " << indices.size());
                            gap_ending_points.erase(gap_ending_points.begin() + indices[1]);
                            gap_ending_points.insert(gap_ending_points.begin() + indices[1], -99);
                        }
                        m = 1;
                    }

                    if (counter == 0)
                    {
                        gap_starting_points.erase(gap_starting_points.begin() + i - 1);
                        gap_starting_points.insert(gap_starting_points.begin() + i - 1, -99);
                    }
                    counter = 0;
                    // ROS_INFO_STREAM("indices are: ");
                    // for (unsigned int z = 0; z < indices.size(); z++)
                    // {
                    //     ROS_INFO_STREAM(" " << indices[z]);
                    // }
                    // ROS_INFO_STREAM("gap ending points are : ");
                    // for (unsigned int z = 0; z < gap_ending_points.size(); z
                    //     ROS_INFO_STREAM(" " << gap_ending_points[z]);
                    // }

                    // ROS_INFO_STREAM("gap starting points are : ");
                    // for (unsigned int z = 0; z < gap_starting_points.size(); z++)
                    // {
                    //     ROS_INFO_STREAM(" " << gap_starting_points[z]);
                    // }
                }
                indices = {};
                l = 0;

                if (i == gap_starting_points.size() - 1)
                {
                    double temp = gap_starting_points[i];
                    // ROS_INFO_STREAM("temp is : " << temp);
                    for (n; n < gap_ending_points.size(); n++)
                    {
                        if (gap_ending_points[n] > temp)
                        {
                            indices.push_back(n);
                            // ROS_INFO_STREAM("indices are : " << indices[counter]);
                            counter++;
                            // ROS_INFO_STREAM("counter is : " << counter);
                        }
                    }
                    if (counter > 1)
                    {
                        for (o; o < indices.size(); o++)
                        {
                            // ROS_INFO_STREAM(" o is " << o);
                            // ROS_INFO_STREAM("indices size is : " << indices.size());
                            gap_ending_points.erase(gap_ending_points.begin() + indices[1]);
                            gap_ending_points.insert(gap_ending_points.begin() + indices[1], -99);
                        }
                    }
                    if (counter == 0)
                    {
                        gap_ending_points.push_back(344);
                    }
                    // ROS_INFO_STREAM("gap ending points are : ");
                    // for (unsigned int z = 0; z < gap_ending_points.size(); z++)
                    // {
                    //     ROS_INFO_STREAM(" " << gap_ending_points[z]);
                    // }

                    // ROS_INFO_STREAM("gap starting points are : ");
                    // for (unsigned int z = 0; z < gap_starting_points.size(); z++)
                    // {
                    //     ROS_INFO_STREAM(" " << gap_starting_points[z]);
                    // }
                }
                counter = 0;
                j = 0;
                k = 1;
                l = 0;
                m = 1;
                n = 0;
                o = 1;
                indices = {};
            }

            // asagidaki donguler iki tane -99 olan eleman varsa patlıyor. i=0da ve i=1 de varsa önce i=0dakini siliyor, i=1deki sıfıra geçtiği için tekrar oraya bakmadan devam ediyor.

            for (unsigned int i = 0; i < gap_starting_points.size(); i++)
            {
                if (gap_starting_points[i] == -99)
                {
                    gap_starting_points.erase(gap_starting_points.begin() + i);
                    i--;
                }
            }
            for (unsigned int i = 0; i < gap_ending_points.size(); i++)
            {
                if (gap_ending_points[i] == -99)
                {
                    gap_ending_points.erase(gap_ending_points.begin() + i);
                    i--;
                }
            }

            gap_starting_points.erase(remove(gap_starting_points.begin(), gap_starting_points.end(), -99), gap_starting_points.end());

            if (gap_ending_points[0] < gap_starting_points[0])
            {
                gap_starting_points.insert(gap_starting_points.begin(), 0);
            }
            // ROS_INFO_STREAM("gap ending points are : ");
            // for (unsigned int z = 0; z < gap_ending_points.size(); z++)
            // {
            //     ROS_INFO_STREAM(" " << gap_ending_points[z]);
            // }

            // ROS_INFO_STREAM("gap starting points are : ");
            // for (unsigned int z = 0; z < gap_starting_points.size(); z++)
            // {
            //     ROS_INFO_STREAM(" " << gap_starting_points[z]);
            // }

            if (!common_angles.empty())
            {
                for (int i = 0; i < common_angles.size(); i++)
                {
                    gap_starting_points.push_back(common_angles[i]);
                    gap_ending_points.push_back(common_angles[i]);
                }
            }

            sort(gap_starting_points.begin(), gap_starting_points.end());
            sort(gap_ending_points.begin(), gap_ending_points.end());

            int min_size;

            // ROS_INFO_STREAM("gap starting points are : ");
            // for (int i = 0; i < gap_starting_points.size(); i++)
            // {
            //     ROS_INFO_STREAM(gap_starting_points[i]);
            // }
            // ROS_INFO_STREAM("gap ending points are : ");
            // for (int i = 0; i < gap_ending_points.size(); i++)
            // {
            //     ROS_INFO_STREAM(gap_ending_points[i]);
            // }

            min_size = min(gap_starting_points.size(), gap_ending_points.size());

            double array_gap[min_size][2];

            for (int i = 0; i < min_size; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    if (j == 0)
                    {
                        array_gap[i][j] = gap_starting_points[i];
                    }
                    else
                    {
                        array_gap[i][j] = gap_ending_points[i];
                    }
                }
            }
            // ROS_INFO_STREAM("min_size is = " << min_size);
            int counter_array = 0;

            // for (int i = 0; i < min_size; i++)
            // {
            //     for (int j = 0; j < 2; j++)
            //     {
            //         counter_array++;
            //         ROS_INFO_STREAM("Array gap's " << counter_array << " element is = " << array_gap[i][j]);
            //     }
            // }

            common_angles.erase(common_angles.begin(), common_angles.end());

            int rows, cols;
            int idx = 0;
            int max_gap = 0;
            double alpha = 0.0;
            double beta = 0.0;

            double d1 = 0.0, d2 = 0.0;
            

            rows = sizeof(array_gap) / sizeof(array_gap[0]);
            cols = sizeof(array_gap[0]) / sizeof(array_gap[0][0]);

            for (int i = 0; i < rows; i ++)
            {
                for (int j = 0 ; j < cols ; j++)
                {
                    array_gap[i][j] = array_gap[i][j] * (163.0/344.0);
                }
            }
            counter_array = 0;
            for (int i = 0; i < min_size; i++)
            {
                for (int j = 0; j < 2; j++)
                {  
                    counter_array++;
                    // ROS_INFO_STREAM("Array gap's " << counter_array << " element is = " << array_gap[i][j]);
                    // ROS_INFO_STREAM("Array gap's rounded " << counter_array << " element is = " << round(array_gap[i][j]));
                }
            }

            double memory_array[min_size][4]; //gaplerin köse noktalarının x ve y koordinatlarını bulunduran array, ilk sütun d1 ölçümünden gelen X koord, 2. sütun d1'in Y koord, 3. sütun d2'nin X koord, 4. sütun d2'nin Y koord
            double midpoint_coords[min_size][3]; //memory arraydeki X Y koordinatlarının ortalarının hesaplanıp her satırda 1 gapin orta noktası ilk sütunda X ikinci sütunda Y koord olarak tutulur

            double lidar_coord_x;
            double lidar_coord_y;

            lidar_coord_x = odomRX + 0.322*sin(robot_pose_theta_manipulated*(M_PI/180.0));  //ön lidarın koordinatının amcl verisi ile hesaplanışı
            lidar_coord_y = odomRY + 0.322*cos(robot_pose_theta_manipulated*(M_PI/180.0));
            // ROS_INFO_STREAM("odomrx: " << odomRX << " odomry: " << odomRY);
            // ROS_INFO_STREAM("lidar coord x: " << lidar_coord_x << " lidar coord y: " << lidar_coord_y);

            
            vector<double> gap_midpoints;  //gap midpointlerinin açı değerlerini tutan vektör
            // vector<double> diff_to_goal; //gap odullendirmede kullanılan ölçüt
            //iç vektör ama sadece indeks olarak tutan
            double d1_temp, d2_temp, alpha_temp, beta_temp, midpoint, gap_width; //midpoint hesaplamada kullanılan, her gap için d1 d2 temp değişkenleri



            for (int i = 0; i < rows; i++) // her gap icin midpoint makaledeki denklemle hesaplanır
            {   // Bu for döngüsü içinde hem açısal olarak midpoint hesabı yapılır hem de hafıza kısmından gelen koordinat oluşturma ve onların midpoint hesabı yapılır.
                // ROS_INFO_STREAM("Total gap count is: " << i+1);
                alpha_temp = array_gap[i][0];
                beta_temp = array_gap[i][1];

                if(alpha_temp == 0.0)
                {
                    alpha_temp = beta_temp;
                }

                if(beta_temp == 163.0)
                {
                    beta_temp = alpha_temp;
                }
                // ROS_INFO_STREAM("d1_temp at: " << alpha_temp*(344.0/163.0));
                // ROS_INFO_STREAM("alpha_temp at: " << alpha_temp);
                d1_temp = currRange.at(round(alpha_temp*(344.0/163.0)));
                // ROS_INFO_STREAM("d2_temp at: " << beta_temp*(344.0/163.0));
                // ROS_INFO_STREAM("beta_temp at: " << beta_temp);
                // if (beta_temp >= 163.0)
                //     beta_temp = 162.01;
                    // ROS_INFO_STREAM("beta_temp at: " << beta_temp);
                d2_temp = currRange.at(round(beta_temp*(344.0/163.0)));
                // ROS_INFO_STREAM("d1temp: " << d1_temp << " d2temp: " <<d2_temp);
                // ROS_INFO_STREAM("robot_pose_theta_manipulated: " << robot_pose_theta_manipulated);


                memory_array[i][0] = lidar_coord_x - d1_temp*cos(M_PI*(robot_pose_theta_manipulated + (alpha_temp+8.5))/180.0); //d1 den gelen X koord
                memory_array[i][1] = lidar_coord_y + d1_temp*sin(M_PI*(robot_pose_theta_manipulated + (alpha_temp+8.5))/180.0); //d1 den gelen y koord
                memory_array[i][2] = lidar_coord_x - d2_temp*cos(M_PI*(robot_pose_theta_manipulated + (beta_temp+8.5))/180.0);  //d2 den gelen X
                memory_array[i][3] = lidar_coord_y + d2_temp*sin(M_PI*(robot_pose_theta_manipulated + (beta_temp+8.5))/180.0);  //d2 den gelen Y
                // ROS_INFO_STREAM("d1X is : " << memory_array[i][0]);
                // ROS_INFO_STREAM("d1Y is : " << memory_array[i][1]);
                // ROS_INFO_STREAM("d2X is : " << memory_array[i][2]);
                // ROS_INFO_STREAM("d2Y is : " << memory_array[i][3]);
                gap_width = sqrt(pow((memory_array[i][0] - memory_array[i][2]),2) + pow((memory_array[i][1] - memory_array[i][3]),2));


                midpoint_coords[i][0] = (memory_array[i][0] + memory_array[i][2])/2.0; //midpointin X koordinatı
                midpoint_coords[i][1] = (memory_array[i][1] + memory_array[i][3])/2.0; //midpointin Y koordinatı
                midpoint_coords[i][2] = gap_width;

                // ROS_INFO_STREAM("gap midpoint coords are, x: " << midpoint_coords[i][0] << " y: "<< midpoint_coords[i][1] << " width: " << gap_width);


                midpoint = 180*(acos((d1_temp + d2_temp * cos((M_PI / 180) * (beta_temp + 8.5) - (M_PI / 180) * (alpha_temp + 8.5))) / sqrt(d1_temp * d1_temp + d2_temp * d2_temp + 2 * d1_temp * d2_temp * cos((M_PI / 180) * (beta_temp + 8.5) - (M_PI / 180) * (alpha_temp + 8.5)))) + (M_PI / 180) * (alpha_temp + 8.5))/M_PI;
                gap_midpoints.push_back(midpoint);
                // diff_to_goal.push_back(fabs(midpoint - phiGoal));
                // ROS_INFO_STREAM("d1 temp is : " << d1_temp);
                // ROS_INFO_STREAM("d2 temp is : " << d2_temp);
            }

            
            for (int i=0; i<rows ;i++) //bu döngünün içince her gap midpointe ait x ve y koordinatları midpoint vektörüne pushlanır. midpoint vektörü hafıza vektörüne pushlanır. bir cycleda 2 gap görüldüyse yine teker teker pushlanır.
            {
                if(midpoint_coords[i][2] > 0.65) //genişliği 0.45'ten küçük olan gapler hafızaya atılmaz
                {
                    vector<double> currentgaps; //iç vektör, koordinat olarak tutan

                    // ROS_INFO_STREAM("rows is : " << rows);

                    currentgaps.push_back(midpoint_coords[i][0]); //içteki küçük vektöre x koordinatının pushlandığı yer
                    currentgaps.push_back(midpoint_coords[i][1]); //içteki küçük vektöre y koordinatının pushlandığı yer
                    currentgaps.push_back(midpoint_coords[i][2]); //içteki küçük vektöre gap genişliğinin pushlandığı yer
                    currentgaps.push_back(memory_array[i][0]); //d1x
                    currentgaps.push_back(memory_array[i][1]); //d1y
                    currentgaps.push_back(memory_array[i][2]); //d2x
                    currentgaps.push_back(memory_array[i][3]); //d2y
                    gaps_in_memory.push_back(currentgaps);  //içteki küçük vektörü dıştaki büyük hafıza vektörüne pushlama
                    gaps_in_memory_predicted.push_back(currentgaps);
                    // for (int i = 0; i < midpoint_x_y.size();i++)
                    // {
                    //     ROS_INFO_STREAM("midpoint_x_y has: " << midpoint_x_y[i]);
                    // }
                }
            }
        }

        visualization_msgs::MarkerArray markers;
        //önceki markları silme bloğu
        visualization_msgs::Marker deleteMarker;
        deleteMarker.header.frame_id= "map";
        deleteMarker.header.stamp = ros::Time::now();
        deleteMarker.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(deleteMarker);
        marker_pub_.publish(markers);
        markers.markers.clear();
        //silme bloğu bitişi
        markers.markers.reserve(gaps_in_memory.size());
        // hafızadaki gapleri boyutları oranında haritaya kırmızı silindir olarak çizdirme
        for (int i = 0; i < gaps_in_memory.size(); i++)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.id = i;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.2;
            // marker.scale.z = 0.1;
            marker.color.a = 1.0;
            marker.color.r = 1.0;

            geometry_msgs::Point start_point;
            start_point.x = gaps_in_memory[i][3]; // x coordinate of the start point
            start_point.y = gaps_in_memory[i][4]; // y coordinate of the start point
            start_point.z = 0.05; // z coordinate of the start point

            geometry_msgs::Point end_point;
            end_point.x = gaps_in_memory[i][5]; // x coordinate of the end point
            end_point.y = gaps_in_memory[i][6]; // y coordinate of the end point
            end_point.z = 0.05; // z coordinate of the end point

            marker.points.push_back(start_point);
            marker.points.push_back(end_point);

            markers.markers.push_back(marker);
            ROS_INFO_STREAM("gaps in memory: " <<gaps_in_memory[i][0] <<"---" << gaps_in_memory[i][1]<<"---" <<gaps_in_memory[i][2]<<"---" <<gaps_in_memory[i][3]<<"---" <<gaps_in_memory[i][4]<<"---" <<gaps_in_memory[i][5]<<"---" <<gaps_in_memory[i][6]);
        }
        // hafızadaki gapleri çizdirme bitişi

        // waypoint olarak seçilen hedef noktanın yeşil küp olarak çizdirilmesi kısmı
        visualization_msgs::Marker goal_marker;
        goal_marker.header.frame_id = "map";
        goal_marker.header.stamp = ros::Time::now();
        goal_marker.id = 50;
        goal_marker.type = visualization_msgs::Marker::CUBE;
        goal_marker.action = visualization_msgs::Marker::ADD;
        goal_marker.pose.orientation.w = 1.0;
        goal_marker.scale.z = 0.1;
        goal_marker.pose.position.x = currentGoalPose_.position.x;
        goal_marker.pose.position.y = currentGoalPose_.position.y;
        goal_marker.pose.position.z = 0.1;
        goal_marker.scale.x = 0.3;
        goal_marker.scale.y = 0.3;
        goal_marker.color.a = 1.0;
        goal_marker.color.g = 1.0;
        markers.markers.push_back(goal_marker);
        // waypoint çizdirmesi bitişi


        vector<double> diff_to_goal_new;
        vector<double> xDiff_new;
        vector<double> yDiff_new;
        vector<double> phi_gap_temp;
        vector<double> gap_sizes_new;
        double phi_gap_calculator;
        double expectedposeX1;
        double expectedposeY1;
        double expectedposeX2;
        double expectedposeY2;
        double expectedposeX3;
        double expectedposeY3;


        if (gaps_in_memory.size() == 0)
        {
            return (M_PI_2 - (M_PI*phiGoal)/180);
        }

        phiGoal += 90; // ödüllendirmede ekseni 90 derece shift etmek için yapıldı.
        double gapcornerthreshold = 1.0;

        for(int i = 0; i < gaps_in_memory.size(); i++)
        {
            if(fabs(gaps_in_memory[i][3] - posX_box1) < gapcornerthreshold && fabs(gaps_in_memory[i][4] - posY_box1) < gapcornerthreshold) //kutu 1 d1 ile ilişkili mi
            {
                ROS_INFO("box1=d1");
                expectedposeX1 = velX_box1*closest_time_t1+gaps_in_memory[i][3];
                expectedposeY1 = velY_box1*closest_time_t1+gaps_in_memory[i][4];
                ROS_INFO_STREAM("expected pose of box 1: " << expectedposeX1 << " and " << expectedposeY1);
                gaps_in_memory_predicted[i][3] = expectedposeX1;
                gaps_in_memory_predicted[i][4] = expectedposeY1;
            }
            else if(fabs(gaps_in_memory[i][5] - posX_box1) < gapcornerthreshold && fabs(gaps_in_memory[i][6] - posY_box1) < gapcornerthreshold) //kutu 1 d2 ile ilişkili mi
            {
                ROS_INFO("box1=d2");
                expectedposeX1 = velX_box1*closest_time_t1+gaps_in_memory[i][5];
                expectedposeY1 = velY_box1*closest_time_t1+gaps_in_memory[i][6];
                ROS_INFO_STREAM("expected pose of box 1: " << expectedposeX1 << " and " << expectedposeY1);
                gaps_in_memory_predicted[i][5] = expectedposeX1;
                gaps_in_memory_predicted[i][6] = expectedposeY1;
            }
            if(fabs(gaps_in_memory[i][3] - posX_box2) < gapcornerthreshold && fabs(gaps_in_memory[i][4] - posY_box2) < gapcornerthreshold) // kutu 2 d1 ile ilişkili mi
            {
                ROS_INFO("box2=d1");
                expectedposeX2 = velX_box2*closest_time_t2+gaps_in_memory[i][3];
                expectedposeY2 = velY_box2*closest_time_t2+gaps_in_memory[i][4];
                ROS_INFO_STREAM("expected pose of box 2: " << expectedposeX2 << " and " << expectedposeY2);
                gaps_in_memory_predicted[i][3] = expectedposeX2;
                gaps_in_memory_predicted[i][4] = expectedposeY2;
            }
            else if(fabs(gaps_in_memory[i][5] - posX_box2) < gapcornerthreshold && fabs(gaps_in_memory[i][6] - posY_box2) < gapcornerthreshold) // kutu 2 d2 ile ilişkili mi
            {
                ROS_INFO("box2=d2");
                expectedposeX2 = velX_box2*closest_time_t2+gaps_in_memory[i][5];
                expectedposeY2 = velY_box2*closest_time_t2+gaps_in_memory[i][6];
                ROS_INFO_STREAM("expected pose of box 2: " << expectedposeX2 << " and " << expectedposeY2);
                gaps_in_memory_predicted[i][5] = expectedposeX2;
                gaps_in_memory_predicted[i][6] = expectedposeY2;
            }
            if(fabs(gaps_in_memory[i][3] - posX_box3) < gapcornerthreshold && fabs(gaps_in_memory[i][4] - posY_box3) < gapcornerthreshold) // kutu 3 d1 ile ilişkili mi
            {
                ROS_INFO("box3=d1");
                expectedposeX3 = velX_box3*closest_time_t3+gaps_in_memory[i][3];
                expectedposeY3 = velY_box3*closest_time_t3+gaps_in_memory[i][4];
                ROS_INFO_STREAM("expected pose of box 3: " << expectedposeX3 << " and " << expectedposeY3);
                gaps_in_memory_predicted[i][3] = expectedposeX3;
                gaps_in_memory_predicted[i][4] = expectedposeY3;
            }
            else if(fabs(gaps_in_memory[i][5] - posX_box3) < gapcornerthreshold && fabs(gaps_in_memory[i][6] - posY_box3) < gapcornerthreshold) // kutu 3 d2 ile ilişkili mi
            {
                ROS_INFO("box3=d2");
                expectedposeX3 = velX_box3*closest_time_t3+gaps_in_memory[i][5];
                expectedposeY3 = velY_box3*closest_time_t3+gaps_in_memory[i][6];
                ROS_INFO_STREAM("expected pose of box 3: " << expectedposeX3 << " and " << expectedposeY3);
                gaps_in_memory_predicted[i][5] = expectedposeX3;
                gaps_in_memory_predicted[i][6] = expectedposeY3;
            }
        }


        //en son hafızada birleştirilmiş gaplerin x, y koordinatları ve genişliği
        // ROS_WARN_STREAM("There are total of: " << gaps_in_memory.size() << " current gaps being detected now");
        // for (int i = 0; i < gaps_in_memory.size(); i++)
        // {
        //     xDiff_new.push_back(gaps_in_memory[i][0] - odomRX);
        //     yDiff_new.push_back(gaps_in_memory[i][1] - odomRY);
        //     phi_gap_calculator = atan2(yDiff_new[i], xDiff_new[i])*180/M_PI;

        //     phi_gap_calculator = robot_pose_theta_real - phi_gap_calculator + 90;

        //     if (phi_gap_calculator < -90)
        //     {
        //         phi_gap_calculator = phi_gap_calculator + 360;
        //     }
        //     else if (phi_gap_calculator > 270)
        //     {
        //         phi_gap_calculator = phi_gap_calculator - 360;
        //     }

        //     phi_gap_temp.push_back(phi_gap_calculator);
        //     // ROS_INFO_STREAM("phi gap temp is : " << phi_gap_temp[i]);

        //     phi_gap_temp[i] += 90; //gap ödüllendirmede 90 derece shift etmek için yapıldı ekseni. gerçek phigap ile alakası yok.
        //     diff_to_goal_new.push_back(min(fabs(phi_gap_temp[i] - phiGoal), 360-fabs(phi_gap_temp[i] - phiGoal)));
        //     // ROS_INFO_STREAM("diff to goal is : " << diff_to_goal_new[i]);
        //     // ROS_INFO_STREAM("gaps are located at: X| " << gaps_in_memory[i][0] << " Y| " << gaps_in_memory[i][1] << " width| " << gaps_in_memory[i][2]);
        // }

        for (int i = 0; i < gaps_in_memory_predicted.size(); i++)
        {
            // 0.2'den sonrası küçük olanı seçsin diye yapılmış manipülasyon!!
            gaps_in_memory_predicted[i][2] = sqrt(pow((gaps_in_memory_predicted[i][3] - gaps_in_memory_predicted[i][5]),2) + pow((gaps_in_memory_predicted[i][4] - gaps_in_memory_predicted[i][6]),2)) +0.2*(sqrt(pow((gaps_in_memory_predicted[i][3] - gaps_in_memory_predicted[i][5]),2) + pow((gaps_in_memory_predicted[i][4] - gaps_in_memory_predicted[i][6]),2))-gaps_in_memory_predicted[i][2]);
            ROS_INFO_STREAM("new width of gap " << i  << " : " << gaps_in_memory_predicted[i][2]);
        }

        for (int i = 0; i < gaps_in_memory_predicted.size(); i++)
        {
            xDiff_new.push_back(gaps_in_memory_predicted[i][0] - odomRX);
            yDiff_new.push_back(gaps_in_memory_predicted[i][1] - odomRY);
            phi_gap_calculator = atan2(yDiff_new[i], xDiff_new[i])*180/M_PI;

            phi_gap_calculator = robot_pose_theta_real - phi_gap_calculator + 90;

            if (phi_gap_calculator < -90)
            {
                phi_gap_calculator = phi_gap_calculator + 360;
            }
            else if (phi_gap_calculator > 270)
            {
                phi_gap_calculator = phi_gap_calculator - 360;
            }

            phi_gap_temp.push_back(phi_gap_calculator);
            // ROS_INFO_STREAM("phi gap temp is : " << phi_gap_temp[i]);

            phi_gap_temp[i] += 90; //gap ödüllendirmede 90 derece shift etmek için yapıldı ekseni. gerçek phigap ile alakası yok.
            diff_to_goal_new.push_back(min(fabs(phi_gap_temp[i] - phiGoal), 360-fabs(phi_gap_temp[i] - phiGoal)));
            // ROS_INFO_STREAM("diff to goal is : " << diff_to_goal_new[i]);
            // ROS_INFO_STREAM("gaps are located at: X| " << gaps_in_memory[i][0] << " Y| " << gaps_in_memory[i][1] << " width| " << gaps_in_memory[i][2]);
        }

        // hafızadaki gapleri ödüllendirme

        for (int i=0; i < gaps_in_memory_predicted.size(); i++)
        {
            if (gaps_in_memory_predicted[i][2] < 0.65) //0,45 ten kucuk olan gapler odullendirilmez.
            {
                gaps_in_memory_predicted[i][2] = 0.1;
            }
            // else if (diff_to_goal_new[i] <= 30)
            // {
            //     gaps_in_memory_predicted[i][2] = gaps_in_memory_predicted[i][2] * 2.2;
            //     // gaps_in_memory[i][2] = gaps_in_memory[i][2] + gaps_in_memory[i][2] * (2/(exp(diff_to_goal_new[i]/20))+1); // 0.75'ten buyuk gaplerin hepsi bu ölçüte göre büyütülür.
            // }
            // else if (diff_to_goal_new[i] <= 60 && diff_to_goal_new[i] > 30)
            // {
            //     gaps_in_memory_predicted[i][2] = gaps_in_memory_predicted[i][2] * 1.8;
            // }
            else if (diff_to_goal_new[i] <= 80)
            {
                gaps_in_memory_predicted[i][2] = gaps_in_memory_predicted[i][2]*1.8;
            }
            else if (diff_to_goal_new[i] <= 90 && diff_to_goal_new[i] > 80)
            {
                gaps_in_memory_predicted[i][2] = gaps_in_memory_predicted[i][2] * 1.2;
            }

            else if (diff_to_goal_new[i] <= 120 && diff_to_goal_new[i] > 90)
            {
                gaps_in_memory_predicted[i][2] = gaps_in_memory_predicted[i][2] * 0.9;
            }
            else if (diff_to_goal_new[i] <= 150 && diff_to_goal_new[i] > 120)
            {
                gaps_in_memory_predicted[i][2] = gaps_in_memory_predicted[i][2] * 0.6;
            }
            else if (diff_to_goal_new[i] <= 180 && diff_to_goal_new[i] > 150)
            {
                gaps_in_memory_predicted[i][2] = gaps_in_memory_predicted[i][2] * 0.3;
            }
            else
            {
                ROS_WARN_STREAM("bir hata var else e girilmemeliydi");
            }

            phi_gap_temp[i] -= 90; // ödüllendirme için eklenmiş olan 90 geri çıkarıldı.
        }
        phiGoal -= 90; // ödüllendirme için eklenmiş olan 90 geri çıkarıldı.

        double largestWidthIndex = 0;
        double largestWidth = gaps_in_memory_predicted[0][2];
        for (int i = 0; i < gaps_in_memory_predicted.size(); i++)
        {
            if (gaps_in_memory_predicted[i][2] > largestWidth)
            {
                largestWidth = gaps_in_memory_predicted[i][2];
                
                largestWidthIndex = i;
            }
        }
        for (int i = 0; i < gaps_in_memory_predicted.size(); i++)
        {
            ROS_INFO_STREAM("odullendirme sonrasi width of gap " << i  << " : " << gaps_in_memory_predicted[i][2]);
        }

        ROS_INFO_STREAM("largestwidth is: " << largestWidth);
        ROS_INFO_STREAM("selected gap is at index " << largestWidthIndex);
        phi_gap = phi_gap_temp[largestWidthIndex];
        

        // seçilmiş olan gap in ortasına mavi bir silindir çizmek için.
        visualization_msgs::Marker selected_gap_marker;
        selected_gap_marker.header.frame_id = "map";
        selected_gap_marker.header.stamp = ros::Time::now();
        selected_gap_marker.id = 51;
        selected_gap_marker.type = visualization_msgs::Marker::CYLINDER;
        selected_gap_marker.action = visualization_msgs::Marker::ADD;
        selected_gap_marker.pose.orientation.w = 1.0;
        selected_gap_marker.scale.z = 0.1;
        selected_gap_marker.pose.position.x = gaps_in_memory[largestWidthIndex][0];
        selected_gap_marker.pose.position.y = gaps_in_memory[largestWidthIndex][1];
        selected_gap_marker.pose.position.z = 0.1;
        selected_gap_marker.scale.x = 0.4;
        selected_gap_marker.scale.y = 0.4;
        selected_gap_marker.color.a = 1.0;
        selected_gap_marker.color.b = 1.0;
        markers.markers.push_back(selected_gap_marker);
        // seçilmiş gap ortasına mavi silindir çizme bitişi
        // ROS_WARN_STREAM("markers has " << markers.markers.size() << " elements");
        marker_pub_.publish(markers);



        gaps_in_memory.clear();
        gaps_in_memory_predicted.clear();
        phi_gap_temp.clear();
        diff_to_goal_new.clear();

        double alpha_weight = 62;
        // Dmin tutma işi yapıldığında kullanılıyordu    
        // if(dmin < 2.0)
        // {
        //     below_2 = true;
        //     time_below_2 = ros::Time::now();
        //     // ROS_WARN_STREAM("true oldu");
            

        //     if(below_2 && dmin < final_dmin)
        //     {
        //         time_below_2 = ros::Time::now();
        //         // ROS_WARN_STREAM("tazelendi");
        //     }
        //     final_dmin = std::min(final_dmin, dmin);
        //     dmin = final_dmin;
        // }

        // ros::Duration elapsed_seconds = ros::Time::now() - time_below_2;
        // double seconds_elapsed = elapsed_seconds.toSec();

        // ROS_WARN_STREAM("geçen zaman: " << seconds_elapsed);
        // if (below_2 && seconds_elapsed < 5.0)
        // {
        //     dmin = final_dmin;
        // }
        // else if (below_2 && seconds_elapsed >= 5.0)
        // {
        //     // ROS_WARN_STREAM("bu ifteyiz");
        //     below_2 = false;
        //     final_dmin = 2.0;
        // }

        // if(below_2)
        // {
        //     final_dmin = std::min(final_dmin, dmin);
        //     ROS_WARN_STREAM("diğer ifteyiz");
        //     dmin = final_dmin;
        // }
        // else
        // {
        //     final_dmin = dmin;
        //     ROS_WARN_STREAM("elsedeyiz");
        // }

        // ROS_WARN_STREAM("dmin now is: " << dmin);//double beta_weight = 2.8;


        phiFinal = (((alpha_weight / exp(dmin)) * (phi_gap * M_PI/180)) + (phiGoal * M_PI/180)) / (alpha_weight / exp(dmin) + 1);
        // phiFinal = phi_gap;
        // ROS_INFO_STREAM("moving to : "<< phiFinal);
        //double phiFinal = 0; //(90-phiGoal)*M_PI/180;

        //aşağıdaki phifinal ayarlamalarının hepsi gereksiz duruyor. Normalde zaten o iflere girecek bir şey yok, sadece ilk else her zaman geçerli gibi devam etmek lazım
        // 31 mayıs 2023

        phiFinal = (M_PI_2) - phiFinal; //alttaki ifleri açacaksan bunu kapat

        // // if (phiFinal > 1.5*M_PI)
        // //     phiFinal = (2.5*M_PI - phiFinal);
        // // else
        // //     phiFinal = (M_PI_2 - phiFinal);

        // // if (phiFinal > M_PI && phiFinal < 2*M_PI)
        // // {
        // //     phiFinal = phiFinal - 2*M_PI;
        // // }
        // // else if (phiFinal > 2*M_PI)
        // // {
        // //     phiFinal = M_PI_2 - (phiFinal - 2*M_PI);
        // // }
        // ROS_INFO_STREAM("alpha_weight/dmin is: " << alpha_weight/dmin);
        ROS_INFO_STREAM("phi gap is : " << phi_gap);
        ROS_INFO_STREAM("phi goal is : " << phiGoal);
        double moving_to;
        moving_to = 90 - phiFinal*180/M_PI;
        ROS_INFO_STREAM("moving to : " << moving_to);

        return phiFinal;
    }

    void LocalPlanner::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped> &path)
    {
        base_local_planner::publishPlan(path, globalPlanPub_);
    } // end function publishGlobalPlan

    // void publishDistToGoal(const ros::Publisher &pub, double dist)
    // {
    //     std_msgs::Float32 msg;
    //     msg.data = dist;

    //     pub.publish(msg);
    // } // end function publishDistToGoal

    void publishWRef(const ros::Publisher &pub, double wRef)
    {
        std_msgs::Float32 msg;
        msg.data = wRef;

        pub.publish(msg);
    } // end function publishWRef
}
