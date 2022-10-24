#include <iostream>
#include <cstdlib>
#include <cmath>
#include <math.h>
#include <vector>

using namespace std;

int main()
{

    <vector>double gap_starting_points = {-99, -99, 3, 5};
    gap_starting_points.erase(remove(gap_starting_points.begin(),gap_starting_points.end(),-99),gap_starting_points.end());
    for (unsigned int i = 0; i < gap_starting_points.size(); i++)
    {
        cout << "gap start is : " << gap_starting_points[i] << endl;
    }
    
    // double robot_pose_theta = 90;
    // double robot_pose_x = 2;
    // double robot_pose_y = 2;
    // double waypoint_x = 5;
    // double waypoint_y = 3;
    // double goal_angle;
    
    
    // if (robot_pose_theta <= 180 && robot_pose_theta > 0)
    // {
    //     robot_pose_theta = robot_pose_theta -360;
    // }
    
    // if (waypoint_x > robot_pose_x && waypoint_y > robot_pose_y)
    // {
    //     goal_angle = atan(fabs(waypoint_x - robot_pose_x)/fabs(waypoint_y - robot_pose_y));
    //     goal_angle = goal_angle * 180 / 3.14159265;
    // }
    
    // if (waypoint_x > robot_pose_x && waypoint_y < robot_pose_y)
    // {
    //     goal_angle = atan(fabs(waypoint_x - robot_pose_x)/fabs(waypoint_y - robot_pose_y));
    //     goal_angle = 180 - (goal_angle * 180 / 3.14159265);
    // }  
    
    // if (waypoint_x < robot_pose_x && waypoint_y < robot_pose_y)
    // {
    //     goal_angle = atan(fabs(waypoint_x - robot_pose_x)/fabs(waypoint_y - robot_pose_y));
    //     goal_angle = 180 + (goal_angle * 180 / 3.14159265);
    // }   
    

    // if (waypoint_x < robot_pose_x && waypoint_y > robot_pose_y)
    // {
    //     goal_angle = atan(fabs(waypoint_x - robot_pose_x)/fabs(waypoint_y - robot_pose_y));
    //     goal_angle = 360 - (goal_angle * 180 / 3.14159265);
    // }
    
    // if (waypoint_x == robot_pose_x && waypoint_y > robot_pose_y)
    // {
    //     goal_angle = 0;
    // }
    
    // if (waypoint_x == robot_pose_x && waypoint_y < robot_pose_y)
    // {
    //     goal_angle = 180;
    // }
    
    // if (waypoint_x > robot_pose_x && waypoint_y == robot_pose_y)
    // { 
    //     goal_angle = 90;
    // }
    
    // if (waypoint_x < robot_pose_x && waypoint_y == robot_pose_y)
    // {
    //     goal_angle = 270;
    // }

    // cout << "Goal angle is : " << goal_angle << endl;
    // cout << "robot_pose_theta is : " << robot_pose_theta << endl;
    
}