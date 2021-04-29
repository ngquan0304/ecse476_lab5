#include <ros/ros.h>
#include <iostream>
#include <string>
#include <detect_object/DetectObjectServiceMsg.h>
#include <mobot_controller/LocomotionAction.h>
#include <mobot_controller/LocomotionActionFeedback.h>
#include <mobot_controller/LocomotionActionGoal.h>
#include <mobot_controller/LocomotionActionResult.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

using namespace std;

nav_msgs::Odometry current_state;
geometry_msgs::PoseStamped current_pose;
ros::ServiceClient client;

void currStateCallback(const nav_msgs::Odometry &odom)
{
    current_state = odom;
    current_pose.pose = current_state.pose.pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_coordinator");
    ros::NodeHandle n;

    ros::Subscriber current_state_sub = n.subscribe("/current_state", 1, currStateCallback);

    float x_t1 = 3.903 - 0.2;
    float y_t1 = 0.412;

    float x_t2 = 0.542; //0.572
    float y_t2 = 2.4;   

    float x_o = current_pose.pose.position.x;
    float y_o = current_pose.pose.position.y;

    ROS_INFO("STEP 1: Pre-table1");
    tryMove(x_t1/2, y_t1, 1);

    ROS_INFO("STEP 2: Docking table1");
    tryMove(x_t1, y_t1, 1);

    ros::Duration(4.0).sleep();

    ROS_INFO("Step 3: Backup from table 1");
    backUp();

    ROS_INFO("Step 4: Pre-table2");
    tryMove(x_t2, y_t2/2, 1);

    ROS_INFO("Step 5: Docking table2");
    tryMove(x_t2, y_t2, 1);

    ros::Duration(4.0).sleep();

    ROS_INFO("Step 6: Backup from table 2");
    backUp();

    ROS_INFO("Step 7: Going back to origin");
    tryMove(x_o, y_o,1);

    float x_e = current_pose.pose.position.x;
    float y_e = current_pose.pose.position.y;

    ROS_INFO("Step 8: Reorienting to original pose");
    tryMove(x_e + 0.001, y_e,1);

    ROS_INFO("Shutting down motor");
    // stop everything
    stop();

    ros::spin();

    return 0;
}