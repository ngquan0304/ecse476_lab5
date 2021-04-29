#include <ros/ros.h>
#include <iostream>
#include <string>
#include "detect_object/DetectTransformServiceMsg.h"
#include <locomotion_action_server/LocomotionAction.h>
#include <locomotion_action_server/LocomotionActionFeedback.h>
#include <locomotion_action_server/LocomotionActionGoal.h>
#include <locomotion_action_server/LocomotionActionResult.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <std_srvs/Empty.h>
using namespace std;

nav_msgs::Odometry current_state;
geometry_msgs::PoseStamped current_pose;
ros::ServiceClient client;
actionlib::SimpleActionClient<locomotion_action_server::LocomotionAction> *locomotion_action_ptr;
double TIME_OUT = 1.0;  //1.0;
ros::ServiceClient detect_block_client, capture_pcl_client;

const int STOP = 0;
const int MOTION = 1;
const int BACKUP = 2;

void currStateCallback(const nav_msgs::Odometry &odom)
{
    current_state = odom;
    current_pose.pose = current_state.pose.pose;
}

bool move(float x, float y){
    locomotion_action_server::LocomotionGoal goal;
    goal.mode = MOTION;
    goal.position_x = x;
    goal.position_y = y;
    locomotion_action_ptr->sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = locomotion_action_ptr->waitForResult(ros::Duration(TIME_OUT));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = locomotion_action_ptr->getState();
        ROS_INFO("Backup Success?: %s",state.toString().c_str());
    }
    else
    ROS_INFO("Backup motion did not finish before the time out.");

    return true;
}

bool backup(){
    locomotion_action_server::LocomotionGoal goal;
    goal.mode = BACKUP;
    locomotion_action_ptr->sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = locomotion_action_ptr->waitForResult(ros::Duration(TIME_OUT));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = locomotion_action_ptr->getState();
        ROS_INFO("Backup Success?: %s",state.toString().c_str());
    }
    else
    ROS_INFO("Backup motion did not finish before the time out.");

    return true;
}

bool stop(){
    locomotion_action_server::LocomotionGoal goal;
    goal.mode = STOP;
    locomotion_action_ptr->sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = locomotion_action_ptr->waitForResult(ros::Duration(TIME_OUT));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = locomotion_action_ptr->getState();
        ROS_INFO("Stop Success?: %s",state.toString().c_str());
    }
    else
    ROS_INFO("Stop motion did not finish before the time out.");

    return true;
}

// find and publish block transform using the service node
bool find_block(std_srvs::Empty &emptysrv, detect_object::DetectTransformServiceMsg &srv)
{
    ros::spinOnce();
    // Capture a snapshot of current pcl
    std_srvs::Empty cap_srv;
    if (capture_pcl_client.call(emptysrv))
    {
        ROS_INFO("Captured a snapshot of pointcloud");
    }
    else
    {
        ROS_ERROR("Failed to capture a snapshot of pointcloud");
        return false;
    }

    // Find the block
    if (detect_block_client.call(srv))
    {
        ROS_INFO("Sent detecting block request");
        if (srv.response.detect_success) return true;
        else return false;
    }
    else
    {
        ROS_ERROR("Failed to call detecting block service");
        return false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "top_commander");
    ros::NodeHandle n;

    ros::Subscriber current_state_sub = n.subscribe("/current_state", 1, currStateCallback);

    // Locomotion Action Server
    actionlib::SimpleActionClient<locomotion_action_server::LocomotionAction> locomotion_action("locomotion_action", true);
    locomotion_action_ptr = &locomotion_action;

    // Perception Service
    capture_pcl_client = n.serviceClient<std_srvs::Empty>("capture_pcl_service");
    detect_block_client = n.serviceClient<detect_object::DetectTransformServiceMsg>("detect_object_service");

    detect_object::DetectTransformServiceMsg srv;
    std_srvs::Empty emptysrv;

    ROS_INFO("Created services");
    ROS_INFO("Waiting for services to start");
    detect_block_client.waitForExistence();
    capture_pcl_client.waitForExistence();

    ROS_INFO("Created action client");
    ROS_INFO("Waiting for action server to start.");

    // wait for the action server to start
    locomotion_action.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, executing locomotion.");

    float x_t1 = 3.903 - 0.2;
    float y_t1 = 0.412;

    float x_t2 = 0.542; //0.572
    float y_t2 = 2.4;   

    float x_o = current_pose.pose.position.x;       //starting pose
    float y_o = current_pose.pose.position.y;       //starting pose

    ROS_INFO("STEP 1: Pre-table1");
    move(x_t1/2, y_t1);

    ROS_INFO("STEP 2: Docking table1");
    move(x_t1, y_t1);

    // locate the block on the table    
    find_block(emptysrv,srv);

    // manipulation

    ROS_INFO("Step 3: Backup from table 1");
    backup();

    ROS_INFO("Step 4: Pre-table2");
    move(x_t2, y_t2/2);

    ROS_INFO("Step 5: Docking table2");
    move(x_t2, y_t2);

    // locate the table plane?

    // manipulation


    ROS_INFO("Step 6: Backup from table 2");
    backup();

    ROS_INFO("Step 7: Going back to origin");
    move(x_o, y_o);

    ros::spinOnce(); // to get current pose

    float x_e = current_pose.pose.position.x;
    float y_e = current_pose.pose.position.y;

    ROS_INFO("Step 8: Reorienting to original pose");
    move(x_e + 0.001, y_e);

    ROS_INFO("Shutting down motor");
    // stop everything
    stop();

    return 0;
}