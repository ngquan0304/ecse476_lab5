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

#include <xform_utils/xform_utils.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <object_grabber/object_grabberAction.h>
#include <object_manipulation_properties/object_ID_codes.h>
#include<generic_gripper_services/genericGripperInterface.h>



using namespace std;

XformUtils xformUtils; //type conversion utilities

nav_msgs::Odometry current_state;
geometry_msgs::PoseStamped current_pose;
ros::ServiceClient client;
actionlib::SimpleActionClient<locomotion_action_server::LocomotionAction> *locomotion_action_ptr;
actionlib::SimpleActionClient<object_grabber::object_grabberAction> *object_grabber_ac_ptr;
ros::ServiceClient detect_block_client, capture_pcl_client;

geometry_msgs::PoseStamped object_pickup_poseStamped;
geometry_msgs::PoseStamped object_dropoff_poseStamped;

int g_object_grabber_return_code;

const double LOCO_TIME_OUT = 1.0;  //1.0;
const double MANI_TIME_OUT = 30.0;
const int STOP = 0;
const int MOTION = 1;
const int BACKUP = 2;

void currStateCallback(const nav_msgs::Odometry &odom)
{
    current_state = odom;
    current_pose.pose = current_state.pose.pose;
}

void objectGrabberDoneCb(const actionlib::SimpleClientGoalState& state,
        const object_grabber::object_grabberResultConstPtr& result) {
    ROS_INFO(" objectGrabberDoneCb: server responded with state [%s]", state.toString().c_str());
    g_object_grabber_return_code = result->return_code;
    ROS_INFO("got result output = %d; ", g_object_grabber_return_code);
}

//! locomotion code:
bool move(float x, float y){
    locomotion_action_server::LocomotionGoal goal;
    goal.mode = MOTION;
    goal.position_x = x;
    goal.position_y = y;
    locomotion_action_ptr->sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = locomotion_action_ptr->waitForResult(ros::Duration(LOCO_TIME_OUT));

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
    bool finished_before_timeout = locomotion_action_ptr->waitForResult(ros::Duration(LOCO_TIME_OUT));

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
    bool finished_before_timeout = locomotion_action_ptr->waitForResult(ros::Duration(LOCO_TIME_OUT));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = locomotion_action_ptr->getState();
        ROS_INFO("Stop Success?: %s",state.toString().c_str());
    }
    else
    ROS_INFO("Stop motion did not finish before the time out.");

    return true;
}

//! perception code: find and publish block transform using the service node
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

void get_block_frame_wrt_torso(geometry_msgs::PoseStamped &object_poseStamped)
{
    bool tferr = true;
    int ntries = 0;
    tf::TransformListener tfListener;
    tf::StampedTransform block_frame_wrt_torso_stf;

    while (tferr)
    {
        tferr = false;
        try
        {
            tfListener.lookupTransform("torso", "block_frame", ros::Time(0), block_frame_wrt_torso_stf); // was rgb
        }
        catch (tf::TransformException &exception)
        {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
            ntries++;
            if (ntries > 5)
            {
                ROS_WARN("did you launch detect_object service?");
                ros::Duration(1.0).sleep();
            }
        }
    }
    ROS_INFO("tf is good for table w/rt camera");
    xformUtils.printStampedTf(block_frame_wrt_torso_stf);

    object_poseStamped.header.frame_id = "torso"; //set object pose; ref frame must be connected via tf
    object_poseStamped.pose.position.x = block_frame_wrt_torso_stf.getOrigin().x();//0.634;//0.527;
    object_poseStamped.pose.position.y = block_frame_wrt_torso_stf.getOrigin().y();//-0.002;//-0.439;
    object_poseStamped.pose.position.z = block_frame_wrt_torso_stf.getOrigin().z() + 0.02; //-0.179;//-0.14; //
    object_poseStamped.pose.orientation.x = block_frame_wrt_torso_stf.getRotation().x();//0.0; //block x-axis parallel to torso y axis
    object_poseStamped.pose.orientation.y = block_frame_wrt_torso_stf.getRotation().y();//0.305;//0.0;
    object_poseStamped.pose.orientation.z = block_frame_wrt_torso_stf.getRotation().z();//0.010;//0.707;
    object_poseStamped.pose.orientation.w = block_frame_wrt_torso_stf.getRotation().w();//0.009;//0.707;
    object_poseStamped.header.stamp = ros::Time(0);

    ROS_INFO("Received block's location and orientaion");

    cout << object_poseStamped << endl;
}

//! manipulation code:
bool grab_ac_send_test_code(object_grabber::object_grabberGoal& object_grabber_goal,double MANI_TIME_OUT)
{
    bool finished_before_timeout;
    ROS_INFO("sending test code: ");
    object_grabber_goal.action_code = object_grabber::object_grabberGoal::TEST_CODE;
    object_grabber_ac_ptr->sendGoal(object_grabber_goal, &objectGrabberDoneCb);
    finished_before_timeout = object_grabber_ac_ptr->waitForResult(ros::Duration(MANI_TIME_OUT));
    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result ");
        return false;
    }
    else return true;

}

bool grab_ac_move_to_wating_pose(object_grabber::object_grabberGoal& object_grabber_goal,double MANI_TIME_OUT)
{
    bool finished_before_timeout;
    //move to waiting pose
    ROS_INFO("sending command to move to waiting pose");
    object_grabber_goal.action_code = object_grabber::object_grabberGoal::MOVE_TO_WAITING_POSE;
    object_grabber_ac_ptr->sendGoal(object_grabber_goal, &objectGrabberDoneCb);
    finished_before_timeout = object_grabber_ac_ptr->waitForResult(ros::Duration(MANI_TIME_OUT));
    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result ");
        return false;
    }
    else return true;
}

bool grab_ac_grab_object_cmd(object_grabber::object_grabberGoal& object_grabber_goal,double MANI_TIME_OUT)
{
    bool finished_before_timeout;
    ROS_INFO("sending a grab-object command");
    object_grabber_goal.action_code = object_grabber::object_grabberGoal::GRAB_OBJECT; //specify the action to be performed 
    object_grabber_goal.object_id = ObjectIdCodes::TOY_BLOCK_ID; // specify the object to manipulate                
    object_grabber_goal.object_frame = object_pickup_poseStamped; //and the object's current pose
    object_grabber_goal.grasp_option = object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY; //from above
    object_grabber_goal.speed_factor = 1.0;
    ROS_INFO("sending goal to grab object: ");
    object_grabber_ac_ptr->sendGoal(object_grabber_goal, &objectGrabberDoneCb);
    ROS_INFO("waiting on result");
    finished_before_timeout = object_grabber_ac_ptr->waitForResult(ros::Duration(MANI_TIME_OUT));

    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result ");
        return false;
    }
    else return true;
}

bool grab_ac_drop_object_cmd(object_grabber::object_grabberGoal& object_grabber_goal,double MANI_TIME_OUT)
{
    bool finished_before_timeout;
    ROS_INFO("sending a dropoff-object command");
    object_grabber_goal.action_code = object_grabber::object_grabberGoal::DROPOFF_OBJECT; //specify the action to be performed 
    object_grabber_goal.object_id = ObjectIdCodes::TOY_BLOCK_ID; // specify the object to manipulate                
    object_grabber_goal.object_frame = object_dropoff_poseStamped; //and the object's current pose
    object_grabber_goal.grasp_option = object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY; //from above
    object_grabber_goal.speed_factor = 1.0;
    ROS_INFO("sending goal to dropoff object: ");
    object_grabber_ac_ptr->sendGoal(object_grabber_goal, &objectGrabberDoneCb);
    ROS_INFO("waiting on result");
    finished_before_timeout = object_grabber_ac_ptr->waitForResult(ros::Duration(MANI_TIME_OUT));

    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result ");
        return false;
    }
    else return true;
}

// ==================================================================================================


int main(int argc, char **argv)
{
    ros::init(argc, argv, "top_commander");
    ros::NodeHandle n;

    int object_id = ObjectIdCodes::TOY_BLOCK_ID; //choose object of interest

    object_grabber::object_grabberGoal object_grabber_goal;


    ros::Subscriber current_state_sub = n.subscribe("/current_state", 1, currStateCallback);

    // Locomotion Action Server
    actionlib::SimpleActionClient<locomotion_action_server::LocomotionAction> locomotion_action("locomotion_action", true);
    locomotion_action_ptr = &locomotion_action;
    actionlib::SimpleActionClient<object_grabber::object_grabberAction> object_grabber_ac("object_grabber_action_service", true);
    object_grabber_ac_ptr = &object_grabber_ac;

    // Perception Service
    capture_pcl_client = n.serviceClient<std_srvs::Empty>("capture_pcl_service");
    detect_block_client = n.serviceClient<detect_object::DetectTransformServiceMsg>("detect_object_service");

    detect_object::DetectTransformServiceMsg srv;
    std_srvs::Empty emptysrv;


    // ! -----------------------------------------------------------------------
    // !              SERVICEs AND ACTION SERVERs INITIALIZATION
    // ! -----------------------------------------------------------------------


    ROS_INFO("Created services");
    ROS_INFO("Waiting for services to start");
    detect_block_client.waitForExistence();
    capture_pcl_client.waitForExistence();

    ROS_INFO("Created action client");
    ROS_INFO("Waiting for action server to start.");

    // wait for the action server to start
    locomotion_action.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, executing locomotion.");

    ROS_INFO("Waiting for object_grabber server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = object_grabber_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_grabber action server"); // if here, then we connected to the server; 
    

    //! -----------------------------------------------------------------------


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

    // locate the block on the table n publish as tf_transform    
    find_block(emptysrv,srv);
    // get the block frame wrt torso from tf tree
    get_block_frame_wrt_torso(object_pickup_poseStamped);
    object_dropoff_poseStamped = object_pickup_poseStamped;

    // manipulation
    
    if (!grab_ac_send_test_code(object_grabber_goal, MANI_TIME_OUT)) return 1;

    if (!grab_ac_move_to_wating_pose(object_grabber_goal, MANI_TIME_OUT)) return 1;

    if (!grab_ac_grab_object_cmd(object_grabber_goal, MANI_TIME_OUT)) return 1;

    if (!grab_ac_move_to_wating_pose(object_grabber_goal, MANI_TIME_OUT)) return 1;


    ROS_INFO("Step 3: Backup from table 1");
    backup();

    ROS_INFO("Step 4: Pre-table2");
    move(x_t2, y_t2/2);

    ROS_INFO("Step 5: Docking table2");
    move(x_t2, y_t2);

    // locate the table plane?

    // manipulation
    if (!grab_ac_move_to_wating_pose(object_grabber_goal, MANI_TIME_OUT)) return 1;

    if (!grab_ac_drop_object_cmd(object_grabber_goal, MANI_TIME_OUT)) return 1;

    if (!grab_ac_move_to_wating_pose(object_grabber_goal, MANI_TIME_OUT)) return 1;

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