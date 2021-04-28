#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <mobot_controller/ServiceMsg.h>
#include <mobot_controller/ArmMotionAction.h>
#include <mobot_controller/ArmMotionFeedback.h>
#include <mobot_controller/ArmMotionResult.h>
#include <traj_builder/traj_builder.h>
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <mobot_controller/ServiceMsg.h>
#include <traj_builder/traj_builder.h>

using namespace std;

// Global current state and current pose (updated by current state subscriber callback)
nav_msgs::Odometry current_state; // Current State (odom)
geometry_msgs::PoseStamped current_pose; // Current Position

class ArmMotionAction {
    private:
        ros::NodeHandle _node_handle; // Node handle

        // *** Action Control (Interface/Master) ***
        // Action Server (Goal, Feedback, Result)
        std::string _action_name; // Action Server's Name
        actionlib::SimpleActionServer<mobot_controller::ArmMotionAction> _action_server; // Action Server
        mobot_controller::ArmMotionFeedback _feedback; // Feedback
        mobot_controller::ArmMotionResult _result; // Result

    public:

        ArmMotionAction(std::string name) : _action_server(_node_handle, name, boost::bind(&ArmMotionAction::executeActionCallback, this, _1), false), _action_name(name)
        {
            _action_server.start(); // Start Action Server
        }

        ~ArmMotionAction(void){}

        // https://github.com/wsnewman/learning_ros_noetic/blob/991c494ffcf29d30e6c40db0f49d88e7bf59513d/Part_5/baxter/baxter_playfile_nodes/src/baxter_playfile_service.cpp
        void executeActionCallback(const mobot_controller::ArmMotionGoalConstPtr &goal_pos)
        {
            ros::Rate r(1);
            bool success = false;

            _feedback.distance_to_goal = distance_to_goal(current_pose, goal_pos->position_x, goal_pos->position_y);

            // publish info to the console for the user
            ROS_INFO("%s: Executing, locomotion to position: (%f,%f) -> distance to goal_pos (%f)", _action_name.c_str(), goal_pos->position_x, goal_pos->position_y, _feedback.distance_to_goal);

            // start executing the action
            for(int i=1; i<=DEFAULT_RETRY_MOVES; i++)
            {

                // check that preempt has not been requested by the client
                if (_action_server.isPreemptRequested() || !ros::ok())
                {
                    ROS_INFO("%s: Preempted", _action_name.c_str());
                    // set the action state to preempted
                    _action_server.setPreempted();
                    success = false;
                    break;
                }

                // If we have yet to be successful - calculate distance to the goal_pos
                if(success == false)
                {
                    _feedback.distance_to_goal = distance_to_goal(current_pose, goal_pos->position_x, goal_pos->position_y);
                
                    _action_server.publishFeedback(_feedback); // publish the feedback

                    success = move2coord(goal_pos->position_x, goal_pos->position_y);

                    _feedback.distance_to_goal = distance_to_goal(current_pose, goal_pos->position_x, goal_pos->position_y);

                    _action_server.publishFeedback(_feedback); // publish the feedback
                }
            }

            // If successful, set the action server's message-result and print information for the user
            if(success)
            {
                _result.success = success;
                ROS_INFO("%s: Succeeded", _action_name.c_str());
                // set the action state to succeeded
                _action_server.setSucceeded(_result);
            }
        }

        // Play the JSP file
        void play(string filepath){
            //Have to figure out how to play the jsp from a file:
            //[Learning ROS ws] roslaunch baxter_gazeo baxter_world.launch
            //[Learning ROS ws] roslaunch baxter_launch_files baxter_playfile_nodes.launch
            //[This Repo ws] rosrun pcd_utils display_pcd_files then arm1.pcd
            //[This Repo ws] rosrun baxter_playfile_nodes baxter_playback arm1.jsp
            //System("rosrun baxter_playfile_nodes baxter_playback " + filepath); 
            
            // mobot_controller::ServiceMsg srv;
            // srv.request.start_pos = current_pose;
            // srv.request.goal_pos = current_pose;
            // srv.request.mode = "0"; 
            // if (desired_state_client.call(srv))
            // {
            //     ROS_INFO("Stopped");
            // }
        }

        bool difference(){
            //Compare between the angles in jsp file and angles of arm
        }

    
};

//Action Server
int main(int argc, char** argv)
{
    char filename[150];
    cout << "Enter full path for playfile: ";
    cin.getline(filename, 100);

    ros::init(argc, argv, "arm_motion_action_server");
    ArmMotionAction ArmMotionAction("arm_motion_action_server");
    ros::spin();

    
    return 0;
}