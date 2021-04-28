#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <mobot_controller/ServiceMsg.h>
#include <mobot_controller/LocomotionAction.h>
#include <mobot_controller/LocomotionFeedback.h>
#include <mobot_controller/LocomotionResult.h>
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

class LocomotionAction {
    private:
        ros::NodeHandle _node_handle; // Node handle

        // *** Action Control (Interface/Master) ***
        // Action Server (Goal, Feedback, Result)
        std::string _action_name; // Action Server's Name
        actionlib::SimpleActionServer<mobot_controller::LocomotionAction> _action_server; // Action Server
        mobot_controller::LocomotionFeedback _feedback; // Feedback
        mobot_controller::LocomotionResult _result; // Result

        // *** Motion Control (Subcomponent/Slaves) *** 
        // Desired State
        ros::ServiceClient desired_state_client = _node_handle.serviceClient<mobot_controller::ServiceMsg>("des_state_publisher_service");

        // Current State is outside the singleton

        // Number of times to retry motion commands (if they fail)
        const int DEFAULT_RETRY_MOVES = 3;
    
    public:

        LocomotionAction(std::string name) : _action_server(_node_handle, name, boost::bind(&LocomotionAction::executeActionCallback, this, _1), false), _action_name(name)
        {
            _action_server.start(); // Start Action Server
        }

        ~LocomotionAction(void){}

        void executeActionCallback(const mobot_controller::LocomotionGoalConstPtr &goal_pos)
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

        // Take in a goal (x,y) and issue desired state, service calls, to command the robot's motion
        bool move2coord(float goal_pose_x, float goal_pose_y)
        {
            // Trajectory Builder
            TrajBuilder trajBuilder;
            
            // Variables for Constructing Desired State Service Message
            mobot_controller::ServiceMsg srv; // Desired State's Service Message (start_pos, goal_pos, mode -> success?)
            geometry_msgs::PoseStamped start_pose, goal_pose_trans, goal_pose_rot; // Desired States' Service Messages (Request portion)
            std::string mode; // Operational Mode (Spin, Forward, Halt)
            
            // Tracking state (successful operation?)
            bool success = true, success_rotate, success_translate;

            start_pose.pose = current_state.pose.pose; // Initialize request's start pose to current state's pose

            // Start x,y
            // For now: rotate to head forward to goal point, then move toward the place.
            double x_start = start_pose.pose.position.x;
            double y_start = start_pose.pose.position.y;
            
            // Goal x,y
            double x_end = goal_pose_x;
            double y_end = goal_pose_y;
            
            // Change in x,y
            double dx = x_end - x_start;
            double dy = y_end - y_start;

            // Desired rotation
            double des_psi = atan2(dy, dx);
            
            // Output some information for the user
            ROS_INFO("start_x = %f, start_y = %f, goal_x = %f, goal_y = %f", x_start, y_start, x_end, y_end);

            // Construct a viable rotation
            // Keep x,y the same; rotate to des_psi
            goal_pose_rot = trajBuilder.xyPsi2PoseStamped(current_pose.pose.position.x, current_pose.pose.position.y, des_psi);
            
            // Start Constructing the Service's Request message
            srv.request.start_pos = current_pose;
            srv.request.goal_pos = goal_pose_rot;
            srv.request.mode = "2"; // Mode 2 (SPIN) - rotate toward the goal.

            // If Desired State Service returns a successful response:
            if (desired_state_client.call(srv))
            {
                success_rotate = srv.response.success;
                ROS_INFO("rotate success? %d", success_rotate);
            }
            ros::spinOnce(); // Update


            // *** FORWARD MOTION ***
            // Plan a forward motion
            goal_pose_trans = trajBuilder.xyPsi2PoseStamped(goal_pose_x, goal_pose_y, des_psi); // Preserve des_psi (rotation), change x,y

            // Start Constructing the Service's Request message
            srv.request.start_pos = goal_pose_rot;
            srv.request.goal_pos = goal_pose_trans;
            srv.request.mode = "1"; // Mode 1 (FORWARD) - head toward the goal.
            
            // If Desired State Service returns a successful response:
            if (desired_state_client.call(srv))
            {
                success_translate = srv.response.success;
                ROS_INFO("translate success? %d", success_translate);
            }
            ros::spinOnce(); // Update

            // If Desired State Service returns a un-successful response
            if (!success_translate)
            {
                ROS_INFO("Cannot move, obstacle. braking");

                // Construct a new request (stay at current position)
                srv.request.start_pos = current_pose;
                srv.request.goal_pos = current_pose; //anything is fine.
                srv.request.mode = "3"; // Mode 3 (HALT) - stop motion
                desired_state_client.call(srv);
                success = false;
            }
            ros::spinOnce();

            return success;
        }

        // Stop motion of the robot
        bool stop(){
            mobot_controller::ServiceMsg srv;
            srv.request.start_pos = current_pose;
            srv.request.goal_pos = current_pose;
            srv.request.mode = "0"; 
            if (desired_state_client.call(srv))
            {
                ROS_INFO("Stopped");
            }
            return true;
        }

        // Move the robot backward
        bool backUp()
        {
            ROS_INFO("Backing up");
            TrajBuilder trajBuilder;
            mobot_controller::ServiceMsg srv;
            geometry_msgs::PoseStamped start_pose;

            start_pose.pose = current_state.pose.pose;

            srv.request.start_pos = current_pose;
            srv.request.goal_pos = current_pose;
            srv.request.mode = "4"; // spin so that head toward the goal.
            if (desired_state_client.call(srv))
            {
                bool success_backup = srv.response.success;
                ROS_INFO("rotate success? %d", success_backup);
            }
            //ros::spinOnce();
            return true;
        }

        // Function to calculate distance to the goal
        float distance_to_goal(geometry_msgs::PoseStamped current_pose, float goal_position_x, float goal_positon_y){
            return distance((int) current_pose.pose.position.x, (int) current_pose.pose.position.y, (int) goal_position_x, (int) goal_positon_y);
        }

        // Function to calculate distance
        float distance(int x1, int y1, int x2, int y2)
        {
            return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0); // Calculating distance
        }
};

// Callback function for the current state subscriber (do something when the current state publisher publishes - and we hear it)
void currentStateCallback(const nav_msgs::Odometry &odom)
{
    current_state = odom;
    current_pose.pose = current_state.pose.pose;
}

//Action Server
int main(int argc, char** argv)
{
    ros::init(argc, argv, "locomotion_action_server");
    ros::NodeHandle n;
    ros::Subscriber current_state_sub = n.subscribe("/current_state", 1, currentStateCallback);
    LocomotionAction LocomotionAction("locomotion_action_server");
    ros::spin();
    
    return 0;
}