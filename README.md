## Full code run

---
only for sim:
    
    roslaunch detect_object real_robot_sim.launch 

run: 
    
    rviz

    rosrun map_server map_server [map]

    roslaunch detect_object launch_detect_object_nodes_real_robot.launch 

    (roslaunch detect_object launch_detect_object_nodes_sim_robot.launch)

    roslaunch locomotion_action_server launch_locomotion_nodes.launch 

    roslaunch object_grabber baxter_object_grabber_nodes.launch

    rosrun jinx_merry_commander top_commander

debugging cmd:

    rosservice call /capture_pcl_service

    rosservice call /detect_pcl_service

    rosrun baxter_playfile_nodes baxter_playback_pre_pose_right.jsp

    rosrun pcl_utils display_pcd_file

---
## To run top_commander:
- Must source to learning_ros before catkin_make
- Run locomotion launch file:
  
`roslaunch locomotion_action_server launch_locomotion_nodes.launch`

- Run perception launch file:

`roslaunch detect_object launch_detect_object_nodes.launch`

- Run top_commander:

`rosrun jinx_merry_commander top_commander`

---
## Locomotion Package
- Initialize robot/sim and maps:

`roslaunch mobot_urdf mobot_in_pen.launch`  (no need if run jinx/merry)

`rosrun map_server map_server starting_pen_map.yaml`

or

`rosrun map_server map_server lab_map.yaml`

- Locomotion nodes:

`rosrun amcl amcl`

`rosrun locomotion_action_server current_state_publisher`

`rosrun locomotion_action_server des_state_publisher_service`

`rosrun locomotion_action_server lin_steering_wrt_amcl_and_odom`

`rosrun locomotion_action_server locomotion_action_server`

- Testing command:

`rostopic pub locomotion_action_server/goal locomotion_action_server/LocomotionActionGoal` 

then tab to fill in the rest
## Perception Package
- Setup launch files:
  - For real Baxter:
    
    `roslaunch detect_object launch_detect_object_nodes_real_robot.launch`

  - For sim Baxter:

    `roslaucnh detect_object real_robot_sim.launch`

    `roslaunch detect_object launch_detect_object_nodes_sim_robot.launch`

- Initialize the pcd/robot kinect

    `roslaunch pcl_utils display_pcd_file` (no need if run jinx/merry)

- Testing command:

    `rosservice call /detect_object_service`

    `rosservice call /detect_table_wrt_cam_service`

- Check for block position and orientation

    `rosrun tf tf_echo block_frame torso`

## Object Grabber Client

---


