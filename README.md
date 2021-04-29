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

`rostopic pub locomotion_action_server/goal locomotion_action_server/LocomotionActionGoal` then tab to fill in the rest
## Perception Package
- Initialize the pcd/robot kinect
  
`roslaunch pcl_utils display_pcd_file` (no need if run jinx/merry)

- Perception services:

`rosrun detect_object detect_object_service`

`rosrun detect_object detect_table_wrt_cam_service`

- Testing command:

`rosservice call /detect_object_service`

`rosservice call /detect_table_wrt_cam_service`

`rosrun tf tf_echo block_frame torso`
## Manipulation Package


## Gripping Package