## Locomotion Package
- Initialize robot/sim and maps:

`roslaunch mobot_urdf mobot_in_pen.laucnh`  (no need if run jinx/merry)

`rosrun map_server map_server starting_pen_map.yaml`

or

`rosrun map_server map_server lab_map.yaml`

- Locomotion nodes:

`rosrun amcl amcl`

`rosrun mobot_controller current_state_publisher`

`rosrun mobot_controller des_state_publisher_service`

`rosrun mobot_controller lin_steering_wrt_amcl_and_odom`

`rosrun mobot_controller locomotion_action_server`

- Testing command:

`rostopic pub locomotion_action_server/goal mobot_controller/LocomotionActionGoal` then tab to fill in the rest
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