# Packages overview

## hector_quadrotor
The `hector_quadrotor` package and its subfolders provide a model, control and simulation of the hector_quadrotor drone.

## MoveIt
* `hector:moveit_actions> `: Determine and execute trajectory
* `hector:moveit_config> `: Configure the drone with MoveIt
* `hector:moveit_exploration> `: Do we want to mention this 1?
* `hector:moveit_gazebo> `: Launcing MoveIt in Gazebo
* `hector:moveit_gazebo_worlds> `: World files for Gazebo
* `hector:moveit_move_to_goal> `: Send goal positions to the drone using the action server /action/pose

## Mapping
* `load_octomap> `: Functionality of loading a saved .bt file
* `maps > `: Contains the saved .bt files
* `octomap_mapping > `: ROS stack for mapping with OctoMap

## wayPoints
* `betweenTrees> `: Send goal positions to fly between the trees (after mapping)
* `send_waypoints > `: Send goal positions to fly above the trees (used for performing mapping)







