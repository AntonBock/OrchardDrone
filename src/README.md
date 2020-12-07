# Packages overview

## `hector_quadrotor`: model, control and simulation of the hector_quadrotor drone

## MoveIt
* `hector:moveit_actions> `: Determine and execute trajectory
* `hector:moveit_config> `: Configure the drone with MoveIt
* `hector:moveit_actions> `: Do we want to mention this 1?
* `hector:moveit_gazebo> `: Launcing MoveIt in Gazebo
* `hector:moveit_gazebo_worlds> `: World files for Gazebo
* `hector:moveit_move_to_goal> `: Send goal positions to the drone using the action serve /action/pose

## Mapping
* `load_octomap> `: Functionality of loading a saved .bt file
* `maps > `: Containts the saved .bt files
* `octomap_mapping > `: ROS stack for mapping with OctoMap

## wayPoints
* `load_octomap> `: Functionality of loading a saved .bt file
* `maps > `: Containts the saved .bt files
* `octomap_mapping > `: ROS stack for mapping with OctoMap






mapping -> load_octomap

mapping -> maps

mapping -> octomap_mapping


moveit -> hector_moveit_config

moveit -> ...

waypoints -> betweenTrees
