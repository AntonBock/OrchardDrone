# OrchardDrone
This project focuses on mapping and data gathering of trees at an orchard using a drone. The project is simulated using Gazebo 7.16.1, ROS kinetic. 
In this repository we provide a simple model of a rectangular orchard in which there is 15 similar tree for the drone to fly among and perform mapping. The drone is a simulated hector_quadrotor equipped with a ASUS xtion pro live RGB-D camera. The mapping produces a 3D occupancy grid map in which we can use MoveIt to perform 3D obstacle avoidance and plan paths between waypoints.



# Prerequisites (click for installation guide)
  - [ROS Kinetic Installation](#ROS-Kinetic-Installation)
  - [Gazebo 7.16.1 Installation](#Gazebo-Installation)
  - [hector_quadrotor package installation](#hector_quadrotor-installation)
  - [MoveIt installation](#MoveIt-installation)
  - [Octomap Installation](#Octomap-installation)
  - [Fix 05/11](#Fix)
  
## ROS Kinetic Installation
http://wiki.ros.org/kinetic/Installation/Ubuntu

## Gazebo Installation
NOTE this will install a debian version of Gazebo 7.16.1
First remove the old version of Gazebo
```c
sudo apt-get remove *gazebo*
``` 


Next, setup to allow for installation from osrfounation.org
```c
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' 
``` 

```c
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
``` 

Update
```c
sudo apt-get update
``` 

Install the latest version of Gazebo 7.16
```c
sudo apt-get install gazebo7=7.16*
``` 


Install additional packages manually
```c
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
``` 


Add some extra math files: 
Navigate to the correct folder, and add the math files found in the math folder in this package
```c
sudo nautilus /usr/include/ignition/math2/ignition/math
```   

## MoveIt installation
### Binary installation:
```c
sudo apt-get install ros-kinetic-moveit 
``` 

### Source installation:
```c
wstool init src
``` 


```c
wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
``` 


```c
wstool update -t src
``` 


```c
rosdep install -y --from-paths src --ignore-src --rosdistro kinetic
``` 


```c
catkin config --extend /opt/ros/kinetic --cmake-args -DCMAKE_BUILD_TYPE=Release 
``` 

## Octomap installation
Download both octomap_mapping from github

```c
git clone https://github.com/OctoMap/octomap_mapping
```

Install dependencies
```c 
sudo apt-get install ros-kinetic-octomap ros-kinetic-octomap-mapping ros-kinetic-octomap-msgs ros-kinetic-octomap-ros ros-kinetic-octomap-rviz-plugins ros-kinetic-octomap-server
```

```c
catkin_make
```

navigate to octomap_mapping/octomap_server/launch

open octomap_mapping.launch in editor

Make sure frame_id and cloud_in are set correctly. Our standard should be "world" and "/camera/depth/points":

<param name="frame_id" type="string" value="world" />
<remap from="cloud_in" to="/camera/depth/points" />

You might want to edit the resolution as well.

Now launch simulation+RViz.
In second window, launch octomapping: roslaunch octomap_server octomap_mapping.launch

When done mapping, save octomap by writing rosrun octomap_server octomap_saver -f NameOfOctomap.bt

Saving can (maybe?) be done after stopping the mapping process in the terminal


When loading in RViz, you may need to restart RViz. use MarkerArray and occupied_cells_vis_array

When making the load_octomap.py, give executable permission

## How to install ground plane and oak_tree models in Gazebo:
1\. Download the folder "Gazebo Models" from drive (Project/Simulation)
2\. In terminal write the following:

```c
cd .gazebo/models/
nautilus .
```

3\. Place the folders "my_ground_plane" and "oak_tree" inside the Models folder which will appear in Nautilus. 
4\. Accept and replace all files of oak_tree (backup of old files is within the one downloaded from Drive)

# How to launch
Remember to give executable rights to your files.
```
cd **desired folder** 
```

```
sudo chmod +x nameOfFile
```

## Launching simulation for creating a 3D occupancy grid map using OctoMap
Launch Gazebo and Rviz with a hector_quadrotor
```
roslaunch hector_quadrotor_demo basicForest_zeroed_flying_hector_withLaser.launch
```  
In another terminal window launch octomapping
```
roslaunch octomap_server octomap_mapping.launch
```
Send waypoints to the drone to get full 2D coverage
```
rosrun sendwaypoints sendwaypoints_node
```

When the mapping is done, save the 3D occupancy grid map by running
```
rosrun octomap_server octomap_saver -f /home/User/workspaceName/src/mapping/maps/NameOfOctomap.bt
```
/home/ros/robotignite_ws/src/mapping/maps/NameOfOctomap.bt

## Launching simulation with octomap for data gathering
Launch Gazebo and Rviz with a hector_quadrotor
```
 roslaunch hector_moveit_gazebo orchardyard_navigation.launch
```  
Load the octomap, the exact path to the .bt file must be specified within the load_octomap.launch
```  
roslaunch load_octomap load_octomap.launch
```  
Send waypoints to the drone
```  
 roslaunch hector_moveit_move_to_goal move_to_goal.launch
```  

## Acknoweledgements and references
The following resources has been widely used by the authors of this repository and to them we extend our sincere gratitude.

https://github.com/wilselby/ROS_quadrotor_simulator 

https://github.com/tahsinkose/hector-moveit

https://github.com/AlessioTonioni/Autonomous-Flight-ROS

https://github.com/mSimon12/multiple_quadrotors





