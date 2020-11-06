# OrchardDrone

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



## hector_quadrotor installation
Pretty standard, only "special thing" is to remember to use rosdep:

Go to your workspace (replace to fit your own src folder)
```c
cd /path/to/your/catkin_ws/src
``` 

Clone the hector_quadrotor package to your workspace
```c
git clone https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor.git
``` 


Go one level up (base level of ws)
```c
cd ..
``` 

Update
```c
rosdep update
``` 

Install dependencies (replace "kinetic" with your distribution, if you for some reason use anything else)
```c
rosdep install --from-paths src/ --ignore-src --rosdistro kinetic
``` 

Install further dependencies
```c
rosinstall . hector_quadrotor.rosinstall
``` 

You can now catkin_make your project
```c
catkin_make
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

### Start use
```c
roslaunch moveit_setup_assistant setup_assistant.launch
``` 

Follow RobotIgnite course: Programming drones with ROS - 3D navigation with MoveIt


## Octomap installation
Download both octomap_mapping from github

```c
git clone https://github.com/OctoMap/octomap_mapping
```

Install dependencies
```c 
sudo apt-get install ros-kinetic-octomap ros-kinetic-octomap-mapping ros-kinetic-octomap-msgs ros-kinetic-octomap-ros ros-kinetic-octomap-rviz-plugins ros-kinetic-octomap-server
```


cmake

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

## tum_ardrone package
In ws/src
```c
git clone https://bitbucket.org/theconstructcore/tum_ardrone_sim.git
```

Go into tum_ardrone_sim/tum-simulator-indigo, and remove the action_controller folder.

### To install ground plane and oak_tree models in Gazebo:
1\. Download the folder "Gazebo Models" from drive (Project/Simulation)
2\. In terminal write the following:

```c
cd .gazebo/models/
nautilus .
```

3\. Place the folders "my_ground_plane" and "oak_tree" inside the Models folder which will appear in Nautilus. 
4\. Accept and replace all files of oak_tree (backup of old files is within the one downloaded from Drive)


## Fix

Hector, move away from package, cmake.


packages you need:

```c
https://github.com/wilselby/ROS_quadrotor_simulator
(maybe) https://github.com/ros/common_msgs/tree/jade-devel (INSTALL BY writing "svn co https://github.com/ros/common_msgs.git")
git clone https://bitbucket.org/theconstructcore/tum_ardrone_sim.git
```

in package /catkin_ws/src/tum_ardrone_sim/tum_ardrone/src/UINode
RosThread.h, 

add around the #includes:
```c
#ifndef Q_MOC_RUN
 
#endif
```

in package ROS_quadrotor_simulator, you only need "action_controller" and "moveit_simple_controller_manager". Move these folders to your ws, the rest you can keep somewhere else. 

Now open the cmake lists in those two packages. In action_controller cmake, copy and add this under target_link_libraries:

```c
add_dependencies(action_controller action_controller_generate_messages_cpp)
```

in moveit_simple_controller_manager replace cmake with this:

```c
cmake_minimum_required(VERSION 2.8.3)
project(moveit_simple_controller_manager)

add_definitions(-std=c++11)
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread)
link_directories(${Boost_LIBRARY_DIRS})

find_package(catkin COMPONENTS
  moveit_core
  pluginlib
  actionlib
  roscpp
  control_msgs
  REQUIRED action_controller)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
link_directories(${catkin_LIBRARY_DIRS})

generate_messages(
  DEPENDENCIES
  action_controller
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  DEPENDS
    moveit_core
)

include_directories(include)

add_library(moveit_simple_controller_manager src/moveit_simple_controller_manager.cpp)
add_dependencies(moveit_simple_controller_manager ${catkin_EXPORTED_TARGETS})
target_link_libraries(moveit_simple_controller_manager ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS moveit_simple_controller_manager LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES moveit_simple_controller_manager_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
       )
```

go to tum_ardr

Move hector back to package, cmake
