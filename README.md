# OrchardDrone

# Prerequisites (click for installation guide)
  - [ROS Kinetic Installation](#ROS-Kinetic-Installation)
  - [Gazebo 7.16.1 Installation](#Gazebo-Installation)
  - [hector_quadrotor package installation](#hector_quadrotor-installation)
  - [MoveIt installation](#MoveIt-installation)
  - [Octomap Installation](#Octomap-installation)
  
## ROS Kinetic Installation

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


```c

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
Download both octomap_mapping, and octomap_server from github

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
