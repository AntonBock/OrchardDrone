# OrchardDrone

# Prerequisites (click for installation guide)
  - [ROS Kinetic Installation](#ROS-Kinetic-Installation)
  - [Gazebo 7.16.1 Installation](#Gazebo-7.16.1-Installation)
  - [hector_quadrotor package installation](#hector_quadrotor-installation)
  - [MoveIt installation](#MoveIt-installation)
  
## ROS Kinetic Installation

## Gazebo 7.16.1 Installation
NOTE this will install a debian version of Gazebo
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
