#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/conversions.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <hector_moveit_actions/ExecuteDroneTrajectoryAction.h>

#include <octomap/OcTree.h>


#define _USE_MATH_DEFINES
#include <cmath>
#include <queue>
#define XMIN -25
#define XMAX 25
#define YMIN -25
#define YMAX 25
#define ZMIN -25.0
#define ZMAX 25.0

#define EPSILON 1e-4
#define M_PI 3.14159265358979323846  /* pi */

typedef std::pair<double,geometry_msgs::Pose> DistancedPoint;

class Compare{
    public:
        bool operator()(DistancedPoint& lhs, DistancedPoint & rhs)
        {
            return lhs.first < rhs.first;
        }
};
typedef std::priority_queue<DistancedPoint,std::vector<DistancedPoint>, Compare> DistancedPointPriorityQueue;

#include <iostream>
#include <chrono>

using namespace std;
using  ns = chrono::nanoseconds;
using get_time = chrono::steady_clock;

#include <omp.h>
#define PATCH_LIMIT 1
class Quadrotor{
    private:
        std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
        actionlib::SimpleActionClient<hector_moveit_actions::ExecuteDroneTrajectoryAction> trajectory_client;
        std::unique_ptr<robot_state::RobotState> start_state;
        std::unique_ptr<planning_scene::PlanningScene> planning_scene;
        const double takeoff_altitude = 1.0;
        int GRID;
        bool odom_received,trajectory_received;
        bool isPathValid;
        bool collision;

        geometry_msgs::Pose odometry_information;
        std::vector<geometry_msgs::Pose> trajectory;

        std::vector<geometry_msgs::Pose> invalid_poses;
        std::vector<std::vector<int> > patches;
        std::queue<DistancedPoint> frontiers;
        std::vector<geometry_msgs::Pose> explored;

        ros::Subscriber base_sub,plan_sub;
        ros::Publisher gui_ack,rate_ack;
        ros::ServiceClient motor_enable_service;
        ros::ServiceClient planning_scene_service;

        moveit_msgs::RobotState plan_start_state;
        moveit_msgs::RobotTrajectory plan_trajectory;

        const std::string PLANNING_GROUP = "DroneBody";

        void poseCallback(const nav_msgs::Odometry::ConstPtr & msg);

        void planCallback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg);

        void collisionCallback(const hector_moveit_actions::ExecuteDroneTrajectoryFeedbackConstPtr& feedback);

        double countFreeVolume(const octomap::OcTree *octree);
        double calc_MI(const octomap::OcTree *octree, const geometry_msgs::Point& point, const octomap::Pointcloud &hits, const double before);
        octomap::Pointcloud castSensorRays(const octomap::OcTree* curr_tree,const geometry_msgs::Pose& pose);
        void findFrontier();


        bool go(geometry_msgs::Pose& target_);

    public:
        Quadrotor(ros::NodeHandle& nh);
        void takeoff();
        void run(double x, double y, double z, double roll, double pitch, double yaw);

};
