#include <ros/ros.h>
#include <sstream>
#include <tf/tf.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <hector_uav_msgs/PoseAction.h>
#include <hector_uav_msgs/EnableMotors.h>

#include <cmath>

ros::Publisher pub;

std::string world = "world";
double z = 1.65;
std::vector<std::vector<double>> cord{{-1.05,	2.45, z, 0.707, 0.707},
		{1.8,	2.45, z, 0.707, 0.707},
		{5.8,	2.45, z, 0.707, 0.707},
		{9.8,	2.45, z, 0.707, 0.707},
		{13.8,	2.45, z, 0.707, 0.707},
		{17.8,	2.45, z, 0.707, 0.707},
		{20.65,	2.45, z, 0.707, 0.707},
		{20.65,	7.5,  z, -0.707, 0.707},
		{17.8,	7.45, z, -0.707, 0.707},
		{13.8,	7.45, z, -0.707, 0.707},
		{9.8,	7.45, z, -0.707, 0.707},
		{5.8,	7.45, z, -0.707, 0.707},
		{1.8,	7.45, z, -0.707, 0.707},
		{-1.05,	7.5,  z, -0.707, 0.707},
		{-1.05,	12.5, z, 0.707, 0.707},
		{1.8,	12.45,z, 0.707, 0.707},
		{5.8,	12.45,z, 0.707, 0.707},
		{9.8,	12.45,z, 0.707, 0.707},
		{13.8,	12.45,z, 0.707, 0.707},
		{17.8,	12.45,z, 0.707, 0.707},
		{20.65,	12.5, z, 0.707, 0.707},
		{20.65,	17.55,z, -0.707, 0.707},
		{17.8,	17.55,z, -0.707, 0.707},
		{13.8,	17.55,z, -0.707, 0.707},
		{9.8,	17.55,z, -0.707, 0.707},
		{5.8,	17.55,z, -0.707, 0.707},
		{1.8,	17.55,z, -0.707, 0.707},
		{-1.05,	17.55,z, 0.707, 0.707}};

hector_uav_msgs::PoseActionGoal tempGoal;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "between_trees");
  ros::NodeHandle nh;

  ros::Rate loop_rate(10);

  while(ros::ok())
  {

for(int i = 0; i < cord.size(); i++){
    tempGoal.goal.target_pose.header.frame_id = world;
    tempGoal.goal.target_pose.header.stamp = ros::Time::now();
    tempGoal.goal.target_pose.pose.position.x = cord[i][0];
    tempGoal.goal.target_pose.pose.position.y = cord[i][1];
    tempGoal.goal.target_pose.pose.position.z = cord[i][2];
    tempGoal.goal.target_pose.pose.orientation.w = cord[i][3];
    tempGoal.goal.target_pose.pose.orientation.z = cord[i][4];
    pub = nh.advertise<hector_uav_msgs::PoseActionGoal>("/action/pose/goal", 1);
    pub.publish(tempGoal);
    ros::spinOnce();
    loop_rate.sleep();
    ros::Duration(5.0).sleep();
    }
  }
  ROS_INFO("DONE");
  return 0;
}
