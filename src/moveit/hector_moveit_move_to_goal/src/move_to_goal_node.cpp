#include <move_to_goal.h>

//The following information is the waypoints use for sending to the drone.
double z = 1.65000;
std::vector<std::vector<double>> cord{{-1.05,	2.45, z, 0.0, 0.0, M_PI/2},
		{1.8,	2.45, z, 0.0, 0.0, M_PI/2},
		{5.8,	2.45, z, 0.0, 0.0, M_PI/2},
		{9.8,	2.45, z, 0.0, 0.0, M_PI/2},
		{13.8,	2.45, z, 0.0, 0.0, M_PI/2},
		{17.8,	2.45, z, 0.0, 0.0, M_PI/2},
		{20.65,	2.45, z, 0.0, 0.0, M_PI/2},
		{20.65,	7.5,  z, 0.0, 0.0, M_PI/2},
		{17.8,	7.45, z, 0.0, 0.0, M_PI/2},
		{13.8,	7.45, z, 0.0, 0.0, M_PI/2},
		{9.8,	7.45, z, 0.0, 0.0, M_PI/2},
		{5.8,	7.45, z, 0.0, 0.0, M_PI/2},
		{1.8,	7.45, z, 0.0, 0.0, M_PI/2},
		{-1.05,	7.5,  z, 0.0, 0.0, M_PI/2},
		{-1.05,	12.5, z, 0.0, 0.0, M_PI/2},
		{1.8,	12.45,z, 0.0, 0.0, M_PI/2},
		{5.8,	12.45,z, 0.0, 0.0, M_PI/2},
		{9.8,	12.45,z, 0.0, 0.0, M_PI/2},
		{13.8,	12.45,z, 0.0, 0.0, M_PI/2},
		{17.8,	12.45,z, 0.0, 0.0, M_PI/2},
		{20.65,	12.5, z, 0.0, 0.0, M_PI/2},
		{20.65,	17.55,z, 0.0, 0.0, M_PI/2},
		{17.8,	17.55,z, 0.0, 0.0, M_PI/2},
		{13.8,	17.55,z, 0.0, 0.0, M_PI/2},
		{9.8,	17.55,z, 0.0, 0.0, M_PI/2},
		{5.8,	17.55,z, 0.0, 0.0, M_PI/2},
		{1.8,	17.55,z, 0.0, 0.0, M_PI/2},
		{-1.05,	17.55,z, 0.0, 0.0, M_PI/2}};

		//INSERT POINTS FOR RETURN TO HOME



int main(int argc, char** argv)
{

    ros::init(argc, argv, "move_to_goal");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    Quadrotor quad(std::ref(node_handle));
    quad.takeoff();
    while(ros::ok()){
      for(int i = 0; i < cord.size(); i++){
        quad.run(cord[i][0], cord[i][1], cord[i][2], cord[i][3], cord[i][4], cord[i][5]);
      }
    }
		//INSERT LAND THE DRONE FUNCTION
    return 0;
}
