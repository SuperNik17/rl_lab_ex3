#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "ros_exercise2/transform_srv.h"

using namespace std;

int main(int argc, char **argv){

	//Init the ROS node with service_client name
	ros::init(argc, argv, "transformation_service_client");
	ros::NodeHandle n_client;

	//Init the service client. Data to use for the service (the .srv file) and the name of the service
	ros::ServiceClient client = n_client.serviceClient<ros_exercise2::transform_srv>("transformation");


	//Define and initialize the service data structure 
	// This datastructure brings with it the input value (in the request fields) 
	// and the output values, in the response field
	ros_exercise2::transform_srv srv;
	srv.request.target_frame = "/base_link";
	srv.request.chaser_frame = "/sixth_link";




}











