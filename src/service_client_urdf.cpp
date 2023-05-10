#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "ros_exercise2/transform_srv.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>

using namespace std;

int main(int argc, char **argv){

	//Init the ROS node with service_client name
	ros::init(argc, argv, "transformation_service_client");
	ros::NodeHandle n_client;

	//Init the service client. Data to use for the service (the .srv file) and the name of the service
	ros::ServiceClient client = n_client.serviceClient<ros_exercise2::transform_srv>("T");


	//Define and initialize the service data structure 
	// This datastructure brings with it the input value (in the request fields) 
	// and the output values, in the response field
	ros_exercise2::transform_srv srv;
	srv.request.target_frame.data = "/base_link";
	srv.request.chaser_frame.data = "/first_link";

	ROS_INFO("Waiting for the server\n");
	client.waitForExistence();
	ROS_INFO("Okay...the server is up!");
	
	tf::TransformBroadcaster br;
	tf::Transform transform;

	ros::Rate rate(10);

	while (ros::ok()){
		//Call the service callback
		//The return value is false if:
		//	- the callback returns false
		//	- the service has not been found in the ROS network
		if(!client.call(srv)){
			ROS_ERROR("Error calling the service");
			return 1;
		}

		//Just print the output
		cout << "Service output: " << srv.response.pose << endl;
		tf::Quaternion q(srv.response.pose.orientation.x,srv.response.pose.orientation.y,srv.response.pose.orientation.z,srv.response.pose.orientation.w);
	
		transform.setOrigin(tf::Vector3(srv.response.pose.position.x,srv.response.pose.position.y,srv.response.pose.position.z));
		transform.setRotation(q);
	
		br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"base_link","sixth_link"));
		rate.sleep();
	}
	return 0;
}











