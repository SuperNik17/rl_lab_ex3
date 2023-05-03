#include "ros/ros.h"
#include <tf/transform_listener.h>
//Header of the service message. 
//	The service message belongs to the ros_service package
#include "ros_exercise2/transform_srv.h"

using namespace std;


//Callback function
//	Return value: boolean
//		If this function returns true, the service function has been corretly called
//		You can use this value to check if the function has been called with correct parameters
//		i.e. call a service calculating the square of a number, calling the service with a negative number
//	Input values:  the request part of the servive 
//				   the output of the service to fill
bool service_callback( ros_exercise2::transform_srv::Request &req, ros_exercise2::transform_srv::Response &res) {


	cout << "Service transform received: " << endl;
	//We know that the service is called with 2 parameters: target frame string and chaser frame string
	// These parameters are put in a data structure called req
	cout << req.target_frame << req.chaser_frame << endl;

	//The return value is store in the res datastrcutre
	cout << "Returning the transformation of the chaser on the target" << endl;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    try{
        listener.waitForTransform(req.target_frame,req.chaser_frame,ros::Time(0),ros::Duration(3.0));
		listener.lookupTransform(req.target_frame,req.chaser_frame,ros::Time(0),transform);
    }catch(tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
	}
	
	res.pose.position.x = transform.getOrigin().x();
	res.pose.position.y = transform.getOrigin().y();
	res.pose.position.z = transform.getOrigin().z();
	res.pose.orientation.x = transform.getRotation().x();
	res.pose.orientation.y = transform.getRotation().y();
	res.pose.orientation.z = transform.getRotation().z();
	res.pose.orientation.w = transform.getRotation().w();

	ROS_INFO_STREAM(" Transform: " << 
        
            res.pose.position.x << ", " << 
            res.pose.position.y << ", " <<
            res.pose.position.z << ", " << "/tQuaternion: "<<
            res.pose.orientation.x << ", " << 
            res.pose.orientation.y << ", " << 
            res.pose.orientation.z << ", " <<
			res.pose.orientation.w
        );

	return true;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "transformation_service");
	ros::NodeHandle n;
	
	ros::Rate rate(1.0);
    
    while (ros::ok()) {
	ROS_INFO_STREAM(" Waiting for a service client");

	//Initialize the service object: name of the service and callback function
	//	Like subscribers, also tje callback function can be declared as a class function
	ros::ServiceServer service = n.advertiseService("transformation", service_callback);
	rate.sleep(); 
	}

	// Call the spin to
	ros::spin();

	return 0;

}
