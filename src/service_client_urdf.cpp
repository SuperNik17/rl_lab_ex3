#include "ros/ros.h"

//Header of the service message. 
//	The service message belongs to the ros_exercise2 package
#include "ros_exercise2/transform_srv.h"
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>

using namespace std;

int main(int argc, char **argv) {

	
	//Init the ROS node with service_client name
	ros::init(argc, argv, "service_client");
	string base_link;
	string follower_link;
	//Se il numero di argomenti argc è diverso da 2, 
	//viene stampato un messaggio di errore ROS utilizzando la funzione ROS_ERROR(), 
	//e il programma termina restituendo un valore negativo (-1)
    if (argc != 3) {
        ROS_ERROR("need camera link name as argument"); 
        return -1;
    }
	//Se il numero di argomenti è 3, 
	//la variabile base_link viene inizializzata con il valore della seconda stringa nell'array argv, 
	// mente la variabile follower_link la terza
	//che corrisponde all'argomento specificato sulla linea di comando. 
	base_link = argv[2];
    follower_link = argv[3];

	ros::NodeHandle n;

	//Init the service client. Data to use for the service (the .srv file) and the name of the service
	ros::ServiceClient client = n.serviceClient<ros_exercise2::transform_srv>("get_frame_transform");

	tf::TransformBroadcaster br;
    tf::Transform transform;
	
	//Define and initialize the service data structure 
	//	This datastructure brings with it the input value (in the request fields) and the output values, in the response field
	ros_exercise2::transform_srv srv
	srv.request.frame_a = base_link.;
	srv.request.frame_b = follower_link;


	ros::Rate r(100);
	while (ros::ok()){

		ROS_INFO("Waiting for the client server");
		client.waitForExistence();
		ROS_INFO("Client server up now");

		



		r.sleep();
	}












	
	//Wait that in the ROS network, the service sum is advertised
	//	If you call a service and the service has not been advertised, you will have back an error
	ROS_INFO("Waiting for the client server");
	client.waitForExistence();
	ROS_INFO("Client server up now");
	
	//Call the service callback
	//	The return value is false if:
	//		- the callback returns false
	//		- the service has not been found in the ROS network
	if (!client.call(srv)) {
		ROS_ERROR("Error calling the service");
		return 1;
	}

	//Just print the output
	cout << "Service output: " << srv.response.sum << endl;
	
	return 0;
}
