// This program publishes randomly−generated velocity messages for turtlesim
#include <ros/ros.h>
//#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelState.h>
#include <stdlib.h> //For rand() and RAND_MAX
#include "gazebo_msgs/GetModelState.h" 	//to use the getmodelstate service. alternatively, could have subscribed directly to /gazebo/model/states

int main(int argc, char **argv){
	//Initialise the ROS system and become a node
	ros::init(argc, argv, "publish_velocity");
	ros::NodeHandle nh;

	//Create a publisher object
	//ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
	ros::Publisher pub=nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 1000);
	gazebo_msgs::ModelState msg1;
	gazebo_msgs::ModelState msg2;
	gazebo_msgs::ModelState msg3;
	msg1.model_name="box1";
	msg2.model_name="box2";
	msg3.model_name="box3";
	
	ros::ServiceClient gms_c = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState getmodelstate1;
	gazebo_msgs::GetModelState getmodelstate2;
	gazebo_msgs::GetModelState getmodelstate3;
    getmodelstate1.request.model_name="box1";
	getmodelstate2.request.model_name="box2";
	getmodelstate3.request.model_name="box3";
	
	//seed the random number generator
	srand(time(0));

	//Loop at 2Hz until the node is shutdown
	ros::Rate rate(2);
	while(ros::ok()){
		//Create and fill in the message. The other four fields which are ignored by turtlesim, default ot 0
		//geometry_msgs::Twist msg;

		
		gms_c.call(getmodelstate1);
		gms_c.call(getmodelstate2);
		gms_c.call(getmodelstate3);
		
		msg1.pose.position.x=getmodelstate1.response.pose.position.x;
		msg1.pose.position.y=getmodelstate1.response.pose.position.y;
		msg1.pose.position.z=getmodelstate1.response.pose.position.z;
		msg1.pose.orientation.x=getmodelstate1.response.pose.orientation.x;
		msg1.pose.orientation.y=getmodelstate1.response.pose.orientation.y;
		msg1.pose.orientation.z=getmodelstate1.response.pose.orientation.z;
		msg1.pose.orientation.w=getmodelstate1.response.pose.orientation.w;
			
		msg1.twist.linear.x=-3;
		msg1.twist.linear.y=0;
		msg1.twist.linear.z=0;
		msg1.twist.angular.x=0;
		msg1.twist.angular.y=0;
		msg1.twist.angular.z=0;

		msg2.pose.position.x=getmodelstate2.response.pose.position.x;
		msg2.pose.position.y=getmodelstate2.response.pose.position.y;
		msg2.pose.position.z=getmodelstate2.response.pose.position.z;
		msg2.pose.orientation.x=getmodelstate2.response.pose.orientation.x;
		msg2.pose.orientation.y=getmodelstate2.response.pose.orientation.y;
		msg2.pose.orientation.z=getmodelstate2.response.pose.orientation.z;
		msg2.pose.orientation.w=getmodelstate2.response.pose.orientation.w;
			
		msg2.twist.linear.x=1.5;
		msg2.twist.linear.y=-1.5;
		msg2.twist.linear.z=0;
		msg2.twist.angular.x=0;
		msg2.twist.angular.y=0;
		msg2.twist.angular.z=0;
		
		
		msg3.pose.position.x=getmodelstate3.response.pose.position.x;
		msg3.pose.position.y=getmodelstate3.response.pose.position.y;
		msg3.pose.position.z=getmodelstate3.response.pose.position.z;
		msg3.pose.orientation.x=getmodelstate3.response.pose.orientation.x;
		msg3.pose.orientation.y=getmodelstate3.response.pose.orientation.y;
		msg3.pose.orientation.z=getmodelstate3.response.pose.orientation.z;
		msg3.pose.orientation.w=getmodelstate3.response.pose.orientation.w;
			
		msg3.twist.linear.x=1.5;
		msg3.twist.linear.y=1.5;
		msg3.twist.linear.z=0;
		msg3.twist.angular.x=0;
		msg3.twist.angular.y=0;
		msg3.twist.angular.z=0;
		//Publish the messafe
		pub.publish(msg1);
		pub.publish(msg2);
		pub.publish(msg3);

		//send a message to rosout with the details
		ROS_INFO_STREAM("sending_vel_command:"<<"_linear="<<msg1.twist.linear.x<<"_angular="<<msg1.twist.angular.z);

		//wait until it's time for another iteration
		rate.sleep();
		}
}
