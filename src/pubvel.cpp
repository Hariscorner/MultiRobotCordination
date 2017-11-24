// This program publishes randomly−generated velocity messages for turtlesim
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h> //For rand() and RAND_MAX

int main(int argc, char **argv){
//Initialise the ROS system and become a node
ros::init(argc, argv, "publish_velocity");
ros::NodeHandle nh;

//Create a publisher object
ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

//seed the random number generator
srand(time(0));

//Loop at 2Hz until the node is shutdown
ros::Rate rate(2);
while(ros::ok()){
	//Create and fill in the message. The other four fields which are ignored by turtlesim, default ot 0
	geometry_msgs::Twist msg;
	msg.linear.x=double(rand())/double(RAND_MAX);
	msg.angular.z=double(rand())/double(RAND_MAX)-1;
	
	//Publish the messafe
	pub.publish(msg);
	
	//send a message to rosout with the details
	ROS_INFO_STREAM("sending_random_vel_command:"<<"_linear="<<msg.linear.x<<"_angular="<<msg.angular.z);
	
	//wait until it's time for another iteration
	rate.sleep();
	}
}
