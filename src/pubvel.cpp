// This program publishes randomly−generated velocity messages for turtlesim
#include <ros/ros.h>
//#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelState.h>
#include <stdlib.h> //For rand() and RAND_MAX
#include "gazebo_msgs/GetModelState.h" 	//to use the getmodelstate service. alternatively, could have subscribed directly to /gazebo/model/states
#include <tf/transform_datatypes.h>

#define NROBOT 3

double constrainAngle(double x);
double getYawAngle(gazebo_msgs::ModelState state[], int m);
void getCurrState(gazebo_msgs::GetModelState getmodelstate[], int m, gazebo_msgs::ModelState state[]);

int main(int argc, char **argv){
	//Initialise the ROS system and become a node
	ros::init(argc, argv, "publish_velocity");
	ros::NodeHandle nh;
	
	//Create a publisher object
	//ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
	ros::Publisher pub=nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 1000);
	gazebo_msgs::ModelState state[NROBOT];
	
	ros::ServiceClient gms_c = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	gazebo_msgs::GetModelState getmodelstate[NROBOT];

	
	state[0].model_name="myrobot1";
	state[1].model_name="myrobot2";
	state[2].model_name="myrobot3";
	
    getmodelstate[0].request.model_name="myrobot1";
	getmodelstate[1].request.model_name="myrobot2";
	getmodelstate[2].request.model_name="myrobot3";
	
	
	//seed the random number generator
	srand(time(0));

	//Loop at 2Hz until the node is shutdown
	ros::Rate rate(2);
	while(ros::ok()){
		//Create and fill in the message. The other four fields which are ignored by turtlesim, default ot 0
		//geometry_msgs::Twist msg;

		for (int m=0; m<NROBOT; ++m) {
			gms_c.call(getmodelstate[m]);
			getCurrState(getmodelstate, m, state);
			pub.publish(state[m]);
		}

		//wait until it's time for another iteration
		rate.sleep();
		}
}

void getCurrState(gazebo_msgs::GetModelState getmodelstate[], int m, gazebo_msgs::ModelState state[]) {

	double yaw_angle;
	
	state[m].pose.position.x=getmodelstate[m].response.pose.position.x;
	state[m].pose.position.y=getmodelstate[m].response.pose.position.y;
	state[m].pose.position.z=getmodelstate[m].response.pose.position.z;
	state[m].pose.orientation.x=getmodelstate[m].response.pose.orientation.x;
	state[m].pose.orientation.y=getmodelstate[m].response.pose.orientation.y;
	state[m].pose.orientation.z=getmodelstate[m].response.pose.orientation.z;
	state[m].pose.orientation.w=getmodelstate[m].response.pose.orientation.w;

	yaw_angle = getYawAngle(state,m);

	state[m].twist.linear.x=3*cos(yaw_angle);
	state[m].twist.linear.y=3*sin(yaw_angle);
	state[m].twist.linear.z=0;
	state[m].twist.angular.x=0;
	state[m].twist.angular.y=0;
	state[m].twist.angular.z=0;

	ROS_INFO_STREAM("sending_vel_command:"<<"_linear="<<state[m].twist.linear.x<<"_angular="<<state[m].twist.linear.y);
	return;
}

double getYawAngle(gazebo_msgs::ModelState state[], int m){
	tf::Pose pose;
	double yaw_angle;
	tf::poseMsgToTF(state[m].pose, pose);
	yaw_angle = tf::getYaw(pose.getRotation());
	yaw_angle = constrainAngle(yaw_angle + M_PI);
	
	return yaw_angle;
}

double constrainAngle(double x){		//Normalize to [-180,180)
    x = fmod(x+M_PI,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}