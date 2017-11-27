#include <ros/ros.h>
#include <iostream>
#include <turtlesim/Spawn.h>
#include <ilcplex/ilocplex.h>
#include <chrono>	//for endl
#include <math.h> 	//for hypot
#include <cmath> 	//for abs
#include "turtlesim/Pose.h"		
#include <limits.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include <boost/assign/list_of.hpp>
#include <gazebo_msgs/ModelState.h>
#include "gazebo_msgs/GetModelState.h" 
#include <tf/transform_datatypes.h>

using namespace std;
#define NROBOT 3
#define OFFSET 9*M_PI/180
#define TOLERANCE 0*M_PI/180
// #define NROBOT 7
// #define OFFSET 9*M_PI/180
// #define TOLERANCE 0*M_PI/180
#define isNear(X, A) ( ((X > (A - TOLERANCE)) && ( X < (A + TOLERANCE))) )

double constrainAngle(double x);
double getYawAngle(gazebo_msgs::ModelState state[], int m);
void getCurrState(ros::ServiceClient& gms_c, gazebo_msgs::GetModelState getmodelstate[], int m, gazebo_msgs::ModelState state[]);
void reverseTransformVelocity(ros::ServiceClient& gms_c, gazebo_msgs::GetModelState getmodelstate[], geometry_msgs::Twist cmd_vel[], int m, gazebo_msgs::ModelState state[]);
void forwardTransformVelocity(gazebo_msgs::ModelState state[], int m, geometry_msgs::Twist cmd_vel[]);


void rotate_world(geometry_msgs::Pose2D TPose[], double del_angle);
void evalcoeffs(double h[NROBOT][NROBOT], double k[NROBOT][NROBOT],  geometry_msgs::Pose2D TPose[]);
//void spawn_my_turtles(ros::NodeHandle& nh, turtlesim::Spawn::Request req[], turtlesim::Spawn::Response resp[]);

void optimizeme(IloModel model, IloNumVarArray var, IloRangeArray con, geometry_msgs::Twist cmd_vel[], double Vinit[]);
static void populatebyrow (IloModel model, IloNumVarArray x, IloRangeArray c, geometry_msgs::Pose2D TPose[], double h[NROBOT][NROBOT], double k[NROBOT][NROBOT], double max_vel, double min_vel, double Vinit[]);
	   
int main(int argc, char **argv) {
	
	ros::init(argc, argv, "defaultnode");
	ros::NodeHandle nh;
	
	double max_vel,min_vel,Vinit[NROBOT], pub_rate, h[NROBOT][NROBOT], k[NROBOT][NROBOT];
	geometry_msgs::Pose2D RPose[NROBOT];
	geometry_msgs::Twist cmd_vel[NROBOT];
	
	ros::param::get("pubrate",	pub_rate);
	ros::param::get("maxvel",	max_vel);
	//ros::param::get("minvel",	min_vel);
	
	ros::Rate my_rate(pub_rate);
	
	//spawn_my_turtles();	//spawn both the turtles and initiate their pose and check if the spawnings were successful
	
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
	sleep(8); //wait until gazebo startsups

	ros::Rate rate(2);
		
	for (int m=0; m<NROBOT; ++m) {
		getCurrState(gms_c, getmodelstate, m, state);
		
		RPose[m].x		= state[m].pose.position.x;
		RPose[m].y		= state[m].pose.position.y;
		RPose[m].theta	= getYawAngle(state,m);

		cmd_vel[m].linear.x=5;		//set initial velocity
		reverseTransformVelocity(gms_c, getmodelstate,cmd_vel,m,state);	//sets the reverse_trasnformed velocity in new_state
		//pub.publish(state[m]);
			cout<<"Pose before optimisation\n";
			cout<<"TPose.x\t= " << RPose[m].x;
			cout<<"\tTPose.y\t= " << RPose[m].y;
			cout<<"\tTPose.theta\t=" << RPose[m].theta*180/M_PI<<endl;
	}	
	
	
	sleep(1);
	ros::spinOnce();
	
	IloEnv   env;				//create environment handle which also creates the implementation object internally
	IloModel model(env);		//create modelling object to define optimisation models with our enviroment env
	IloNumVarArray var(env);	//create modelling variables
	IloRangeArray con(env);		//create range objects for defining constraints
	
	evalcoeffs(h,k,RPose);				//calculates the coefficients of the constraint eqns etc
	populatebyrow (model, var, con, RPose,h,k, max_vel,min_vel,Vinit );	//populate the model
	optimizeme(model,var,con, cmd_vel, Vinit);	//the real optimisation happens here
	

	
	while(ros::ok() && nh.ok() ){
		for (int a=0; a<NROBOT; ++a) {
			pub.publish(state[a]);
		}
		//my_rate.sleep();
		//ros::spinOnce();
		my_rate.sleep();
		//++i;
		for (int a=0; a<NROBOT; ++a) {
			//turtle_vel_pub[a].publish(cmdVel[a]);
			reverseTransformVelocity(gms_c, getmodelstate,cmd_vel,a,state);
		}
	}
	
	cout<<"done!"<<endl;
	sleep(1000);
   	return 0;
}	//END of main


static void populatebyrow (IloModel model, IloNumVarArray x, IloRangeArray c, geometry_msgs::Pose2D TPose[], double h[NROBOT][NROBOT], double k[NROBOT][NROBOT], double max_vel, double min_vel, double Vinit[])
{
	IloEnv env = model.getEnv();		//environment handle

	for(int m=0; m<NROBOT; ++m) {
		//define upper and lower bounds for variables and the variable type (continous(float) or discrete (int/bool)
		x.add(IloNumVar(env, min_vel-Vinit[m], max_vel-Vinit[m], ILOFLOAT));	//qi
	}
	
	IloExpr expr(env);
	for(int j=0; j<NROBOT; ++j) {
	  expr += - x[j] ;
	}
	model.add(IloMinimize(env,expr));
	//model.add(IloMinimize(env, - x[0] - x[1] - x[2] ));		//objective function for minimisation (Note: done in single step instead of two) 

	double theta1, theta2, h1, h2, k1, k2, v1, v2;
	for(int i=0; i<NROBOT; ++i) {
		for(int j=0; j<NROBOT; ++j) {
			if (i<j) {
				theta1=TPose[i].theta;
				theta2=TPose[j].theta;
				h1=h[i][j];
				h2=h[j][i];
				k1=k[i][j];
				k2=k[j][i];
				v1=Vinit[i];
				v2=Vinit[j];
				IloRange R1(-cos(theta1)*x[i] 	+ cos(theta2)*x[j] 	<=  v1*cos(theta1) 	- v2*cos(theta2));
				IloRange R2(h1*x[i] 			- h2*x[j] 	<= -v1*h1		+ v2*h2);
				IloRange R3(cos(theta1)*x[i] 	- cos(theta2)*x[j] 	<= -v1*cos(theta1) 	+ v2*cos(theta2));
				IloRange R4(-h1*x[i] 			+ h2*x[j] 	<=  v1*h1 		- v2*h2);
				IloRange R5(-cos(theta1)*x[i] 	+ cos(theta2)*x[j] 	<=  v1*cos(theta1) 	- v2*cos(theta2));
				IloRange R6(-k1*x[i] 			+ k2*x[j] 	<=  v1*k1		- v2*k2);
				IloRange R7(cos(theta1)*x[i] 	- cos(theta2)*x[j] 	<= -v1*cos(theta1) 	+ v2*cos(theta2));
				IloRange R8(k1*x[i] 			- k2*x[j] 	<= -v1*k1 		+ v2*k2);
				
				IloOr myOR(env);
				myOR.add(R1 && R2);
				myOR.add(R3 && R4);
				myOR.add(R5 && R6);
				myOR.add(R7 && R8);
				
				model.add(myOR);
			}
		}
	}

}  // END populatebyrow


void optimizeme(IloModel model, IloNumVarArray var, IloRangeArray con, geometry_msgs::Twist cmd_vel[], double Vinit[]) {
	IloEnv env = model.getEnv();
    try {
		
      	IloCplex cplex(model);				//create cplex object for solving the problem (Note: two steps combined)
      	cplex.setParam( IloCplex::Param::Emphasis::MIP,1);
	cplex.setParam( IloCplex::Param::MIP::Tolerances::Integrality, 0);
	//cplex.setParam( IloCplex::Param::Simplex::Tolerances::Feasibility, 1e-9);
	cplex.setParam( IloCplex::Param::Barrier::ConvergeTol, 1e-12);
      	cplex.exportModel("/home/hari/my_iliad/src/shadow_algorithm/src/lpex1.lp");		//write the extracted model to lplex1.lp file (optional)
		cplex.setOut(env.getNullStream()); 	//discard the output from next step(solving) by dumping it to null stream rather than to the screen
		
		// Optimize the problem and obtain solution.
		if ( !cplex.solve() ) { //this returns an ILOBOOL value (true/false) depending on whether it obtained a feasible soln or not
		 env.error() << "Failed to optimize LP" << endl;
		 throw(-1);
		}
		
		IloNumArray vals(env);	//to store the solution values of all the variables
		IloNumArray vals2(env);	//to store the slack variable outputs
		env.out() << "Solution status = " << cplex.getStatus() << endl;		//to get more detailed status of the soln
		env.out() << "Cplex status = " << cplex.getCplexStatus() << endl;		//to get more detailed status of the soln
		env.out() << "Solution value  = " << cplex.getObjValue() << endl;	//query the objective function value that resulted from optimisation
		
		cout<<"Solved as MIP?\t: " <<cplex.isMIP()<<endl;
		cplex.getValues(vals, var);		//solution values for all variables will be stored to vals
		env.out() << "Values        = " << vals << endl;
		//cplex.getSlacks(vals2, con);
		//env.out() << "Slacks        = " << vals2 << endl;
      
		IloNum violation = cplex.getQuality(IloCplex::MaxPrimalInfeas);
		cout << "max violation\t: " << violation << endl;
		  		
		cout<< "q1\t:" << vals[0] << "\tq2\t: " << vals[1] << endl;
		for(int y=0; y< NROBOT; ++y) {
			cmd_vel[y].linear.x=Vinit[y]+vals[y];
		}
		
	}
   	catch (IloException& e) {
      	cerr << "Concert exception caught: " << e << endl;
  	}
   	catch (...) {
      	cerr << "Unknown exception caught" << endl;
   	}//end of try catch
}

void evalcoeffs(double h[NROBOT][NROBOT], double k[NROBOT][NROBOT], geometry_msgs::Pose2D TPose[]) {
	
	double safe_dia, alpha, A[NROBOT][NROBOT], w[NROBOT][NROBOT], l[NROBOT][NROBOT], r[NROBOT][NROBOT];
			
	ros::param::get("safedia",	safe_dia);
	
	for(int i=0; i<NROBOT; ++i) {
		for(int j=0; j<NROBOT; ++j) {
			if (i != j) {
				A[i][j]=hypot(TPose[j].x-TPose[i].x,TPose[j].y-TPose[i].y);
				w[i][j]=atan2(TPose[j].y-TPose[i].y,TPose[j].x-TPose[i].x);
				cout << endl<< "A[i][j]\t: " << A[i][j]  << "\tSafe_dia\t: " << safe_dia << "\tDifference\t: " << A[i][j] - safe_dia << endl;

				if(A[i][j] - safe_dia < 0) {
					cout << "Safety disc overlap identified!!!" <<endl;
					sleep(1500);
					exit (0);
				}
				else if(safe_dia/A[i][j] > 1) {
					cout << "It's too late to avoid collision...Sine Error...exiting" <<endl;
					sleep(1500);
					exit (0);
				}
				alpha=asin(safe_dia/A[i][j]);
				l[i][j]=constrainAngle(w[i][j]+alpha);	//convert to the range [-180,180) if it goes out of bound
				r[i][j]=constrainAngle(w[i][j]-alpha);
				cout << "alpha\t: " << alpha*180/M_PI <<"\tw["<<i<<"]["<<j<<"]\t: " << w[i][j]*180/M_PI << "\tl[i][j]\t: " << l[i][j]*180/M_PI << "\tr[i][j]\t: " << r[i][j]*180/M_PI << endl;
				
				if(isNear(w[i][j], M_PI/2)){
				  cout<<"if 1"<<endl;
				  double del_angle=TOLERANCE -abs(M_PI/2-w[i][j]);
				  del_angle=((w[i][j] > M_PI/2) ? del_angle : -del_angle);
				  rotate_world(TPose, del_angle);
				  memset(h, 0, NROBOT*NROBOT);	//set arrays to zero
				  memset(k, 0, NROBOT*NROBOT);
				  evalcoeffs(h,k,TPose);
				  return;
				}
				else if(isNear(l[i][j], M_PI/2)){
				  cout<<"if 2"<<endl;
				  double del_angle=TOLERANCE -abs(M_PI/2-l[i][j]);
				  del_angle=((l[i][j] > M_PI/2) ? del_angle : -del_angle);
				  rotate_world(TPose, del_angle);
				  memset(h, 0, NROBOT*NROBOT);	//set arrays to zero
				  memset(k, 0, NROBOT*NROBOT);
				  evalcoeffs(h,k,TPose);
				  return;
				}
				else if (isNear(r[i][j], M_PI/2)){
				  cout<<"if 3"<<endl;
				  double del_angle=TOLERANCE -abs(M_PI/2-r[i][j]);
				  del_angle=((r[i][j] > M_PI/2) ? del_angle : -del_angle);
				  rotate_world(TPose, del_angle);
				  memset(h, 0, NROBOT*NROBOT);	//set arrays to zero
				  memset(k, 0, NROBOT*NROBOT);
				  evalcoeffs(h,k,TPose);
				  return;
				}
				else if(isNear(w[i][j], -M_PI/2)){
				  cout<<"if 4"<<endl;
				  double del_angle=TOLERANCE -abs(-M_PI/2-w[i][j]);
				  del_angle=((w[i][j] > -M_PI/2) ? del_angle : -del_angle);
				  rotate_world(TPose, del_angle);
				  memset(h, 0, NROBOT*NROBOT);	//set arrays to zero
				  memset(k, 0, NROBOT*NROBOT);
				  evalcoeffs(h,k,TPose);
				  return;
				}  
				else if(isNear(l[i][j], -M_PI/2)){
				  cout<<"if 5"<<endl;
				  double del_angle=TOLERANCE -abs(-M_PI/2-l[i][j]);
				  del_angle=((l[i][j] > -M_PI/2) ? del_angle : -del_angle);
				  rotate_world(TPose, del_angle);
				  memset(h, 0, NROBOT*NROBOT);	//set arrays to zero
				  memset(k, 0, NROBOT*NROBOT);
				  evalcoeffs(h,k,TPose);
				  return;
				}  
				else if (isNear(r[i][j], -M_PI/2)){
				  cout<<"if 6"<<endl;
				  double del_angle=TOLERANCE -abs(-M_PI/2-r[i][j]);
				  del_angle=((r[i][j] > -M_PI/2) ? del_angle : -del_angle);
				  rotate_world(TPose, del_angle);
				  memset(h, 0, NROBOT*NROBOT);	//set arrays to zero
				  memset(k, 0, NROBOT*NROBOT);
				  evalcoeffs(h,k,TPose);
				  return;
				}
				else  { 
				  //v1=T1Pose.linear_velocity;
				  //v2=T2Pose.linear_velocity;

				  cout << "tan(l[i][j])\t: " << tan(l[i][j]) << "\ttan(r[i][j])\t: " << tan(r[i][j]) << endl;

				  h[i][j]=tan(l[i][j])*cos(TPose[i].theta) - sin(TPose[i].theta);
				  k[i][j]=tan(r[i][j])*cos(TPose[i].theta) - sin(TPose[i].theta);

				  cout << "h[i][j]\t: " << h[i][j] << "\tk[i][j]\t: " << k[i][j] << endl;
				}
			}
		}
	}

}

void rotate_world(geometry_msgs::Pose2D TPose[], double del_angle) {
  double tempx, tempy;
  del_angle=del_angle+0.01*del_angle;
  cout << "TPose is updated to: " <<endl;
  for(int j=0; j< NROBOT; ++j) {
    tempx		= TPose[j].x * cos(del_angle) + TPose[j].y * sin(del_angle);
    tempy		=-TPose[j].x * sin(del_angle) + TPose[j].y * cos(del_angle);
    TPose[j].x		= tempx;
    TPose[j].y		= tempy;
    TPose[j].theta	= constrainAngle(TPose[j].theta - del_angle);
    	
	cout<<"TPose.x\t= " << TPose[j].x;
	cout<<"\tTPose.y\t= " << TPose[j].y;
	cout<<"\tTPose.theta\t=" << TPose[j].theta*180/M_PI<<endl;
  }
  //sleep(10);
}

/*
void spawn_my_turtles() {
	//spawn two turtles
	//ros::service::waitForService("spawn");
	//ros::ServiceClient spawNROBOT = nh.serviceClient<turtlesim::Spawn>("spawn");
	
	double x=5.5, y=5.5, p=0, r=5.5;

	for (int j=0; j<NROBOT; ++j) {
		req[j].name="myrobot";
		req[j].name += std::to_string(j+1);	//append UID to the end of "myturtle" eg: myturtle3
		p=OFFSET+j*2*M_PI/NROBOT;	
		//p=j*2*M_PI/NROBOT;		//divide 360 equally into "NROBOT parts
		req[j].x=x+r*cos(p);
		req[j].y=y+r*sin(p);
		req[j].theta=M_PI+p;
		
		bool success=spawNROBOT.call(req[j],resp[j]);
		if(success) { ROS_INFO_STREAM ("Spawned turtle\t:" << j ); }
		else { ROS_INFO_STREAM ("Error, unable to spawn turtle"); }
		
	}
}
*/

void getCurrState(ros::ServiceClient& gms_c,gazebo_msgs::GetModelState getmodelstate[], int m, gazebo_msgs::ModelState state[]) {

	gms_c.call(getmodelstate[m]);
	state[m].pose.position.x=getmodelstate[m].response.pose.position.x;
	state[m].pose.position.y=getmodelstate[m].response.pose.position.y;
	state[m].pose.position.z=getmodelstate[m].response.pose.position.z;
	state[m].pose.orientation.x=getmodelstate[m].response.pose.orientation.x;
	state[m].pose.orientation.y=getmodelstate[m].response.pose.orientation.y;
	state[m].pose.orientation.z=getmodelstate[m].response.pose.orientation.z;
	state[m].pose.orientation.w=getmodelstate[m].response.pose.orientation.w;
	
	state[m].twist.linear.x	=getmodelstate[m].response.twist.linear.x;
	state[m].twist.linear.y	=getmodelstate[m].response.twist.linear.y;
	state[m].twist.linear.z	=getmodelstate[m].response.twist.linear.z;
	state[m].twist.angular.x=getmodelstate[m].response.twist.angular.x;
	state[m].twist.angular.y=getmodelstate[m].response.twist.angular.y;
	state[m].twist.angular.z=getmodelstate[m].response.twist.angular.z;

	return;
}

void forwardTransformVelocity(gazebo_msgs::ModelState state[], int m, geometry_msgs::Twist cmd_vel[]){
	
	cmd_vel[m].linear.x	= hypot(state[m].twist.linear.x, state[m].twist.linear.y);
	cmd_vel[m].linear.y	= 0;
	cmd_vel[m].linear.z	= 0;
	cmd_vel[m].angular.x	= 0;
	cmd_vel[m].angular.y	= 0;
	cmd_vel[m].angular.z	= 0;	
	
	//cout<<"cmd_vel[m].linear.x" << cmd_vel[m].linear.x << endl;
}

void reverseTransformVelocity(ros::ServiceClient& gms_c, gazebo_msgs::GetModelState getmodelstate[], geometry_msgs::Twist cmd_vel[], int m, gazebo_msgs::ModelState state[]){		
	double yaw_angle;
	
	getCurrState(gms_c, getmodelstate, m, state);
	
	yaw_angle = getYawAngle(state,m);
	state[m].twist.linear.x=cmd_vel[m].linear.x*cos(yaw_angle);
	state[m].twist.linear.y=cmd_vel[m].linear.x*sin(yaw_angle);
	state[m].twist.linear.z=0;
	state[m].twist.angular.x=0;
	state[m].twist.angular.y=0;
	state[m].twist.angular.z=0;
	
	//cout<<"state[m].twist.linear.x"<<state[m].twist.linear.x << endl;
	//cout<<"state[m].twist.linear.y"<<state[m].twist.linear.y << endl;
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