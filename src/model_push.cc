#include <boost/bind.hpp>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include "ros/subscribe_options.h"
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;
      gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "model_push" ) );

      this->robot_pose_pub 	= gazebo_ros_->node()->advertise<geometry_msgs::Pose2D>("curr_pose", 1000);	//publisher objects to publish the command velocity
      ros::SubscribeOptions so_v = ros::SubscribeOptions::create<geometry_msgs::Twist> ( "cmd_vel", 1,
                                boost::bind ( &ModelPush::currVelCallback, this, _1 ), ros::VoidPtr(), &queue_);
      curr_vel_sub = gazebo_ros_->node()->subscribe ( so_v );
      
      //ros::SubscribeOptions so_p = ros::SubscribeOptions::create<geometry_msgs::Twist> ( "cmd_vel", 1,
      //                          boost::bind ( &ModelPush::currPoseCallback, this, _1 ), ros::VoidPtr(), &queue_);
      //curr_pose_sub = gazebo_ros_->node()->subscribe ( so_p );

      this->callback_queue_thread_ = boost::thread ( boost::bind ( &ModelPush::QueueThread, this ) );

      // Listen to the update event. This event is broadcast every simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelPush::OnUpdate, this, _1));
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      //publish the current pose
      this->robot_pose_pub.publish(curr_pose);
      
      //update the velocity
      this->model->SetLinearVel(math::Vector3(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z));

      //this->model->GetLinearVel(cmd_vel.linear);
      //this->model->GetAngularVel(cmd_vel.angular);

      //cmd_vel.linear.x=this->model->GetWorldLinearVel().x;
      //cmd_vel.linear.y=this->model->GetWorldLinearVel().y;
      //cmd_vel.linear.z=this->model->GetWorldLinearVel().z;
      //cmd_vel.angular.z=this->model->GetWorldAngularVel().z;

      //ROS_INFO_STREAM("cmd_vel.linear.x: "<<cmd_vel.linear.x);
      //ROS_INFO_STREAM("cmd_vel.linear.y: "<<cmd_vel.linear.y);
      //ROS_INFO_STREAM("cmd_vel.linear.z: "<<cmd_vel.linear.z);
      //cmd_vel.linear.x=1;
      //cmd_vel.linear.y=1;

    }
    
    public: void currVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
      cmd_vel.linear.x= msg->linear.x;
      cmd_vel.linear.y= msg->linear.y;
      cmd_vel.angular.z= msg->angular.z;
    }
    
    public: void currPoseCallback(const geometry_msgs::Twist::ConstPtr& msg) {
      //curr_pose.x= msg->x;
      //curr_pose.y= msg->y;
      //curr_pose.theta= msg-> //FIX
    }
    
    private: void QueueThread()
    {
        static const double timeout = 0.01;

        while ( gazebo_ros_->node()->ok() ) {
            queue_.callAvailable ( ros::WallDuration ( timeout ) );
        }
    }

    private: 
    ros::CallbackQueue queue_;
    boost::thread callback_queue_thread_;
    // Pointer to the model
    physics::ModelPtr model;
    //pointer to publisher
    ros::Publisher robot_pose_pub;
    ros::Subscriber curr_vel_sub, curr_pose_sub;
    GazeboRosPtr gazebo_ros_;
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::Pose2D curr_pose;
    
    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
