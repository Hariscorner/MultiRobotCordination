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
#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"
//#include "LinearMath/btMatrix3x3.h"
#include <geometry_msgs/Quaternion.h>

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;
      gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "model_push" ) );

      this->robot_pose_pub 	= gazebo_ros_->node()->advertise<geometry_msgs::Pose2D>("pose", 1000);	//publisher objects to publish the command velocity
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
      //gazebo::math::Pose pose = this->model->gazebo::physics::ModelState::GetPose();
      gazebo::math::Pose pose = this->model->GetWorldPose();
      geometry_msgs::Quaternion g_quat;
      g_quat.x=pose.rot.x;
      g_quat.y=pose.rot.y;
      g_quat.z=pose.rot.z;
      g_quat.w=pose.rot.w;
      
      //ROS_INFO_STREAM("pose.pos.x: "<< pose.pos.x);
      //ROS_INFO_STREAM("pose.pos.y: "<< pose.pos.y);
      //ROS_INFO_STREAM("pose.pos.z: "<< pose.pos.z);
      //ROS_INFO_STREAM("pose.rot.x: "<< pose.rot.x);
      //ROS_INFO_STREAM("pose.rot.y: "<< pose.rot.y);
      //ROS_INFO_STREAM("pose.rot.z: "<< pose.rot.z);
      //ROS_INFO_STREAM("pose.rot.w: "<< pose.rot.w);
      
      
      // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
      tf::Quaternion quat;
      tf::quaternionMsgToTF(g_quat, quat);

      // the tf::Quaternion has a method to acess roll pitch and yaw
      double roll, pitch, yaw;
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      
      curr_pose.x     = pose.pos.x;
      curr_pose.y     = pose.pos.y;
      curr_pose.theta = yaw;
      
      //publish the current pose
      this->robot_pose_pub.publish(curr_pose);
      
      //update the velocity
      this->model->SetLinearVel(math::Vector3(-cmd_vel.linear.x*cos(yaw),-cmd_vel.linear.x*sin(yaw), 0));

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
