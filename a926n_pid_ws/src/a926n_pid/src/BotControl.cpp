// EE3305/ME3243
// Name: Sim Justin
// NUSNET ID: E0968898

#include "botcontrol.hpp"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
using namespace botcontrol;

BotControl::BotControl(ros::NodeHandle& nh) : nodehandle_(nh){

	//load the param
	if(!loadParam()){
		ROS_ERROR("Error in loading the parameters.");
		// ros::requestShutdown();
	}

	// declare all the subscriber and publisher
	odom_sub_ = nodehandle_.subscribe("/odom", 1, &BotControl::odomCallBack, this);
	vel_pub_ = nodehandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	error_forward_pub_ = nodehandle_.advertise<std_msgs::Float32>("/error_forward", 1); 
	error_angle_pub_ = nodehandle_.advertise<std_msgs::Float32>("/error_angle", 1);
	control_signal_forward_pub_ = nodehandle_.advertise<std_msgs::Float32>("/control_signal_forward", 1);
	control_signal_angle_pub_ = nodehandle_.advertise<std_msgs::Float32>("/control_signal_angle", 1);

	//initialize variables
	error_forward_ = 0;
	error_angle_ = 0;
	error_forward_prev_ = 0;
	error_angle_prev_ = 0;
	I_forward_ = 0;
	I_angle_ = 0;
	D_forward_ = 0;
	D_angle_ = 0;

	lin_max = 0.22;
	ang_max = M_PI;

	// FOR DIAGNOSTICS
	// 1 for velocity +ve, -1 for velocity -ve
	curr_state_a = 1; 
	curr_state_f = 1;
	minima_a = 0;
	minima_f = 0;
	maxima_a = 0;
	maxima_f = 0;

	ROS_INFO("Node Initialized");
}

BotControl::~BotControl(){}

// double pos_x, pos_y, ang_z_ = NAN;

void BotControl::odomCallBack(const nav_msgs::Odometry::ConstPtr& msg)
{ // ground truth in simulation for turtlebot
  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;

  pos_x_ = msg->pose.pose.position.x;
  pos_y_ = msg->pose.pose.position.y;
  ang_z_ = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qz * qz + qy * qy));
}

void BotControl::pidAlgorithm(){
	std_msgs::Float32 linear_error;
	std_msgs::Float32 angle_error;
	std_msgs::Float32 linear_velocity; // command
	std_msgs::Float32 angle_velocity; // command

    double Dx = pillar_x - pos_x_; // pos_x_ from odom (true sim. position)
    double Dy = pillar_y - pos_y_; // pos_y_ from odom (true sim. position)
    
	// update the pid status
	error_forward_prev_ = error_forward_;
	error_angle_prev_ = error_angle_;
	
	error_forward_ = sqrt(Dx*Dx + Dy*Dy) - target_distance;
	error_angle_ = atan2(Dy, Dx) - ang_z_;
    
	// regularize the error_angle_ within [-PI, PI]
	while (error_angle_ < -M_PI) error_angle_ += 2*M_PI;
	while (error_angle_ > M_PI) error_angle_ -= 2*M_PI;

	double P_term_f = Kp_f * error_forward_;
	double P_term_a = Kp_a * error_angle_;

	// ENTER YOUR CODE HERE

	// define the integral term

	// Classical Riemann Sum
	// I_forward_ += error_forward_ * dt;
	// I_angle_ += error_angle_ * dt;

	// Trapezoidal Rieman Sum
	// We only apply integrator in the controllable region (Conditional Integration)
	if (P_term_f > -lin_max && P_term_f < lin_max)
		I_forward_ += (error_forward_ + error_forward_prev_) * dt / 2;
	if (Kp_a * P_term_a > -ang_max && Kp_a * P_term_a < ang_max)
		I_angle_ += (error_angle_ + error_angle_prev_) * dt / 2;

	// define the derivative term
	D_forward_ = (error_forward_ - error_forward_prev_) / dt;
	D_angle_ = (error_angle_ - error_angle_prev_) / dt;

	// define the PID control term
	trans_forward_ = P_term_f + Ki_f * I_forward_ + Kd_f * D_forward_;
	trans_angle_ = P_term_a + Ki_a * I_angle_ + Kd_a * D_angle_;

	// END YOUR CODE HERE

	// // FOR DIAGNOSTICS
	// // Angular
	// if (curr_state_a == 1 && trans_angle_ <= 0) {
	// 	curr_state_a = -1;
	// 	minima_a = error_angle_;
	// 	ROS_INFO("Angle Velocity Negative, Minima Error: %f", minima_a);
	// }
	// if (curr_state_a == -1 && trans_angle_ >= 0) {
	// 	curr_state_a = 1;
	// 	maxima_a = error_angle_;
	// 	ROS_INFO("Angle Velocity Positive, Maxima Error: %f", maxima_a);
	// }
	// ROS_INFO("Ang: %f of %f", atan2(Dy, Dx), ang_z_);

	// // Linear
	// if (curr_state_f == 1 && trans_forward_ <= 0) {
	// 	curr_state_f = -1;
	// 	minima_f = error_forward_;
	// 	ROS_INFO("Forward Velocity Negative, Minima Error: %f", minima_f);
	// }
	// if (curr_state_f == -1 && trans_forward_ >= 0) {
	// 	curr_state_f = 1;
	// 	maxima_f = error_forward_;
	// 	ROS_INFO("Forward Velocity Positive, Maxima Error: %f", maxima_f);
	// }
	// ROS_INFO("Lin: %f of %f", sqrt(Dx*Dx + Dy*Dy), target_distance);

	// limiting trans_angle
	if (trans_angle_ > ang_max)
      trans_angle_ = ang_max;
    if (trans_angle_ < -ang_max)
      trans_angle_ = -ang_max;

	// set limit
	if(trans_forward_ > lin_max) trans_forward_ = lin_max;
	if(trans_forward_ < -lin_max) trans_forward_ = -lin_max;

	// UNCOMMENT BELOW

	ROS_INFO("1----Forward Velocity: %f; Angle Velocity: %f; Orientation_error: %f, Linear_error: %f",  
		trans_forward_, trans_angle_, error_angle_, error_forward_);
	ROS_INFO("Pillar x: %f; Pillar y: %f; Pos x: %f, Pos y: %f",  
		pillar_x, pillar_y, pos_x_, pos_y_);

	// ROS_INFO("Forward Velocity: %f; Angle Velocity: %f; Orientation_error: %f, Distance: %f",  //#
	// 	trans_forward_, trans_angle_, error_angle_, scan_range_);
 	// ROS_INFO("2----Forward Velocity: %f; Angle Velocity: %f; Orientation_error: %f, Linear_error: %f",  
	// 	trans_forward_, trans_angle_, error_angle_, error_forward_);


	//publish all
	vel_cmd_.linear.x = trans_forward_;
	vel_cmd_.angular.z = trans_angle_; //euler angle
	vel_pub_.publish(vel_cmd_);

	linear_error.data = error_forward_;
	error_forward_pub_.publish(linear_error);

	linear_velocity.data = trans_forward_;
	control_signal_angle_pub_.publish(linear_velocity);

	angle_error.data = error_angle_;
	error_angle_pub_.publish(angle_error);

	angle_velocity.data = trans_angle_;
	control_signal_angle_pub_.publish(angle_velocity);

}

void BotControl::spin(){
	ros::Rate loop_rate(1/dt);

    //# sleep at start to wait for windows to load
	ros::Rate init_rate(1);
    for (int i=3; i>0; --i) {
        ROS_INFO("%d", i);
        init_rate.sleep();
    } // sleep for # seconds where i=# above
	
	while(ros::ok()){
		ros::spinOnce();
        pidAlgorithm(); 
		loop_rate.sleep();
	}

}

bool BotControl::loadParam(){


	if(!nodehandle_.getParam("/Kd_a", Kd_a)){
		ROS_ERROR("Kd_a Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/Kp_f", Kp_f)){
		ROS_ERROR("Kp_f Load Error");
		return false;
	}

	if(!nodehandle_.getParam("/Ki_f", Ki_f)){
		ROS_ERROR("Ki_f Load Error");
		return false;
	}

	if(!nodehandle_.getParam("/Kd_f", Kd_f)){
		ROS_ERROR("Kd_f Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/Kp_a", Kp_a)){
		ROS_ERROR("Kp_a Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/Ki_a", Ki_a)){
		ROS_ERROR("Ki_a Load Error");
		return false;
	}

	if(!nodehandle_.getParam("/target_angle", target_angle)){
		ROS_ERROR("target_angle Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/target_distance", target_distance)){
		ROS_ERROR("target_distance Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/pillar_x", pillar_x)){ //#
		ROS_ERROR("pillar_x Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/pillar_y", pillar_y)){ //#
		ROS_ERROR("pillar_y Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/dt", dt)){
		ROS_ERROR("dt Load Error");
		return false;
	}

	return true;

}
