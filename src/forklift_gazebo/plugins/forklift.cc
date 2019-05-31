#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <string.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>


namespace gazebo {
class ForkLiftPlugin: public ModelPlugin {

private:
	//double prev_publish_time, prev_odom_calc_time;
	//ros::Publisher butler_odom_pub_;
	//ros::Publisher tf;
	ros::Subscriber forklift_joint_state;
	ros::NodeHandle nh_;
	physics::ModelPtr model;
	event::ConnectionPtr updateConnection;
	std::string model_name;
	physics::JointPtr fork_shift, fork_lift;
	//int64_t left_wheel_enc_ticks, right_wheel_enc_ticks;
	//int64_t prev_left_wheel_enc_ticks = 0, prev_right_wheel_enc_ticks = 0;
	//tf::TransformBroadcaster odom_broadcaster, world_broadcaster;
	// double x = 0.0;
	// double y = 0.0;
	// double th = 0.0;
	//double left_wheel_linear_vel=0.0;
	//double right_wheel_linear_vel=0.0;


public:
	void Init() {
		// this->prev_publish_time = 0.0f;
		// this->prev_odom_calc_time = 0.0f;
		this->model->Reset();
	}

public:
	void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

		// Store the pointer to the model
		this->model = _parent;
		this->model_name = model->GetName();

		ROS_INFO("%s Loaded.", model_name.c_str());

		this->fork_shift = this->model->GetJoint("shift");
		this->fork_lift = this->model->GetJoint("lift");


		this->forklift_joint_state = this->nh_.subscribe( "/joint_states", 200,
				&ForkLiftPlugin::set_joint_states, this);

		// Listen to the update event. This event is broadcast every
		// simulation iteration.
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&ForkLiftPlugin::OnUpdate, this, _1));
	}

	// Called by the world update start event
public:
	void OnUpdate(const common::UpdateInfo & /*_info*/) {

		ros::spinOnce();
		
		// ROS_INFO("World Butler Velo x:%f, y:%f, z:%f, Angular vel:%f",
		// 				this->model->GetWorldLinearVel().x,
		// 				this->model->GetWorldLinearVel().y,
		// 				this->model->GetWorldLinearVel().z,
		// 				this->model->GetWorldAngularVel().z);

		// ROS_INFO("Velo: left : %f right: %f",left_wheel_linear_vel,right_wheel_linear_vel);

// 		this->right_wheel_joint->SetVelocity(0,
//                                 (1) * (right_wheel_linear_vel) / (WHEEL_RADIUS));

//                 this->left_wheel_joint->SetVelocity(0,
//                                 (1) * (left_wheel_linear_vel) / (WHEEL_RADIUS));

// 		math::Pose model_world_pose = this->model->GetWorldPose();

// 		tf::Transform transform;
// 		transform.setOrigin(
// 				tf::Vector3(model_world_pose.pos.x, model_world_pose.pos.y,
// 						model_world_pose.pos.z));
// 		transform.setRotation(
// 				tf::Quaternion(model_world_pose.rot.x, model_world_pose.rot.y,
// 						model_world_pose.rot.z, model_world_pose.rot.w));
// 		this->world_broadcaster.sendTransform(
// 				tf::StampedTransform(transform, ros::Time::now(), "world",
// 						model_name));

// 		ros::Time curr_time = ros::Time::now();

// 		if ( (curr_time.toSec() - this->prev_publish_time) > 0.01 ) {

// 			this->prev_publish_time = curr_time.toSec();

// 			butler_gazebo_2::wheel_encoder encoder_msg;

// 			this->left_wheel_enc_ticks = (long) ANGLETOENC(
// 					this->left_wheel_joint->GetAngle(0).Radian());

// 			this->right_wheel_enc_ticks = (long) ANGLETOENC(
// 					this->right_wheel_joint->GetAngle(0).Radian());

// 			encoder_msg.header.frame_id = 1;
// 			encoder_msg.header.stamp = curr_time;

// 			encoder_msg.left_wheel = left_wheel_enc_ticks;
// 			encoder_msg.right_wheel = right_wheel_enc_ticks;

// 			this->butler_wheel_encoder_pub_.publish(encoder_msg);


// 			double dt = (curr_time.toSec() - prev_odom_calc_time);

// 			double dist_left = ENCTOLEN(
// 					left_wheel_enc_ticks - prev_left_wheel_enc_ticks);

// 			double dist_right = ENCTOLEN(
// 					right_wheel_enc_ticks - prev_right_wheel_enc_ticks);

// 			//ROS_INFO("Dist left:%f, right:%f", dist_left, dist_right);

// 			double dist_x = (dist_left + dist_right) / 2;
// 			double th_in_dt = (dist_right - dist_left) / WHEEL_BASE;

// 			double vx = dist_x / dt;
// 			double vth = th_in_dt / dt;

// 			double x_in_dt = cos(th_in_dt) * dist_x;
// 			double y_in_dt = sin(th_in_dt) * dist_x;

// 			//ROS_INFO("POS IN dT x:%f, y:%f, th:%f", x_in_dt, y_in_dt, th_in_dt);

// 			// TODO: Verify this
// 			this->x += (cos(th) * x_in_dt - sin(th) * y_in_dt);
// 			this->y += (sin(th) * x_in_dt + cos(th) * y_in_dt);
// 			this->th += th_in_dt;

// //			ROS_INFO("POS x:%f, y:%f, th:%f", x, y, th);
// 			geometry_msgs::Quaternion odom_quat =
// 					tf::createQuaternionMsgFromYaw(this->th);

// 			geometry_msgs::TransformStamped odom_trans;
// 			odom_trans.header.stamp = curr_time;
// 			odom_trans.header.frame_id = "odom0_" + model_name;
// 			odom_trans.child_frame_id = model_name;

// 			odom_trans.transform.translation.x = this->x;
// 			odom_trans.transform.translation.y = this->y;
// 			odom_trans.transform.translation.z = 0.0;
// 			odom_trans.transform.rotation = odom_quat;

// 			this->odom_broadcaster.sendTransform(odom_trans);

// 			nav_msgs::Odometry odom;
// 			odom.header.stamp = curr_time;
// 			odom.header.frame_id = "odom0_" + model_name;

// 			// set the position
// 			odom.pose.pose.position.x = this->x;
// 			odom.pose.pose.position.y = this->y;
// 			odom.pose.pose.position.z = 0.0;
// 			odom.pose.pose.orientation = odom_quat;

// 			// set the velocity
// 			odom.child_frame_id = "base_link";
// 			odom.twist.twist.linear.x = vx;
// 			odom.twist.twist.linear.y = 0.0;
// 			odom.twist.twist.angular.z = vth;

// 			// publish the message
// 			this->butler_odom_pub_.publish(odom);

// 			this->prev_odom_calc_time = curr_time.toSec();
// 			this->prev_left_wheel_enc_ticks = this->left_wheel_enc_ticks;
// 			this->prev_right_wheel_enc_ticks = this->right_wheel_enc_ticks;

 	//	}

	}

private:
	void set_joint_states(const sensor_msgs::JointState& joint_msg) {
	
		ROS_INFO("set_joint_states Callback triggered: %s", joint_msg.name[0].c_str());
		ROS_INFO("Pos: %f", joint_msg.position[0]);
		// if(!("shift" == joint_msg.name))
		// {
		// 	ROS_INFO("Shifting");

		// }
		
		// if(!("lift" == joint_msg.name))
		// {
		// 	ROS_INFO("Lifting");

		// }

		
		//ROS_INFO("Velo: left : %f right: %f",left_wheel_linear_vel,right_wheel_linear_vel);

	}

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ForkLiftPlugin)
}
