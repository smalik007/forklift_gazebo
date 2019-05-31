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

	ros::Subscriber forklift_joint_state;
	ros::NodeHandle nh_;
	physics::ModelPtr model;
	event::ConnectionPtr updateConnection;
	std::string model_name;
	physics::JointPtr fork_shift, fork_lift;


public:
	void Init() {
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
	}

private:
	void set_joint_states(const sensor_msgs::JointState& joint_msg) {
	
		ROS_INFO("set_joint_states Callback triggered: %s", joint_msg.name[0].c_str());
		ROS_INFO("Pos: %f", joint_msg.position[0]);
		bool status;
		
		if(!strcmp("shift", joint_msg.name[0].c_str()))
		{
		 	ROS_INFO("Shifting by pos: %f", joint_msg.position[0]);
			status = this->fork_shift->SetPosition(0,  joint_msg.position[0]);
			ROS_INFO("Status: %d", status);
			
		}

		else if(!strcmp("lift", joint_msg.name[0].c_str()))
		{
		 	ROS_INFO("Lift by pos: %f", joint_msg.position[0]);
			status = this->fork_lift->SetPosition(0,  joint_msg.position[0]);
			ROS_INFO("Status: %d", status);
		}
		else
		{
			/* code */
		}
		
	}

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ForkLiftPlugin)
}
