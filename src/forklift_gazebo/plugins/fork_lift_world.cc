#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <ros/ros.h>

#define NUM_BOTS 				1


namespace gazebo {
class ForkLiftWorld: public WorldPlugin {

private:
	physics::WorldPtr world;

private:
	void addModel(std::string filename, std::string model_name, double x,
			double y, double z) {
		sdf::SDFPtr modelSDF;
		modelSDF.reset(new sdf::SDF);
		std::string file_name = common::ModelDatabase::Instance()->GetModelFile(
				filename);

		sdf::initFile("root.sdf", modelSDF);
		sdf::readFile(file_name, modelSDF);
		sdf::ElementPtr modelElem;

		if (modelSDF->Root()->HasElement("model")) {
			modelElem = modelSDF->Root()->GetElement("model");
		}

		std::string modelName = modelElem->GetAttribute("name")->GetAsString();

		modelName = model_name;

		modelElem->GetAttribute("name")->Set(modelName);
		modelElem->GetElement("pose")->Set(
				math::Pose(
						math::Vector3(x, y, z),
						math::Quaternion(0, 0, 0)));
		this->world->InsertModelSDF(*modelSDF);
	}

public:
	void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/) {

		this->world = _parent;

		/* Add Bots */
		ROS_INFO("Total bots: %d", NUM_BOTS);

		
			addModel("model://fork_lift_shift",
					"forklift" , 0.0, 0.0, 0.0);


		/* Add Racks */

		//addModel("model://MSU", "MSU", 1.65, 0, 0);
	}

};

GZ_REGISTER_WORLD_PLUGIN(ForkLiftWorld)
}
