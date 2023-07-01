#include <iostream>


//TODO: remove unnecesssary libraries
#include "drake/common/eigen_types.h"
#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"

#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"

#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

#include "drake/geometry/drake_visualizer.h"


#include <drake/manipulation/kuka_iiwa/iiwa_constants.h>

#define KUKA_DT 0.002
#define KUKA_MODEL_PATH "drake/manipulation/models/iiwa_description/urdf/iiwa14_polytope_collision.urdf"



namespace drake {
namespace manipulation {
namespace kuka_iiwa {

    int runMain()
    {
        systems::DiagramBuilder<double> builder;

        //Adding all the "blocks" to the diagrams

        //args (systems::DiagramBuilder<double>*, double timestep)
        auto [plant, scene_graph] = multibody::AddMultibodyPlantSceneGraph(&builder, KUKA_DT);
        const std::string urdf = FindResourceOrThrow(KUKA_MODEL_PATH);

        //create iiwa
        auto iiwa_instance = multibody::Parser(&plant, &scene_graph).AddModels(urdf).at(0);
        
        //base is from urdf file, u would have to read it. just weld base to the ground
        plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"));
        plant.Finalize();


        //define constants
        const int num_joints = plant.num_positions();

        //shitty pid gains
        Eigen::VectorXd kp = Eigen::VectorXd::Zero(num_joints);
        Eigen::VectorXd ki = Eigen::VectorXd::Zero(num_joints);
        Eigen::VectorXd kd = Eigen::VectorXd::Zero(num_joints);
        kp.fill(0.3); //PD controller
        auto controller = builder.AddSystem<systems::controllers::InverseDynamicsController<double>>(plant,kp,ki,kd,false);

        drake::VectorX<double> q_des(7);
        q_des << -0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7;
        drake::VectorX<double> v_des(7);
        v_des << -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1;
        drake::VectorX<double> desired_state_input(2 * kIiwaArmNumJoints);
        desired_state_input << q_des, v_des;

        auto desired_state_source = builder.AddSystem<systems::ConstantVectorSource<double>>(desired_state_input);
        desired_state_source->set_name("desired_state_constant");

        //create visualizer
        geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);

        // create diagram
        builder.Connect(plant.get_state_output_port(),
                        controller->get_input_port_estimated_state());
        builder.Connect(controller->get_output_port_control(),
                        plant.get_actuation_input_port(iiwa_instance));
        builder.Connect(desired_state_source->get_output_port(), controller->get_input_port_desired_state());
        
        auto sys = builder.Build();
        systems::Simulator<double> simulator(*sys);

        simulator.set_publish_every_time_step(false);
        simulator.set_target_realtime_rate(1);
        simulator.Initialize();

        // Simulate for a very long time.
        simulator.AdvanceTo(300);
        return -1;
    }
}//namespace kuka_iiwa
}//namespace manipulation
}//namespace drake

int main()
{
    drake::manipulation::kuka_iiwa::runMain();
    return 0;
}