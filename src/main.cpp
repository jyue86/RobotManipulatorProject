/**
 * Manipulation Basic Pick and Place
 * ==================================
 * At this point, we should be able to get a robot and gripper to move around.
 * Our goal now is to perform grasping.
 *  In order to do so, we need to have an understanding of forward/inverse kinematics and 
 *  and how that integrates into the drake libary / other libraries.
 * 
 * This code......
 * 
 * 
*/

#include <drake/common/eigen_types.h>
#include <drake/common/find_resource.h>

#include <drake/geometry/drake_visualizer.h>
#include <drake/geometry/scene_graph.h>

#include <drake/multibody/parsing/parser.h>

#include <drake/systems/analysis/simulator.h>
#include <drake/systems/controllers/inverse_dynamics_controller.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/constant_vector_source.h>

#include <drake/geometry/drake_visualizer.h>

#include <drake/manipulation/kuka_iiwa/iiwa_constants.h>

#include <iostream>

//define constants needed here
#define MULTIBODY_DT 0.002
#define ARM_PATH "drake/manipulation/models/iiwa_description/urdf/iiwa14_polytope_collision.urdf"
#define GRIPPER_PATH "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50_no_tip.sdf"
#define BRICK_PATH "drake/examples/manipulation_station/models/061_foam_brick.sdf"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

//NOTE: UNUSABLE
//TODO: integrate software with manipulation station or use inspired software from manipulation station.
//      -> we are interested in separating the state ports of the MultibodyPlant to not use the foam brick.
int runMain()
{
    systems::DiagramBuilder<double> builder;
    auto [plant, scene_graph] = multibody::AddMultibodyPlantSceneGraph(&builder, MULTIBODY_DT);
    const std::string urdf = FindResourceOrThrow(ARM_PATH);
    const std::string gripper_sdf = FindResourceOrThrow(GRIPPER_PATH);
    const std::string brick_sdf = FindResourceOrThrow(BRICK_PATH);

    auto arm_instance = multibody::Parser(&plant, &scene_graph).AddModels(urdf).at(0);
    auto gripper_instance = multibody::Parser(&plant, &scene_graph).AddModels(gripper_sdf).at(0);
    auto brick_instance = multibody::Parser(&plant, &scene_graph).AddModels(brick_sdf).at(0);
    
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"));
    plant.WeldFrames(plant.GetFrameByName("iiwa_link_7"), plant.GetFrameByName("body"));
    plant.Finalize();

    const int num_joints = plant.num_positions(); //9 actuators total (7 iiwa, 2 gripper) .... (this returns 16 positions for brick)


    const int num_joints_iiwa = 7;
    //shitty pid gains
    Eigen::VectorXd kp = Eigen::VectorXd::Zero(num_joints);
    Eigen::VectorXd ki = Eigen::VectorXd::Zero(num_joints);
    Eigen::VectorXd kd = Eigen::VectorXd::Zero(num_joints);
    kp.fill(0.3); //PD controller
    auto controller = builder.AddSystem<systems::controllers::InverseDynamicsController<double>>(plant,kp,ki,kd,false);

    drake::VectorX<double> q_des(num_joints);
    q_des.fill(1);
    drake::VectorX<double> v_des(num_joints);
    v_des.fill(0.0);
    drake::VectorX<double> desired_state_input(2 * num_joints);
    desired_state_input << q_des, v_des;

    auto desired_state_source = builder.AddSystem<systems::ConstantVectorSource<double>>(desired_state_input);
    desired_state_source->set_name("desired_state_constant");

    //connect only arm/iiwa
    builder.Connect(plant.get_state_output_port(),controller->get_input_port_estimated_state());
    builder.Connect(controller->get_output_port_control(),plant.get_actuation_input_port());

    builder.Connect(desired_state_source->get_output_port(), controller->get_input_port_desired_state());

    //start visual + simulation
    geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);

    auto sys = builder.Build();
    systems::Simulator<double> simulator(*sys);

    simulator.set_publish_every_time_step(false);
    simulator.set_target_realtime_rate(1);
    simulator.Initialize();

    simulator.AdvanceTo(300);
    return 0;
}

}//namespace kuka_iiwa
}//namespace manipulation
}//namespace drake

int main()
{
    drake::manipulation::kuka_iiwa::runMain();
    return 0;
}