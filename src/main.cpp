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
#include <drake/systems/framework/leaf_system.h>

#include <drake/geometry/drake_visualizer.h>

#include <drake/manipulation/kuka_iiwa/iiwa_constants.h>
#include <drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h>

#include <drake/math/rigid_transform.h>
#include <drake/math/roll_pitch_yaw.h>
#include <math.h>

#include <iostream>

//define constants needed here
#define MULTIBODY_DT 0.002
#define ARM_PATH "drake/manipulation/models/iiwa_description/urdf/iiwa14_polytope_collision.urdf"
#define GRIPPER_PATH "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50_no_tip.sdf"
#define BRICK_PATH "drake/examples/manipulation_station/models/061_foam_brick.sdf"

# define M_PI 3.14159265358979323846


namespace drake {
namespace manipulation {
namespace kuka_iiwa {

//NOTE: UNUSABLE
//TODO: Got a better idea of how manipulation station works... proposed changes
//      -> create three multibody plants
//          -> plant for everything
//          -> plant for just robot arm
//          -> plant for just gripper
//      -> run inversedynamicscontroller, each for robot arm plant and gripper plant
//      -> connect "plant for everything" outputs into the controllers.... 
//              don't actually use the other multibody plants 

int runMain()
{
    systems::DiagramBuilder<double> builder;
    auto [plant, scene_graph] = multibody::AddMultibodyPlantSceneGraph(&builder, MULTIBODY_DT);
    multibody::MultibodyPlant<double> plant_arm(MULTIBODY_DT);

    const std::string urdf = FindResourceOrThrow(ARM_PATH);
    const std::string gripper_sdf = FindResourceOrThrow(GRIPPER_PATH);
    const std::string brick_sdf = FindResourceOrThrow(BRICK_PATH);

    auto parser = multibody::Parser(&plant, &scene_graph);
    auto arm_instance =     parser.AddModels(urdf).at(0);
    auto gripper_instance = parser.AddModels(gripper_sdf).at(0);
    auto brick_instance =   parser.AddModels(brick_sdf).at(0);
    
    math::RigidTransform<double> schunk_pose = math::RigidTransform<double>::Identity();
    drake::Vector3<double> schunk_t;
    schunk_t << 0,0,0;
    schunk_pose.set_rotation(math::RollPitchYaw<double>(M_PI/2,0,0));
    schunk_pose.set_translation(schunk_t);

    drake::Vector3<double> brick_t;
    brick_t << 1,0,0;
    math::RigidTransform<double> brick_pose0 = math::RigidTransform<double>::Identity();
    brick_pose0.set_translation(brick_t);

    //WeldFrames has a 3rd argument representing the pose of the object to be welded
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"));
    plant.WeldFrames(plant.GetFrameByName("iiwa_link_ee_kuka"), plant.GetFrameByName("body",gripper_instance), schunk_pose);

    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link", brick_instance), brick_pose0);
    plant.Finalize();


    multibody::Parser parser_arm(&plant_arm);
    parser_arm.AddModels(urdf).at(0);
    plant_arm.WeldFrames(plant_arm.world_frame(), plant_arm.GetFrameByName("base"));
    plant_arm.Finalize();

    const int num_joints = plant.num_positions(); //9 actuators total (7 iiwa, 2 gripper) .... (this returns 16 positions for brick)


    const int num_joints_iiwa = 7;
    //shitty pid gains
    Eigen::VectorXd kp = Eigen::VectorXd::Zero(kIiwaArmNumJoints);
    Eigen::VectorXd ki = Eigen::VectorXd::Zero(kIiwaArmNumJoints);
    Eigen::VectorXd kd = Eigen::VectorXd::Zero(kIiwaArmNumJoints);
    kp.fill(0.3); //PD controller
    auto controller = builder.AddSystem<systems::controllers::InverseDynamicsController<double>>(plant_arm,kp,ki,kd,false);

    drake::VectorX<double> q_des(kIiwaArmNumJoints);
    q_des.fill(1);
    drake::VectorX<double> v_des(kIiwaArmNumJoints);
    v_des.fill(0.0);
    drake::VectorX<double> desired_state_input(2 * kIiwaArmNumJoints);
    desired_state_input << q_des, v_des;

    drake::VectorX<double> zero_act_schunk(2);
    zero_act_schunk << 0, 0;

    auto desired_state_source = builder.AddSystem<systems::ConstantVectorSource<double>>(desired_state_input);
    desired_state_source->set_name("desired_state_constant");

    auto act_schunk = builder.AddSystem<systems::ConstantVectorSource<double>>(zero_act_schunk);
    act_schunk->set_name("act_schunk");

    //connect only arm/iiwa
    builder.Connect(plant.get_state_output_port(arm_instance),controller->get_input_port_estimated_state());
    builder.Connect(controller->get_output_port_control(),plant.get_actuation_input_port(arm_instance));

    builder.Connect(desired_state_source->get_output_port(), controller->get_input_port_desired_state());
    builder.Connect(act_schunk->get_output_port(), plant.get_actuation_input_port(gripper_instance));

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