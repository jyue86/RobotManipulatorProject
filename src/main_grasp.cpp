/**
 * Manipulation Basic Pick and Place
 * ==================================
 * At this point, we should be able to get a robot and gripper to move around.
 * Our goal now is to perform grasping.
 *  In order to do so, we need to have an understanding of forward/inverse
 * kinematics and and how that integrates into the drake libary / other
 * libraries.
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

#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/leaf_system.h>

#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/integrator.h>
#include <drake/systems/primitives/trajectory_source.h>
#include <drake/systems/primitives/discrete_derivative.h>

#include <drake/geometry/drake_visualizer.h>

#include <drake/manipulation/kuka_iiwa/iiwa_constants.h>
#include <drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h>

#include <drake/math/rigid_transform.h>
#include <drake/math/roll_pitch_yaw.h>
#include <drake/math/rotation_matrix.h>
#include <exception>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <drake/common/trajectories/piecewise_pose.h>
#include <iostream>
#include <map>
#include <vector>
#include <cmath>

// define constants needed here
#define MULTIBODY_DT 0.002
#define ARM_PATH                                                               \
  "drake/manipulation/models/iiwa_description/urdf/"                           \
  "iiwa14_polytope_collision.urdf"
#define GRIPPER_PATH                                                           \
  "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50_no_tip.sdf"
#define BRICK_PATH                                                             \
  "drake/examples/manipulation_station/models/061_foam_brick.sdf"

#define M_PI 3.14159265358979323846

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

/**
 * Get robot to follow a setpoint. (grasp)
*/

int runMain() {
  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, MULTIBODY_DT);
  multibody::MultibodyPlant<double> plant_arm(MULTIBODY_DT);

  const std::string urdf = FindResourceOrThrow(ARM_PATH);
  const std::string gripper_sdf = FindResourceOrThrow(GRIPPER_PATH);
  const std::string brick_sdf = FindResourceOrThrow(BRICK_PATH);

  auto parser = multibody::Parser(&plant, &scene_graph);
  parser.SetAutoRenaming(
      true); // needed to differentiate multiple objects of same type

  auto armParser = multibody::Parser(&plant_arm);
  auto armParserInstance = armParser.AddModels(urdf).at(0);

  plant_arm.WeldFrames(plant_arm.world_frame(),
                       plant_arm.GetFrameByName("base"));
  plant_arm.Finalize();

  auto arm_instance = parser.AddModels(urdf).at(0);
  auto gripper_instance = parser.AddModels(gripper_sdf).at(0);
  auto brick_instance = parser.AddModels(brick_sdf).at(0);


  const math::RigidTransform<double> X_7G(math::RollPitchYaw<double>(M_PI_2, 0, M_PI_2),
                                    Eigen::Vector3d(0, 0, 0.114));

  drake::Vector3<double> brick_t(1, 0.35, 0);
  math::RigidTransform<double> brick_pose0 =
      math::RigidTransform<double>::Identity();
  brick_pose0.set_translation(brick_t);

  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"));
  plant.WeldFrames(plant.GetFrameByName("iiwa_link_7"),
                   plant.GetFrameByName("body", gripper_instance), X_7G);

  plant.Finalize();
  const int num_joints = plant_arm.num_positions();


  auto desired_state_from_position = builder.AddSystem<
        systems::StateInterpolatorWithDiscreteDerivative>(
            7, plant.time_step(),
            true /* suppress_initial_transient */);
  desired_state_from_position->set_name("desired_state_from_position");

  //base controller for robot
  Eigen::VectorXd kp = Eigen::VectorXd::Zero(kIiwaArmNumJoints);
  Eigen::VectorXd ki = Eigen::VectorXd::Zero(kIiwaArmNumJoints);
  Eigen::VectorXd kd = Eigen::VectorXd::Zero(kIiwaArmNumJoints);  
  kp.fill(100); // PD controller
  ki.fill(0);
  kd.fill(200);

  //setting up gripper contrlyoller (doesn't really do anything)
  auto gripperDesiredStateSource =
      builder.AddSystem<systems::ConstantVectorSource<double>>(
          systems::BasicVector<double>{0, 0});
  gripperDesiredStateSource->set_name("gripper_desired_state_constant");
  auto wsgController =
      builder.AddSystem<manipulation::schunk_wsg::SchunkWsgPdController>();

  auto armController = builder.AddSystem<systems::controllers::InverseDynamicsController<double>>(plant_arm, kp, ki, kd, false);

  drake::VectorX<double> q_des(7);
  q_des.fill(1);
  drake::VectorX<double> desired_pos_input(num_joints);
  desired_pos_input << q_des;

  auto desired_pos_source = builder.AddSystem<systems::ConstantVectorSource<double>>(desired_pos_input);
  desired_pos_source->set_name("desired_pos_source");

  builder.Connect(gripperDesiredStateSource->get_output_port(),
                  wsgController->get_desired_state_input_port());
  builder.Connect(wsgController->get_generalized_force_output_port(),
                  plant.get_actuation_input_port(gripper_instance));
  builder.Connect(plant.get_state_output_port(gripper_instance),
                  wsgController->get_state_input_port());

  builder.Connect(plant.get_state_output_port(arm_instance),
                  armController->get_input_port_estimated_state());
  builder.Connect(desired_pos_source->get_output_port(), desired_state_from_position->get_input_port());

  builder.Connect(armController->get_output_port_control(),
                  plant.get_actuation_input_port(arm_instance));
                  
  builder.Connect(desired_state_from_position->get_output_port(),
                    armController->get_input_port_desired_state());

  // start visual + simulation
  geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);

  auto sys = builder.Build();

  systems::Simulator<double> simulator(*sys);  

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(1);
  simulator.Initialize();
  simulator.AdvanceTo(100.0);
  return 0;
}

} // namespace kuka_iiwa
} // namespace manipulation
} // namespace drake

int main() {
  drake::manipulation::kuka_iiwa::runMain();
  return 0;
}
