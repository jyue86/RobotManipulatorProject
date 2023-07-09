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

#include <drake/math/rotation_matrix.h>
#include <drake/multibody/parsing/parser.h>

#include <drake/systems/analysis/simulator.h>
#include <drake/systems/controllers/inverse_dynamics_controller.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/primitives/constant_vector_source.h>

#include <drake/geometry/drake_visualizer.h>

#include <drake/manipulation/kuka_iiwa/iiwa_constants.h>
#include <drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h>

#include <drake/math/rigid_transform.h>
#include <drake/math/roll_pitch_yaw.h>
#include <math.h>

#include <iostream>
#include <map>
#include <set>

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

int runMain() {
  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, MULTIBODY_DT);
  multibody::MultibodyPlant<double> plant_arm(MULTIBODY_DT);

  const std::string urdf = FindResourceOrThrow(ARM_PATH);
  const std::string gripper_sdf = FindResourceOrThrow(GRIPPER_PATH);
  const std::string brick_sdf = FindResourceOrThrow(BRICK_PATH);

  auto parser = multibody::Parser(&plant, &scene_graph);
  parser.SetAutoRenaming(true); // needed to differentiate multiple objects of same type
  

  math::RigidTransform<double> schunk_pose = math::RigidTransform<double>::Identity();
  drake::Vector3<double> schunk_t(0, 0, 0);
  schunk_pose.set_rotation(math::RollPitchYaw<double>(M_PI / 2, 0, 0));
  schunk_pose.set_translation(schunk_t);

  drake::Vector3<double> brick_t(0.65, 0.65, 0);
  math::RigidTransform<double> brick_pose0 = math::RigidTransform<double>::Identity();
  brick_pose0.set_translation(brick_t);

  // Gripper to brick
  Vector3<double> p_GgraspO(0, 0.11, 0);
  math::RotationMatrixd r_Ggrasp0 = math::RotationMatrixd(math::RotationMatrixd::MakeXRotation(M_PI / 2) * math::RotationMatrixd::MakeZRotation(M_PI / 2));
  math::RigidTransform<double> X_OGgrasp = math::RigidTransform(r_Ggrasp0, p_GgraspO).inverse();
  math::RigidTransform<double> X_GgraspGpregrasp(Vector3<double>(0, -0.08, 0));

  std::map<std::string, math::RigidTransform<double>> X_O{
      {"initial", brick_pose0},
      {"goal", math::RigidTransform<double>(Vector3<double>(0, -0.5, 0))}};
  std::map<std::string, math::RigidTransform<double>> X_G{
      {"initial", math::RigidTransform<double>(
                      math::RotationMatrixd::MakeXRotation(-M_PI / 2),
                      Vector3<double>(0, -0.25, 0.25))}};
  X_G["pick"] = X_O["initial"] * X_OGgrasp;
  X_G["prepick"] = X_G["pick"] * X_GgraspGpregrasp;
  X_G["place"] = X_O["goal"] * X_OGgrasp;
  X_G["preplace"] = X_G["place"] * X_GgraspGpregrasp;

  // interpolating halfway pose
  math::RigidTransform<double> X_GprepickGpreplace =
      X_G["prepick"].inverse() * X_G["preplace"];
  Eigen::AngleAxis<double> angleAxis =
      X_GprepickGpreplace.rotation().ToAngleAxis();
  math::RigidTransform<double> X_GprepickGclearance =
      math::RigidTransform<double>(
          Eigen::AngleAxis<double>(angleAxis.angle() / 2, angleAxis.axis()),
          X_GprepickGpreplace.translation() / 2 + Vector3<double>(0, -0.3, 0));
  X_G["clearance"] = X_G["prepick"] * X_GprepickGclearance;

  // timing
  std::map<std::string, double> times{{"initial", 0}};
  math::RigidTransform<double> X_GinitialGprepick =
      X_G["initial"].inverse() * X_G["prepick"];
  times["prepick"] =
      times["initial"] + 10 + X_GinitialGprepick.translation().norm();

  times["pick_start"] = times["prepick"] + 2;
  times["pick_end"] = times["pick_start"] + 2;
  X_G["pick_start"] = X_G["pick"];
  X_G["pick_end"] = X_G["pick"];
  times["postpick"] = times["pick_end"] + 2;
  X_G["postpick"] = X_G["prepick"];

  double timeToFromClearance = 10 + X_GprepickGclearance.translation().norm();
  times["clearance"] = times["postpick"] + timeToFromClearance;
  times["preplace"] = times["clearance"] + timeToFromClearance;
  times["place_start"] = times["preplace"] + 2;
  times["place_end"] = times["place_start"] + 2;
  X_G["place_start"] = X_G["place"];
  X_G["place_end"] = X_G["place"];
  times["postplace"] = times["place_end"] + 2.0;
  X_G["postplace"] = X_G["preplace"];
  std::cout << times["postplace"] << std::endl;

  // Weld bricks down for trajectory visualization
  for (auto item : X_O) {
    math::RigidTransform<double> X = item.second;

    auto brick_instance = parser.AddModels(brick_sdf).at(0);
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("base_link", brick_instance), X);
  }

  // Weld grippers down for trajectory visualization
  std::set<std::string> allowedGripperPoses{
      "initial", "prepick", "pick", "place", "preplace", "clearance", "goal"};
  for (auto item : X_G) {
    if (allowedGripperPoses.count(item.first) == 0)
      continue;
    math::RigidTransform<double> X = item.second;
    auto gripper_instance = parser.AddModels(gripper_sdf).at(0);
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("body", gripper_instance), X);
  }
  
  plant.Finalize();
  multibody::Parser parser_arm(&plant_arm);
  parser_arm.AddModels(urdf);
  plant_arm.WeldFrames(plant_arm.world_frame(),
                       plant_arm.GetFrameByName("base"));
  plant_arm.Finalize();



  // start visual + simulation
  geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);

  auto sys = builder.Build();

  systems::Simulator<double> simulator(*sys);

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(1);
  simulator.Initialize();

  // Commented out so no Logic error complaining about actuation input error not
  // connected
  // simulator.AdvanceTo(300);
  return 0;
}

} // namespace kuka_iiwa
} // namespace manipulation
} // namespace drake

int main() {
  drake::manipulation::kuka_iiwa::runMain();
  return 0;
}
