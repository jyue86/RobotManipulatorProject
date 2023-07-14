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
#define MULTIBODY_DT 0.1
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

using TrajHashBrown = std::map<std::string, math::RigidTransform<double>>;
using TimeHashBrown = std::map<std::string, double>;

TimeHashBrown makeGripperFrames(TrajHashBrown &X_G, TrajHashBrown X_O) {
  Vector3<double> p_GgraspO(0, 0.11, 0);
  math::RotationMatrixd r_Ggrasp0 =
      math::RotationMatrixd(math::RotationMatrixd::MakeXRotation(M_PI / 2) *
                            math::RotationMatrixd::MakeZRotation(M_PI / 2));
  math::RigidTransform<double> X_OGgrasp =
      math::RigidTransform(r_Ggrasp0, p_GgraspO).inverse();
  math::RigidTransform<double> X_GgraspGpregrasp(Vector3<double>(0, -0.08, 0));

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
  TimeHashBrown times{{"initial", 0}};
  math::RigidTransform<double> X_GinitialGprepick =
      X_G["initial"].inverse() * X_G["prepick"];
  times["prepick"] =
      times["initial"] + 10 + X_GinitialGprepick.translation().norm();

  times["pick_start"] = times["prepick"] + 2.0;
  times["pick_end"] = times["pick_start"] + 2.0;
  X_G["pick_start"] = X_G["pick"];
  X_G["pick_end"] = X_G["pick"];
  times["postpick"] = times["pick_end"] + 2.0;
  X_G["postpick"] = X_G["prepick"];

  double timeToFromClearance = 10.0 * X_GprepickGclearance.translation().norm();
  times["clearance"] = times["postpick"] + timeToFromClearance;
  times["preplace"] = times["clearance"] + timeToFromClearance;
  times["place_start"] = times["preplace"] + 2.0;
  times["place_end"] = times["place_start"] + 2.0;
  X_G["place_start"] = X_G["place"];
  X_G["place_end"] = X_G["place"];
  times["postplace"] = times["place_end"] + 2.0;
  X_G["postplace"] = X_G["preplace"];
  // std::cout << times["postplace"] << std::endl;

  return times;
}

/**
 * UNTESTED: test at your own risk
 */
trajectories::PiecewisePose<double>
MakeGripperPoseTrajectory(const TrajHashBrown &X_G, TimeHashBrown &times) {
  std::vector<double> times_vec;
  std::vector<math::RigidTransform<double>> pose_vec;
  std::vector<std::string> names = {
      "initial",   "prepick",  "pick_start",  "pick_end",  "postpick",
      "clearance", "preplace", "place_start", "place_end", "postplace"};
  for (const std::string &name : names) {
    pose_vec.push_back(X_G.at(name));
    times_vec.push_back(times.at(name)); // hope to god it doesn't crash
  }

  return trajectories::PiecewisePose<double>::MakeLinear(times_vec, pose_vec);
}

class PseudoInverseController : public systems::LeafSystem<double> {
public:
  PseudoInverseController(multibody::MultibodyPlant<double> &plant) {
    plant_ = &plant;
    context_ = plant.CreateDefaultContext();
    iiwa_ = plant.GetModelInstanceByName("iiwa14");
    G_ = &(plant.GetBodyByName("body").body_frame());
    W_ = &(plant.world_frame());

    // input port 0 have size 6
    V_G_port = &(DeclareVectorInputPort("V_WG", 6));

    // input port 1 have size 7
    q_port = &(DeclareVectorInputPort("iiwa_position", 14));

    DeclareVectorOutputPort("iiwa_velocity", 7,
                            &PseudoInverseController::CalcOutput);

    iiwa_start = plant.GetJointByName("iiwa_joint_1").velocity_start();
    iiwa_end = plant.GetJointByName("iiwa_joint_7").velocity_start();
  }

private:
  void CalcOutput(const systems::Context<double> &context,
                  systems::BasicVector<double> *output) const {
    auto V_G = V_G_port->Eval(context);
    auto qq = q_port->Eval(context);
    auto q = qq(Eigen::seq(0, 6));
    // std::cout << "Q size: " << q.size() << std::endl;
    // q = q(Eigen::seq(0, 7)); // take first 7 (position);

    plant_->SetPositions(context_.get(), iiwa_, q);
    // std::cout << plant_->num_velocities() << std::endl;
    MatrixX<double> J_G = MatrixX<double>(6, plant_->num_velocities());
    // std::cout << "Cols: " << J_G.cols() << std::endl;

    drake::Vector3<double> zeros;
    zeros << 0, 0, 0;
    plant_->CalcJacobianSpatialVelocity(*(context_.get()),
                                        multibody::JacobianWrtVariable::kV, *G_,
                                        zeros, *W_, *W_, &J_G);

    auto J_G_val = J_G;
    // std::cout << "J_G val:" << J_G_val << std::endl;
    auto realJ_G = J_G_val(Eigen::all, Eigen::seq(iiwa_start, iiwa_end));
    std::cout << "Real J_G:" << realJ_G << std::endl;

    auto v = realJ_G.completeOrthogonalDecomposition().pseudoInverse() * V_G;
    // std::cout << "Basic Vector shape: " << v.size() << std::endl;
    // std::cout << "V: " << v << std::endl;
    output->SetFromVector(v);
  }

  multibody::MultibodyPlant<double> *plant_;
  std::unique_ptr<systems::Context<double>> context_;
  multibody::ModelInstanceIndex iiwa_;
  const multibody::BodyFrame<double> *G_;
  const multibody::BodyFrame<double> *W_;

  systems::InputPort<double> *V_G_port;
  systems::InputPort<double> *q_port;

  int iiwa_start, iiwa_end;
};

class DummyPosVel : public systems::LeafSystem<double> {
public:
  DummyPosVel() {
    Q_port = &(DeclareVectorInputPort("iiwa_position", 7));
    V_port = &(DeclareVectorInputPort("iiwa_velocity", 7));

    DeclareVectorOutputPort("iiwa_pos_velocity", 14, &DummyPosVel::CalcOutput);
  }

private:
  void CalcOutput(const systems::Context<double> &context,
                  systems::BasicVector<double> *output) const {
    auto q = Q_port->Eval(context);
    auto v = V_port->Eval(context);

    VectorX<double> vecJoined(q.size() + v.size());
    vecJoined << q, v;
    output->SetFromVector(vecJoined);
  }

  systems::InputPort<double> *Q_port;
  systems::InputPort<double> *V_port;
};

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

  math::RigidTransform<double> schunk_pose =
      math::RigidTransform<double>::Identity();
  drake::Vector3<double> schunk_t;
  schunk_t << 0, 0, 0;
  schunk_pose.set_rotation(math::RollPitchYaw<double>(M_PI / 2, 0, 0));
  schunk_pose.set_translation(schunk_t);

  drake::Vector3<double> brick_t(0.35, 0.35, 0);
  math::RigidTransform<double> brick_pose0 =
      math::RigidTransform<double>::Identity();
  brick_pose0.set_translation(brick_t);

  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"));
  plant.WeldFrames(plant.GetFrameByName("iiwa_link_ee_kuka"),
                   plant.GetFrameByName("body", gripper_instance), schunk_pose);

  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("base_link", brick_instance),
                   brick_pose0);
  plant.Finalize();

  TrajHashBrown X_O{
      {"initial", brick_pose0},
      {"goal", math::RigidTransform<double>(Vector3<double>(0, -0.5, 0))}};
  TrajHashBrown X_G{
      {"initial", math::RigidTransform<double>(
                      math::RotationMatrixd::MakeXRotation(-M_PI / 2),
                      Vector3<double>(0, -0.25, 0.25))}};

  TimeHashBrown times = makeGripperFrames(X_G, X_O);

  trajectories::PiecewisePose<double> traj =
      MakeGripperPoseTrajectory(X_G, times);
  std::unique_ptr<trajectories::Trajectory<double>> traj_V_G =
      traj.MakeDerivative(); // might need std::move()

  auto V_G_source =
      builder.AddSystem<systems::TrajectorySource<double>>(*traj_V_G.get());
  V_G_source->set_name("v_WG");

  // pseudoinversecontroller IMPLEMENT
  auto controller = builder.AddSystem<PseudoInverseController>(plant);
  controller->set_name("PseudoInverseController");

  auto integrator = builder.AddSystem<systems::Integrator<double>>(7);
  integrator->set_name("integrator");

  auto intermediateSys = builder.AddSystem<DummyPosVel>();
  intermediateSys->set_name("IntermediateSys");

  Eigen::VectorXd kp = Eigen::VectorXd::Zero(kIiwaArmNumJoints);
  Eigen::VectorXd ki = Eigen::VectorXd::Zero(kIiwaArmNumJoints);
  Eigen::VectorXd kd = Eigen::VectorXd::Zero(kIiwaArmNumJoints);
  kp.fill(100); // PD controller
  ki.fill(1);
  kd.fill(2*std::sqrt(100));

  auto armController =
      builder
          .AddSystem<systems::controllers::InverseDynamicsController<double>>(
              plant_arm, kp, ki, kd, false);
  auto gripperDesiredStateSource =
      builder.AddSystem<systems::ConstantVectorSource<double>>(
          systems::BasicVector<double>{0, 0});
  gripperDesiredStateSource->set_name("gripper_desired_state_constant");
  auto wsgController =
      builder.AddSystem<manipulation::schunk_wsg::SchunkWsgPdController>();
  builder.Connect(gripperDesiredStateSource->get_output_port(),
                  wsgController->get_desired_state_input_port());
  builder.Connect(wsgController->get_generalized_force_output_port(),
                  plant.get_actuation_input_port(gripper_instance));
  builder.Connect(plant.get_state_output_port(gripper_instance),
                  wsgController->get_state_input_port());

  builder.Connect(V_G_source->get_output_port(),
                  controller->GetInputPort("V_WG"));
  builder.Connect(plant.get_state_output_port(arm_instance),
                  controller->GetInputPort("iiwa_position"));
  builder.Connect(controller->get_output_port(), integrator->get_input_port());
  builder.Connect(integrator->get_output_port(),
                  intermediateSys->GetInputPort("iiwa_position"));
  builder.Connect(controller->get_output_port(),
                  intermediateSys->GetInputPort("iiwa_velocity"));

  builder.Connect(intermediateSys->get_output_port(),
                  armController->get_input_port_desired_state());
  builder.Connect(plant.get_state_output_port(arm_instance),
                  armController->get_input_port_estimated_state());

  builder.Connect(armController->get_output_port_control(),
                  plant.get_actuation_input_port(arm_instance));

  // start visual + simulation
  geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);

  auto sys = builder.Build();

  systems::Simulator<double> simulator(*sys);
  // systems::Context<double> *context = &simulator.get_mutable_context();
  systems::Context<double> &context = simulator.get_mutable_context();
  systems::Context<double> *contextPtr =
      &(integrator->GetMyMutableContextFromRoot(&context));
  integrator->set_integral_value(
      contextPtr, plant.GetPositions(plant.GetMyContextFromRoot(context),
                                     plant.GetModelInstanceByName("iiwa14")));

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(1);
  simulator.Initialize();

  simulator.AdvanceTo(traj.get_position_trajectory().end_time());
  return 0;
}

} // namespace kuka_iiwa
} // namespace manipulation
} // namespace drake

int main() {
  drake::manipulation::kuka_iiwa::runMain();
  return 0;
}
