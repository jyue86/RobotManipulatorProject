#include <chrono>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/solvers/solve.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/input_port.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/output_port.h>
#include <iostream>
#include <vector>

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

using time = std::chrono::time_point<std::chrono::system_clock>;

class IKTrajectorySystem : public systems::LeafSystem<double> {
public:
  IKTrajectorySystem(
      multibody::MultibodyPlant<double> &plant,
      const std::vector<math::RigidTransform<double>> &positions) {
    // iiwaInputPort_ = &DeclareVectorInputPort("iiwa_current_state", 7);
    // brickInputPort_ = &DeclareVectorInputPort("brick_current_state", 7);
    // wsgInputPort_ = &DeclareVectorInputPort("wsg_current_state", 2);

    plant_ = &plant;
    G_ = plant.GetModelInstanceByName("Schunk_Gripper");

    for (int i = 0; i < positions.size(); i++) {
      math::RigidTransform<double> goalPose = positions[i];
      if (i == 0) {
        multibody::InverseKinematics ik(*plant_, true);
        ik.AddPositionConstraint(plant_->GetFrameByName("body", G_),
                                 Eigen::Vector3d::Zero(3),
                                 plant_->world_frame(), goalPose.translation(),
                                 goalPose.translation());
        ik.AddOrientationConstraint(
            plant_->GetFrameByName("body", G_), math::RotationMatrix<double>(),
            plant_->world_frame(), goalPose.rotation(), 0.0);
        solvers::MathematicalProgram *prog = ik.get_mutable_prog();
        solvers::VectorXDecisionVariable q = ik.q();
        prog->AddQuadraticErrorCost(Eigen::MatrixXd::Identity(16, 16),
                                    Eigen::VectorXd::Ones(16), q);
        prog->SetInitialGuess(q, Eigen::VectorXd::Ones(16));
        solvers::MathematicalProgramResult result = solvers::Solve(ik.prog());
        Eigen::VectorXd solution = result.GetSolution(ik.q());

        Eigen::VectorX<double> qVec = solution(Eigen::seq(0, 6));
        std::cout << "Qvec: " << qVec << std::endl;
        positions_.push_back(qVec);
      } else {
        Eigen::VectorXd initialGuess(16);
        initialGuess << positions_[i - 1], Eigen::VectorXd::Ones(9);
        multibody::InverseKinematics ik(*plant_, true);
        ik.AddPositionConstraint(plant_->GetFrameByName("body", G_),
                                 Eigen::Vector3d::Zero(3),
                                 plant_->world_frame(), goalPose.translation(),
                                 goalPose.translation());
        ik.AddOrientationConstraint(
            plant_->GetFrameByName("body", G_), math::RotationMatrix<double>(),
            plant_->world_frame(), goalPose.rotation(), 0.0);
        solvers::MathematicalProgram *prog = ik.get_mutable_prog();
        solvers::VectorXDecisionVariable q = ik.q();
        prog->AddQuadraticErrorCost(Eigen::MatrixXd::Identity(16, 16),
                                    Eigen::VectorXd::Ones(16), q);
        prog->SetInitialGuess(q, initialGuess);
        solvers::MathematicalProgramResult result = solvers::Solve(ik.prog());
        Eigen::VectorXd solution = result.GetSolution(ik.q());

        Eigen::VectorX<double> qVec = solution(Eigen::seq(0, 6));
        std::cout << "Qvec: " << qVec << std::endl;
        positions_.push_back(qVec);
      }
    }

    iiwaOutputPort_ = &DeclareVectorOutputPort(
        "iiwa_position", 7, &IKTrajectorySystem::ManipulateArm);
    wsgOutputPort_ = &DeclareVectorOutputPort(
        "wsg_position", 2, &IKTrajectorySystem::ManipulateGripper);

    startTime_ = std::chrono::system_clock::now();
    currentStep_ = 0;
  }

private:
  void ManipulateArm(const systems::Context<double> &context,
                     systems::BasicVector<double> *output) const {
    time currentTime = std::chrono::system_clock::now();
    int seconds = std::chrono::duration_cast<std::chrono::milliseconds>(
                      currentTime - startTime_)
                      .count() /
                  1000;

    if (seconds < 10) {
      std::cout << "Stage 1" << std::endl;
      output->set_value(positions_[0]);
    } else if (seconds < 15) {
      std::cout << "Stage 2" << std::endl;
      output->set_value(positions_[1]);
    } else if (seconds < 20) {
      std::cout << "Stage 3" << std::endl;
      output->set_value(positions_[1]);
    } else if (seconds < 25) {
      std::cout << "Stage 4" << std::endl;
      output->set_value(positions_[0]);
    } else {
      std::cout << "Stage 5" << std::endl;
      output->set_value(positions_[2]);
    }
  }

  void ManipulateGripper(const systems::Context<double> &context,
                         systems::BasicVector<double> *output) const {
    time currentTime = std::chrono::system_clock::now();
    int seconds = std::chrono::duration_cast<std::chrono::milliseconds>(
                      currentTime - startTime_)
                      .count() /
                  1000;

    if (seconds < 10) {
      output->set_value(Eigen::Vector2d(0.1, 0.1));
    } else if (seconds < 15) {
      output->set_value(Eigen::Vector2d(0.1, 0.1));
    } else if (seconds < 20) {
      std::cout << "GRIPPER BETTER CLOSE" << std::endl;
      output->set_value(Eigen::Vector2d(-0.5, -0.5));
    } else if (seconds < 30) {
      output->set_value(Eigen::Vector2d(-0.5, -0.5));
    } else {
      output->set_value(Eigen::Vector2d(0.1, 0.1));
    }
  }

  multibody::MultibodyPlant<double> *plant_;
  multibody::ModelInstanceIndex G_;
  systems::InputPort<double> *iiwaInputPort_;
  // systems::InputPort<double> *wsgInputPort_;
  // systems::InputPort<double> *brickInputPort_;
  const systems::OutputPort<double> *iiwaOutputPort_{};
  const systems::OutputPort<double> *wsgOutputPort_{};

  std::vector<Eigen::VectorX<double>> positions_;
  time startTime_;
  int currentStep_;
};
} // namespace kuka_iiwa
} // namespace manipulation
} // namespace drake
