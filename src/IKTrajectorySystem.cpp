#include <chrono>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/solvers/solve.h>
#include <drake/systems/framework/leaf_system.h>
#include <iostream>
#include <vector>

namespace drake {
namespace manipulation {
namespace kuka_iiwa {
class IKTrajectorySystem : public systems::LeafSystem<double> {
public:
  IKTrajectorySystem(
      multibody::MultibodyPlant<double> &plant,
      const std::vector<math::RigidTransform<double>> &positions) {
    plant_ = &plant;
    G_ = plant.GetModelInstanceByName("Schunk_Gripper");
    // positions_ = positions;

    for (math::RigidTransform<double> goalPose : positions) {
      multibody::InverseKinematics ik(*plant_, true);
      ik.AddPositionConstraint(plant_->GetFrameByName("body", G_),
                               Eigen::Vector3d::Zero(3), plant_->world_frame(),
                               goalPose.translation(), goalPose.translation());
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
    }

    DeclareVectorOutputPort("iiwa_position", 7,
                            &IKTrajectorySystem::CalcOutput);

    startTime_ = std::chrono::system_clock::now();
  }

private:
  void CalcOutput(const systems::Context<double> &context,
                  systems::BasicVector<double> *output) const {
    output->set_value(positions_[0]);
  }

  multibody::MultibodyPlant<double> *plant_;
  multibody::ModelInstanceIndex G_;
  std::vector<Eigen::VectorX<double>> positions_;

  std::chrono::time_point<std::chrono::system_clock> startTime_;
};
} // namespace kuka_iiwa
} // namespace manipulation
} // namespace drake
