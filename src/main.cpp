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

#include <drake/common/value.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/geometry/scene_graph.h>

#include <drake/multibody/parsing/parser.h>

#include <drake/systems/analysis/simulator.h>
#include <drake/systems/controllers/inverse_dynamics_controller.h>

#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/leaf_system.h>

#include <drake/geometry/drake_visualizer.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_point_cloud_visualizer.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/geometry/render/render_camera.h>
#include <drake/perception/depth_image_to_point_cloud.h>
#include <drake/systems/primitives/constant_value_source.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/integrator.h>
#include <drake/systems/sensors/rgbd_sensor.h>

#include <drake/manipulation/kuka_iiwa/iiwa_constants.h>
#include <drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h>

#include <drake/math/rigid_transform.h>
#include <drake/math/roll_pitch_yaw.h>
#include <drake/math/rotation_matrix.h>
#include <exception>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <vector>

// define constants needed here
#define MULTIBODY_DT 0.002
#define ARM_PATH                                                               \
  "drake/manipulation/models/iiwa_description/urdf/"                           \
  "iiwa14_polytope_collision.urdf"
#define GRIPPER_PATH                                                           \
  "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50_no_tip.sdf"
#define BRICK_PATH                                                             \
  "drake/examples/manipulation_station/models/061_foam_brick.sdf"

#define CAMERA_PATH "models/camera_box.sdf"
#define M_PI 3.14159265358979323846

namespace drake {

namespace manipulation {
namespace kuka_iiwa {

void runMain() {
  std::shared_ptr<geometry::Meshcat> meshCat =
      std::make_shared<geometry::Meshcat>();

  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, MULTIBODY_DT);
  multibody::MultibodyPlant<double> plant_arm(MULTIBODY_DT);

  const std::string urdf = FindResourceOrThrow(ARM_PATH);
  const std::string gripper_sdf = FindResourceOrThrow(GRIPPER_PATH);
  const std::string cam_sdf = CAMERA_PATH;

  auto parser = multibody::Parser(&plant, &scene_graph);
  auto arm_instance = parser.AddModels(urdf).at(0);
  auto gripper_instance = parser.AddModels(gripper_sdf).at(0);
  auto cam_instance = parser.AddModels(cam_sdf).at(0);

  const math::RigidTransform<double> X_7G(
      math::RollPitchYaw<double>(M_PI_2, 0, M_PI_2),
      Eigen::Vector3d(0, 0, 0.114));

  const math::RigidTransform<double> X_CAM(
      math::RollPitchYaw<double>(0, -0.2, 0.2), Eigen::Vector3d(0.5, 0.1, 0.2));

  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("base", arm_instance));
  plant.WeldFrames(plant.GetFrameByName("iiwa_link_7"),
                   plant.GetFrameByName("body", gripper_instance), X_7G);

  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("base", cam_instance), X_CAM);
  plant.Finalize();

  const int kHeight = 480;
  const int kWidth = 848;

  // From color camera.
  const systems::sensors::CameraInfo intrinsics{kWidth,  kHeight, 616.285,
                                                615.778, 405.418, 232.864};

  const math::RigidTransformd X_BC;
  // This is not necessarily true, but we simplify this s.t. we don't have a
  // lie for generating point clouds.
  const math::RigidTransformd X_BD;

  geometry::render::ColorRenderCamera color_camera{
      {"rgbd_sensor", intrinsics, {0.01, 3.0} /* clipping_range */, X_BC},
      false};
  geometry::render::DepthRenderCamera depth_camera{
      {"rgbd_sensor", intrinsics, {0.01, 3.0} /* clipping_range */, X_BD},
      {0.1, 2.0} /* depth_range */};

  auto camera = builder.AddSystem<systems::sensors::RgbdSensor>(
      plant.GetBodyFrameIdOrThrow(
          plant.GetFrameByName("base", cam_instance).body().index()),
      math::RigidTransform<double>(), color_camera, depth_camera);
  camera->set_name("rgbd_sensor");

  builder.ExportOutput(camera->color_image_output_port(), "color_image");
  builder.ExportOutput(camera->depth_image_32F_output_port(), "depth_image");

  auto ToPC = builder.AddSystem<perception::DepthImageToPointCloud>(
      camera->depth_camera_info());
  builder.Connect(camera->depth_image_32F_output_port(),
                  ToPC->depth_image_input_port());
  builder.Connect(camera->color_image_output_port(),
                  ToPC->color_image_input_port());

  auto pointCloudVisualizer =
      builder.AddSystem<geometry::MeshcatPointCloudVisualizer<double>>(meshCat,
                                                                       "cloud");
  pointCloudVisualizer->set_name("point_cloud_visualizer");
  builder.Connect(ToPC->point_cloud_output_port(),
                  pointCloudVisualizer->cloud_input_port());

  std::unique_ptr<AbstractValue> abstractX_CAM = AbstractValue::Make(X_CAM);
  auto cameraPose = builder.template AddSystem<systems::ConstantValueSource>(
      Value<math::RigidTransformd>(X_CAM));
  cameraPose->get_output_port();
  cameraPose->set_name("camera_pose");

  builder.Connect(cameraPose->get_output_port(),
                  pointCloudVisualizer->pose_input_port());
  builder.ExportOutput(ToPC->point_cloud_output_port(), "point_cloud");

  geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);
  auto sys = builder.Build();
  systems::Simulator<double> simulator(*sys);

  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(1);
  simulator.Initialize();

  while (true) {
  }
}

} // namespace kuka_iiwa
} // namespace manipulation
} // namespace drake

int main() {
  drake::manipulation::kuka_iiwa::runMain();
  return 0;
}
