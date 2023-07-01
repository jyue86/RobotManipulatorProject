#include <iostream>


//TODO: remove unnecesssary libraries
#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include <drake/common/drake_assert.h>
#include <drake/common/find_resource.h>

#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_robot_plan.hpp"
#include "drake/lcm/drake_lcm.h"

#include "drake/multibody/parsing/parser.h"

#include <drake/geometry/drake_visualizer.h>
#include <drake/geometry/scene_graph.h>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/multiplexer.h"

#include <drake/manipulation/kuka_iiwa/iiwa_command_sender.h>
#include <drake/manipulation/kuka_iiwa/iiwa_constants.h>
#include <drake/manipulation/kuka_iiwa/iiwa_status_receiver.h>
#include <drake/manipulation/util/robot_plan_interpolator.h>


#include <drake/lcmt_iiwa_command.hpp>
#include <drake/lcmt_iiwa_status.hpp>

#define KUKA_DT 1.0
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
        const char* const kLcmStatusChannel = "KUKA_STATUS";
        const char* const kLcmCommandChannel = "KUKA_COMMAND";
        const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";

        //interpolator
        auto plan_source = builder.AddSystem<util::RobotPlanInterpolator>(urdf);


        //shitty pid gains
        Eigen::VectorXd kp = Eigen::VectorXd::Zero(num_joints);
        Eigen::VectorXd ki = Eigen::VectorXd::Zero(num_joints);
        Eigen::VectorXd kd = Eigen::VectorXd::Zero(num_joints);
        kp.fill(1.0); //P controller
        auto pid_controller = builder.AddSystem<systems::controllers::PidController>(kp,ki,kd);


        //create lcm related objects
        lcm::DrakeLcm lcms;
        auto plant_sub = builder.AddSystem(systems::lcm::LcmSubscriberSystem::Make<lcmt_robot_plan>(kLcmPlanChannel, &lcms));
        auto status_recv = builder.AddSystem<IiwaStatusReceiver>(num_joints);
        auto status_mux = builder.AddSystem<systems::Multiplexer>(std::vector<int>({kIiwaArmNumJoints, kIiwaArmNumJoints}));


        auto target_demux = builder.AddSystem<systems::Demultiplexer>(num_joints*2, num_joints);
        auto adder = builder.AddSystem<systems::Adder>(2,num_joints);
        auto command_pub = builder.AddSystem(systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_command>("IIWA_COMMAND",&lcms));
        auto command_sender = builder.AddSystem<IiwaCommandSender>(num_joints);


        // create diagram
        builder.Connect(plant_sub->get_output_port(),plan_source->get_plan_input_port());
        builder.Connect(status_recv->get_position_measured_output_port(), status_mux->get_input_port(0));
        builder.Connect(status_recv->get_velocity_estimated_output_port(), status_mux->get_input_port(1));
        builder.Connect(status_mux->get_output_port(), pid_controller->get_input_port_estimated_state());
        builder.Connect(plan_source->get_output_port(0),pid_controller->get_input_port_desired_state());

        builder.Connect(pid_controller->get_output_port_control(), adder->get_input_port(0));
        builder.Connect(target_demux->get_output_port(1),adder->get_input_port(1));

        builder.Connect(target_demux->get_output_port(0),command_sender->get_position_input_port());
        builder.Connect(adder->get_output_port(),command_sender->get_torque_input_port());
        builder.Connect(command_sender->get_output_port(),command_pub->get_input_port());


        auto sys = builder.Build(); //auto = pointer of System
        const systems::Diagram<double>* diagram = sys.get();
        systems::Simulator<double> simulator(std::move(sys));

        lcm::Subscriber<lcmt_iiwa_status> status_sub(&lcms, kLcmStatusChannel);
        LcmHandleSubscriptionsUntil(&lcms, [&]() {return status_sub.count() > 0;});

        const lcmt_iiwa_status& first_status = status_sub.message();

        //C++ version of context
        systems::Context<double>& diagram_context = simulator.get_mutable_context();
        const double t0 = first_status.utime * 1e-6;
        diagram_context.SetTime(t0);

        systems::Context<double>& status_context = diagram->GetMutableSubsystemContext(*status_recv, &diagram_context);
        auto& status_value = status_recv->get_input_port().FixValue(&status_context, first_status);
        auto& plan_source_context = diagram->GetMutableSubsystemContext(*plan_source, &diagram_context);
        plan_source->Initialize(t0,status_recv->get_position_measured_output_port().Eval(status_context),&plan_source_context.get_mutable_state());

        drake::log()->info("Controller started");
        while(1)
        {
            std::cout << "Fuck you\n";
            status_sub.clear();
            LcmHandleSubscriptionsUntil(&lcms, [&]() { return status_sub.count() > 0; });
            status_value.GetMutableData()->set_value(status_sub.message());
            const double time = status_sub.message().utime * 1e-6;
            simulator.AdvanceTo(time);
            diagram->ForcedPublish(diagram_context);

        }
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