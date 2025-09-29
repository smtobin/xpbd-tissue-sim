#include <cstdio>

#include "sim_bridge/SimBridge.hpp"
#include "sim_bridge/VirtuosoSimBridge.hpp"

#include "config/simulation/GraspingSimulationConfig.hpp"
#include "config/simulation/VirtuosoTissueGraspingSimulationConfig.hpp"
#include "config/simulation/PalpationSimulationConfig.hpp"
#include "config/simulation/FixedCubeSimulationConfig.hpp"
#include "simulation/VirtuosoTissueGraspingSimulation.hpp"
#include "simulation/PalpationSimulation.hpp"
#include "simulation/GraspingSimulation.hpp"
#include "simulation/FixedCubeSimulation.hpp"

#include <mutex>
#include <condition_variable>

// Global synchronization objects
std::mutex mtx;
std::condition_variable cv;
bool setup_complete = false;

void runSim(Sim::Simulation* sim)
{
    // setup MUST be called in this thread
    // because the OpenGL context MUST be initialized in the same thread
    sim->setup();

    // notify the main thread that the simulation has completed setup
    {
        std::lock_guard<std::mutex> l(mtx);
        setup_complete = true;
    }
    cv.notify_one();

    // begin running the simulation
    sim->run();
}

template<typename SimulationType>
void startNode(SimulationType* sim)
{
    // start up the simulation in a separate thread
    std::thread sim_thread(runSim, sim);

    // wait for the simulation to be set up
    // required for loading meshes, setting up sim objects, etc.
    {
        std::unique_lock<std::mutex> l(mtx);
        cv.wait(l, [] { return setup_complete; });
    }

    // then start up the SimBridge ROS node
    rclcpp::spin(std::make_shared<SimBridge<SimulationType>>(sim));

    sim_thread.join();

    rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);


    // parse command line arguments for config filename and simulation type
    std::string config_filename;
    std::string simulation_type;
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        if (arg == "--config-filename" && i+1 < argc)
        {
            config_filename = argv[++i];
        }

        if (arg == "--simulation-type" && i+1 < argc)
        {
            simulation_type = argv[++i];
        }
    }

    if (simulation_type == "VirtuosoTissueGraspingSimulation")
    {
        // create the simulation config object from the yaml config file
        Config::VirtuosoTissueGraspingSimulationConfig config(YAML::LoadFile(config_filename));
        // create the simulation from the config object
        Sim::VirtuosoTissueGraspingSimulation sim(&config);

        startNode<Sim::VirtuosoSimulation>(&sim);
    }
    else if (simulation_type == "VirtuosoSimulation")
    {
        Config::VirtuosoSimulationConfig config(YAML::LoadFile(config_filename));
        Sim::VirtuosoSimulation sim(&config);

        startNode<Sim::VirtuosoSimulation>(&sim);
    }
    else if (simulation_type == "PalpationSimulation")
    {
        Config::PalpationSimulationConfig config(YAML::LoadFile(config_filename));
        Sim::PalpationSimulation sim(&config);

        startNode<Sim::VirtuosoSimulation>(&sim);
    }
    else if (simulation_type == "GraspingSimulation")
    {
        Config::GraspingSimulationConfig config(YAML::LoadFile(config_filename));
        Sim::GraspingSimulation sim(&config);

        startNode<Sim::GraspingSimulation>(&sim);
    }
    else if (simulation_type == "FixedCubeSimulation")
    {
        Config::FixedCubeSimulationConfig config(YAML::LoadFile(config_filename));
        Sim::FixedCubeSimulation sim(&config);

        startNode<Sim::FixedCubeSimulation>(&sim);
    }
    else if (simulation_type == "Simulation")
    {
        Config::SimulationConfig config(YAML::LoadFile(config_filename));
        Sim::Simulation sim(&config);
        startNode<Sim::Simulation>(&sim);
    }
    else
    {
        std::cerr << "Unrecognized simulation type: " << simulation_type << std::endl;
        assert(0);
    }

    return 0;
}
