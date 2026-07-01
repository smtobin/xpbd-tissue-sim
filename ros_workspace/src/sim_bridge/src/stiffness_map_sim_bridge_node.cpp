#include <cstdio>

#include "sim_bridge/StiffnessMapSimBridge.hpp"
#include "simulation/StiffnessMapSimulation.hpp"

#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <condition_variable>
#include <filesystem>
#include <thread>

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
    rclcpp::spin(std::make_shared<StiffnessMapSimBridge>(sim));

    sim_thread.join();

    rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // read the config filename
    std::string config_filename;
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        std::cout << "arg: " << arg << std::endl;
        if (arg == "--config-filename" && i+1 < argc)
        {
            config_filename = argv[++i];
        }
    }

    Config::StiffnessMapSimulationConfig sim_config(YAML::LoadFile(config_filename));
    Sim::StiffnessMapSimulation sim(&sim_config);
    startNode<Sim::StiffnessMapSimulation>(&sim);
    

    return 0;
}
