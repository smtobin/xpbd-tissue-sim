#include <cstdio>

#include "sim_bridge/SimBridge.hpp"
#include "sim_bridge/VirtuosoSimBridge.hpp"
#include "sim_bridge/VirtuosoCTAnatomySimBridge.hpp"
#include "sim_bridge/CAOSimBridge.hpp"
#include "sim_bridge/BPHSimBridge.hpp"
#include "sim_bridge/FocalLesionSimBridge.hpp"
#include "sim_bridge/FixedObjectSimBridge.hpp"

#include "config/simulation/GraspingSimulationConfig.hpp"
#include "config/simulation/VirtuosoCTAnatomySimulationConfig.hpp"
#include "config/simulation/PalpationSimulationConfig.hpp"
#include "config/simulation/FixedObjectSimulationConfig.hpp"
#include "simulation/VirtuosoCTAnatomySimulation.hpp"
#include "simulation/PalpationSimulation.hpp"
#include "simulation/GraspingSimulation.hpp"
#include "simulation/FixedObjectSimulation.hpp"

#include <mutex>
#include <condition_variable>
#include <filesystem>

// Global synchronization objects
std::mutex mtx;
std::condition_variable cv;
bool setup_complete = false;


Vec3r parseVector3(const std::string& s)
{
    Vec3r v;
    char c;

    std::stringstream ss(s);
    ss >> c >> v[0] >> c >> v[1] >> c >> v[2] >> c;

    return v;
}

void runSim(Sim::Simulation* sim)
{
    try
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
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("Sim Bridge"),
            "Simulation exception: %s",
            e.what());
        rclcpp::shutdown();
    }
}

template<typename SimulationType, typename SimBridgeType=SimBridge<SimulationType>>
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
    rclcpp::spin(std::make_shared<SimBridgeType>(sim));

    sim_thread.join();

    rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::cout << "\n\n\nSIM BRIDGE MAIN" << std::endl;

    // parse command line arguments for config filename and simulation type
    std::string config_filename;
    std::string simulation_type;
    std::string tumor_mesh_filename;
    std::string trachea_mesh_filename;
    std::string prostate_mesh_filename;

    Vec3r CTtoVB_translation, CTtoVB_rotation;
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        std::cout << "arg: " << arg << std::endl;
        if (arg == "--config-filename" && i+1 < argc)
        {
            config_filename = argv[++i];
        }
        if (arg == "--simulation-type" && i+1 < argc)
        {
            simulation_type = argv[++i];
        }
        if (arg == "--tumor-mesh-filename" && i+1 < argc)
        {
            tumor_mesh_filename = argv[++i];
        }
        if (arg == "--trachea-mesh-filename" && i+1 < argc)
        {
            trachea_mesh_filename = argv[++i];
        }
        if (arg == "--prostate-mesh-filename" && i+1 < argc)
        {
            prostate_mesh_filename = argv[++i];
        }
        if (arg == "--CT-to-VB-translation" && i+1 < argc)
        {
            std::string str = argv[++i];
            CTtoVB_translation = parseVector3(str);
        }
        if (arg == "--CT-to-VB-rotation" && i+1 < argc)
        {
            std::string str = argv[++i];
            CTtoVB_rotation = parseVector3(str);
        }
    }

    if (simulation_type == "CAOSimulation")
    {
        // create the simulation config object from the yaml config file
        Config::VirtuosoCTAnatomySimulationConfig config(YAML::LoadFile(config_filename));

        // edit the CT mesh filename, when a different one is given by the user
        if (!tumor_mesh_filename.empty())
        {
            auto& object_configs = config.objectConfigs();
            auto& xpbd_obj_configs = object_configs.template get<Config::FirstOrderXPBDMeshObjectConfig>();
            if (xpbd_obj_configs.size() > 0)
            {
                std::filesystem::path mesh_file = tumor_mesh_filename;
                std::string fixed_faces_filename = (mesh_file.parent_path() / (mesh_file.stem().string() + "_fixed_faces.txt")).string();
                xpbd_obj_configs[0].setFilename(tumor_mesh_filename);
                xpbd_obj_configs[0].setFixedFacesFilename(fixed_faces_filename);
            }
        }

        if (!trachea_mesh_filename.empty())
        {
            auto& object_configs = config.objectConfigs();
            auto& rigid_mesh_obj_configs = object_configs.template get<Config::RigidMeshObjectConfig>();
            if (rigid_mesh_obj_configs.size() > 0)
            {
                rigid_mesh_obj_configs[0].setFilename(trachea_mesh_filename);
            }
        }

        std::cout << "CTtoVB translation: " << CTtoVB_translation.transpose() << std::endl;
        std::cout << "CTtoVB rotation: " << CTtoVB_rotation.transpose() << std::endl;
        config.setCTtoVBTranslation(CTtoVB_translation);
        config.setCTtoVBRotation(CTtoVB_rotation);

        // create the simulation from the config object
        Sim::VirtuosoCTAnatomySimulation sim(&config);

        startNode<Sim::VirtuosoCTAnatomySimulation, CAOSimBridge>(&sim);
    }
    else if (simulation_type == "BPHSimulation")
    {
        // create the simulation config object from the yaml config file
        Config::VirtuosoCTAnatomySimulationConfig config(YAML::LoadFile(config_filename));

        // edit the CT mesh filename, when a different one is given by the user
        if (!prostate_mesh_filename.empty())
        {
            auto& object_configs = config.objectConfigs();
            auto& xpbd_obj_configs = object_configs.template get<Config::FirstOrderXPBDMeshObjectConfig>();
            if (xpbd_obj_configs.size() > 0)
            {
                std::filesystem::path mesh_file = prostate_mesh_filename;
                std::string fixed_faces_filename = (mesh_file.parent_path() / (mesh_file.stem().string() + "_fixed_faces.txt")).string();
                xpbd_obj_configs[0].setFilename(prostate_mesh_filename);
                xpbd_obj_configs[0].setFixedFacesFilename(fixed_faces_filename);
            }
        }

        config.setCTtoVBTranslation(CTtoVB_translation);
        config.setCTtoVBRotation(CTtoVB_rotation);

        // create the simulation from the config object
        Sim::VirtuosoCTAnatomySimulation sim(&config);

        startNode<Sim::VirtuosoCTAnatomySimulation, BPHSimBridge>(&sim);
    }
    else if (simulation_type == "FocalLesionSimulation")
    {
        // create the simulation config object from the yaml config file
        Config::VirtuosoCTAnatomySimulationConfig config(YAML::LoadFile(config_filename));

        // edit the CT mesh filename, when a different one is given by the user
        if (!prostate_mesh_filename.empty())
        {
            auto& object_configs = config.objectConfigs();
            auto& xpbd_obj_configs = object_configs.template get<Config::FirstOrderXPBDMeshObjectConfig>();
            if (xpbd_obj_configs.size() > 0)
            {
                std::filesystem::path mesh_file = prostate_mesh_filename;
                std::string fixed_faces_filename = (mesh_file.parent_path() / (mesh_file.stem().string() + "_fixed_faces.txt")).string();
                std::string element_classes_filename = (mesh_file.parent_path() / (mesh_file.stem().string() + "_element_classes.txt")).string();
                xpbd_obj_configs[0].setFilename(prostate_mesh_filename);
                /** TODO: uncomment this after fixed faces file is fixed (07/13/26) */
                // xpbd_obj_configs[0].setFixedFacesFilename(fixed_faces_filename);
                xpbd_obj_configs[0].setElementClassesFilename(element_classes_filename);
            }
        }

        config.setCTtoVBTranslation(CTtoVB_translation);
        config.setCTtoVBRotation(CTtoVB_rotation);

        // create the simulation from the config object
        Sim::VirtuosoCTAnatomySimulation sim(&config);

        startNode<Sim::VirtuosoCTAnatomySimulation, FocalLesionSimBridge>(&sim);
    }
    else if (simulation_type == "VirtuosoCTAnatomySimulation")
    {
        // create the simulation config object from the yaml config file
        Config::VirtuosoCTAnatomySimulationConfig config(YAML::LoadFile(config_filename));
        // create the simulation from the config object
        Sim::VirtuosoCTAnatomySimulation sim(&config);

        startNode<Sim::VirtuosoCTAnatomySimulation, VirtuosoCTAnatomySimBridge>(&sim);
    }
    else if (simulation_type == "VirtuosoSimulation")
    {
        Config::VirtuosoSimulationConfig config(YAML::LoadFile(config_filename));
        Sim::VirtuosoSimulation sim(&config);

        startNode<Sim::VirtuosoSimulation, VirtuosoSimBridge>(&sim);
    }
    else if (simulation_type == "PalpationSimulation")
    {
        Config::PalpationSimulationConfig config(YAML::LoadFile(config_filename));
        Sim::PalpationSimulation sim(&config);

        startNode<Sim::VirtuosoSimulation, VirtuosoSimBridge>(&sim);
    }
    else if (simulation_type == "GraspingSimulation")
    {
        Config::GraspingSimulationConfig config(YAML::LoadFile(config_filename));
        Sim::GraspingSimulation sim(&config);

        startNode<Sim::GraspingSimulation>(&sim);
    }
    else if (simulation_type == "FixedObjectSimulation")
    {
        Config::FixedObjectSimulationConfig config(YAML::LoadFile(config_filename));
        Sim::FixedObjectSimulation sim(&config);

        startNode<Sim::FixedObjectSimulation, FixedObjectSimBridge>(&sim);
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
