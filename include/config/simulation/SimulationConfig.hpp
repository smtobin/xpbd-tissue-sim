#ifndef __SIMULATION_CONFIG_HPP
#define __SIMULATION_CONFIG_HPP

#include "config/simobject/ObjectConfig.hpp"
#include "config/simobject/XPBDMeshObjectConfig.hpp"
#include "config/simobject/FirstOrderXPBDMeshObjectConfig.hpp"
#include "config/simobject/RigidMeshObjectConfig.hpp"
#include "config/simobject/RigidPrimitiveConfigs.hpp"
#include "config/simobject/VirtuosoArmConfig.hpp"
#include "config/simobject/VirtuosoRobotConfig.hpp"
#include "config/render/SimulationRenderConfig.hpp"

#include "common/SimulationTypeDefs.hpp"
#include "common/VariadicVectorContainer.hpp"

#include <optional>
#include <string>

namespace Config
{

/** Enum defining the different ways the simulation can be run 
 * VISUALIZATION: if simulation is running faster than real-time, slow down updates so that sim time = wall time
 * AFAP: run the simulation As Fast As Possible - i.e. do not slow updates
 * FRAME_BY_FRAME: allow the user to step the simulation forward, time step by time step, using the keyboard
*/
enum class SimulationMode
{
    VISUALIZATION=0,
    AFAP,
    FRAME_BY_FRAME
};

enum class Visualization
{
    NONE=0,
    EASY3D,
    VTK
};

class SimulationConfig : public Config
{

    public:
    using ConfigVectorType = VariadicVectorContainerFromTypeList<SimulationObjectConfigTypes>::type;

    /** Static predifined options for the simulation mode. Maps strings to the Simulation mode enum. */
    static std::map<std::string, SimulationMode> SIM_MODE_OPTIONS()
    {
        static std::map<std::string, SimulationMode> sim_mode_options{{"Visualization", SimulationMode::VISUALIZATION},
                                                                      {"AFAP", SimulationMode::AFAP},
                                                                      {"Frame-by-frame", SimulationMode::FRAME_BY_FRAME}};
        return sim_mode_options;
    }

    /** Static predifined options for the visualization type. */
    static std::map<std::string, Visualization> VISUALIZATION_OPTIONS()
    {
        static std::map<std::string, Visualization> visualization{{"None", Visualization::NONE},
                                                                  {"Easy3D", Visualization::EASY3D},
                                                                  {"VTK", Visualization::VTK}};
        return visualization;
    }

    public:
    explicit SimulationConfig()
        : Config(), _render_config()
    {

    }

    /** Creates a Config from a YAML node, which consists of parameters needed for Simulation.
     * @param node : the YAML node (i.e. dictionary of key-value pairs) that information is pulled from
     */
    explicit SimulationConfig(const YAML::Node& node)
        : Config(node), _render_config(node)
    {
        // extract parameters
        _extractParameter("time-step", node, _time_step);
        _extractParameter("end-time", node, _end_time);
        _extractParameterWithOptions("sim-mode", node, _sim_mode, SIM_MODE_OPTIONS());
        _extractParameterWithOptions("visualization", node, _visualization, VISUALIZATION_OPTIONS());
        _extractParameter("enable-mouse-interaction", node, _enable_mouse_interaction);
        _extractParameter("logging", node, _logging);
        _extractParameter("logging-output-folder", node, _logging_output_dir);
        _extractParameter("g-accel", node, _g_accel);
        _extractParameter("description", node, _description);
        _extractParameter("fps", node, _fps);
        _extractParameter("collision-rate", node, _collision_rate);

        // create a MeshObject for each object specified in the YAML file
        for (const auto& obj_node : node["objects"])
        {
            std::string type;
            try 
            {
                // extract type information
                type = obj_node["type"].as<std::string>();
            }
            catch (const std::exception& e)
            {
                std::cerr << e.what() << std::endl;
                std::cerr << "Type of object is needed!" << std::endl;
                continue;
            }
            

            // create the specified type of object based on type string
            if (type == "XPBDMeshObject")
                _object_configs.template emplace_back<XPBDMeshObjectConfig>(obj_node);
            else if (type == "FirstOrderXPBDMeshObject")    
                _object_configs.template emplace_back<FirstOrderXPBDMeshObjectConfig>(obj_node);
            else if (type == "RigidMeshObject")
                _object_configs.template emplace_back<RigidMeshObjectConfig>(obj_node);
            else if (type == "RigidSphere")
                _object_configs.template emplace_back<RigidSphereConfig>(obj_node);
            else if (type == "RigidBox")
                _object_configs.template emplace_back<RigidBoxConfig>(obj_node);
            else if (type == "RigidCylinder")
                _object_configs.template emplace_back<RigidCylinderConfig>(obj_node);
            else if (type == "VirtuosoArm")
                _object_configs.template emplace_back<VirtuosoArmConfig>(obj_node);
            else if (type == "VirtuosoRobot")
                _object_configs.template emplace_back<VirtuosoRobotConfig>(obj_node);
            else
            {
                std::cerr << "Unknown type of object! \"" << type << "\" is not a type of simulation object." << std::endl;
                assert(0);
            }
            
        }

        // create a material config for each material specified
        for (const auto& mat_node : node["materials"])
        {
            _material_configs.emplace_back(mat_node);
        }
    }

    explicit SimulationConfig(const std::string& name, const std::string& description,
                             Real time_step, Real end_time, Real g_accel,
                             SimulationMode sim_mode, Visualization visualization, bool enable_mouse_interaction, 
                             bool logging, const std::string& logging_output_dir,
                            Real fps,  Real collision_rate, const SimulationRenderConfig& render_config)
        : Config(name), _render_config(render_config)
    {
        _description.value = description;
        _time_step.value = time_step;
        _end_time.value = end_time;
        _g_accel.value = g_accel;
        _sim_mode.value = sim_mode;
        _visualization.value = visualization;
        _enable_mouse_interaction.value = enable_mouse_interaction;
        _logging.value = logging;
        _logging_output_dir.value = logging_output_dir;
        _fps.value = fps;
        _collision_rate.value = collision_rate;
    }
    
    SimulationConfig(const SimulationConfig& other) = delete;
    SimulationConfig(SimulationConfig&& other) = default;

    // Getters
    Real timeStep() const { return _time_step.value; }
    Real endTime() const { return _end_time.value; }
    SimulationMode simMode() const { return _sim_mode.value; }
    Visualization visualization() const { return _visualization.value; }
    bool enableMouseInteraction() const { return _enable_mouse_interaction.value; }
    bool logging() const { return _logging.value; }
    std::string loggingOutputDir() const { return _logging_output_dir.value; }
    Real gAccel() const { return _g_accel.value; }
    std::string description() const { return _description.value; }
    Real fps() const { return _fps.value; }
    Real collisionRate() const { return _collision_rate.value; }

    // get list of MeshObject configs that will be used to create MeshObjects
    const ConfigVectorType& objectConfigs() const { return _object_configs; }

    const std::vector<MaterialClassConfig>& materialConfigs() const { return _material_configs; }

    const SimulationRenderConfig& renderConfig() const { return _render_config; }

    protected:
    // Parameters
    ConfigParameter<std::string> _description = ConfigParameter<std::string>("");
    ConfigParameter<Real> _time_step = ConfigParameter<Real>(1e-3);
    ConfigParameter<Real> _end_time = ConfigParameter<Real>(10);
    ConfigParameter<SimulationMode> _sim_mode = ConfigParameter<SimulationMode>(SimulationMode::VISUALIZATION);
    ConfigParameter<Visualization> _visualization = ConfigParameter<Visualization>(Visualization::VTK);
    ConfigParameter<bool> _enable_mouse_interaction = ConfigParameter<bool>(true);
    ConfigParameter<bool> _logging = ConfigParameter<bool>(false);
    ConfigParameter<std::string> _logging_output_dir = ConfigParameter<std::string>("../sim/output/");
    ConfigParameter<Real> _g_accel = ConfigParameter<Real>(9.81);
    ConfigParameter<Real> _fps = ConfigParameter<Real>(30.0);
    ConfigParameter<Real> _collision_rate = ConfigParameter<Real>(100);

    /** List of object configs for each object in the Simulation */
    ConfigVectorType _object_configs;

    /** List of material configs that can be used by objects in the simulation. */
    std::vector<MaterialClassConfig> _material_configs;

    SimulationRenderConfig _render_config;
};

} // namespace Config

#endif // __SIMULATION_CONFIG_HPP