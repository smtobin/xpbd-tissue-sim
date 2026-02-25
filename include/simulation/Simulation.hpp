#ifndef __SIMULATION_HPP
#define __SIMULATION_HPP

#include <assimp/Importer.hpp>

#include "simulation/SimulationLogger.hpp"

#include "simobject/Object.hpp"
#include "simobject/XPBDMeshObject.hpp"
#include "simobject/RigidMeshObject.hpp"
#include "simobject/RigidObject.hpp"
#include "simobject/RigidPrimitives.hpp"
#include "simobject/VirtuosoArm.hpp"
#include "simobject/VirtuosoRobot.hpp"

#include "config/simulation/SimulationConfig.hpp"
#include "collision/CollisionScene.hpp"
#include "graphics/GraphicsScene.hpp"
#include "geometry/embree/EmbreeScene.hpp"

#include "common/VariadicVectorContainer.hpp"
#include "common/SimulationTypeDefs.hpp"
#include "common/SimulationInput.hpp"

#include <yaml-cpp/yaml.h>

#include <thread>
#include <optional>
#include <functional>

namespace Sim
{

struct SimulationCheckpoint
{
    Real time;
    Real last_collision_detection_time;
    std::vector<std::byte> object_bytes;
};

/** A class for managing the simulation being performed.
 * Owns the Objects, keeps track fo the sim time, etc.
 * 
 */
class Simulation
{
    public:
    using ObjectVectorType = VariadicVectorContainerFromTypeList<SimulationObjectTypes>::unique_ptr_type;

    public:
    struct CallbackInfo
    {
        std::function<void()> callback;     // the function to execute
        double interval;                    // the time (in s) between calls
        double next_exec_time;              // the next time this callback should be executed
        bool use_wall_time;                 // if true, measure time in terms of wall clock time. if false, use sim time

        CallbackInfo(std::function<void()> cb, double intvl, double start_time, bool use_wall_time_)
            : callback(std::move(cb)), interval(intvl), next_exec_time(start_time + interval), use_wall_time(use_wall_time_)
        {}
    };

    public:
        explicit Simulation(const Config::SimulationConfig* config);

    protected:
        /** Protected default constructor - only callable from derived objects
         * Assumes that the _config object is set and exists
         */
        // explicit Simulation();
    
    public:
        virtual std::string toString(const int indent) const;
        virtual std::string type() const { return "Simulation"; }

        /** Adds a MeshObject to the simulation. Will add its Drawables to the Viewer as well.
         * @param mesh_obj : the MeshObject being added  
        */        
        // void addObject(std::shared_ptr<MeshObject> mesh_obj);

        Real time() const { return _time; }

        Real dt() const { return _time_step; }
        
        Real wallClockdt() const { return _wall_clock_dt; }
        
        Real gAccel() const { return _g_accel; }

        const Graphics::GraphicsScene* graphicsScene() const { return _graphics_scene.get(); }
        Graphics::GraphicsScene* graphicsScene() { return _graphics_scene.get(); }
        const Geometry::EmbreeScene* embreeScene() const { return _embree_scene.get(); }
        void updateEmbreeScene() { _embree_scene->update(); }
        void updateEmbreeRayScene() { _embree_scene->updateRayScene(); }
        const CollisionScene* collisionScene() const { return _collision_scene.get(); }

        const ObjectVectorType& objects() const { return _objects; }
        const ObjectVectorType& graphicsObjects() const { return _graphics_only_objects; }

        /** Adds a new material to the sim. Useful for setting up sims without a config file. */
        void addMaterial(const MaterialClass& mat) { _material_classes.push_back(std::make_unique<MaterialClass>(mat)); }

        /** Returns the material with the specified name, if it exists. If it doesn't exist,
         * the program will throw an error and exit.
         */
        const MaterialClass* getMaterialClass(const std::string& name) const 
        { 
            for (const auto& mat : _material_classes)
            {
                if (mat->name() == name)
                    return mat.get();
            }

            std::stringstream ss;
            ss << "Material with name " << name << " does not exist in the simuilation!";
            throw std::runtime_error(ss.str());

            return _material_classes[0].get();
        }

        /** Performs setup for the Simulation.
         * Creates initial MeshObjects, sets up Viewer, etc.
         */
        virtual void setup();

        /** Runs the simulation.
         * Spawns a separate thread to do updates.
         */
        int run();

        /** Updates the simulation at a fixed time step. */
        virtual void update();

        /** Resets the simulation to its initial state. */
        void reset();

        /** Notifies the simulation that a key has been pressed in the viewer.
         * @param key : the key that was pressed
         * @param action : the action performed on the keyboard
         * @param modifiers : the modifiers (i.e. Shift, Ctrl, Alt)
         */
        virtual void notifyKeyPressed(SimulationInput::Key key, SimulationInput::KeyAction action, int modifiers);

        virtual void notifyMouseButtonPressed(SimulationInput::MouseButton button, SimulationInput::MouseAction action, int modifiers);

        virtual void notifyMouseMoved(double x, double y);

        virtual void notifyMouseScrolled(double dx, double dy);

        template<typename CallbackT>
        void addCallback(double interval, CallbackT&& lambda, bool use_wall_time=true)
        {
            std::function<void()> wrapper = [lambda = std::forward<CallbackT>(lambda)]() {
                lambda();
            };

            _callbacks.emplace_back(std::move(wrapper), interval, _time, use_wall_time);
        }
    
    protected:
        /** Helper to add an object to the simulation given an ObjectConfig.
         * Will create an object (RigidMeshObject, RigidSphere, XPBDMeshObject, etc.) depending on the type of ObjectConfig given.
         * Adds the object to the appropriate part of the simulation (i.e. to the CollisionScene if collisions are enabled, GraphicsScene if graphics are enabled, etc.)
        */
        template<typename ConfigType>        
        typename ConfigType::ObjectType* _addObjectFromConfig(const ConfigType* obj_config)
        {
            using ObjPtrType = std::unique_ptr<typename ConfigType::ObjectType>;

            ObjPtrType new_obj = obj_config->createObject(this);
            new_obj->setup();

            // handle XPBDMeshObjects slightly differently so that we can tell the CollisionScene if self-collisions are enabled
            if constexpr (std::is_convertible_v<ConfigType*, Config::XPBDMeshObjectConfig*>)
            {
                if (!obj_config->graphicsOnly())
                    _collision_scene->addObject(new_obj.get(), obj_config->collisions(), obj_config->selfCollisions());
            }
            else
            {
                // add the new object to the collision scene if collisions are enabled
                if (!obj_config->graphicsOnly())
                    _collision_scene->addObject(new_obj.get(), obj_config->collisions());
            }
            
            // add the new object to the graphics scene to be visualized
            if (_graphics_scene)
            {
                // for XPBDMeshObjects can have multiple materials associated with different parts of the mesh
                if constexpr (std::is_convertible_v<ConfigType*, Config::XPBDMeshObjectConfig*>)
                {
                    /** TODO: handle multiple render configs at the same time
                     * 
                     * for now, just use the first material
                     */
                    const Config::ObjectRenderConfig& render_config = new_obj->materialClasses().front()->renderConfig();
                    _graphics_scene->addObject(new_obj.get(), render_config);
                }
                else
                {
                    const Config::ObjectRenderConfig& render_config = getMaterialClass(obj_config->materialClass())->renderConfig();
                    _graphics_scene->addObject(new_obj.get(), render_config);
                }
                
            }

            // if we get to here, we have successfully created a new MeshObject of some kind
            // so add the new object to the simulation
            typename ConfigType::ObjectType* tmp_ptr = new_obj.get();
            if (obj_config->graphicsOnly())
            {
                _graphics_only_objects.template push_back<ObjPtrType>(std::move(new_obj));
            }
            else
            {
                _objects.template push_back<ObjPtrType>(std::move(new_obj));
            }
            return tmp_ptr;
        }

        /** Time step the simulation */
        virtual void _timeStep();

        /** Update graphics in the sim */
        virtual void _updateGraphics();

    protected:
        /** Whether or not the simulation has been setup already with a call to setup()  */
        bool _setup;
        
        /** YAML config dictionary for setting up the simulation */
        const Config::SimulationConfig* _config;

        /** Name of the simulation */
        std::string _name;

        /** Description of the simulation */
        std::string _description;

        /** How the simulation should be run */
        Config::SimulationMode _sim_mode;

        /** Current sim time */
        Real _time;
        /** Wall clock sim start time */
        std::chrono::time_point<std::chrono::steady_clock> _wall_time_start;
        /** The time step to take */
        Real _time_step;
        /** End time of the simulation */
        Real _end_time;
        /** Number of time steps taken */
        size_t _steps_taken;
        /** Acceleration due to gravity */
        Real _g_accel;
        /** Time to wait inbetween viewer updates (in ms). This is 1/fps */
        int _viewer_refresh_time;
        /** Time to wait inbetween collision checks (in seconds). This is 1/collision_rate */
        Real _time_between_collision_checks;

        Real _last_collision_detection_time;

        /** The wall clock time taken for the last simulation time step. */
        Real _wall_clock_dt;

        /** scheduled callbacks */
        std::vector<CallbackInfo> _callbacks;

        /** storage of all Objects in the simulation.
         * These objects will evolve in time through the update() method that they all provide
         */
        ObjectVectorType _objects;

        /** storage of objects in the simulation that are purely visual (i.e. no physics involved, just graphics)
         * update() will NOT get called on these objects.
         */
        ObjectVectorType _graphics_only_objects;

        /** Master list of the materials used by all objects in the simulation */
        std::vector<std::unique_ptr<MaterialClass>> _material_classes;

        /** Manages collision detection and creating constraints for collision response.
         * Only objects with collisions enabled will be added to the CollisionScene.
         */
        std::unique_ptr<CollisionScene> _collision_scene;

        /** Manages graphics objects and displaying things to the screen. */
        std::unique_ptr<Graphics::GraphicsScene> _graphics_scene;

        /** Embree is used to make some ray-tracing and collision queries.
         * The EmbreeScene acts as an interface between the Simulation and the Embree library.
          */
        std::unique_ptr<Geometry::EmbreeScene> _embree_scene;

        /** Responsible for logging various simulation quantities. */
        std::unique_ptr<SimulationLogger> _logger;

        /** Saves the simulation state at various checkpoints. */
        std::vector<SimulationCheckpoint> _checkpoint_states;
};

} // namespace Sim

#endif

