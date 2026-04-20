#include "simulation/Simulation.hpp"

#include "config/simobject/RigidMeshObjectConfig.hpp"
#include "config/simobject/XPBDMeshObjectConfig.hpp"
#include "config/simobject/FirstOrderXPBDMeshObjectConfig.hpp"
#include "config/simobject/RigidPrimitiveConfigs.hpp"
#include "config/simobject/VirtuosoArmConfig.hpp"
#include "config/simobject/VirtuosoRobotConfig.hpp"

#include "graphics/easy3d/Easy3DGraphicsScene.hpp"
#include "graphics/vtk/VTKGraphicsScene.hpp"

#include "simobject/RigidMeshObject.hpp"
#include "simobject/XPBDMeshObject.hpp"
#include "simobject/RigidPrimitives.hpp"
#include "simobject/VirtuosoArm.hpp"
#include "simobject/VirtuosoRobot.hpp"

#include "simobject/XPBDObjectFactory.hpp"

#include "utils/MeshUtils.hpp"

#include <gmsh.h>

namespace Sim
{


Simulation::Simulation(const Config::SimulationConfig* config)
    : _setup(false), _config(config)
{
    // initialize gmsh
    gmsh::initialize();

    // set simulation properties based on YAML file
    _name = _config->name();
    _description = _config->description();
    _time_step = _config->timeStep();
    _end_time = _config->endTime();
    _time = 0;
    _g_accel = _config->gAccel();
    _viewer_refresh_time = 1/_config->fps()*1000;
    _time_between_collision_checks = 1.0/_config->collisionRate();

    // add a default material to the simulation (for objects that the user does not specify a material for)
    Config::MaterialClassConfig default_mat_config("default");
    addMaterial(MaterialClass(&default_mat_config));

    // set the Simulation mode from the YAML config
    _sim_mode = _config->simMode();

    // initialize the graphics scene according to the type specified by the user
    // if "None", don't create a graphics scene
    if (_config->visualization() == Config::Visualization::EASY3D)
    {
        _graphics_scene = std::make_unique<Graphics::Easy3DGraphicsScene>("main", config->renderConfig());
    }

    if (_config->visualization() == Config::Visualization::VTK)
    {
        _graphics_scene = std::make_unique<Graphics::VTKGraphicsScene>("main", config->renderConfig());
    }

    // initialize the Embree scene
    _embree_scene = std::make_unique<Geometry::EmbreeScene>();

    // initialize the collision scene
    // _collision_scene = std::make_unique<CollisionScene>(1.0/_config->fps().value(), 0.05, 10007);
    _collision_scene = std::make_unique<CollisionScene>(this, _embree_scene.get());
    _last_collision_detection_time = 0;

    // initialize the logger
    if (_config->logging())
    {
        // get datetime string
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d_%H:%M:%S") << ".txt";
        std::string filename = ss.str();

        std::filesystem::path output_dir(config->loggingOutputDir());
        std::filesystem::path filepath = output_dir / filename;

        // create the logger
        _logger = std::make_unique<SimulationLogger>(filepath.string());
    }

    /** Create the materials */
    for (const auto& mat_config : config->materialConfigs())
    {
        _material_classes.push_back(std::make_unique<MaterialClass>(&mat_config));
    }
}

std::string Simulation::toString(const int indent) const
{
    std::string indent_str(indent, '\t');
    std::stringstream ss;
    ss << indent_str << "=====" << type() << " '" << _name << "'=====" << std::endl;
    ss << indent_str << "Time step: " << _time_step << " s" << std::endl;
    ss << indent_str << "End time: " << _end_time << " s" << std::endl;
    ss << indent_str << "Gravity: " << _g_accel << " m/s2" << std::endl;
    return  ss.str();
}

void Simulation::setup()
{   
    // make sure we haven't set up already before
    assert(!_setup);

    _setup = true;


    /** Configure graphics scene... */ 
    if (_graphics_scene)
    {
        _graphics_scene->init();
        _graphics_scene->viewer()->registerSimulation(this);
        // add text that displays the current Sim Time  
        _graphics_scene->viewer()->addText("time", "Sim Time: 0.000 s", 10.0f, 10.0f, 15.0f, Graphics::Viewer::TextAlignment::LEFT, Graphics::Viewer::Font::MAO, std::array<float,3>({0,0,0}), 0.5f, false);
    
        _graphics_scene->viewer()->enableMouseInteraction(_config->enableMouseInteraction());
    }

    /** Add simulation objects from Config object... */
    auto& object_configs = _config->objectConfigs();
    object_configs.for_each_element([this](const auto& config)
    {
        this->_addObjectFromConfig(&config);
    });

    /** Setup callbacks for adaptive mesh refinement.
     * TODO: should this go somewhere else? I.e. not at the Simulation level but in XPBDMeshObject?
     */
    _objects.for_each_element<std::unique_ptr<VirtuosoRobot>>([&](auto& robot) {
        _objects.for_each_element<std::unique_ptr<XPBDMeshObject_Base>, std::unique_ptr<FirstOrderXPBDMeshObject_Base>>([&](auto& xpbd_obj) {
            if (!xpbd_obj->adaptiveMeshRefinement())
                return;

            this->addCallback(0.1, [&xpbd_obj, &robot]() {
                auto t1 = std::chrono::high_resolution_clock::now();

                int max_refinement_level = xpbd_obj->maxRefinementLevel();

                const typename Sim::VirtuosoArm::SDFType* sdf1 = nullptr;
                const typename Sim::VirtuosoArm::SDFType* sdf2 = nullptr;
                if (robot->hasArm1() && robot->arm1()->toolType() == Sim::VirtuosoArm::ToolType::CAUTERY)   sdf1 = robot->arm1()->SDF();
                if (robot->hasArm2() && robot->arm2()->toolType() == Sim::VirtuosoArm::ToolType::CAUTERY)   sdf2 = robot->arm2()->SDF();
                const Geometry::Mesh* mesh = xpbd_obj->mesh();
                std::unordered_set<int> elems_to_refine;
                std::unordered_set<int> elems_to_coarsen;
                // std::unordered_set<int> verts_to_refine;
                // std::unordered_set<int> verts_to_coarsen;
                for (const auto& i : mesh->faces().validIndices())
                {
                    const Vec3i& f = mesh->face(i);
                    const Vec3r& p1 = mesh->vertex(f[0]);
                    const Vec3r& p2 = mesh->vertex(f[1]);
                    const Vec3r& p3 = mesh->vertex(f[2]);

                    std::array<Vec3r, 4> pts_to_test = {p1, p2, p3, (p1+p2+p3)/3.0};

                    // check if face centroid is close to either arm by querying each SDF
                    Real sdf_dist1 = std::numeric_limits<Real>::max();
                    Real sdf_dist2 = std::numeric_limits<Real>::max();

                    int element_with_face = xpbd_obj->tetMesh()->elementWithFace(i);

                    // only refine around cautery tool tip (i.e. not the whole tube)
                    if (sdf1)
                    {
                        Geometry::VirtuosoArmSDF::DistanceAndGradientWithNodeInfo best_result;
                        best_result.distance = std::numeric_limits<Real>::max();
                        for (const auto& pt : pts_to_test)
                        {
                            auto result = sdf1->evaluateWithGradientAndNodeInfo(pt);
                            if (result.distance < best_result.distance)
                                best_result = result;
                        }
                        sdf_dist1 = best_result.distance;
                    }
                    if (sdf2)
                    {
                        Geometry::VirtuosoArmSDF::DistanceAndGradientWithNodeInfo best_result;
                        best_result.distance = std::numeric_limits<Real>::max();
                        for (const auto& pt : pts_to_test)
                        {
                            auto result = sdf2->evaluateWithGradientAndNodeInfo(pt);
                            if (result.distance < best_result.distance)
                                best_result = result;
                        }
                        sdf_dist2 = best_result.distance;
                    }

                    Real min_dist = std::min(sdf_dist1, sdf_dist2);
                    if (min_dist < 1.5e-3)
                    {

                        if (xpbd_obj->refinedTetMesh()->elementRefinementLevel(element_with_face) < max_refinement_level)
                        {
                            elems_to_refine.insert(element_with_face);
                        }

                        
                    }
                    else if (min_dist > 5e-3)
                    {
                        if (xpbd_obj->refinedTetMesh()->elementRefinementLevel(element_with_face) > 0)
                        {
                            elems_to_coarsen.insert(element_with_face);
                        }
                    }
                }

                auto t2 = std::chrono::high_resolution_clock::now();

                for (const auto& elem : elems_to_refine)
                {
                    xpbd_obj->refineElement(elem, max_refinement_level, true);
                }
                for (const auto& elem : elems_to_coarsen)
                {
                    xpbd_obj->coarsenElement(elem, max_refinement_level, false);
                }

                auto t3 = std::chrono::high_resolution_clock::now();
                // double search_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1.0e6;
                // double refine_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(t3 - t2).count() / 1.0e6;
                // std::cout << "Time for searching over faces: " << search_ms << " ms" << std::endl;
                // std::cout << "Time for refining " << elems_to_refine.size() << " elements: " << refine_ms << " ms" << std::endl;
                

            }, true);
        });
    });
        
    /** Configure the logger */
    if (_logger)
    {
        // add time as a variable
        _logger->addOutput("time [s]", &_time);
    }
}

void Simulation::update()
{
    // we assume that other derived Simulation classes have already added their logged quantities
    // so we can start logging now (which will print the header and prevent us from adding new logged quantities)
    if (_logger)
        _logger->startLogging();


    // same logic - other derived Simulation classes have finished whatever setup they're doing
    // now we can save the initial state
    saveCheckpoint("initial_state");


    auto start = std::chrono::steady_clock::now();

    // the start time in wall clock time of the simulation
    _wall_time_start = std::chrono::steady_clock::now();
    // the wall time of the last viewer redraw
    auto last_redraw = std::chrono::steady_clock::now();

    // loop until end time is reached
    while(_time < _end_time)
    {
        // the elapsed seconds in wall time since the simulation has started
        Real wall_time_elapsed_s = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - _wall_time_start).count() / 1000000000.0;
        
        // check if any callbacks need to be called
        for (auto& cb : _callbacks)
        {
            if ((cb.use_wall_time && wall_time_elapsed_s > cb.next_exec_time) ||
                (!cb.use_wall_time && _time > cb.next_exec_time)
            )
            {
                cb.callback();
                cb.next_exec_time = cb.next_exec_time + cb.interval;
            }
        }

        // execute one-time callbacks
        for (auto& cb : _one_time_callbacks)
        {
            cb();
        }
        // clear the one-time callbacks after executing them
        _one_time_callbacks.clear();

        // if the simulation is ahead of the current elapsed wall time, stall
        if (_sim_mode == Config::SimulationMode::VISUALIZATION && _time > wall_time_elapsed_s)
        {
            continue;
        }

        _timeStep();

        // the time in ms since the viewer was last redrawn
        auto time_since_last_redraw_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - last_redraw).count();
        // we want ~30 fps, so update the viewer every 33 ms
        if (time_since_last_redraw_ms > _viewer_refresh_time)
        {
            // std::cout << _haptic_device_manager->getPosition() << std::endl;
            _updateGraphics();

            last_redraw = std::chrono::steady_clock::now();
        }
        
    }

    // one final redraw of final state
    _updateGraphics();

    auto end = std::chrono::steady_clock::now();
    std::cout << "Simulating " << _end_time << " seconds took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;
}

void Simulation::_timeStep()
{
    // std::cout << "\n===Time step===" << std::endl;
    auto t1 = std::chrono::steady_clock::now();

    if (_time - _last_collision_detection_time > _time_between_collision_checks)
    {
        // run collision detection
        auto t1 = std::chrono::steady_clock::now();
        auto& xpbd_mesh_objs = _objects.get<std::unique_ptr<XPBDMeshObject_Base>>();
        for (auto& obj : xpbd_mesh_objs)
        {
            obj->clearCollisionConstraints();
        }
        auto& fo_xpbd_mesh_objs = _objects.get<std::unique_ptr<FirstOrderXPBDMeshObject_Base>>();
        for (auto& obj : fo_xpbd_mesh_objs)
        {
            obj->clearCollisionConstraints();
        }
        auto& virtuoso_robots = _objects.get<std::unique_ptr<VirtuosoRobot>>();
        for (auto& obj : virtuoso_robots)
        {
            if (obj->hasArm1())
                obj->arm1()->clearCollisionConstraints();
            if (obj->hasArm2())
                obj->arm2()->clearCollisionConstraints();
        }
        auto& virtuoso_arms = _objects.get<std::unique_ptr<VirtuosoArm>>();
        for (auto& obj : virtuoso_arms)
        {
            obj->clearCollisionConstraints();
        }
        // update the Embree scene before colliding objects
        // auto embree_t1 = std::chrono::steady_clock::now();
        // _embree_scene->update();
        // auto embree_t2 = std::chrono::steady_clock::now();
        // std::cout << "Embree update took " << std::chrono::duration_cast<std::chrono::microseconds>(embree_t2 - embree_t1).count() << " us\n";


        _collision_scene->collideObjects();
        // auto t2 = std::chrono::steady_clock::now();
        // std::cout << "Collision detection took " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us\n";

        
    }

    
    // auto update_t1 = std::chrono::steady_clock::now();

    _objects.for_each_element([](auto& obj)
    {
        obj->update();
    });

    // update each object's velocities
    _objects.for_each_element([](auto& obj)
    {
        obj->velocityUpdate();
    });
    // auto update_t2 = std::chrono::steady_clock::now();
    // std::cout << "Update took " << std::chrono::duration_cast<std::chrono::microseconds>(update_t2 - update_t1).count() << " us\n";

    if (_time - _last_collision_detection_time > _time_between_collision_checks)
    {
        // _collision_scene->updatePrevPositions();
        _last_collision_detection_time = _time;
    }
    
    // log quantities
    if (_logger)
    {
        _logger->logToFile();
    }

    // increment the time by the time step
    _time += _time_step;

    auto t2 = std::chrono::steady_clock::now();
    _wall_clock_dt = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1.0e9;
    // std::cout << "Time step took " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us" << std::endl;
}

void Simulation::_updateGraphics()
{
    if (_graphics_scene)
    {
        // for meshes, we need to update all mesh normals
        _objects.for_each_element([&](auto& obj) {
            // get type of obj
            using UniquePtrType = std::remove_reference_t<decltype(obj)>;
            using ObjType = typename UniquePtrType::element_type;

            // make sure that ObjType is a derived class of mesh obj
            if constexpr (std::is_base_of_v<MeshObject, ObjType>)
            {
                obj->mesh()->updateVertexNormals();
            }
        });

        _graphics_scene->update();
    }
    // update the sim time text
    if (_graphics_scene)
    {
        _graphics_scene->viewer()->editText("time", "Sim Time: " + std::to_string(_time) + " s");
    }
}

void Simulation::notifyKeyPressed(SimulationInput::Key key , SimulationInput::KeyAction action, int /* modifiers */)
{
    // action = 0 ==> key up event
    // action = 1 ==> key down event
    // action = 2 ==> key hold event
    
    // if key is pressed down or held, we want to time step
    if (_sim_mode == Config::SimulationMode::FRAME_BY_FRAME && action == SimulationInput::KeyAction::PRESS)
    {
        _timeStep();
        _updateGraphics();
    }

    if (action == SimulationInput::KeyAction::PRESS && key == SimulationInput::Key::BACKSPACE)
    {
        addOneTimeCallback([&]() {
            this->reset();
        });
    }
}

void Simulation::notifyMouseButtonPressed(SimulationInput::MouseButton /* button */, SimulationInput::MouseAction /* action */, int /* modifiers */)
{
    // button = 0 ==> left mouse button
    // button = 1 ==> right mouse button
    // action = 0 ==> mouse up
    // action = 1 ==> mouse down
    
    // do nothing
}

void Simulation::notifyMouseMoved(double /* x */, double /* y */)
{
    // do nothing
}

void Simulation::notifyMouseScrolled(double /* dx */, double /* dy */)
{
    // do nothing
}

int Simulation::run()
{
    // first, setup if we haven't done so already
    if (!_setup)
        setup();

    // spwan the update thread
    std::thread update_thread;
    if (_sim_mode != Config::SimulationMode::FRAME_BY_FRAME)
    {
        update_thread = std::thread(&Simulation::update, this);
    }

    // run the Viewer
    // _viewer->fit_screen();
    // return _viewer->run();
    if (_graphics_scene)
    {
        _graphics_scene->run();
        return 0;
    }
    else
    {
        update_thread.join();
        return 0;
    }
    
}

void Simulation::reset()
{
    restoreCheckpoint("initial_state");
}

bool Simulation::saveCheckpoint(const std::string& label)
{
    if (_sim_checkpoints.count(label) > 0)
        return false;
    
    addOneTimeCallback([this, label]() {
        std::cout << "Saving checkpoint with label: " << label << std::endl;
        SimulationCheckpoint checkpoint;
        checkpoint.time = this->_time;
        checkpoint.last_collision_detection_time = this->_last_collision_detection_time;
        pack(checkpoint.object_bytes, this->_objects);
        _sim_checkpoints.insert({label, std::move(checkpoint)});
    });


    return true;
}

bool Simulation::restoreCheckpoint(const std::string& label)
{
    auto it = _sim_checkpoints.find(label);
    bool found = it != _sim_checkpoints.end();
    if (found)
    {
        addOneTimeCallback([this, it]() {
            this->_time = it->second.time;
            this->_last_collision_detection_time = it->second.last_collision_detection_time;
            const std::byte* cursor = it->second.object_bytes.data();
            unpack(cursor, _objects);
        });
    }

    return found;
}

} // namespace Sim