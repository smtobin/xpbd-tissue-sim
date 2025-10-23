#include "simulation/VirtuosoTissueGraspingSimulation.hpp"
#include "config/simulation/VirtuosoTissueGraspingSimulationConfig.hpp"

#include "simulation/VirtuosoSimulation.hpp"

#include "utils/GeometryUtils.hpp"

#include <filesystem>
#include <stdlib.h>

namespace Sim
{

VirtuosoTissueGraspingSimulation::VirtuosoTissueGraspingSimulation(const Config::VirtuosoTissueGraspingSimulationConfig* config)
: VirtuosoSimulation(config), _grasping(false)
{
    // extract parameters from config object
    _input_device = config->inputDevice();
    _fixed_faces_filename = config->fixedFacesFilename();
    _tumor_faces_filename = config->tumorFacesFilename();
    _goal_filename = config->goalFilename();
    _goals_folder = config->goalsFolder();
    _goal_active = false;
    _current_score = 0;
}

void VirtuosoTissueGraspingSimulation::setup()
{
    VirtuosoSimulation::setup();

    // find the XPBDMeshObject (which we're assuming to be the tissue)
    // first check 1st-order XPBDMeshObjects
    auto& fo_xpbd_objs = _objects.template get<std::unique_ptr<FirstOrderXPBDMeshObject_Base>>();
    auto& xpbd_objs = _objects.template get<std::unique_ptr<XPBDMeshObject_Base>>();
    if (fo_xpbd_objs.size() > 0)
        _tissue_obj = fo_xpbd_objs.front().get();
    else if (xpbd_objs.size() > 0)
        _tissue_obj = xpbd_objs.front().get();
    else
        assert((xpbd_objs.size() + fo_xpbd_objs.size() > 0) && "There must be at least 1 XPBDMeshObject or FirstOrderXPBDMeshObject in the simulation (there is no tissue object!).");
    

    

    // once we've found the tissue object, make sure that each virtuoso arm knows that this is the object that they're manipulating
    // (the VirtuosoArm class handles the grasping logic)
    if (_virtuoso_robot->hasArm1())
        _virtuoso_robot->arm1()->setToolManipulatedObject(_tissue_obj);
    if (_virtuoso_robot->hasArm2())
        _virtuoso_robot->arm2()->setToolManipulatedObject(_tissue_obj);


    if (_fixed_faces_filename.has_value())
    {
        std::set<int> vertices;
        std::vector<int> faces;
        MeshUtils::verticesAndFacesFromFixedFacesFile(_fixed_faces_filename.value(), vertices, faces);
        for (const auto& v : vertices)
        {
            _tissue_obj.fixVertex(v);
        }
    }
    

    // create a goal visualization object that the user tries to match
    if (_goals_folder.has_value())
    {
        std::cout << "Loading goals..." << std::endl;
        for (const auto & entry : std::filesystem::directory_iterator(_goals_folder.value()))
        {
            Config::RigidMeshObjectConfig goal_config(entry.path(), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0),
                1.0, false, true, false,
                entry.path(), 0.06, std::nullopt, false, true, false, Vec4r(0.4, 0.0, 0.0, 0.0), std::nullopt, Config::ObjectRenderConfig());
            RigidMeshObject* goal_obj = dynamic_cast<RigidMeshObject*>(_addObjectFromConfig(&goal_config));
            goal_obj->mesh()->addFaceProperty<bool>("draw", false);
            
            // align meshes based on the first face (the meshes are loaded and moved about their center of mass, which may be different between the two)
            const Vec3r& v0_f0_tissue_obj = _tissue_obj.mesh()->vertex(_tissue_obj.mesh()->face(0)[0]);
            const Vec3r& v0_f0_goal_obj = goal_obj->mesh()->vertex(goal_obj->mesh()->face(0)[0]);

            goal_obj->setPosition( goal_obj->position() + (v0_f0_tissue_obj - v0_f0_goal_obj) );

            _goal_objs.push_back(goal_obj);

            std::cout << entry.path() << std::endl;
        }
        
        // get a random goal obj to start
        _goal_obj_ind = rand() % _goal_objs.size();
    }
    
    else if (_goal_filename.has_value())
    {
        Config::RigidMeshObjectConfig goal_config("goal_mesh", Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0),
            1.0, false, true, false,
            _goal_filename.value(), 0.06, std::nullopt, false, true, false, Vec4r(0.4, 0.0, 0.0, 0.0), std::nullopt, Config::ObjectRenderConfig());
        RigidMeshObject* goal_obj = dynamic_cast<RigidMeshObject*>(_addObjectFromConfig(&goal_config));
        goal_obj->mesh()->addFaceProperty<bool>("draw", false);

        // align meshes based on the first face (the meshes are loaded and moved about their center of mass, which may be different between the two)
        const Vec3r& v0_f0_tissue_obj = _tissue_obj.mesh()->vertex(_tissue_obj.mesh()->face(0)[0]);
        const Vec3r& v0_f0_goal_obj = goal_obj->mesh()->vertex(goal_obj->mesh()->face(0)[0]);

        goal_obj->setPosition( goal_obj->position() + (v0_f0_tissue_obj - v0_f0_goal_obj) );

        _goal_objs.push_back(goal_obj);
        _goal_obj_ind = 0;

        // std::cout << "Difference: " << v0_f0_goal_obj - v0_f0_tissue_obj << std::endl;
    }

    // if (_tumor_faces_filename.has_value())
    // {
    //     std::set<int> vertices;
    //     MeshUtils::verticesAndFacesFromFixedFacesFile(_tumor_faces_filename.value(), vertices, _tumor_faces);

    //     // set tumor property
    //     _tissue_obj.mesh()->addFaceProperty<int>("class", TissueClasses::TRACHEA);
    //     Geometry::MeshProperty<int>& tissue_class_property = _tissue_obj.mesh()->getFaceProperty<int>("class");
    //     for (const auto& face_index : _tumor_faces)
    //     {
    //         tissue_class_property.set(face_index, TissueClasses::TUMOR);
    //     }
    // }

    if (_goal_objs.size() > 0)
    {
        _graphics_scene->viewer()->addText("score", "Current Score: ", 10.0f, 35.0f, 15.0f, Graphics::Viewer::TextAlignment::LEFT, Graphics::Viewer::Font::MAO, std::array<float,3>({0,0,0}), 0.5f, false);
    }
    
}

void VirtuosoTissueGraspingSimulation::notifyMouseButtonPressed(SimulationInput::MouseButton button, SimulationInput::MouseAction action, int modifiers)
{   
    if (_input_device == SimulationInput::Device::MOUSE && button == SimulationInput::MouseButton::LEFT && action == SimulationInput::MouseAction::PRESS)
    {
        // _toggleTissueGrasping();
        _active_arm->setToolState(!_active_arm->toolState());
    }

    VirtuosoSimulation::notifyMouseButtonPressed(button, action, modifiers);

}

void VirtuosoTissueGraspingSimulation::notifyMouseMoved(double x, double y)
{
    VirtuosoSimulation::notifyMouseMoved(x, y);
}

void VirtuosoTissueGraspingSimulation::notifyKeyPressed(SimulationInput::Key key, SimulationInput::KeyAction action, int modifiers)
{

    // if input mode is keyboard, space bar grasps
    if (_input_device == SimulationInput::Device::KEYBOARD && key == SimulationInput::Key::SPACE && action == SimulationInput::KeyAction::PRESS)
    {
        _active_arm->setToolState(!_active_arm->toolState());
        // _toggleTissueGrasping();
    }

    // if 'B' is pressed, save the tissue mesh to file
    if (key == SimulationInput::Key::B && action == SimulationInput::KeyAction::PRESS)
    {
        const std::string filename = "tissue_mesh_" + std::to_string(_time) + "_s.obj";
        _tissue_obj.mesh()->writeMeshToObjFile(filename);
    }

    // if 'Z' is pressed, toggle the goal
    if (key == SimulationInput::Key::Z && action == SimulationInput::KeyAction::PRESS && _goal_objs.size() > 0)
    {
        _toggleGoal();
    }

    // if 'C' is pressed, change goals
    if (key == SimulationInput::Key::C && action == SimulationInput::KeyAction::PRESS && _goal_objs.size() > 0)
    {
        _changeGoal();
    }

    VirtuosoSimulation::notifyKeyPressed(key, action, modifiers);

}

void VirtuosoTissueGraspingSimulation::notifyMouseScrolled(double dx, double dy)
{
    VirtuosoSimulation::notifyMouseScrolled(dx, dy);
}

void VirtuosoTissueGraspingSimulation::_updateGraphics()
{

    Simulation::_updateGraphics();
}

void VirtuosoTissueGraspingSimulation::_timeStep()
{

    VirtuosoSimulation::_timeStep();

    if (_input_device == SimulationInput::Device::HAPTIC)
    {
        HHD handle = _haptic_device_manager->deviceHandles()[0];

        bool button1_pressed = _haptic_device_manager->button1Pressed(handle);

        if (!_grasping && button1_pressed)
        {
            _active_arm->setToolState(!_active_arm->toolState());
        }
        else if (_grasping && !button1_pressed)
        {
            _active_arm->setToolState(!_active_arm->toolState());
        }
    }
}

void VirtuosoTissueGraspingSimulation::_toggleGoal()
{
    if (_goal_objs.size() == 0)
        return;

    // toggle class variable
    _goal_active = !_goal_active;

    // update draw property
    Geometry::MeshProperty<bool>& draw_face_property = _goal_objs[_goal_obj_ind]->mesh()->getFaceProperty<bool>("draw");
    for (const auto& face_index : _tumor_faces)
    {
        draw_face_property.set(face_index, _goal_active);
    }
}

void VirtuosoTissueGraspingSimulation::_changeGoal()
{
    // update draw property
    Geometry::MeshProperty<bool>& draw_face_property_old = _goal_objs[_goal_obj_ind]->mesh()->getFaceProperty<bool>("draw");
    for (const auto& face_index : _tumor_faces)
    {
        draw_face_property_old.set(face_index, false);
    }

    // increment goal object index
    _goal_obj_ind++;
    if (_goal_obj_ind >= _goal_objs.size())
        _goal_obj_ind = 0;
    
    // show the new goal
    Geometry::MeshProperty<bool>& draw_face_property_new = _goal_objs[_goal_obj_ind]->mesh()->getFaceProperty<bool>("draw");
    for (const auto& face_index : _tumor_faces)
    {
        draw_face_property_new.set(face_index, _goal_active);
    }
    
}

int VirtuosoTissueGraspingSimulation::_calculateScore()
{
    assert(_goal_objs.size() > 0 && "Cannot calculate score when goal obj does not exist!");

    RigidMeshObject* goal_obj = _goal_objs[_goal_obj_ind];
    double total_mse = 0;
    for (const auto& face_index : _tumor_faces)
    {
        const Vec3i& tissue_face = _tissue_obj.mesh()->face(face_index);
        const Vec3r& tissue_v1 = _tissue_obj.mesh()->vertex(tissue_face[0]);
        const Vec3r& tissue_v2 = _tissue_obj.mesh()->vertex(tissue_face[1]);
        const Vec3r& tissue_v3 = _tissue_obj.mesh()->vertex(tissue_face[2]);

        const Vec3i& goal_face = goal_obj->mesh()->face(face_index);
        const Vec3r& goal_v1 = goal_obj->mesh()->vertex(goal_face[0]);
        const Vec3r& goal_v2 = goal_obj->mesh()->vertex(goal_face[1]);
        const Vec3r& goal_v3 = goal_obj->mesh()->vertex(goal_face[2]);
        total_mse += (tissue_v1 - goal_v1).squaredNorm() + (tissue_v2 - goal_v2).squaredNorm() + (tissue_v3 - goal_v3).squaredNorm();
    }

    return std::max(0, 10000 - static_cast<int>(100000*total_mse));
}

} // namespace Sim
