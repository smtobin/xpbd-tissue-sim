#include "simulation/VirtuosoTissueGraspingSimulation.hpp"
#include "config/simulation/VirtuosoTissueGraspingSimulationConfig.hpp"

#include "simulation/VirtuosoSimulation.hpp"

#include "utils/GeometryUtils.hpp"

// #include "simobject/MeshObject.hpp"

#include <filesystem>
#include <stdlib.h>
#include <type_traits>

namespace Sim
{

VirtuosoTissueGraspingSimulation::VirtuosoTissueGraspingSimulation(const Config::VirtuosoTissueGraspingSimulationConfig* config)
: VirtuosoSimulation(config)
{
    // extract parameters from config object
    _input_device = config->inputDevice();
    _ct_to_vb_transform = config->CTtoVBTransform();
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
    

    // go through all objects that are not the virtuoso robot and transform them to be w.r.t. the VB frame
    _objects.for_each_element([&](auto& obj){
        
        // VB -> sim world
        const Geometry::CoordinateFrame& vb_frame = _virtuoso_robot->VBFrame();
        // CT -> sim world
        Geometry::TransformationMatrix ct_to_sim = vb_frame.transform() * _ct_to_vb_transform;

        if constexpr (std::is_base_of_v<Sim::RigidMeshObject, typename std::remove_reference_t<decltype(obj)>::element_type>)
        {
            /** 12/5/25 TODO: this needs to work for initial rotations of VB frame. Right now, only works if VB orientation is identity. */
            std::cout << "Moving RigidMeshObject to express it in VB frame..." << std::endl;

            // by default, when we load meshes into the sim, we center their positions about the mesh's center of mass
            // (this normally makes meshes easier to position in the sim)
            // however, since we're assuming these meshes were loaded from a CT scan (with a defined CT origin), we need to 
            //  move the meshes back to recover the original positioning
            Vec3r dp = obj->mesh()->massCenter() - obj->mesh()->meshOrigin();
            obj->setPosition(obj->position() + dp);

            // now, use the CT -> sim world transform to put the mesh in the appropriate place
            obj->rotateAboutOrigin(ct_to_sim.rotMat());
            obj->setPosition(obj->position() + ct_to_sim.translation());
        }
        else if constexpr (std::is_base_of_v<Sim::MeshObject, typename std::remove_reference_t<decltype(obj)>::element_type>)
        {
            std::cout << "Moving mesh to express it in VB frame..." << std::endl;
            // by default, when we load meshes into the sim, we center their positions about the mesh's center of mass
            // (this normally makes meshes easier to position in the sim)
            // however, since we're assuming these meshes were loaded from a CT scan (with a defined CT origin), we need to 
            //  move the meshes back to recover the original positioning
            Vec3r dp = obj->mesh()->massCenter() - obj->mesh()->meshOrigin();
            obj->mesh()->moveTogether(dp);

            // now, use the CT -> sim world transform to put the mesh in the appropriate place
            obj->mesh()->rotateAbout(Vec3r::Zero(), ct_to_sim.rotMat());
            obj->mesh()->moveTogether(ct_to_sim.translation());
        }
    });

    // once we've found the tissue object, make sure that each virtuoso arm knows that this is the object that they're manipulating
    // (the VirtuosoArm class handles the grasping logic)
    if (_virtuoso_robot->hasArm1())
        _virtuoso_robot->arm1()->setToolManipulatedObject(_tissue_obj);
    if (_virtuoso_robot->hasArm2())
        _virtuoso_robot->arm2()->setToolManipulatedObject(_tissue_obj);
    
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

    // if (_input_device == SimulationInput::Device::HAPTIC)
    // {
    //     HHD handle = _haptic_device_manager->deviceHandles()[0];

    //     bool button1_pressed = _haptic_device_manager->button1Pressed(handle);

    //     if (!_grasping && button1_pressed)
    //     {
    //         _active_arm->setToolState(!_active_arm->toolState());
    //     }
    //     else if (_grasping && !button1_pressed)
    //     {
    //         _active_arm->setToolState(!_active_arm->toolState());
    //     }
    // }
}

} // namespace Sim
