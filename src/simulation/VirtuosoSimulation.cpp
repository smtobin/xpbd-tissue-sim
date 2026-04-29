#include "simulation/VirtuosoSimulation.hpp"

#include "utils/GeometryUtils.hpp"
#include "utils/MathUtils.hpp"

#include <filesystem>
#include <stdlib.h>

namespace Sim
{

VirtuosoSimulation::VirtuosoSimulation(const Config::VirtuosoSimulationConfig* config)
    : Simulation(config), _virtuoso_robot(nullptr), _active_arm(nullptr)
{
    _input_device = config->inputDevice();
    _show_tip_cursor = config->showTipCursor();

    // initialize the haptic device if using haptic device input
    if (_input_device == SimulationInput::Device::HAPTIC)
    {
        std::cout << BOLD << "Initializing haptic device(s)..." << RST << std::endl;
        if (config->hapticDeviceName1().has_value() && config->hapticDeviceName2().has_value())
        {
            _haptic_device_manager = std::make_unique<HapticDeviceManager>(config->hapticDeviceName1().value(), config->hapticDeviceName2().value());
        }
        else if (config->hapticDeviceName1().has_value())
        {
            _haptic_device_manager = std::make_unique<HapticDeviceManager>(config->hapticDeviceName1().value());
        }
        else
        {
            _haptic_device_manager = std::make_unique<HapticDeviceManager>();
        }

        
        _haptic_device_manager->setForceScaling(config->hapticForceScaling());
        
    }

    // initialize the keys map with relevant keycodes for controlling the Virtuoso robot with the keyboard
    SimulationInput::Key keys[] = {
        SimulationInput::Key::SPACE, // space bar
        SimulationInput::Key::Q, // Q
        SimulationInput::Key::W, // W
        SimulationInput::Key::E, // E
        SimulationInput::Key::R, // R
        SimulationInput::Key::A, // A
        SimulationInput::Key::S, // S
        SimulationInput::Key::D, // D
        SimulationInput::Key::F  // F
    };

    size_t num_keys = sizeof(keys) / sizeof(keys[0]);
    for (unsigned i = 0; i < num_keys; i++)
        _keys_held[keys[i]] = 0;
}

void VirtuosoSimulation::setup()
{
    Simulation::setup();

    // find the VirtuosoRobot object - necessary for Virtuoso simulation controls
    auto& _virtuoso_robot_objs = _objects.template get<std::unique_ptr<VirtuosoRobot>>();
    assert((_virtuoso_robot_objs.size() == 1) && "There must be exactly 1 VirtuosoRobot in the sim!");
    _virtuoso_robot = _virtuoso_robot_objs.front().get();


    // set the active arm to be arm1
    _active_arm = _virtuoso_robot->arm1();
    assert(_active_arm);

    
    // create an object at the tip of the robot to show where grasping is
    if (_show_tip_cursor)
    {
        Config::ObjectRenderConfig cursor_render_config(Config::ObjectRenderConfig::RenderType::PBR, std::nullopt, std::nullopt, std::nullopt,
                0.0, 0.5, 0.3, Vec3r(1.0, 1.0, 0.0), true, true, false, false);
        Config::MaterialClassConfig cursor_material_config("cursor", Config::ElasticMaterialConfig("cursor"), cursor_render_config);
        MaterialClass cursor_mat(&cursor_material_config);
        addMaterial(cursor_mat);
        
        if (_virtuoso_robot->hasArm1())
        {
            Config::RigidSphereConfig cursor_config("tip_cursor1", "cursor", Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0),
                1.0, 0.001, false, true, false, cursor_render_config);
            _tip_cursor1 = _addObjectFromConfig(&cursor_config);
            _tip_cursor1->setPosition(_virtuoso_robot->arm1()->actualTipPosition());
        }
        if (_virtuoso_robot->hasArm2() && _input_device == SimulationInput::Device::HAPTIC && _haptic_device_manager->deviceHandles().size() == 2)
        {
            Config::RigidSphereConfig cursor_config("tip_cursor2", "cursor", Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0),
                1.0, 0.001, false, true, false, cursor_render_config);
            _tip_cursor2 = _addObjectFromConfig(&cursor_config);
            _tip_cursor2->setPosition(_virtuoso_robot->arm2()->actualTipPosition());
        }
    }
    
}

void VirtuosoSimulation::updateGraphicsCameraPoseToRobotCamFrame()
{
    if (_graphics_scene)
    {
        const Geometry::TransformationMatrix& cam_transform = _virtuoso_robot->camFrame().transform();
        _graphics_scene->setCameraPosition(cam_transform.translation());

        // find view dir
        const Vec3r& z_axis_pt = cam_transform.rotMat() * Vec3r(0,0,1) + cam_transform.translation();
        const Vec3r& y_axis_pt = cam_transform.rotMat() * Vec3r(0,-1,0) + cam_transform.translation();  // use the negative y direction, since by convention y-axis points down for cam frame
        _graphics_scene->setCameraViewDirection(z_axis_pt - cam_transform.translation());
        _graphics_scene->setCameraUpDirection(y_axis_pt - cam_transform.translation());
        _graphics_scene->setCameraFOV(80.0);
    }
}

void VirtuosoSimulation::notifyMouseButtonPressed(SimulationInput::MouseButton button, SimulationInput::MouseAction action, int modifiers)
{

    Simulation::notifyMouseButtonPressed(button, action, modifiers);

}

void VirtuosoSimulation::notifyMouseMoved(double x, double y)
{
    if (_input_device == SimulationInput::Device::MOUSE)
    {
        if (_keys_held[SimulationInput::Key::SPACE] > 0) // space bar = clutch
        {
            const Real scaling = 0.0005;
            Real dx = x - _last_mouse_pos[0];
            Real dy = y - _last_mouse_pos[1];

            // camera plane defined by camera up direction and camera right direction
            // changes in mouse y position = changes along camera up direction
            // changes in mouse x position = changes along camera right direction
            const Vec3r up_vec = _graphics_scene->cameraUpDirection();
            const Vec3r right_vec = _graphics_scene->cameraRightDirection();
            
            const Vec3r current_tip_position = _active_arm->commandedTipPosition();
            const Vec3r offset = right_vec*dx + up_vec*dy;
            _moveArm(_active_arm, _tip_cursor1, offset*scaling, Vec3r::Zero());
        }
    }

    _last_mouse_pos[0] = x;
    _last_mouse_pos[1] = y;
}

void VirtuosoSimulation::notifyKeyPressed(SimulationInput::Key key, SimulationInput::KeyAction action, int modifiers)
{

    // find key in map
    auto it = _keys_held.find(key);
    if (it != _keys_held.end())
    {
        it->second = (action == SimulationInput::KeyAction::PRESS); // if action > 0, key is pressed or held
    }

    // when 'ALT' is pressed, switch which arm is active
    if (key == SimulationInput::Key::ALT && action == SimulationInput::KeyAction::PRESS)
    {
        if (_virtuoso_robot->hasArm2() && _active_arm == _virtuoso_robot->arm1())
        {
            _active_arm = _virtuoso_robot->arm2();
        }
        else if (_virtuoso_robot->hasArm1() && _active_arm == _virtuoso_robot->arm2())
        {
            _active_arm = _virtuoso_robot->arm1();
        }

        if (_show_tip_cursor)
            _tip_cursor1->setPosition(_active_arm->commandedTipPosition());
    }
    // when 'TAB' is pressed, switch the camera view to the endoscope view
    else if (key == SimulationInput::Key::TAB && action == SimulationInput::KeyAction::PRESS)
    {
        updateGraphicsCameraPoseToRobotCamFrame();
    }

    else if (key == SimulationInput::Key::SPACE && action == SimulationInput::KeyAction::PRESS)
    {
        if (_input_device == SimulationInput::Device::MOUSE)
        {
            _graphics_scene->viewer()->enableMouseInteraction(false);   // disable mouse interaction with the viewer when actively using mouse control (space bar is pressed)
        }
    }
    else if (key == SimulationInput::Key::SPACE && action == SimulationInput::KeyAction::RELEASE)
    {
        if (_input_device == SimulationInput::Device::MOUSE)
        {
            _graphics_scene->viewer()->enableMouseInteraction(true);   // enable mouse interaction with the viewer when not actively using mouse control (space bar is released)
        }
    }

    Simulation::notifyKeyPressed(key, action, modifiers);

}

void VirtuosoSimulation::notifyMouseScrolled(double dx, double dy)
{
    // when using mouse input, mouse scrolling moves the robot tip in and out of the page
    if (_input_device == SimulationInput::Device::MOUSE)
    {
        if (_keys_held[SimulationInput::Key::SPACE] > 0) // space bar = clutch
        {
            const Real scaling = 0.0001;
            const Vec3r view_dir = _graphics_scene->cameraViewDirection();

            const Vec3r current_tip_position = _active_arm->commandedTipPosition();
            const Vec3r offset = view_dir*dy;
            _moveArm(_active_arm, _tip_cursor1, offset*scaling, Vec3r::Zero());
        }
    }

    Simulation::notifyMouseScrolled(dx, dy);
    
}

void VirtuosoSimulation::setArm1JointState(double ot_rot, double ot_trans, double it_rot, double it_trans, int tool)
{
    assert(_virtuoso_robot && _virtuoso_robot->hasArm1());

    {
        std::lock_guard<std::mutex> l(_arm1_joint_state.mtx);
        _arm1_joint_state.state = VirtuosoArmJointState(ot_rot, ot_trans, it_rot, it_trans, tool);
        _arm1_joint_state.has_new = true;
    }
}
void VirtuosoSimulation::setArm2JointState(double ot_rot, double ot_trans, double it_rot, double it_trans, int tool)
{
    assert(_virtuoso_robot && _virtuoso_robot->hasArm2());

    {
        std::lock_guard<std::mutex> l(_arm2_joint_state.mtx);
        _arm2_joint_state.state = VirtuosoArmJointState(ot_rot, ot_trans, it_rot, it_trans, tool);
        _arm2_joint_state.has_new = true;
    }
}
void VirtuosoSimulation::setArm1TipPosition(const Vec3r& tip_pos)
{
    assert(_virtuoso_robot && _virtuoso_robot->hasArm1());

    {
        std::lock_guard<std::mutex> l(_arm1_tip_pose.mtx);
        _arm1_tip_pose.state.setTranslation(tip_pos);
        _arm1_tip_pose.has_new = true;
    }
}
void VirtuosoSimulation::setArm2TipPosition(const Vec3r& tip_pos)
{
    assert(_virtuoso_robot && _virtuoso_robot->hasArm2());

    {
        std::lock_guard<std::mutex> l(_arm2_tip_pose.mtx);
        _arm2_tip_pose.state.setTranslation(tip_pos);
        _arm2_tip_pose.has_new = true;
    }
}
void VirtuosoSimulation::setArm1TipPose(const Geometry::TransformationMatrix& tip_pose)
{
    assert(_virtuoso_robot && _virtuoso_robot->hasArm1());

    {
        std::lock_guard<std::mutex> l(_arm1_tip_pose.mtx);
        _arm1_tip_pose.state = tip_pose;
        _arm1_tip_pose.has_new = true;
    }
}
void VirtuosoSimulation::setArm2TipPose(const Geometry::TransformationMatrix& tip_pose)
{
    assert(_virtuoso_robot && _virtuoso_robot->hasArm2());

    {
        std::lock_guard<std::mutex> l(_arm2_tip_pose.mtx);
        _arm2_tip_pose.state = tip_pose;
        _arm2_tip_pose.has_new = true;
    }
}
void VirtuosoSimulation::setArm1ToolState(int tool_state)
{
    assert(_virtuoso_robot && _virtuoso_robot->hasArm1());

    {
        std::lock_guard<std::mutex> l(_arm1_tool_state.mtx);
        _arm1_tool_state.state = tool_state;
        _arm1_tool_state.has_new = true;
    }
}
void VirtuosoSimulation::setArm2ToolState(int tool_state)
{
    assert(_virtuoso_robot && _virtuoso_robot->hasArm2());

    {
        std::lock_guard<std::mutex> l(_arm2_tool_state.mtx);
        _arm2_tool_state.state = tool_state;
        _arm2_tool_state.has_new = true;
    }
}

void VirtuosoSimulation::_moveArm(Sim::VirtuosoArm* arm, RigidSphere* cursor, const Vec3r& dp, const Vec3r& dR)
{
    Vec3r dp_clamped = dp;
    Real max_dp = 2e-5;
    if (dp.norm() > 2e-5)
    {
        dp_clamped = dp * (2e-5 / dp.norm());
    }
    // move the tip cursor and the active arm tip position
    Geometry::TransformationMatrix cur_commanded_pose = arm->commandedTipPose();
    const Vec3r current_tip_position = cur_commanded_pose.translation();
    const Vec3r new_commanded_position = current_tip_position + dp_clamped;

    const Mat3r current_rot = cur_commanded_pose.rotMat();
    const Mat3r new_rot = current_rot * MathUtils::Exp_so3(dR);

    Geometry::TransformationMatrix new_pose(new_rot, new_commanded_position);
    arm->setCommandedTipPose(new_pose);

    if (_show_tip_cursor)
        cursor->setPosition(new_commanded_position);
}

void VirtuosoSimulation::_updateGraphics()
{

    Simulation::_updateGraphics();
}

void VirtuosoSimulation::_timeStep()
{
    if (_input_device == SimulationInput::Device::KEYBOARD)
    {
        if (_keys_held[SimulationInput::Key::Q] > 0) // Q = CCW inner tube rotation
        {
            const Real cur_rot = _active_arm->innerTubeRotation();
            _active_arm->setInnerTubeRotation(cur_rot + IT_ROT_RATE*dt());

        }
        if (_keys_held[SimulationInput::Key::W] > 0) // W = CCW outer tube rotation
        {
            const Real cur_rot = _active_arm->outerTubeRotation();
            _active_arm->setOuterTubeRotation(cur_rot + OT_ROT_RATE*dt());
        }
        if (_keys_held[SimulationInput::Key::E] > 0) // E = inner tube extension
        {
            const Real cur_trans = _active_arm->innerTubeTranslation();
            _active_arm->setInnerTubeTranslation(cur_trans + IT_TRANS_RATE*dt());
        }
        if (_keys_held[SimulationInput::Key::R] > 0) // R = outer tube extension
        {
            const Real cur_trans = _active_arm->outerTubeTranslation();
            _active_arm->setOuterTubeTranslation(cur_trans + OT_TRANS_RATE*dt());
        }
        if (_keys_held[SimulationInput::Key::A] > 0) // A = CW inner tube rotation
        {
            const Real cur_rot = _active_arm->innerTubeRotation();
            _active_arm->setInnerTubeRotation(cur_rot - IT_ROT_RATE*dt()); 
        }
        if (_keys_held[SimulationInput::Key::S] > 0) // S = CW outer tube rotation
        {
            const Real cur_rot = _active_arm->outerTubeRotation();
            _active_arm->setOuterTubeRotation(cur_rot - OT_ROT_RATE*dt());
        }
        if (_keys_held[SimulationInput::Key::D] > 0) // D = inner tube retraction
        {
            const Real cur_trans = _active_arm->innerTubeTranslation();
            _active_arm->setInnerTubeTranslation(cur_trans - IT_TRANS_RATE*dt());
        }
        if (_keys_held[SimulationInput::Key::F] > 0) // F = outer tube retraction
        {
            const Real cur_trans = _active_arm->outerTubeTranslation();
            _active_arm->setOuterTubeTranslation(cur_trans - OT_TRANS_RATE*dt());
        }

        if (_show_tip_cursor)
            _tip_cursor1->setPosition(_active_arm->commandedTipPosition());
    }

    if (_input_device == SimulationInput::Device::HAPTIC)
    {
        const std::vector<HHD>& device_handles = _haptic_device_manager->deviceHandles();
        // std::cout << "Num devicec handles: " << device_handles.size() << std::endl;

        for (unsigned i = 0; i < device_handles.size(); i++)
        {
            Sim::VirtuosoArm* arm = nullptr;
            Sim::RigidSphere* cursor = nullptr;
            if (device_handles.size() == 1)
            {
                arm = _active_arm;
                cursor = _tip_cursor1;
            }
            if (device_handles.size() == 2 && i == 0)
            {
                arm = _virtuoso_robot->arm1();
                cursor = _tip_cursor1;
            }
            if (device_handles.size() == 2 && i == 1)
            {
                arm = _virtuoso_robot->arm2();
                cursor = _tip_cursor2;
            }
            
            if (!arm)
            {
                std::cout << "i = " << i << ": ARM NOT VALID!" << std::endl;
                continue;
            }

            HHD handle = device_handles[i];
            Vec3r last_pos = _haptic_device_manager->lastPosition(handle);
            Vec3r cur_pos = _haptic_device_manager->position(handle);
            Mat3r last_rot = _haptic_device_manager->lastOrientation(handle);
            Mat3r cur_rot = _haptic_device_manager->orientation(handle);
            const Vec3r cur_force = _haptic_device_manager->force(handle);

            bool button1_pressed = _haptic_device_manager->button1Pressed(handle);
            bool button2_pressed = _haptic_device_manager->button2Pressed(handle);
            
            arm->setToolState((int)button1_pressed);

            if (button2_pressed)
            {
                Vec3r dx = cur_pos - last_pos;

                // transform dx from haptic input frame to camera frame
                Vec3r dx_camera = GeometryUtils::Ry(M_PI) * dx;
                Mat3r rot_mat;
                rot_mat.col(1) = _graphics_scene->cameraUpDirection();
                rot_mat.col(2) = _graphics_scene->cameraViewDirection();
                rot_mat.col(0) = rot_mat.col(1).cross(rot_mat.col(2));
                // transform from camera frame to global frame
                Vec3r dx_sim = rot_mat * dx_camera;

                Vec3r dR = MathUtils::Minus_SO3(cur_rot, last_rot);
                _moveArm(arm, cursor, dx_sim*0.00005, dR);

                // transform force from global frame to haptic frame
                Vec3r cam_force = rot_mat.transpose() * arm->unfilteredCollisionForce();
                Vec3r haptic_force = GeometryUtils::Ry(-M_PI) * cam_force;
                
                
                Real frac = 0.3;
                const Vec3r new_force = frac*haptic_force + (1-frac)*cur_force;
                
                // std::cout << "i = " << i << ": Setting haptic force to " << new_force.transpose() << std::endl;
                _haptic_device_manager->setForce(handle, new_force);
            }
            
        }
        
    }


    // check if the joint state has been updated externally
    if (_arm1_joint_state.has_new.load())
    {
        std::lock_guard<std::mutex> l(_arm1_joint_state.mtx);
        _virtuoso_robot->arm1()->setJointState(
            _arm1_joint_state.state.outer_tube_rotation,
            _arm1_joint_state.state.outer_tube_translation,
            _arm1_joint_state.state.inner_tube_rotation,
            _arm1_joint_state.state.inner_tube_translation,
            _arm1_joint_state.state.tool
        );
        _arm1_joint_state.has_new = false;
    }

    if (_arm2_joint_state.has_new.load())
    {
        std::lock_guard<std::mutex> l(_arm2_joint_state.mtx);
        _virtuoso_robot->arm2()->setJointState(
            _arm2_joint_state.state.outer_tube_rotation,
            _arm2_joint_state.state.outer_tube_translation,
            _arm2_joint_state.state.inner_tube_rotation,
            _arm2_joint_state.state.inner_tube_translation,
            _arm2_joint_state.state.tool
        );
        _arm2_joint_state.has_new = false;
    }

    if (_arm1_tip_pose.has_new.load())
    {
        std::lock_guard<std::mutex> l(_arm1_tip_pose.mtx);
        _virtuoso_robot->arm1()->setCommandedTipPose(_arm1_tip_pose.state);
        _arm1_tip_pose.has_new = false;
    }
    if (_arm2_tip_pose.has_new.load())
    {
        std::lock_guard<std::mutex> l(_arm2_tip_pose.mtx);
        _virtuoso_robot->arm2()->setCommandedTipPose(_arm2_tip_pose.state);
        _arm2_tip_pose.has_new = false;
    }

    if (_arm1_tool_state.has_new.load())
    {
        std::lock_guard<std::mutex> l(_arm1_tool_state.mtx);
        _virtuoso_robot->arm1()->setToolState(_arm1_tool_state.state);
        _arm1_tool_state.has_new = false;
    }
    if (_arm2_tool_state.has_new.load())
    {
        std::lock_guard<std::mutex> l(_arm2_tool_state.mtx);
        _virtuoso_robot->arm2()->setToolState(_arm2_tool_state.state);
        _arm2_tool_state.has_new = false;
    }

    

    Simulation::_timeStep();
}


} // namespace Sim
