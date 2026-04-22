#ifndef __VIRTUOSO_SIMULATION_HPP
#define __VIRTUOSO_SIMULATION_HPP

#include "simulation/Simulation.hpp"
#include "simobject/VirtuosoRobot.hpp"
#include "simobject/VirtuosoArm.hpp"
#include "simobject/RigidMeshObject.hpp"
#include "simobject/RigidPrimitives.hpp"
#include "simobject/XPBDMeshObject.hpp"

#include "config/simulation/VirtuosoSimulationConfig.hpp"

#include "haptics/HapticDeviceManager.hpp"

#include <map>
#include <atomic>

namespace Sim
{

class VirtuosoSimulation : public Simulation
{
    public:
    VirtuosoSimulation(const Config::VirtuosoSimulationConfig* config);

    virtual std::string type() const override { return "VirtuosoSimulation"; }

    virtual void setup() override;

    virtual void notifyKeyPressed(SimulationInput::Key key, SimulationInput::KeyAction action, int modifiers) override;

    virtual void notifyMouseButtonPressed(SimulationInput::MouseButton button, SimulationInput::MouseAction action, int modifiers) override;

    virtual void notifyMouseMoved(double x, double y) override;

    virtual void notifyMouseScrolled(double dx, double dy) override;

    const VirtuosoRobot* virtuosoRobot() const { return _virtuoso_robot; }
    VirtuosoRobot* virtuosoRobot() { return _virtuoso_robot; }
    const VirtuosoArm* activeArm() const { return _active_arm; }
    const Vec3r activeTipPosition() const { return _tip_cursor1->position(); }

    void setArm1JointState(double ot_rot, double ot_trans, double it_rot, double it_trans, int tool);
    void setArm2JointState(double ot_rot, double ot_trans, double it_rot, double it_trans, int tool);

    void setArm1TipPosition(const Vec3r& new_pos);
    void setArm2TipPosition(const Vec3r& new_pos);

    void setArm1TipPose(const Geometry::TransformationMatrix& new_pose);
    void setArm2TipPose(const Geometry::TransformationMatrix& new_pose);

    void setArm1ToolState(int tool);
    void setArm2ToolState(int tool);

    /** Updates the camera position and orientation in the graphics scene to match that of the endoscopic camera on the Virtuoso robot. */
    void updateGraphicsCameraPoseToRobotCamFrame();

    protected:

    struct VirtuosoArmJointState
    {
        double outer_tube_rotation;
        double outer_tube_translation;
        double inner_tube_rotation;
        double inner_tube_translation;
        int tool;

        VirtuosoArmJointState(double ot_rot, double ot_trans, double it_rot, double it_trans, int tool_)
            : outer_tube_rotation(ot_rot), outer_tube_translation(ot_trans), inner_tube_rotation(it_rot), inner_tube_translation(it_trans), tool(tool_)
        {}

        VirtuosoArmJointState()
            : outer_tube_rotation(0.0), outer_tube_translation(0.0), inner_tube_rotation(0.0), inner_tube_translation(0.0), tool(0)
        {}
    };

    void _updateGraphics() override;
    
    void _timeStep() override;

    void _moveArm(Sim::VirtuosoArm* arm, Sim::RigidSphere* cursor, const Vec3r& dp, const Vec3r& dR);

    protected:
    /** Struct for receiving state updates in a thread-safe way.
     * Parts of the simulation (Virtuoso arm state) may be set through ROS, from a different thread. We need to make sure that these updates
     * do not get applied in the middle of a time step, but instead inbetween time steps.
     */
    template<typename State>
    struct _AsyncState
    {
        std::mutex mtx;
        std::atomic<bool> has_new;
        State state;

        _AsyncState()
            : has_new(false) {}
    };

    VirtuosoRobot* _virtuoso_robot = nullptr; // the Virtuoso robot (includes both arms)
    VirtuosoArm* _active_arm = nullptr;       // whichever arm is being actively controlled (assuming only one input device)

    _AsyncState<VirtuosoArmJointState> _arm1_joint_state;   // arm1's joint state may be set through ROS    
    _AsyncState<VirtuosoArmJointState> _arm2_joint_state;   // arm2's joint state may be set through ROS

    _AsyncState<Geometry::TransformationMatrix> _arm1_tip_pose; // arm1's commanded tip pose may be set through ROS
    _AsyncState<Geometry::TransformationMatrix> _arm2_tip_pose; // arm2's commanded tip pose may be set through ROS

    _AsyncState<int> _arm1_tool_state;  // arm1's tool state may be set through ROS
    _AsyncState<int> _arm2_tool_state;  // arm2's tool state may be set through ROS

    
    SimulationInput::Device _input_device;    // the type of input device used (Keyboard, Mouse, or Haptic)
    bool _show_tip_cursor;  // whether or not to show the tip cursor

    RigidSphere* _tip_cursor1 = nullptr;       // spherical object for visualizing grasp area 
    RigidSphere* _tip_cursor2 = nullptr;

    /** MOUSE INPUT */
    Vec2r _last_mouse_pos;    // tracks the last mouse position (used in when mouse input is used to control the arms)

    /** HAPTIC INPUT */
    std::unique_ptr<HapticDeviceManager> _haptic_device_manager;

    /** KEYBOARD INPUT */
    constexpr static Real IT_ROT_RATE = 3; // rad/s
    constexpr static Real IT_TRANS_RATE = 0.005; // m/s
    constexpr static Real OT_ROT_RATE = 3; // rad/s
    constexpr static Real OT_TRANS_RATE = 0.005; // m/s

    std::map<SimulationInput::Key, bool> _keys_held;     // tracks which keys are held (from a pre-defined set of keys)
};

} // namespace Sim

#endif // __VIRTUOSO_SIMULATION_HPP