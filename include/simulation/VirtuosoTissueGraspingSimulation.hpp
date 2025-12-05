#ifndef __VIRTUOSO_TISSUE_GRASPING_SIMULATION_HPP
#define __VIRTUOSO_TISSUE_GRASPING_SIMULATION_HPP

#include "simulation/VirtuosoSimulation.hpp"
#include "config/simulation/VirtuosoTissueGraspingSimulationConfig.hpp"

#include "simobject/XPBDMeshObjectBaseWrapper.hpp"

namespace Sim
{

class VirtuosoTissueGraspingSimulation : public VirtuosoSimulation
{
    public:
    struct TissueClasses
    {
        constexpr static int TRACHEA=1;
        constexpr static int TUMOR=0;
    };

    VirtuosoTissueGraspingSimulation(const Config::VirtuosoTissueGraspingSimulationConfig* config);

    virtual std::string type() const override { return "VirtuosoTissueGraspingSimulation"; }

    virtual void setup() override;

    /** Notifies the simulation that a key has been pressed in the viewer.
     * @param key : the key that was pressed
     * @param action : the action performed on the keyboard
     * @param modifiers : the modifiers (i.e. Shift, Ctrl, Alt)
     */
    virtual void notifyKeyPressed(SimulationInput::Key key, SimulationInput::KeyAction action, int modifiers) override;

    virtual void notifyMouseButtonPressed(SimulationInput::MouseButton button, SimulationInput::MouseAction action, int modifiers) override;

    virtual void notifyMouseMoved(double x, double y) override;

    virtual void notifyMouseScrolled(double dx, double dy) override;

    const Geometry::TetMesh* tissueMesh() const { assert(_tissue_obj); return _tissue_obj.tetMesh(); }

    protected:

    void _updateGraphics() override;
    
    void _timeStep() override;

    void _moveCursor(const Vec3r& dp);

    void _toggleGoal();
    void _changeGoal();

    int _calculateScore();

    protected:

    XPBDMeshObject_BasePtrWrapper _tissue_obj;    // the tissue XPBD object that is being manipulated
    std::optional<std::string> _fixed_faces_filename;   // a .txt filename that lists all the faces that should be held fixed

    bool _express_meshes_in_vb_frame;   // if true, all meshes loaded into the scene will be put into the base frame of the Virtuoso robot in the scene
};

} // namespace Sim

#endif // __VIRTUOSO_TISSUE_GRASPING_SIMULATION_HPP