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

    protected:

    XPBDMeshObject_BasePtrWrapper _tissue_obj;    // the tissue XPBD object that is being manipulated

    bool _express_meshes_in_vb_frame;   // if true, all meshes loaded into the scene will be put into the base frame of the Virtuoso robot in the scene
};

} // namespace Sim

#endif // __VIRTUOSO_TISSUE_GRASPING_SIMULATION_HPP