#ifndef __PALPATION_SIMULATION_HPP
#define __PALPATION_SIMULATION_HPP

#include "simulation/VirtuosoSimulation.hpp"
#include "config/simulation/PalpationSimulationConfig.hpp"

#include "simobject/XPBDMeshObjectBaseWrapper.hpp"

namespace Sim
{

class PalpationSimulation : public VirtuosoSimulation
{
    public:

    PalpationSimulation(const Config::PalpationSimulationConfig* config);

    virtual std::string type() const override { return "PalpationSimulation"; }

    virtual void setup() override;

    /** Notifies the simulation that a key has been pressed in the viewer.
     * @param key : the key that was pressed
     * @param action : the action performed on the keyboard
     * @param modifiers : the modifiers (i.e. Shift, Ctrl, Alt)
     */
    // virtual void notifyKeyPressed(SimulationInput::Key key, SimulationInput::KeyAction action, int modifiers) override;

    // virtual void notifyMouseButtonPressed(SimulationInput::MouseButton button, SimulationInput::MouseAction action, int modifiers) override;

    // virtual void notifyMouseMoved(double x, double y) override;

    // virtual void notifyMouseScrolled(double dx, double dy) override;

    const Geometry::TetMesh* tissueMesh() const { assert(_tissue_obj); return _tissue_obj.tetMesh(); }

    protected:

    void _updateGraphics() override;
    
    void _timeStep() override;

    void _moveCursor(const Vec3r& dp);

    protected:

    XPBDMeshObject_BasePtrWrapper _tissue_obj;    // the tissue XPBD object that is being manipulated
};

} // namespace Sim

#endif // __PALPATION_SIMULATION_HPP