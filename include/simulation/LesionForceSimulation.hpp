#ifndef __LESION_FORCE_SIMULATION_HPP
#define __LESION_FORCE_SIMULATION_HPP

#include "simulation/VirtuosoCTAnatomySimulation.hpp"

namespace Sim
{

class LesionForceSimulation : public VirtuosoCTAnatomySimulation
{
public:
    LesionForceSimulation(const Config::VirtuosoCTAnatomySimulationConfig* config);

    virtual std::string type() const override { return "LesionForceSimulation"; }

    virtual void setup() override;

    const Vec3r& lesionBodyForce() const { return _lesion_body_force; }
    void setLesionBodyForce(const Vec3r& force);

private:
    Vec3r _lesion_body_force;
    unsigned _lesion_class_index;

    std::vector<int> _lesion_elements;
};

} // namespace Sim

#endif // __LESION_FORCE_SIMULATION_HPP