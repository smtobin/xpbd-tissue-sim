#ifndef __BEAM_STRETCH_SIMULATION_HPP
#define __BEAM_STRETCH_SIMULATION_HPP

#include "simulation/sim/outputSimulation.hpp"
#include "common/types.hpp"

namespace Sim
{

namespace Sim
{

class BeamStretchSimulation : public OutputSimulation
{
    public:

    explicit BeamStretchSimulation(const std::string& config_filename);

    virtual std::string toString() const override;
    virtual std::string type() const override { return "BeamStretchSimulation"; }

    virtual void setup() override;

    virtual void printInfo() const override;

    Real _stretch_velocity;
    Real _stretch_time;
};

} // namespace Sim

#endif // __BEAM_STRETCH_SIMULATION_HPP