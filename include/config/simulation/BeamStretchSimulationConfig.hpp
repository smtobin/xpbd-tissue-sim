#ifndef __BEAM_STRETCH_SIMULATION_CONFIG_HPP
#define __BEAM_STRETCH_SIMULATION_CONFIG_HPP

#include "config/simulation/sim/outputSimulationConfig.hpp"

namespace Config
{

class BeamStretchSimulationConfig : public OutputSimulationConfig
{
    public:
    explicit BeamStretchSimulationConfig(const YAML::Node& node)
        : OutputSimulationConfig(node)
    {
        // extract the parameters from the config file
        _extractParameter("stretch-velocity", node, _stretch_velocity);
        _extractParameter("stretch-time", node, _stretch_time);
    }

    // getters
    Real stretchVelocity() const { return _stretch_velocity.value; }
    Real stretchTime() const { return _stretch_time.value; }

    protected:
    /** The velocity of the beam stretching, in m/s. Use a negative value for beam compression. */
    ConfigParameter<Real> _stretch_velocity = ConfigParameter<Real>(0.5);
    /** The amount of time to stretch the beam. */
    ConfigParameter<Real> _stretch_time = ConfigParameter<Real>(1.0); // in s
};

} // namespace Config

#endif // __BEAM_STRETCH_SIMULATION_CONFIG_HPP