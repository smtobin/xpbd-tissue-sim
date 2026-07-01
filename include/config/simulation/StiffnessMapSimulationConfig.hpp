#ifndef __STIFFNESS_MAP_SIMULATION_CONFIG_HPP
#define __STIFFNESS_MAP_SIMULATION_CONFIG_HPP

#include "config/simulation/SimulationConfig.hpp"

namespace Config
{

class StiffnessMapSimulationConfig : public SimulationConfig
{

public:
    explicit StiffnessMapSimulationConfig(const YAML::Node& node)
        : SimulationConfig(node)
    {
        _extractParameter("time-to-steady-state", node, _time_to_steady_state);
        _extractParameter("displacement-magnitude", node, _displacement_magnitude);
    }

    Real timeToSteadyState() const { return _time_to_steady_state.value; }
    Real displacementMagnitude() const { return _displacement_magnitude.value; }

protected:
    ConfigParameter<Real> _time_to_steady_state = ConfigParameter<Real>(1);
    ConfigParameter<Real> _displacement_magnitude = ConfigParameter<Real>(5e-3);
};

} // namespace Config

#endif // __STIFFNESS_MAP_SIMULATION_CONFIG_HPP