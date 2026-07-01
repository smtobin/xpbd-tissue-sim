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
    }

protected:
};

} // namespace Config

#endif // __STIFFNESS_MAP_SIMULATION_CONFIG_HPP