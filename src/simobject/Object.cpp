#include "simobject/Object.hpp"
#include "simulation/Simulation.hpp"

namespace Sim
{

Object::Object(const Simulation* sim, const ConfigType* config)
    : _name(config->name()), _sim(sim)
{
    _material_class = _sim->getMaterialClass(config->materialClass());
}
} // namespace Sim