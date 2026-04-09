#include "simobject/Object.hpp"
#include "simulation/Simulation.hpp"

namespace Sim
{

Object::Object(const Simulation* sim, const ConfigType* config)
    : _name(config->name()), _sim(sim)
{
    _material_class = _sim->getMaterialClass(config->materialClass());
}

void Object::serialize(std::vector<std::byte>& buf) const
{
    pack(buf, _name);
    pack(buf, _char_dim);
    pack(buf, _material_class);
    pack(buf, _sim);
}
void Object::deserialize(const std::byte*& buf)
{
    unpack(buf, _name);
    unpack(buf, _char_dim);
    unpack(buf, _material_class);
    unpack(buf, _sim);
}

} // namespace Sim