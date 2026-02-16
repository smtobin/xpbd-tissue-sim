#ifndef __SIMULATION_TYPE_DEFS_HPP
#define __SIMULATION_TYPE_DEFS_HPP

#include "config/simobject/FirstOrderXPBDMeshObjectConfig.hpp"
#include "config/simobject/XPBDMeshObjectConfig.hpp"
#include "config/simobject/RigidMeshObjectConfig.hpp"
#include "config/simobject/RigidPrimitiveConfigs.hpp"
#include "config/simobject/VirtuosoArmConfig.hpp"
#include "config/simobject/VirtuosoRobotConfig.hpp"

#include "common/TypeList.hpp"

/** The types of Config classes the Simulation should expect to appear in the .yaml config file.
 * These should have a one-to-one correspondance with the different types of "objects" supported by the simulation
 */
using SimulationObjectConfigTypes = TypeList<Config::XPBDMeshObjectConfig,
                                             Config::FirstOrderXPBDMeshObjectConfig,
                                             Config::RigidSphereConfig,
                                             Config::RigidBoxConfig,
                                             Config::RigidCylinderConfig,
                                             Config::RigidMeshObjectConfig,
                                             Config::VirtuosoArmConfig,
                                             Config::VirtuosoRobotConfig>;
// Helper to get a list of Object types with no duplicates
template <typename List>
struct GetObjectTypesFromConfigTypes;

template <typename... ConfigTypes>
struct GetObjectTypesFromConfigTypes<TypeList<ConfigTypes...>>
{
    using duped_type = TypeList<typename ConfigTypes::ObjectType...>;
    using deduped_type = typename TypeListRemoveDuplicates<duped_type>::type;
};

/** The types of simulation Objects the Simulation should expect. These come directly from the SimulationObjectconfigTypes. */
using SimulationObjectTypes = GetObjectTypesFromConfigTypes<SimulationObjectConfigTypes>::deduped_type;

// define std::variant with const ptr types of all object types
using SimulationObjectConstPtrVariantType = VariantFromTypeList<SimulationObjectTypes>::const_ptr_type;

#endif // __SIMULATION_TYPE_DEFS_HPP