cmake_minimum_required(VERSION 3.13)
project(XPBDSim)

set_target_properties(Config PROPERTIES LINKER_LANGUAGE CXX)

set(CONFIG_INCLUDE_DIR "../../include/config")
set(CONFIG_SRC_DIR "./")

set(CONFIG_HEADERS
    ${CONFIG_INCLUDE_DIR}/Config.hpp

    ${CONFIG_INCLUDE_DIR}/simulation/BeamSimulationConfig.hpp
    ${CONFIG_INCLUDE_DIR}/simulation/BeamStretchSimulationConfig.hpp
    ${CONFIG_INCLUDE_DIR}/simulation/InitialDeformationSimulationConfig.hpp
    ${CONFIG_INCLUDE_DIR}/simulation/SimulationConfig.hpp
    ${CONFIG_INCLUDE_DIR}/simulation/GraspingSimulationConfig.hpp
    ${CONFIG_INCLUDE_DIR}/simulation/VirtuosoSimulationConfig.hpp
    ${CONFIG_INCLUDE_DIR}/simulation/VirtuosoCTAnatomySimulationConfig.hpp
    
    ${CONFIG_INCLUDE_DIR}/simobject/ElasticMaterialConfig.hpp
    ${CONFIG_INCLUDE_DIR}/simobject/FirstOrderXPBDMeshObjectConfig.hpp
    ${CONFIG_INCLUDE_DIR}/simobject/MeshObjectConfig.hpp
    ${CONFIG_INCLUDE_DIR}/simobject/ObjectConfig.hpp    
    ${CONFIG_INCLUDE_DIR}/simobject/RigidMeshObjectConfig.hpp
    ${CONFIG_INCLUDE_DIR}/simobject/RigidObjectConfig.hpp
    ${CONFIG_INCLUDE_DIR}/simobject/RigidPrimitiveConfigs.hpp
    ${CONFIG_INCLUDE_DIR}/simobject/VirtuosoArmConfig.hpp
    ${CONFIG_INCLUDE_DIR}/simobject/VirtuosoRobotConfig.hpp
    ${CONFIG_INCLUDE_DIR}/simobject/XPBDMeshObjectConfig.hpp
)

set(CONFIG_SOURCES
    ${CONFIG_SRC_DIR}/simobject/FirstOrderXPBDMeshObjectConfig.cpp
    ${CONFIG_SRC_DIR}/simobject/RigidMeshObjectConfig.cpp
    ${CONFIG_SRC_DIR}/simobject/RigidPrimitiveConfigs.cpp
    ${CONFIG_SRC_DIR}/simobject/VirtuosoArmConfig.cpp
    ${CONFIG_SRC_DIR}/simobject/VirtuosoRobotConfig.cpp
    ${CONFIG_SRC_DIR}/simobject/XPBDMeshObjectConfig.cpp
)

target_sources(Config
    PRIVATE
        ${CONFIG_HEADERS}
        ${CONFIG_SOURCES}       
)

install(TARGETS Config EXPORT XPBDSim-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES ${CONFIG_HEADERS}
    DESTINATION include/${PROJECT_NAME}/config)