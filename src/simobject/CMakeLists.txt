cmake_minimum_required(VERSION 3.13)
project(XPBDSim)

set(SIMOBJECT_INCLUDE_DIR "../../include/simobject")
set(SIMOBJECT_SRC_DIR "./")

set(SIMOBJECT_HEADERS
    ${SIMOBJECT_INCLUDE_DIR}/Object.hpp
    ${SIMOBJECT_INCLUDE_DIR}/RigidObject.hpp
    ${SIMOBJECT_INCLUDE_DIR}/RigidPrimitives.hpp
    ${SIMOBJECT_INCLUDE_DIR}/XPBDMeshObject.hpp
    ${SIMOBJECT_INCLUDE_DIR}/RigidMeshObject.hpp
    ${SIMOBJECT_INCLUDE_DIR}/ElasticMaterial.hpp
    ${SIMOBJECT_INCLUDE_DIR}/VirtuosoArm.hpp
    ${SIMOBJECT_INCLUDE_DIR}/VirtuosoRobot.hpp
    ${SIMOBJECT_INCLUDE_DIR}/VirtuosoArmTools.hpp
)

set(SIMOBJECT_SOURCES
    ${SIMOBJECT_SRC_DIR}/Object.cpp
    ${SIMOBJECT_SRC_DIR}/RigidObject.cpp
    ${SIMOBJECT_SRC_DIR}/RigidPrimitives.cpp
    ${SIMOBJECT_SRC_DIR}/XPBDMeshObject.cpp
    ${SIMOBJECT_SRC_DIR}/RigidMeshObject.cpp
    ${SIMOBJECT_SRC_DIR}/VirtuosoArm.cpp
    ${SIMOBJECT_SRC_DIR}/VirtuosoRobot.cpp
    ${SIMOBJECT_SRC_DIR}/VirtuosoArmTools.cpp
)

target_sources(SimObject
    PRIVATE
        ${SIMOBJECT_HEADERS}
        ${SIMOBJECT_SOURCES}
)
install(TARGETS SimObject EXPORT XPBDSim-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES ${SIMOBJECT_HEADERS}
    DESTINATION include/${PROJECT_NAME}/simobject)