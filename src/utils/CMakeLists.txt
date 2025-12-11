cmake_minimum_required(VERSION 3.13)
project(XPBDSim)

set_target_properties(MeshUtils PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(GeometryUtils PROPERTIES LINKER_LANGUAGE CXX)

set(UTILS_INCLUDE_DIR "../../include/utils")
set(UTILS_SRC_DIR "./")

set(MESH_UTILS_HEADERS
    ${UTILS_INCLUDE_DIR}/MeshUtils.hpp
)

set(MESH_UTILS_SOURCES
    ${UTILS_SRC_DIR}/MeshUtils.cpp
)

set(GEOMETRY_UTILS_HEADERS
    ${UTILS_INCLUDE_DIR}/GeometryUtils.hpp
)   

set(GEOMETRY_UTILS_SOURCES
    ${UTILS_SRC_DIR}/GeometryUtils.cpp
)

target_sources(MeshUtils
    PRIVATE
        ${MESH_UTILS_HEADERS}
        ${MESH_UTILS_SOURCES}       
)

install(TARGETS MeshUtils EXPORT XPBDSim-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES ${MESH_UTILS_HEADERS}
    DESTINATION include/${PROJECT_NAME}/utils)

target_sources(GeometryUtils
    PRIVATE
        ${GEOMETRY_UTILS_HEADERS}
        ${GEOMETRY_UTILS_SOURCES}
)

install(TARGETS GeometryUtils EXPORT XPBDSim-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES ${GEOMETRY_UTILS_HEADERS}
    DESTINATION include/${PROJECT_NAME}/utils)

