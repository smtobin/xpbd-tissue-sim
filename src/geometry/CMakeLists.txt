cmake_minimum_required(VERSION 3.13)
project(XPBDSim)

set(GEOMETRY_INCLUDE_DIR "../../include/geometry")
set(GEOMETRY_SRC_DIR "./")

set(GEOMETRY_HEADERS
    ${GEOMETRY_INCLUDE_DIR}/TransformationMatrix.hpp
    ${GEOMETRY_INCLUDE_DIR}/CoordinateFrame.hpp
    ${GEOMETRY_INCLUDE_DIR}/AABB.hpp
    ${GEOMETRY_INCLUDE_DIR}/Mesh.hpp
    ${GEOMETRY_INCLUDE_DIR}/TetMesh.hpp
    ${GEOMETRY_INCLUDE_DIR}/RefinedTetMesh.hpp
    ${GEOMETRY_INCLUDE_DIR}/MeshProperty.hpp
    ${GEOMETRY_INCLUDE_DIR}/SDF.hpp
    ${GEOMETRY_INCLUDE_DIR}/BoxSDF.hpp
    ${GEOMETRY_INCLUDE_DIR}/SphereSDF.hpp
    ${GEOMETRY_INCLUDE_DIR}/MeshSDF.hpp
    ${GEOMETRY_INCLUDE_DIR}/CylinderSDF.hpp
    ${GEOMETRY_INCLUDE_DIR}/VirtuosoArmSDF.hpp
    ${GEOMETRY_INCLUDE_DIR}/VirtuosoRobotSDF.hpp
    ${GEOMETRY_INCLUDE_DIR}/VirtuosoArmToolSDFs.hpp
    ${GEOMETRY_INCLUDE_DIR}/DeformableMeshSDF.hpp
    ${GEOMETRY_INCLUDE_DIR}/embree/EmbreeScene.hpp
    ${GEOMETRY_INCLUDE_DIR}/embree/EmbreeMeshGeometry.hpp
    ${GEOMETRY_INCLUDE_DIR}/embree/EmbreeTetMeshGeometry.hpp
    ${GEOMETRY_INCLUDE_DIR}/embree/EmbreeVirtuosoArmGeometry.hpp
)

set(GEOMETRY_SOURCES
    ${GEOMETRY_SRC_DIR}/Mesh.cpp
    ${GEOMETRY_SRC_DIR}/TetMesh.cpp
    ${GEOMETRY_SRC_DIR}/RefinedTetMesh.cpp
    ${GEOMETRY_SRC_DIR}/SphereSDF.cpp
    ${GEOMETRY_SRC_DIR}/BoxSDF.cpp
    ${GEOMETRY_SRC_DIR}/CylinderSDF.cpp
    ${GEOMETRY_SRC_DIR}/MeshSDF.cpp
    ${GEOMETRY_SRC_DIR}/VirtuosoArmSDF.cpp
    ${GEOMETRY_SRC_DIR}/VirtuosoRobotSDF.cpp
    ${GEOMETRY_SRC_DIR}/VirtuosoArmToolSDFs.cpp
    ${GEOMETRY_SRC_DIR}/DeformableMeshSDF.cpp  
    ${GEOMETRY_SRC_DIR}/embree/EmbreeScene.cpp
    ${GEOMETRY_SRC_DIR}/embree/EmbreeMeshGeometry.cpp
    ${GEOMETRY_SRC_DIR}/embree/EmbreeTetMeshGeometry.cpp
    ${GEOMETRY_SRC_DIR}/embree/EmbreeVirtuosoArmGeometry.cpp
)

target_sources(Geometry
    PRIVATE
        ${GEOMETRY_HEADERS}
        ${GEOMETRY_SOURCES}
)

install(TARGETS Geometry EXPORT XPBDSim-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES ${GEOMETRY_HEADERS}
    DESTINATION include/${PROJECT_NAME}/geometry)