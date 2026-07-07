cmake_minimum_required(VERSION 3.13)
project(XPBDSim)

set(GRAPHICS_INCLUDE_DIR "../../include/graphics")
set(GRAPHICS_SRC_DIR "./")

set(GRAPHICS_HEADERS
    ${GRAPHICS_INCLUDE_DIR}/Viewer.hpp
    ${GRAPHICS_INCLUDE_DIR}/GraphicsScene.hpp
    ${GRAPHICS_INCLUDE_DIR}/GraphicsObject.hpp
    ${GRAPHICS_INCLUDE_DIR}/MeshGraphicsObject.hpp
    ${GRAPHICS_INCLUDE_DIR}/TetMeshGraphicsObject.hpp
    ${GRAPHICS_INCLUDE_DIR}/easy3d/Easy3DGraphicsScene.hpp
    ${GRAPHICS_INCLUDE_DIR}/easy3d/Easy3DSphereGraphicsObject.hpp
    ${GRAPHICS_INCLUDE_DIR}/easy3d/Easy3DBoxGraphicsObject.hpp
    ${GRAPHICS_INCLUDE_DIR}/easy3d/Easy3DCylinderGraphicsObject.hpp
    ${GRAPHICS_INCLUDE_DIR}/easy3d/Easy3DMeshGraphicsObject.hpp
    ${GRAPHICS_INCLUDE_DIR}/easy3d/Easy3DVirtuosoArmGraphicsObject.hpp
    ${GRAPHICS_INCLUDE_DIR}/easy3d/Easy3DVirtuosoRobotGraphicsObject.hpp
    ${GRAPHICS_INCLUDE_DIR}/easy3d/Easy3DTextRenderingViewer.hpp
    ${GRAPHICS_INCLUDE_DIR}/vtk/VTKGraphicsScene.hpp
    ${GRAPHICS_INCLUDE_DIR}/vtk/VTKViewer.hpp
    ${GRAPHICS_INCLUDE_DIR}/vtk/CustomVTKInteractorStyle.hpp
    ${GRAPHICS_INCLUDE_DIR}/vtk/VTKBoxGraphicsObject.hpp
    ${GRAPHICS_INCLUDE_DIR}/vtk/VTKCylinderGraphicsObject.hpp
    ${GRAPHICS_INCLUDE_DIR}/vtk/VTKSphereGraphicsObject.hpp
    ${GRAPHICS_INCLUDE_DIR}/vtk/VTKMeshGraphicsObject.hpp
    ${GRAPHICS_INCLUDE_DIR}/vtk/VTKTetMeshGraphicsObject.hpp
    ${GRAPHICS_INCLUDE_DIR}/vtk/VTKVirtuosoArmGraphicsObject.hpp
    ${GRAPHICS_INCLUDE_DIR}/vtk/VTKVirtuosoRobotGraphicsObject.hpp
    ${GRAPHICS_INCLUDE_DIR}/vtk/VTKVirtuosoArmSpatulaToolGraphicsObject.hpp
    ${GRAPHICS_INCLUDE_DIR}/vtk/VTKVirtuosoArmCauteryToolGraphicsObject.hpp
    ${GRAPHICS_INCLUDE_DIR}/vtk/VTKVirtuosoArmGraspingToolGraphicsObject.hpp
)

set(GRAPHICS_SOURCES
    ${GRAPHICS_SRC_DIR}/Viewer.cpp
    ${GRAPHICS_SRC_DIR}/GraphicsScene.cpp
    ${GRAPHICS_SRC_DIR}/MeshGraphicsObject.cpp
    ${GRAPHICS_SRC_DIR}/TetMeshGraphicsObject.cpp
    ${GRAPHICS_SRC_DIR}/easy3d/Easy3DGraphicsScene.cpp
    ${GRAPHICS_SRC_DIR}/easy3d/Easy3DSphereGraphicsObject.cpp
    ${GRAPHICS_SRC_DIR}/easy3d/Easy3DBoxGraphicsObject.cpp
    ${GRAPHICS_SRC_DIR}/easy3d/Easy3DCylinderGraphicsObject.cpp
    ${GRAPHICS_SRC_DIR}/easy3d/Easy3DMeshGraphicsObject.cpp
    ${GRAPHICS_SRC_DIR}/easy3d/Easy3DVirtuosoArmGraphicsObject.cpp
    ${GRAPHICS_SRC_DIR}/easy3d/Easy3DVirtuosoRobotGraphicsObject.cpp
    ${GRAPHICS_SRC_DIR}/easy3d/Easy3DTextRenderingViewer.cpp
    ${GRAPHICS_SRC_DIR}/vtk/VTKGraphicsScene.cpp
    ${GRAPHICS_SRC_DIR}/vtk/VTKViewer.cpp
    ${GRAPHICS_SRC_DIR}/vtk/CustomVTKInteractorStyle.cpp
    ${GRAPHICS_SRC_DIR}/vtk/VTKBoxGraphicsObject.cpp
    ${GRAPHICS_SRC_DIR}/vtk/VTKCylinderGraphicsObject.cpp
    ${GRAPHICS_SRC_DIR}/vtk/VTKSphereGraphicsObject.cpp
    ${GRAPHICS_SRC_DIR}/vtk/VTKMeshGraphicsObject.cpp
    ${GRAPHICS_SRC_DIR}/vtk/VTKTetMeshGraphicsObject.cpp
    ${GRAPHICS_SRC_DIR}/vtk/VTKVirtuosoArmGraphicsObject.cpp
    ${GRAPHICS_SRC_DIR}/vtk/VTKVirtuosoRobotGraphicsObject.cpp
    ${GRAPHICS_SRC_DIR}/vtk/VTKVirtuosoArmSpatulaToolGraphicsObject.cpp
    ${GRAPHICS_SRC_DIR}/vtk/VTKVirtuosoArmCauteryToolGraphicsObject.cpp
    ${GRAPHICS_SRC_DIR}/vtk/VTKVirtuosoArmGraspingToolGraphicsObject.cpp
)

target_sources(Graphics
    PRIVATE
        ${GRAPHICS_HEADERS}
        ${GRAPHICS_SOURCES} 
)

install(TARGETS Graphics EXPORT XPBDSim-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES ${GRAPHICS_HEADERS}
    DESTINATION include/${PROJECT_NAME}/graphics)