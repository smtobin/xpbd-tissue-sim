cmake_minimum_required(VERSION 3.8)
project(sim_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include_directories("/workspace/include")
include_directories("${XPBD_SIM_BASE_DIR}/include")

list(APPEND CMAKE_PREFIX_PATH "${XPBD_SIM_EASY3D_CMAKE_PREFIX_PATH}") # needed in order to find Easy3D

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2)
find_package(Eigen3 3.3 REQUIRED)   # find Eigen
find_package(Easy3D COMPONENTS viewer fileio renderer core util REQUIRED)  # request Easy3D (recommended to request only needed components)
find_package(assimp REQUIRED)   # find assimp (for mesh loading/conversion)
find_package(embree 4 REQUIRED)
find_package(VTK 9.5 COMPONENTS
    CommonColor
    CommonCore
    CommonDataModel
    CommonExecutionModel
    FiltersSources
    IOImage
    InteractionStyle
    RenderingContextOpenGL2
    RenderingCore
    RenderingFreeType
    RenderingGL2PSOpenGL2
    RenderingOpenGL2
    RenderingAnnotation
    InteractionWidgets
)
if (NOT VTK_FOUND)
  message(FATAL_ERROR "Unable to find the VTK build folder.")
endif()

add_executable(${PROJECT_NAME}_node src/sim_bridge_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs geometry_msgs shape_msgs sensor_msgs tf2_ros tf2)
target_link_libraries(${PROJECT_NAME}_node
  Simulation
  VirtuosoSimulation
  GraspingSimulation
  PalpationSimulation
  FixedCubeSimulation
  Collision
  Config
  Geometry
  SimObject
  Solver
  Graphics
  Haptics
  Geometry
  GeometryUtils
  MeshUtils
  assimp
  gmsh
  Eigen3::Eigen
  yaml-cpp
  easy3d::core
  easy3d::util
  easy3d::renderer
  easy3d::viewer
  embree
  HD
  HDU
  mesh2sdf
  ${VTK_LIBRARIES}
)

vtk_module_autoinit(
  TARGETS ${PROJECT_NAME}_node
  MODULES ${VTK_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
