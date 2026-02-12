cmake_minimum_required(VERSION 3.8)
project(sim_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# specify C++ standard - C++17 is used mainly for template metaprogramming stuff
set(CMAKE_CXX_STANDARD 17)
# use 03 optimizer, turn on all warnings
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")
# by default, use Debug build so we can use gdb easily
set(CMAKE_BUILD_TYPE RelWithDebInfo)

# Set RPATH to find local dependencies
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(SIM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../..")

# Explicitly add your dependency library paths
set(CMAKE_INSTALL_RPATH 
    "${SIM_DIR}/build"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../dependencies/install/gmsh/lib"
    "${SIM_DIR}/dependencies/src/openhaptics_3.4-0-developer-edition-amd64/usr/lib")

set(CMAKE_BUILD_RPATH 
    "${SIM_DIR}/build"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../dependencies/install/gmsh/lib"
    "${SIM_DIR}/dependencies/src/openhaptics_3.4-0-developer-edition-amd64/usr/lib")

# === Find ROS Dependencies ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2)

# Build custom message types
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SparseMatrix.msg"
  "msg/RemovedElement.msg"
  "msg/RemovedElementArray.msg"
  DEPENDENCIES std_msgs
  DEPENDENCIES geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)


# === Find External Dependencies ===

# the paths to the CMake config files for each of the external dependencies
set(yaml-cpp_DIR "${SIM_DIR}/dependencies/install/yaml-cpp/lib/cmake")
set(gmsh_DIR "${SIM_DIR}/dependencies/install/gmsh/share/gmsh")
set(Easy3D_DIR "${SIM_DIR}/dependencies/src/Easy3D/build")
set(Eigen3_DIR "${SIM_DIR}/dependencies/install/eigen/share/eigen3/cmake")
set(VTK_DIR "${SIM_DIR}/dependencies/install/vtk/lib/cmake")
set(embree_DIR "${SIM_DIR}/dependencies/install/embree/lib/cmake")

# note: PETSc does not provide a CMake config file
set(PETSC_DIR "${SIM_DIR}/dependencies/src/petsc")
set(PETSC_ARCH "arch-linux-c-debug")

set(XPBDSim_DIR "${SIM_DIR}/build")

find_package(XPBDSim REQUIRED)

# find yaml-cpp
message(STATUS "\nFinding yaml-cpp...")
find_package(yaml-cpp REQUIRED
    PATHS ${yaml-cpp_DIR}
    NO_DEFAULT_PATH)
message(STATUS "yaml-cpp found at: ${yaml-cpp_DIR}")


# find gmsh
message(STATUS "\nFinding gmsh...")
find_package(gmsh REQUIRED
    PATHS ${gmsh_DIR}
    NO_DEFAULT_PATH)
message(STATUS "gmsh found at: ${gmsh_DIR}")

# find Easy3D
message(STATUS "\nFinding Easy3D...")
find_package(Easy3D COMPONENTS viewer fileio renderer core util REQUIRED)
message(STATUS "Easy3D found at: ${Easy3D_DIR}")
message(STATUS "Easy3D include: ${Easy3D_INCLUDE_DIRS}")

# find Eigen
message(STATUS "\nFinding Eigen3...")
find_package(Eigen3 REQUIRED
    PATHS ${Eigen3_DIR}
    NO_DEFAULT_PATH)
message(STATUS "Eigen3 found at: ${Eigen3_DIR}")

# find embree
message(STATUS "\nFinding embree...")
find_package(embree 4 REQUIRED
    PATHS ${embree_DIR}
    NO_DEFAULT_PATH)
message(STATUS "embree found at: ${embree_DIR}")

# find VTK
message(STATUS "\nFinding VTK...")
find_package(VTK 9.5 REQUIRED
    PATHS ${VTK_DIR}
    NO_DEFAULT_PATH
    COMPONENTS
    CommonColor
    CommonCore
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
message(STATUS "VTK_VERSION: ${VTK_VERSION}")
message(STATUS "VTK found at: ${VTK_DIR}")

if (NOT VTK_FOUND)
  message(FATAL_ERROR "Unable to find the VTK build folder.")
endif()

# find assimp
find_package(assimp REQUIRED)

# include PETSc directories
# (PETSc does not provide a CMake config file)
message(STATUS "including: ${PETSC_DIR}/include")
include_directories("${PETSC_DIR}/include")
include_directories("${PETSC_DIR}/${PETSC_ARCH}/include")
set(PETSC_LIB "${PETSC_DIR}/${PETSC_ARCH}/lib/libpetsc.so")

# Add Mesh2SDF library install directory to the library search path
link_directories("${SIM_DIR}/dependencies/install/Mesh2SDF/lib")

# Add OpenHaptics library directory
link_directories("${SIM_DIR}/dependencies/src/openhaptics_3.4-0-developer-edition-amd64/usr/lib")

# === Create Node Executable ===
add_executable(${PROJECT_NAME}_node 
  src/sim_bridge_node.cpp 
  src/VirtuosoSimBridge.cpp 
  src/VirtuosoCTAnatomySimBridge.cpp
  src/CAOSimBridge.cpp
)

# === Link Dependencies ===
include_directories("${SIM_DIR}/include")
# Add Mesh2SDF include directory
include_directories("${SIM_DIR}/dependencies/src/")
include_directories("${SIM_DIR}/dependencies/install/Mesh2SDF/include")
# Add OpenHaptics include directory
include_directories("${SIM_DIR}/dependencies/src/openhaptics_3.4-0-developer-edition-amd64/usr/include")

# ROS dependencies
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs geometry_msgs shape_msgs sensor_msgs tf2_ros tf2)
# sim dependencies
target_link_libraries(${PROJECT_NAME}_node
  XPBDSim::Simulation
  XPBDSim::VirtuosoSimulation
  XPBDSim::GraspingSimulation
  XPBDSim::PalpationSimulation
  XPBDSim::FixedObjectSimulation
  XPBDSim::Collision
  XPBDSim::Config
  XPBDSim::Geometry
  XPBDSim::SimObject
  XPBDSim::Solver
  XPBDSim::Graphics
  XPBDSim::Haptics
  XPBDSim::Geometry
  XPBDSim::GeometryUtils
  XPBDSim::MeshUtils
  assimp
  gmsh::shared
  Eigen3::Eigen
  yaml-cpp::yaml-cpp
  easy3d::core
  easy3d::util
  easy3d::renderer
  easy3d::viewer
  embree
  HD
  HDU
  mesh2sdf
  ${VTK_LIBRARIES}
  ${PETSC_LIB}
)

target_link_libraries(${PROJECT_NAME}_node "${cpp_typesupport_target}")

vtk_module_autoinit(
  TARGETS ${PROJECT_NAME}_node
  MODULES ${VTK_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_17)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()