cmake_minimum_required(VERSION 3.13)


# === CUDA-related stuff ===

# look for CUDA on the user's machine
if(EXISTS /usr/local/cuda)
    set(HAVE_CUDA true)
else()
    set(HAVE_CUDA false)
endif()

if (HAVE_CUDA AND NOT NO_GPU)
    # if CUDA is found and the NO_GPU var is false or not set,
    # tell CMake to use the CUDA compiler
    message("\n-- WE HAVE CUDA! -- \n")
    set(CMAKE_CUDA_ARCHITECTURES "native")
    set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc")
    project(XPBDSim LANGUAGES CXX CUDA)
else ()
    # otherwise project is just vanilla C++
    project(XPBDSim LANGUAGES CXX)
endif()



# === Project Options ===

# specify C++ standard - C++17 is used mainly for template metaprogramming stuff
set(CMAKE_CXX_STANDARD 17)
# use 03 optimizer, turn on all warnings
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")
# by default, use Debug build so we can use gdb easily
set(CMAKE_BUILD_TYPE RelWithDebInfo)


# === Find External Dependencies ===

# the paths to the CMake config files for each of the external dependencies
set(yaml-cpp_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/install/yaml-cpp/lib/cmake")
set(gmsh_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/install/gmsh/share/gmsh")
set(Easy3D_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/src/Easy3D/build")
set(Eigen3_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/install/eigen/share/eigen3/cmake")
set(VTK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/install/vtk/lib/cmake")
set(embree_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/install/embree/lib/cmake")
# note: PETSc does not provide a CMake config file
set(PETSC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/src/petsc")
set(PETSC_ARCH "arch-linux-c-debug")

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
find_package(Easy3D COMPONENTS viewer fileio renderer core util REQUIRED)  # request Easy3D (recommended to request only needed components)
message(STATUS "Easy3D found at: ${Easy3D_DIR}")
message(STATUS "Easy3D include: ${Easy3D_INCLUDE_DIRS}")

# find Eigen
message(STATUS "\nFinding Eigen3...")
find_package(Eigen3 REQUIRED
    PATHS ${Eigen3_DIR}
    NO_DEFAULT_PATH)   # find Eigen
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


# === Include Directories ===
# for some reason, we need to manually include these paths
include_directories("./include")
include_directories("./dependencies/src/")
include_directories("./dependencies/install/gmsh/include")
include_directories("./dependencies/install/embree/include")
include_directories("./dependencies/src/openhaptics_3.4-0-developer-edition-amd64/usr/include")

# include PETSc directories
# (PETSc does not provide a CMake config file)
message(STATUS "including: ${PETSC_DIR}/include")
include_directories("${PETSC_DIR}/include")
include_directories("${PETSC_DIR}/${PETSC_ARCH}/include")
set(PETSC_LIB "${PETSC_DIR}/${PETSC_ARCH}/lib/libpetsc.so")

if (HAVE_CUDA AND NOT NO_GPU)
    include_directories("/usr/local/cuda/include")
endif()

# Add Mesh2SDF library install directory to the library search path
# for some reason, we need to manually link these libraries
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/dependencies/install/Mesh2SDF/lib")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/dependencies/src/openhaptics_3.4-0-developer-edition-amd64/usr/lib")
set(HD_LIB "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/src/openhaptics_3.4-0-developer-edition-amd64/usr/lib/libHD.so.3.4.0")


# === Add Child CMake Libraries ===

# Collision library
add_library(Collision)
add_subdirectory("src/collision")

# Config library
add_library(Config)
add_subdirectory("src/config")

# Common library
add_library(Common)
add_subdirectory("src/common")

# FEM library
add_library(FEM)
add_subdirectory("src/fem")

# Graphics library
add_library(Graphics)
add_subdirectory("src/graphics")

# Geometry library
add_library(Geometry)
add_subdirectory("src/geometry")

# GPU library
if (HAVE_CUDA AND NOT NO_GPU)
    add_library(GPU)
    add_subdirectory("src/gpu")
endif()

# Haptics library
add_library(Haptics)
add_subdirectory("src/haptics")

# Solver library
add_library(Solver)
add_subdirectory("src/solver")

# SimObject library
add_library(SimObject)
add_subdirectory("src/simobject")

# Utils libraries
add_library(MeshUtils)
add_library(GeometryUtils)
add_subdirectory("src/utils")



# === Link Libraries ===
target_link_libraries(Collision Eigen3::Eigen mesh2sdf Geometry GeometryUtils)
target_link_libraries(Config Eigen3::Eigen SimObject)
target_link_libraries(Graphics Eigen3::Eigen easy3d::core easy3d::util easy3d::renderer easy3d::viewer ${VTK_LIBRARIES}) # link Easy3D for Graphics library
target_link_libraries(Geometry Eigen3::Eigen mesh2sdf SimObject GeometryUtils embree)
target_link_libraries(Haptics Eigen3::Eigen ${HD_LIB} HDU)   # link OpenHaptics API libraries for Haptics library
target_link_libraries(SimObject Eigen3::Eigen MeshUtils GeometryUtils Geometry Solver Config Collision FEM)   # link MeshUtils and Solver libraries
target_link_libraries(Solver Eigen3::Eigen SimObject)         # link SimObject library
target_link_libraries(MeshUtils gmsh::shared assimp Eigen3::Eigen Geometry) # link MeshUtils with gmsh (for volume mesh generation) and assimp (for mesh loading/conversion)
target_link_libraries(GeometryUtils Eigen3::Eigen Geometry)
target_link_libraries(FEM Eigen3::Eigen Geometry ${PETSC_LIB})

target_link_options(Haptics PRIVATE
    -Wl, -rpath, ${HD_LIB}
)


# === Create Simulation Libraries ===
add_library(Simulation)
add_library(BeamSimulation)
add_library(GraspingSimulation)
add_library(VirtuosoSimulation)
add_library(PalpationSimulation)
add_library(InitialDeformationSimulation)
add_library(FixedObjectSimulation)
add_subdirectory("src/simulation")



# === Link Simulation Libraries ===
target_link_libraries(Simulation Collision Graphics SimObject Geometry SimObject Solver yaml-cpp::yaml-cpp ${VTK_LIBRARIES})
target_link_libraries(BeamSimulation Simulation)
target_link_libraries(GraspingSimulation Simulation)
target_link_libraries(VirtuosoSimulation Simulation Haptics)
target_link_libraries(PalpationSimulation VirtuosoSimulation)
target_link_libraries(InitialDeformationSimulation Config Simulation)
target_link_libraries(FixedObjectSimulation Config Simulation)



# === Create Executables ===
add_executable(Test src/exec/main.cpp)
target_link_libraries(Test Simulation ${VTK_LIBRARIES})

add_executable(BeamTest src/exec/beam_test_main.cpp)
target_link_libraries(BeamTest BeamSimulation)

add_executable(GraspingTest src/exec/grasping_main.cpp)
target_link_libraries(GraspingTest GraspingSimulation)

add_executable(VirtuosoTest src/exec/virtuoso_main.cpp)
target_link_libraries(VirtuosoTest VirtuosoSimulation)

add_executable(VirtuosoTracheaDemo src/exec/demos/virtuoso_trachea_demo.cpp)
target_link_libraries(VirtuosoTracheaDemo VirtuosoSimulation)

add_executable(PalpationDemo src/exec/demos/palpation_demo.cpp)
target_link_libraries(PalpationDemo PalpationSimulation)

add_executable(FixedObjectSim src/exec/fixed_object_main.cpp)
target_link_libraries(FixedObjectSim FixedObjectSimulation)

add_executable(InitialDeformationTest src/exec/initial_deformation_test_main.cpp)
target_link_libraries(InitialDeformationTest InitialDeformationSimulation)

add_executable(VariadicVectorContainerTest src/exec/test/variadic_vector_container_test.cpp)
target_link_libraries(VariadicVectorContainerTest Eigen3::Eigen)

add_executable(TombstoneVectorTest src/exec/test/tombstone_vector_test.cpp)
target_link_libraries(TombstoneVectorTest Eigen3::Eigen)

add_executable(EmbreeTest src/exec/test/embree_test.cpp)
target_link_libraries(EmbreeTest embree Config SimObject Simulation Solver Geometry MeshUtils)

add_executable(VirtuosoStaticsModelTest src/exec/test/virtuoso_statics_model_test.cpp)
target_link_libraries(VirtuosoStaticsModelTest Eigen3::Eigen SimObject Simulation Config Graphics ${VTK_LIBRARIES})

add_executable(VTKTest src/exec/test/vtk_test.cpp)
target_link_libraries(VTKTest Graphics ${VTK_LIBRARIES})

add_executable(StiffnessMatrixTest src/exec/test/stiffness_matrix_test.cpp)
target_link_libraries(StiffnessMatrixTest Config SimObject Solver Simulation MeshUtils)

add_executable(PETScTest src/exec/test/petsc_test.cpp)
target_link_libraries(PETScTest ${PETSC_LIB})

add_executable(LaplaceFEMTest src/exec/test/laplace_fem_test.cpp)
target_link_libraries(LaplaceFEMTest Geometry FEM MeshUtils)

add_executable(MeshRefinementTest src/exec/test/mesh_refinement_test.cpp)
target_link_libraries(MeshRefinementTest Geometry FEM MeshUtils Graphics ${VTK_LIBRARIES})

add_executable(SerializeTest src/exec/test/serialize_test.cpp)
target_link_libraries(SerializeTest Geometry FEM MeshUtils Graphics ${VTK_LIBRARIES})

add_executable(RefinedFEMTest src/exec/test/refined_fem_test.cpp)
target_link_libraries(RefinedFEMTest Geometry FEM MeshUtils Graphics ${PETSC_LIB} ${VTK_LIBRARIES})

add_executable(AssignElements src/exec/util/assign_elements.cpp)
target_link_libraries(AssignElements Config SimObject Simulation Geometry MeshUtils)

vtk_module_autoinit(
  TARGETS Test BeamTest GraspingTest VirtuosoTest VirtuosoTracheaDemo VTKTest VirtuosoStaticsModelTest InitialDeformationTest MeshRefinementTest RefinedFEMTest
  MODULES ${VTK_LIBRARIES}
)


# === Configure Libraries for CUDA, when applicable ===
if (HAVE_CUDA AND NOT NO_GPU)
    target_link_libraries(GPU Geometry SimObject Solver)
    target_link_libraries(SimObject GPU)
    target_link_libraries(Collision GPU)
    target_link_libraries(Simulation GPU)
    # target_link_libraries(Solver GPU)

    target_compile_definitions(Collision PRIVATE HAVE_CUDA)
    target_compile_definitions(Graphics PRIVATE HAVE_CUDA)
    target_compile_definitions(Geometry PRIVATE HAVE_CUDA)
    target_compile_definitions(GeometryUtils PRIVATE HAVE_CUDA)
    target_compile_definitions(GPU PRIVATE HAVE_CUDA)
    target_compile_definitions(MeshUtils PRIVATE HAVE_CUDA)
    target_compile_definitions(Haptics PRIVATE HAVE_CUDA)
    target_compile_definitions(SimObject PRIVATE HAVE_CUDA)
    target_compile_definitions(Solver PRIVATE HAVE_CUDA)
    target_compile_definitions(Simulation PRIVATE HAVE_CUDA)
    target_compile_definitions(BeamSimulation PRIVATE HAVE_CUDA)
    target_compile_definitions(VirtuosoSimulation PRIVATE HAVE_CUDA)

    target_compile_definitions(Test PRIVATE HAVE_CUDA)
    target_compile_definitions(BeamTest PRIVATE HAVE_CUDA)
    target_compile_definitions(VirtuosoTest PRIVATE HAVE_CUDA)
    target_compile_definitions(VirtuosoTracheaDemo PRIVATE HAVE_CUDA)

    add_executable(CudaTest src/exec/cuda_test.cu)
    target_link_libraries(CudaTest Eigen3::Eigen Geometry SimObject)
    target_compile_definitions(CudaTest PRIVATE HAVE_CUDA)

    add_executable(MeshSDFCudaTest src/exec/mesh_sdf_cuda_test.cu)
    target_link_libraries(MeshSDFCudaTest Eigen3::Eigen Geometry SimObject)
    target_compile_definitions(MeshSDFCudaTest PRIVATE HAVE_CUDA)

    add_executable(XPBDCudaTest src/exec/xpbd_cuda_test.cu)
    target_link_libraries(XPBDCudaTest Eigen3::Eigen Geometry SimObject)
    target_compile_definitions(XPBDCudaTest PRIVATE HAVE_CUDA)
endif()



include(CMakePackageConfigHelpers)

export(EXPORT XPBDSim-targets
    FILE ${CMAKE_CURRENT_BINARY_DIR}/XPBDSimTargets.cmake
    NAMESPACE XPBDSim::
)

install(EXPORT XPBDSim-targets
    FILE XPBDSimTargets.cmake
    NAMESPACE XPBDSim::
    DESTINATION lib/cmake/XPBDSim
)

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/XPBDSimConfig.cmake.in
"include(\"\${CMAKE_CURRENT_LIST_DIR}/XPBDSimTargets.cmake\")
"
)

# Generate the config file
configure_package_config_file(
    ${CMAKE_CURRENT_BINARY_DIR}/XPBDSimConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/XPBDSimConfig.cmake
    INSTALL_DESTINATION lib/cmake/XPBDSim
)

# Generate version file
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/XPBDSimConfigVersion.cmake
    VERSION 1.0.0
    COMPATIBILITY AnyNewerVersion
)

# Install the config files
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/XPBDSimConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/XPBDSimConfigVersion.cmake
    DESTINATION lib/cmake/XPBDSim
)