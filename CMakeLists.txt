cmake_minimum_required(VERSION 3.13)

if(EXISTS /usr/local/cuda)
    set(HAVE_CUDA true)
else()
    set(HAVE_CUDA false)
endif()

if (HAVE_CUDA AND NOT NO_GPU)
    message("\n-- WE HAVE CUDA! -- \n")
    set(CMAKE_CUDA_ARCHITECTURES "native")
    set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc")
    project(XPBDSim LANGUAGES CXX CUDA)
else ()
    project(XPBDSim LANGUAGES CXX)
endif()


set(CMAKE_CXX_STANDARD 17)                       # specify C++ standard
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")
set(CMAKE_BUILD_TYPE Debug)

list(APPEND CMAKE_PREFIX_PATH "$ENV{XPBD_SIM_EASY3D_CMAKE_PREFIX_PATH}") # needed in order to find Easy3D

find_package(Easy3D COMPONENTS viewer fileio renderer core util REQUIRED)  # request Easy3D (recommended to request only needed components)
find_package(assimp REQUIRED)   # find assimp (for mesh loading/conversion)
find_package(Eigen3 3.3 REQUIRED)   # find Eigen
find_package(embree 4 REQUIRED)

# find VTK
find_package(VTK 9.5 REQUIRED COMPONENTS
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

if (NOT VTK_FOUND)
  message(FATAL_ERROR "Unable to find the VTK build folder.")
endif()
message(STATUS "VTK_VERSION: ${VTK_VERSION}")
message(STATUS "VTK_LIBRARIES: ${VTK_LIBRARIES}")
message(STATUS "VTK_INCLUDE_DIRS: ${VTK_INCLUDE_DIRS}")


include_directories("./include")

# include PETSc directories
message(STATUS "including: $ENV{PETSC_DIR}/include")
include_directories("$ENV{PETSC_DIR}/include")
include_directories("$ENV{PETSC_DIR}/$ENV{PETSC_ARCH}/include")
set(PETSC_LIB "$ENV{PETSC_DIR}/$ENV{PETSC_ARCH}/lib/libpetsc.so")

if (HAVE_CUDA AND NOT NO_GPU)
    include_directories("/usr/local/cuda/include")
endif()

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

# link in necessary libraries
target_link_libraries(Collision Eigen3::Eigen mesh2sdf Geometry GeometryUtils)
target_link_libraries(Config Eigen3::Eigen SimObject)
target_link_libraries(Graphics Eigen3::Eigen easy3d::core easy3d::util easy3d::renderer easy3d::viewer ${VTK_LIBRARIES}) # link Easy3D for Graphics library
target_link_libraries(Geometry Eigen3::Eigen mesh2sdf SimObject GeometryUtils embree)
target_link_libraries(Haptics Eigen3::Eigen HD HDU)   # link OpenHaptics API libraries for Haptics library
target_link_libraries(SimObject Eigen3::Eigen MeshUtils GeometryUtils Geometry Solver Config FEM)   # link MeshUtils and Solver libraries
target_link_libraries(Solver Eigen3::Eigen SimObject)         # link SimObject library
target_link_libraries(MeshUtils gmsh assimp Eigen3::Eigen Geometry) # link MeshUtils with gmsh (for volume mesh generation) and assimp (for mesh loading/conversion)
target_link_libraries(GeometryUtils Eigen3::Eigen Geometry)
target_link_libraries(FEM Eigen3::Eigen Geometry ${PETSC_LIB})

### Simulation libraries - each has its own executable
add_library(Simulation)
add_library(BeamSimulation)
add_library(GraspingSimulation)
add_library(VirtuosoSimulation)
add_library(PalpationSimulation)
# add_library(BeamStretchSimulation)
# add_library(TissueGraspingSimulation)
add_library(InitialDeformationSimulation)
add_library(FixedObjectSimulation)
add_subdirectory("src/simulation")

target_link_libraries(Simulation Collision Graphics SimObject Geometry SimObject Solver yaml-cpp ${VTK_LIBRARIES})
target_link_libraries(BeamSimulation Simulation)
target_link_libraries(GraspingSimulation Simulation)
target_link_libraries(VirtuosoSimulation Simulation Haptics)
target_link_libraries(PalpationSimulation VirtuosoSimulation)
# target_link_libraries(BeamStretchSimulation Simulation)
target_link_libraries(InitialDeformationSimulation Config Simulation)
target_link_libraries(FixedObjectSimulation Config Simulation)

### Create executables
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

# add_executable(TissueGraspingTest src/exec/tissue_grasping_test_main.cpp)
# target_link_libraries(TissueGraspingTest TissueGraspingSimulation)

# add_executable(BeamStretchTest src/exec/beam_stretch_test_main.cpp)
# target_link_libraries(BeamStretchTest BeamStretchSimulation)

add_executable(InitialDeformationTest src/exec/initial_deformation_test_main.cpp)
target_link_libraries(InitialDeformationTest InitialDeformationSimulation)

# add_executable(StaticLoadingTest src/exec/static_loading_test_main.cpp)
# target_link_libraries(StaticLoadingTest XPBD easy3d::viewer gmsh)

add_executable(VariadicVectorContainerTest src/exec/test/variadic_vector_container_test.cpp)
target_link_libraries(VariadicVectorContainerTest Eigen3::Eigen)

add_executable(TombstoneVectorTest src/exec/test/tombstone_vector_test.cpp)
target_link_libraries(TombstoneVectorTest Eigen3::Eigen)

add_executable(EmbreeTest src/exec/test/embree_test.cpp)
target_link_libraries(EmbreeTest embree Geometry MeshUtils)

add_executable(VirtuosoStaticsModelTest src/exec/test/virtuoso_statics_model_test.cpp)
target_link_libraries(VirtuosoStaticsModelTest Eigen3::Eigen SimObject Config Graphics ${VTK_LIBRARIES})

add_executable(VTKTest src/exec/test/vtk_test.cpp)
target_link_libraries(VTKTest Graphics ${VTK_LIBRARIES})

add_executable(StiffnessMatrixTest src/exec/test/stiffness_matrix_test.cpp)
target_link_libraries(StiffnessMatrixTest Config SimObject Solver Simulation MeshUtils)

add_executable(PETScTest src/exec/test/petsc_test.cpp)
target_link_libraries(PETScTest ${PETSC_LIB})

add_executable(LaplaceFEMTest src/exec/test/laplace_fem_test.cpp)
target_link_libraries(LaplaceFEMTest Geometry FEM MeshUtils)

add_executable(AssignElements src/exec/util/assign_elements.cpp)
target_link_libraries(AssignElements Config SimObject Geometry MeshUtils)

vtk_module_autoinit(
  TARGETS Test BeamTest GraspingTest VirtuosoTest VirtuosoTracheaDemo VTKTest VirtuosoStaticsModelTest InitialDeformationTest
  MODULES ${VTK_LIBRARIES}
)

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

    # target_compile_options(GPU PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:
    #                    -G
    #                    -g
    #                    >)

    add_executable(CudaTest src/exec/cuda_test.cu)
    target_link_libraries(CudaTest Eigen3::Eigen Geometry SimObject)
    target_compile_definitions(CudaTest PRIVATE HAVE_CUDA)

    add_executable(MeshSDFCudaTest src/exec/mesh_sdf_cuda_test.cu)
    target_link_libraries(MeshSDFCudaTest Eigen3::Eigen Geometry SimObject)
    target_compile_definitions(MeshSDFCudaTest PRIVATE HAVE_CUDA)

    add_executable(XPBDCudaTest src/exec/xpbd_cuda_test.cu)
    target_link_libraries(XPBDCudaTest Eigen3::Eigen Geometry SimObject)
    target_compile_definitions(XPBDCudaTest PRIVATE HAVE_CUDA)
    # target_compile_options(XPBDCudaTest PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--keep>)

    # add_executable(XPBDConstraintCudaTest src/exec/constraints_cuda_test.cu)
    # target_link_libraries(XPBDConstraintCudaTest Eigen3::Eigen Geometry SimObject GPU Simulation)
    # target_compile_definitions(XPBDConstraintCudaTest PRIVATE HAVE_CUDA)
endif()


