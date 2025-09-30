cmake_minimum_required(VERSION 3.13)
project(XPBDSim)

set(SIMULATION_INCLUDE_DIR "../../include/simulation")
set(SIMULATION_SRC_DIR "./")


######################################
# Residual Simulation
######################################

set(SIMULATION_HEADERS
    "${SIMULATION_INCLUDE_DIR}/Simulation.hpp"
    "${SIMULATION_INCLUDE_DIR}/SimulationLogger.hpp"
)
set(SIMULATION_SOURCES
    "${SIMULATION_SRC_DIR}/Simulation.cpp"
    "${SIMULATION_SRC_DIR}/SimulationLogger.cpp"
)

target_sources(Simulation
    PRIVATE
        ${SIMULATION_HEADERS}
        ${SIMULATION_SOURCES}
        
)
install(TARGETS Simulation EXPORT Simulation-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES ${SIMULATION_HEADERS}
    DESTINATION include/${PROJECT_NAME}/simulation)



######################################
# Beam Simulation
######################################

target_sources(BeamSimulation
    PRIVATE
        ${SIMULATION_INCLUDE_DIR}/BeamSimulation.hpp
        ${SIMULATION_SRC_DIR}/BeamSimulation.cpp
)

######################################
# Grasping Simulation
######################################

set(GRASPING_SIMULATION_HEADERS
    "${SIMULATION_INCLUDE_DIR}/GraspingSimulation.hpp"
)

set(GRASPING_SIMULATION_SOURCES
    "${SIMULATION_SRC_DIR}/GraspingSimulation.cpp"
)

target_sources(GraspingSimulation
    PRIVATE
        ${GRASPING_SIMULATION_HEADERS}
        ${GRASPING_SIMULATION_SOURCES}
)

install(TARGETS GraspingSimulation EXPORT GraspingSimulation-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
)
install(FILES ${GRASPING_SIMULATION_HEADERS}
    DESTINATION include/${PROJECT_NAME}/simulation)


######################################
# Virtuoso Simulation
######################################
set(VIRTUOSO_SIMULATION_HEADERS
    "${SIMULATION_INCLUDE_DIR}/VirtuosoSimulation.hpp"
    "${SIMULATION_INCLUDE_DIR}/VirtuosoTissueGraspingSimulation.hpp"
)

set(VIRTUOSO_SIMULATION_SOURCES
    "${SIMULATION_SRC_DIR}/VirtuosoSimulation.cpp"
    "${SIMULATION_SRC_DIR}/VirtuosoTissueGraspingSimulation.cpp"
)

target_sources(VirtuosoSimulation
    PRIVATE
        ${VIRTUOSO_SIMULATION_HEADERS}
        ${VIRTUOSO_SIMULATION_SOURCES}
)
install(TARGETS VirtuosoSimulation EXPORT VirtuosoSimulation-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
)
install(FILES ${VIRTUOSO_SIMULATION_HEADERS}
    DESTINATION include/${PROJECT_NAME}/simulation)



######################################
# Palpation Simulation
######################################

set(PALPATION_SIMULATION_HEADERS
    "${SIMULATION_INCLUDE_DIR}/PalpationSimulation.hpp"
)

set(PALPATION_SIMULATION_SOURCES
    "${SIMULATION_SRC_DIR}/PalpationSimulation.cpp"
)

target_sources(PalpationSimulation
    PRIVATE
        ${PALPATION_SIMULATION_HEADERS}
        ${PALPATION_SIMULATION_SOURCES}
)

install(TARGETS PalpationSimulation EXPORT PalpationSimulation-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
)
install(FILES ${PALPATION_SIMULATION_HEADERS}
    DESTINATION include/${PROJECT_NAME}/simulation)



####################################
# Initial deformation simulation
#####################################
set (INITIAL_DEFORMATION_SIMULATION_HEADERS
    "${SIMULATION_INCLUDE_DIR}/InitialDeformationSimulation.hpp"
)
set (INITIAL_DEFORMATION_SIMULATION_SOURCES
    "${SIMULATION_SRC_DIR}/InitialDeformationSimulation.cpp"
)

target_sources(InitialDeformationSimulation
    PRIVATE
        ${INITIAL_DEFORMATION_SIMULATION_HEADERS}
        ${INITIAL_DEFORMATION_SIMULATION_SOURCES}
)
install(TARGETS InitialDeformationSimulation EXPORT InitialDeformationSimulation-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
)
install(FILES ${INITIAL_DEFORMATION_SIMULATION_HEADERS}
    DESTINATION include/${PROJECT_NAME}/simulation)


###################################
# Fixed cube simulation
###################################
set (FIXED_CUBE_SIMULATION_HEADERS
    "${SIMULATION_INCLUDE_DIR}/FixedCubeSimulation.hpp"
)

set (FIXED_CUBE_SIMULATION_SOURCES
    "${SIMULATION_SRC_DIR}/FixedCubeSimulation.cpp"
)

target_sources(FixedCubeSimulation
    PRIVATE
        ${FIXED_CUBE_SIMULATION_HEADERS}
        ${FIXED_CUBE_SIMULATION_SOURCES}    
)

install(TARGETS FixedCubeSimulation EXPORT FixedCubeSimulation-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION bin/${PROJECT_NAME}
)
install(FILES ${FIXED_CUBE_SIMULATION_HEADERS}
    DESTINATION include/${PROJECT_NAME}/simulation)

# target_sources(BeamStretchSimulation
#     PRIVATE
#         ${SIMULATION_INCLUDE_DIR}/BeamStretchSimulation.hpp
#         ${SIMULATION_SRC_DIR}/BeamStretchSimulation.cpp
# )



# target_sources(TissueGraspingSimulation
#     PRIVATE
#         ${SIMULATION_INCLUDE_DIR}/TissueGraspingSimulation.hpp
#         ${SIMULATION_SRC_DIR}/TissueGraspingSimulation.cpp
# )