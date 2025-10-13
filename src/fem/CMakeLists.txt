cmake_minimum_required(VERSION 3.13)
project(XPBDSim)

set(FEM_INCLUDE_DIR "../../include/fem")
set(FEM_SRC_DIR "./")

set(FEM_HEADERS
    ${FEM_INCLUDE_DIR}/FEMTetMesh.hpp
    ${FEM_INCLUDE_DIR}/LaplacianFEMSolver.hpp
    ${FEM_INCLUDE_DIR}/HeatConductionFEMSolver.hpp
)

set(FEM_SOURCES
    ${FEM_SRC_DIR}/FEMTetMesh.cpp
    ${FEM_SRC_DIR}/LaplacianFEMSolver.cpp
    ${FEM_SRC_DIR}/HeatConductionFEMSolver.cpp
)

target_sources(FEM
    PRIVATE
        ${FEM_HEADERS}
        ${FEM_SOURCES}
        
)

install(TARGETS FEM EXPORT FEM-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES ${FEM_HEADERS}
    DESTINATION include/${PROJECT_NAME}/fem)