cmake_minimum_required(VERSION 3.13)
project(XPBDSim)

set(SOLVER_INCLUDE_DIR "../../include/solver")
set(SOLVER_SRC_DIR "./")

set(SOLVER_HEADERS
    ${SOLVER_INCLUDE_DIR}/constraint/OffsetAttachmentConstraint.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/CollisionConstraint.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/Constraint.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/DeviatoricConstraint.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/ElementConstraint.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/HydrostaticConstraint.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/MidpointConstraint.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/RigidBodyConstraint.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/RigidDeformableCollisionConstraint.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/StaticDeformableCollisionConstraint.hpp
    ${SOLVER_INCLUDE_DIR}/constraint/DeformableDeformableCollisionConstraint.hpp

    ${SOLVER_INCLUDE_DIR}/xpbd_projector/CombinedConstraintProjector.hpp
    ${SOLVER_INCLUDE_DIR}/xpbd_projector/CombinedNeohookeanConstraintProjector.hpp
    ${SOLVER_INCLUDE_DIR}/xpbd_projector/ConstraintProjector.hpp
    ${SOLVER_INCLUDE_DIR}/xpbd_projector/RigidBodyConstraintProjector.hpp

    ${SOLVER_INCLUDE_DIR}/xpbd_solver/XPBDGaussSeidelSolver.hpp
    ${SOLVER_INCLUDE_DIR}/xpbd_solver/XPBDJacobiSolver.hpp
    ${SOLVER_INCLUDE_DIR}/xpbd_solver/XPBDParallelJacobiSolver.hpp
    ${SOLVER_INCLUDE_DIR}/xpbd_solver/XPBDSolver.hpp
    ${SOLVER_INCLUDE_DIR}/xpbd_solver/XPBDSolverUpdates.hpp
)

set(SOLVER_SOURCES
    ${SOLVER_SRC_DIR}/constraint/DeviatoricConstraint.cpp
    ${SOLVER_SRC_DIR}/constraint/HydrostaticConstraint.cpp
    ${SOLVER_SRC_DIR}/constraint/MidpointConstraint.cpp
    ${SOLVER_SRC_DIR}/constraint/RigidBodyConstraint.cpp
    ${SOLVER_SRC_DIR}/constraint/StaticDeformableCollisionConstraint.cpp
    ${SOLVER_SRC_DIR}/constraint/RigidDeformableCollisionConstraint.cpp
    ${SOLVER_SRC_DIR}/constraint/DeformableDeformableCollisionConstraint.cpp
    ${SOLVER_SRC_DIR}/constraint/OffsetAttachmentConstraint.cpp
    ${SOLVER_SRC_DIR}/constraint/AttachmentConstraint.cpp

    ${SOLVER_SRC_DIR}/xpbd_projector/CombinedNeohookeanConstraintProjector.cpp
)

target_sources(Solver
    PRIVATE
        ${SOLVER_HEADERS}
        ${SOLVER_SOURCES}
)

install(TARGETS Solver EXPORT XPBDSim-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES ${SOLVER_HEADERS}
    DESTINATION include/${PROJECT_NAME}/solver)
