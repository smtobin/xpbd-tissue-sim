cmake_minimum_required(VERSION 3.13)
project(XPBDSim)

set(COLLISION_INCLUDE_DIR "../../include/collision")
set(COLLISION_SRC_DIR "./")

set(COLLISION_HEADERS
    ${COLLISION_INCLUDE_DIR}/CollisionScene.hpp
)

set(COLLISION_SOURCES
    ${COLLISION_SRC_DIR}/CollisionScene.cpp
)

target_sources(Collision
    PRIVATE
        ${COLLISION_HEADERS}
        ${COLLISION_SOURCES}
        
)

install(TARGETS Collision EXPORT XPBDSim-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES ${COLLISION_HEADERS}
    DESTINATION include/${PROJECT_NAME}/collision)