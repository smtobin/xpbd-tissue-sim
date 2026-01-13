cmake_minimum_required(VERSION 3.13)
project(XPBDSim)

set_target_properties(Common PROPERTIES LINKER_LANGUAGE CXX)

set(COMMON_INCLUDE_DIR "../../include/common")
set(COMMON_SRC_DIR "./")

set(COMMON_HEADERS
    ${COMMON_INCLUDE_DIR}/colors.hpp
    ${COMMON_INCLUDE_DIR}/MonitoredVector.hpp
    ${COMMON_INCLUDE_DIR}/TypeList.hpp
    ${COMMON_INCLUDE_DIR}/types.hpp
    ${COMMON_INCLUDE_DIR}/VariadicContainer.hpp
    ${COMMON_INCLUDE_DIR}/VariadicVectorContainer.hpp
    ${COMMON_INCLUDE_DIR}/XPBDEnumTypes.hpp
    ${COMMON_INCLUDE_DIR}/XPBDTypedefs.hpp
    ${COMMON_INCLUDE_DIR}/SimulationTypeDefs.hpp
)

set(COMMON_SOURCES
)

target_sources(Common
    PRIVATE
        ${COMMON_HEADERS}
        ${COMMON_SOURCES}       
)

install(TARGETS Common EXPORT XPBDSim-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES ${COMMON_HEADERS}
    DESTINATION include/${PROJECT_NAME}/common)