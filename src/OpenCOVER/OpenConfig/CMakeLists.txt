set(CONFIG_SOURCES config.cpp)
set(CONFIG_HEADERS array.h access.h file.h value.h config.h)

set(PREFIX covconfig/)
include(${PREFIX}covconfig.cmake)

add_covise_library(coOpenConfig ${CONFIG_SOURCES} ${CONFIG_HEADERS} ${COVCONFIG_SOURCES} ${COVCONFIG_HEADERS} ${COVCONFIG_DETAIL_HEADERS})
target_include_directories(coOpenConfig SYSTEM PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>)
target_compile_definitions(coOpenConfig PRIVATE CONFIG_NAMESPACE=opencover COVCONFIG_OPENCOVER)
target_include_directories(coOpenConfig PRIVATE ${COVCONFIG_PRIVATE_INCLUDES})
covise_install_target(coOpenConfig)
covise_install_headers(OpenConfig ${CONFIG_HEADERS})
covise_install_headers(OpenConfig/covconfig ${COVCONFIG_HEADERS})
covise_install_headers(OpenConfig/covconfig/detail ${COVCONFIG_DETAIL_HEADERS})
