cmake_minimum_required(VERSION 3.0)
PROJECT(indi_astrofocus CXX C)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

include(GNUInstallDirs)

set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)

find_package(INDI REQUIRED)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${INDI_INCLUDE_DIR})

include(CMakeCommon)

########### AstroFocus ###########
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-error")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error")

SET(astrofocus_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/astrofocus_focuser.cpp)

add_executable(indi_astrofocus_focus ${astrofocus_SRC})
target_link_libraries(indi_astrofocus_focus indidriver)
install(TARGETS indi_astrofocus_focus RUNTIME DESTINATION bin)
