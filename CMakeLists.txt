#===- CMakeLists.txt --------------------------------------------------------===
#
# L2PackageTools
#
# This file is distributed under the Simplified BSD License. See LICENSE.TXT
# for details.
#
#===-------------------------------------------------------------------------===
#
# This is the make CMake file for L2PackageTools
#
#===-------------------------------------------------------------------------===

cmake_minimum_required(VERSION 2.8)

project(L2PackageTools)

set(OGRE_HOME "" CACHE PATH "Blah")

set(CMAKE_MODULE_PATH "${OGRE_HOME}/CMake/;${CMAKE_MODULE_PATH}")

if (CMAKE_BUILD_TYPE STREQUAL "")
  # CMake defaults to leaving CMAKE_BUILD_TYPE empty. This screws up
  # differentiation between debug and release builds.
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build, options are: None (CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif ()

set(CMAKE_DEBUG_POSTFIX "_d")

set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/dist")

find_package(OGRE REQUIRED)

find_package(OIS REQUIRED)

if(NOT OIS_FOUND)
  message(SEND_ERROR "Failed to find OIS.")
endif()

set(Boost_USE_STATIC_LIBS TRUE)
set(Boost_ADDITIONAL_VERSIONS "1.50" "1.50.0" "1.44" "1.44.0" "1.42" "1.42.0" "1.41.0" "1.41" "1.40.0" "1.40" "1.39.0" "1.39" "1.38.0" "1.38" "1.37.0" "1.37" )
# Components that need linking (NB does not include header-only components like bind)
set(OGRE_BOOST_COMPONENTS chrono thread date_time program_options)
find_package(Boost COMPONENTS ${OGRE_BOOST_COMPONENTS} QUIET)
# Set up referencing of Boost
include_directories(${Boost_INCLUDE_DIR})
# add_definitions(-DBOOST_ALL_NO_LIB)
set(OGRE_LIBRARIES ${OGRE_LIBRARIES} ${Boost_LIBRARIES})

set(HDRS
  ./BaseApplication.h
  ./TutorialApplication.h
  ./DynamicLines.h
)

set(SRCS
  ./BaseApplication.cpp
  ./TutorialApplication.cpp
  ./DynamicLines.cpp
)

# add_definitions(-D_HAS_ITERATOR_DEBUGGING=0)

include_directories(.)
add_subdirectory(l2p)
add_subdirectory(l2ptest)

include_directories( ${OIS_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE_SAMPLES_INCLUDEPATH}
)

add_executable(L2MapViewer WIN32 ${HDRS} ${SRCS})

set_target_properties(L2MapViewer PROPERTIES DEBUG_POSTFIX _d)

target_link_libraries(L2MapViewer ${OGRE_LIBRARIES} ${OIS_LIBRARIES} l2p)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/dist/bin)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/dist/media)

# post-build copy for win32
if(WIN32 AND NOT MINGW)
  add_custom_command( TARGET L2MapViewer PRE_BUILD
    COMMAND if not exist .\\dist\\bin mkdir .\\dist\\bin )
  add_custom_command( TARGET L2MapViewer POST_BUILD
    COMMAND copy \"$(TargetPath)\" .\\dist\\bin )
endif(WIN32 AND NOT MINGW)

if(MINGW OR UNIX)
  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/dist/bin)
endif(MINGW OR UNIX)

if(WIN32)

  install(TARGETS L2MapViewer
    RUNTIME DESTINATION bin
    CONFIGURATIONS All)

  install(DIRECTORY ${CMAKE_SOURCE_DIR}/dist/Media
    DESTINATION ./
    CONFIGURATIONS Release RelWithDebInfo Debug
  )

  install(FILES ${CMAKE_SOURCE_DIR}/dist/bin/plugins.cfg
    ${CMAKE_SOURCE_DIR}/dist/bin/resources.cfg
    DESTINATION bin
    CONFIGURATIONS Release RelWithDebInfo
  )

  install(FILES ${CMAKE_SOURCE_DIR}/dist/bin/plugins_d.cfg
    ${CMAKE_SOURCE_DIR}/dist/bin/resources_d.cfg
    DESTINATION bin
    CONFIGURATIONS Debug
  )

        # NOTE: for the 1.7.1 sdk the OIS dll is called OIS.dll instead of libOIS.dll
        # so you'll have to change that to make it work with 1.7.1
  install(FILES ${OGRE_PLUGIN_DIR_REL}/OgreMain.dll
    ${OGRE_PLUGIN_DIR_REL}/RenderSystem_Direct3D9.dll
    ${OGRE_PLUGIN_DIR_REL}/RenderSystem_GL.dll
    ${OGRE_PLUGIN_DIR_REL}/Plugin_CgProgramManager.dll
    ${OGRE_PLUGIN_DIR_REL}/OIS.dll
    ${OGRE_PLUGIN_DIR_REL}/cg.dll
    DESTINATION bin
    CONFIGURATIONS Release RelWithDebInfo
  )

  install(FILES ${OGRE_PLUGIN_DIR_DBG}/OgreMain_d.dll
    ${OGRE_PLUGIN_DIR_DBG}/RenderSystem_Direct3D9_d.dll
    ${OGRE_PLUGIN_DIR_DBG}/RenderSystem_GL_d.dll
    ${OGRE_PLUGIN_DIR_DBG}/Plugin_CgProgramManager_d.dll
    ${OGRE_PLUGIN_DIR_DBG}/OIS_d.dll
    ${OGRE_PLUGIN_DIR_DBG}/cg.dll
    DESTINATION bin
    CONFIGURATIONS Debug
  )
endif(WIN32)

if(UNIX)

  install(TARGETS L2MapViewer
    RUNTIME DESTINATION bin
    CONFIGURATIONS All)

  install(DIRECTORY ${CMAKE_SOURCE_DIR}/dist/media
    DESTINATION ./
    CONFIGURATIONS Release RelWithDebInfo Debug
  )

  install(FILES ${CMAKE_SOURCE_DIR}/dist/bin/plugins.cfg
    ${CMAKE_SOURCE_DIR}/dist/bin/resources.cfg
    DESTINATION bin
    CONFIGURATIONS Release RelWithDebInfo Debug
  )

endif(UNIX)

