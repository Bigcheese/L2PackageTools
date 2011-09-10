#===- l2ptest/CMakeLists.txt ------------------------------------------------===
#
# L2PackageTools
#
# This file is distributed under the Simplified BSD License. See LICENSE.TXT
# for details.
#
#===-------------------------------------------------------------------------===

cmake_minimum_required(VERSION 2.8)
project(l2ptest)

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS program_options REQUIRED)

include_directories(. ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

add_executable(l2ptest
  l2ptest.cpp
  )

target_link_libraries(l2ptest
  l2p
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  )
