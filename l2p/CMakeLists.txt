cmake_minimum_required(VERSION 2.8)
project(l2p)

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS filesystem system REQUIRED)

include_directories(. ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

add_library(l2p STATIC
  Package.h
  Package.cpp
  )

target_link_libraries(l2p
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  )
