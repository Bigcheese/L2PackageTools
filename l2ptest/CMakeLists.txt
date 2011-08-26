cmake_minimum_required(VERSION 2.8)
project(l2ptest)

add_executable(l2ptest
  l2ptest.cpp
  )

target_link_libraries(l2ptest
  l2p
  )
