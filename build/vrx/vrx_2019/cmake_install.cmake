# Install script for directory: /home/brad/vrx_amore/src/vrx/vrx_2019

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/brad/vrx_amore/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/brad/vrx_amore/build/vrx/vrx_2019/catkin_generated/installspace/vrx_2019.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vrx_2019/cmake" TYPE FILE FILES
    "/home/brad/vrx_amore/build/vrx/vrx_2019/catkin_generated/installspace/vrx_2019Config.cmake"
    "/home/brad/vrx_amore/build/vrx/vrx_2019/catkin_generated/installspace/vrx_2019Config-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vrx_2019" TYPE FILE FILES "/home/brad/vrx_amore/src/vrx/vrx_2019/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vrx_2019/worlds" TYPE FILE FILES
    "/home/brad/vrx_amore/build/vrx/vrx_2019/worlds/dock.world"
    "/home/brad/vrx_amore/build/vrx/vrx_2019/worlds/example_course.world"
    "/home/brad/vrx_amore/build/vrx/vrx_2019/worlds/navigation_task.world"
    "/home/brad/vrx_amore/build/vrx/vrx_2019/worlds/ocean.world"
    "/home/brad/vrx_amore/build/vrx/vrx_2019/worlds/perception_task.world"
    "/home/brad/vrx_amore/build/vrx/vrx_2019/worlds/sandisland.world"
    "/home/brad/vrx_amore/build/vrx/vrx_2019/worlds/scan_and_dock.world"
    "/home/brad/vrx_amore/build/vrx/vrx_2019/worlds/stationkeeping_task.world"
    "/home/brad/vrx_amore/build/vrx/vrx_2019/worlds/wayfinding_task.world"
    "/home/brad/vrx_amore/build/vrx/vrx_2019/worlds/wind_test.world"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vrx_2019/config" TYPE DIRECTORY FILES "/home/brad/vrx_amore/src/vrx/vrx_2019/config/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vrx_2019/worlds" TYPE DIRECTORY FILES "/home/brad/vrx_amore/src/vrx/vrx_2019/worlds/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vrx_2019/launch" TYPE DIRECTORY FILES "/home/brad/vrx_amore/src/vrx/vrx_2019/launch/")
endif()

