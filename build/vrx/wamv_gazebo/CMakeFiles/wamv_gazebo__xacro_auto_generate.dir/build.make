# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/shaede/vrx_amore/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/shaede/vrx_amore/build

# Utility rule file for wamv_gazebo__xacro_auto_generate.

# Include the progress variables for this target.
include vrx/wamv_gazebo/CMakeFiles/wamv_gazebo__xacro_auto_generate.dir/progress.make

vrx/wamv_gazebo/CMakeFiles/wamv_gazebo__xacro_auto_generate: vrx/wamv_gazebo/urdf/wamv_gazebo.urdf


vrx/wamv_gazebo/urdf/wamv_gazebo.urdf: /home/shaede/vrx_amore/src/vrx/wamv_gazebo/urdf/wamv_gazebo.urdf.xacro
vrx/wamv_gazebo/urdf/wamv_gazebo.urdf: /home/shaede/vrx_amore/src/vrx/wamv_gazebo/urdf/components/wamv_p3d.xacro
vrx/wamv_gazebo/urdf/wamv_gazebo.urdf: /home/shaede/vrx_amore/src/vrx/wamv_description/urdf/battery.xacro
vrx/wamv_gazebo/urdf/wamv_gazebo.urdf: /home/shaede/vrx_amore/src/vrx/wamv_description/urdf/thrusters/engine.xacro
vrx/wamv_gazebo/urdf/wamv_gazebo.urdf: /home/shaede/vrx_amore/src/vrx/wamv_gazebo/urdf/components/wamv_camera.xacro
vrx/wamv_gazebo/urdf/wamv_gazebo.urdf: /home/shaede/vrx_amore/src/vrx/wamv_gazebo/urdf/components/wamv_planar_lidar.xacro
vrx/wamv_gazebo/urdf/wamv_gazebo.urdf: /home/shaede/vrx_amore/src/vrx/wamv_gazebo/urdf/thruster_layouts/wamv_aft_thrusters.xacro
vrx/wamv_gazebo/urdf/wamv_gazebo.urdf: /home/shaede/vrx_amore/src/vrx/wamv_gazebo/urdf/dynamics/wamv_gazebo_dynamics_plugin.xacro
vrx/wamv_gazebo/urdf/wamv_gazebo.urdf: /home/shaede/vrx_amore/src/vrx/wamv_gazebo/urdf/components/wamv_pinger.xacro
vrx/wamv_gazebo/urdf/wamv_gazebo.urdf: /home/shaede/vrx_amore/src/vrx/wamv_gazebo/urdf/wamv_gazebo.xacro
vrx/wamv_gazebo/urdf/wamv_gazebo.urdf: /home/shaede/vrx_amore/src/vrx/wamv_gazebo/urdf/macros.xacro
vrx/wamv_gazebo/urdf/wamv_gazebo.urdf: /home/shaede/vrx_amore/src/vrx/wamv_gazebo/urdf/components/ball_shooter.xacro
vrx/wamv_gazebo/urdf/wamv_gazebo.urdf: /home/shaede/vrx_amore/src/vrx/wamv_gazebo/urdf/components/wamv_3d_lidar.xacro
vrx/wamv_gazebo/urdf/wamv_gazebo.urdf: /home/shaede/vrx_amore/src/vrx/wamv_gazebo/urdf/components/wamv_gps.xacro
vrx/wamv_gazebo/urdf/wamv_gazebo.urdf: /home/shaede/vrx_amore/src/vrx/wamv_gazebo/urdf/components/lidar.xacro
vrx/wamv_gazebo/urdf/wamv_gazebo.urdf: /home/shaede/vrx_amore/src/vrx/wamv_gazebo/urdf/thruster_layouts/wamv_gazebo_thruster_config.xacro
vrx/wamv_gazebo/urdf/wamv_gazebo.urdf: /home/shaede/vrx_amore/src/vrx/wamv_gazebo/urdf/components/wamv_imu.xacro
vrx/wamv_gazebo/urdf/wamv_gazebo.urdf: /home/shaede/vrx_amore/src/vrx/wamv_description/urdf/wamv_base.urdf.xacro
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/shaede/vrx_amore/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "xacro: generating urdf/wamv_gazebo.urdf from urdf/wamv_gazebo.urdf.xacro"
	cd /home/shaede/vrx_amore/src/vrx/wamv_gazebo && /home/shaede/vrx_amore/build/catkin_generated/env_cached.sh xacro -o /home/shaede/vrx_amore/build/vrx/wamv_gazebo/urdf/wamv_gazebo.urdf urdf/wamv_gazebo.urdf.xacro

wamv_gazebo__xacro_auto_generate: vrx/wamv_gazebo/CMakeFiles/wamv_gazebo__xacro_auto_generate
wamv_gazebo__xacro_auto_generate: vrx/wamv_gazebo/urdf/wamv_gazebo.urdf
wamv_gazebo__xacro_auto_generate: vrx/wamv_gazebo/CMakeFiles/wamv_gazebo__xacro_auto_generate.dir/build.make

.PHONY : wamv_gazebo__xacro_auto_generate

# Rule to build all files generated by this target.
vrx/wamv_gazebo/CMakeFiles/wamv_gazebo__xacro_auto_generate.dir/build: wamv_gazebo__xacro_auto_generate

.PHONY : vrx/wamv_gazebo/CMakeFiles/wamv_gazebo__xacro_auto_generate.dir/build

vrx/wamv_gazebo/CMakeFiles/wamv_gazebo__xacro_auto_generate.dir/clean:
	cd /home/shaede/vrx_amore/build/vrx/wamv_gazebo && $(CMAKE_COMMAND) -P CMakeFiles/wamv_gazebo__xacro_auto_generate.dir/cmake_clean.cmake
.PHONY : vrx/wamv_gazebo/CMakeFiles/wamv_gazebo__xacro_auto_generate.dir/clean

vrx/wamv_gazebo/CMakeFiles/wamv_gazebo__xacro_auto_generate.dir/depend:
	cd /home/shaede/vrx_amore/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/shaede/vrx_amore/src /home/shaede/vrx_amore/src/vrx/wamv_gazebo /home/shaede/vrx_amore/build /home/shaede/vrx_amore/build/vrx/wamv_gazebo /home/shaede/vrx_amore/build/vrx/wamv_gazebo/CMakeFiles/wamv_gazebo__xacro_auto_generate.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : vrx/wamv_gazebo/CMakeFiles/wamv_gazebo__xacro_auto_generate.dir/depend

