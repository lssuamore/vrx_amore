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

# Utility rule file for wave_gazebo__xacro_auto_generate_to_devel_space_.

# Include the progress variables for this target.
include vrx/wave_gazebo/CMakeFiles/wave_gazebo__xacro_auto_generate_to_devel_space_.dir/progress.make

vrx/wave_gazebo/CMakeFiles/wave_gazebo__xacro_auto_generate_to_devel_space_: /home/shaede/vrx_amore/devel/share/wave_gazebo/worlds/ocean.world
vrx/wave_gazebo/CMakeFiles/wave_gazebo__xacro_auto_generate_to_devel_space_: /home/shaede/vrx_amore/devel/share/wave_gazebo/worlds/ocean_buoys.world
vrx/wave_gazebo/CMakeFiles/wave_gazebo__xacro_auto_generate_to_devel_space_: /home/shaede/vrx_amore/devel/share/wave_gazebo/worlds/ocean_wamv.world


/home/shaede/vrx_amore/devel/share/wave_gazebo/worlds/ocean.world: /home/shaede/vrx_amore/devel/share/wave_gazebo/worlds
/home/shaede/vrx_amore/devel/share/wave_gazebo/worlds/ocean.world: vrx/wave_gazebo/worlds/ocean.world
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/shaede/vrx_amore/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Copying to devel space: /home/shaede/vrx_amore/devel/share/wave_gazebo/worlds/ocean.world"
	cd /home/shaede/vrx_amore/build/vrx/wave_gazebo && /usr/bin/cmake -E copy_if_different /home/shaede/vrx_amore/build/vrx/wave_gazebo/worlds/ocean.world /home/shaede/vrx_amore/devel/share/wave_gazebo/worlds/ocean.world

/home/shaede/vrx_amore/devel/share/wave_gazebo/worlds/ocean_buoys.world: /home/shaede/vrx_amore/devel/share/wave_gazebo/worlds
/home/shaede/vrx_amore/devel/share/wave_gazebo/worlds/ocean_buoys.world: vrx/wave_gazebo/worlds/ocean_buoys.world
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/shaede/vrx_amore/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Copying to devel space: /home/shaede/vrx_amore/devel/share/wave_gazebo/worlds/ocean_buoys.world"
	cd /home/shaede/vrx_amore/build/vrx/wave_gazebo && /usr/bin/cmake -E copy_if_different /home/shaede/vrx_amore/build/vrx/wave_gazebo/worlds/ocean_buoys.world /home/shaede/vrx_amore/devel/share/wave_gazebo/worlds/ocean_buoys.world

/home/shaede/vrx_amore/devel/share/wave_gazebo/worlds/ocean_wamv.world: /home/shaede/vrx_amore/devel/share/wave_gazebo/worlds
/home/shaede/vrx_amore/devel/share/wave_gazebo/worlds/ocean_wamv.world: vrx/wave_gazebo/worlds/ocean_wamv.world
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/shaede/vrx_amore/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Copying to devel space: /home/shaede/vrx_amore/devel/share/wave_gazebo/worlds/ocean_wamv.world"
	cd /home/shaede/vrx_amore/build/vrx/wave_gazebo && /usr/bin/cmake -E copy_if_different /home/shaede/vrx_amore/build/vrx/wave_gazebo/worlds/ocean_wamv.world /home/shaede/vrx_amore/devel/share/wave_gazebo/worlds/ocean_wamv.world

/home/shaede/vrx_amore/devel/share/wave_gazebo/worlds:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/shaede/vrx_amore/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "creating dir /home/shaede/vrx_amore/devel/share/wave_gazebo/worlds"
	cd /home/shaede/vrx_amore/build/vrx/wave_gazebo && /usr/bin/cmake -E make_directory /home/shaede/vrx_amore/devel/share/wave_gazebo/worlds

vrx/wave_gazebo/worlds/ocean.world: /home/shaede/vrx_amore/src/vrx/wave_gazebo/worlds/ocean.world.xacro
vrx/wave_gazebo/worlds/ocean.world: /home/shaede/vrx_amore/src/vrx/wave_gazebo/world_models/ocean_waves/model.xacro
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/shaede/vrx_amore/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "xacro: generating worlds/ocean.world from worlds/ocean.world.xacro"
	cd /home/shaede/vrx_amore/src/vrx/wave_gazebo && /home/shaede/vrx_amore/build/catkin_generated/env_cached.sh xacro -o /home/shaede/vrx_amore/build/vrx/wave_gazebo/worlds/ocean.world worlds/ocean.world.xacro

vrx/wave_gazebo/worlds/ocean_buoys.world: /home/shaede/vrx_amore/src/vrx/wave_gazebo/worlds/ocean_buoys.world.xacro
vrx/wave_gazebo/worlds/ocean_buoys.world: /home/shaede/vrx_amore/src/vrx/wave_gazebo/world_models/ocean_waves/model.xacro
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/shaede/vrx_amore/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "xacro: generating worlds/ocean_buoys.world from worlds/ocean_buoys.world.xacro"
	cd /home/shaede/vrx_amore/src/vrx/wave_gazebo && /home/shaede/vrx_amore/build/catkin_generated/env_cached.sh xacro -o /home/shaede/vrx_amore/build/vrx/wave_gazebo/worlds/ocean_buoys.world worlds/ocean_buoys.world.xacro

vrx/wave_gazebo/worlds/ocean_wamv.world: /home/shaede/vrx_amore/src/vrx/wave_gazebo/worlds/ocean_wamv.world.xacro
vrx/wave_gazebo/worlds/ocean_wamv.world: /home/shaede/vrx_amore/src/vrx/wave_gazebo/world_models/ocean_waves/model.xacro
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/shaede/vrx_amore/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "xacro: generating worlds/ocean_wamv.world from worlds/ocean_wamv.world.xacro"
	cd /home/shaede/vrx_amore/src/vrx/wave_gazebo && /home/shaede/vrx_amore/build/catkin_generated/env_cached.sh xacro -o /home/shaede/vrx_amore/build/vrx/wave_gazebo/worlds/ocean_wamv.world worlds/ocean_wamv.world.xacro

/home/shaede/vrx_amore/src/vrx/wave_gazebo/world_models/ocean_waves/model.xacro: /home/shaede/vrx_amore/src/vrx/wave_gazebo/world_models/ocean_waves/model.xacro.erb
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/shaede/vrx_amore/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Generating /home/shaede/vrx_amore/src/vrx/wave_gazebo/world_models/ocean_waves/model.xacro"
	cd /home/shaede/vrx_amore/src/vrx/wave_gazebo && /usr/bin/erb world_models/ocean_waves/model.xacro.erb > /home/shaede/vrx_amore/src/vrx/wave_gazebo/world_models/ocean_waves/model.xacro

wave_gazebo__xacro_auto_generate_to_devel_space_: vrx/wave_gazebo/CMakeFiles/wave_gazebo__xacro_auto_generate_to_devel_space_
wave_gazebo__xacro_auto_generate_to_devel_space_: /home/shaede/vrx_amore/devel/share/wave_gazebo/worlds/ocean.world
wave_gazebo__xacro_auto_generate_to_devel_space_: /home/shaede/vrx_amore/devel/share/wave_gazebo/worlds/ocean_buoys.world
wave_gazebo__xacro_auto_generate_to_devel_space_: /home/shaede/vrx_amore/devel/share/wave_gazebo/worlds/ocean_wamv.world
wave_gazebo__xacro_auto_generate_to_devel_space_: /home/shaede/vrx_amore/devel/share/wave_gazebo/worlds
wave_gazebo__xacro_auto_generate_to_devel_space_: vrx/wave_gazebo/worlds/ocean.world
wave_gazebo__xacro_auto_generate_to_devel_space_: vrx/wave_gazebo/worlds/ocean_buoys.world
wave_gazebo__xacro_auto_generate_to_devel_space_: vrx/wave_gazebo/worlds/ocean_wamv.world
wave_gazebo__xacro_auto_generate_to_devel_space_: /home/shaede/vrx_amore/src/vrx/wave_gazebo/world_models/ocean_waves/model.xacro
wave_gazebo__xacro_auto_generate_to_devel_space_: vrx/wave_gazebo/CMakeFiles/wave_gazebo__xacro_auto_generate_to_devel_space_.dir/build.make

.PHONY : wave_gazebo__xacro_auto_generate_to_devel_space_

# Rule to build all files generated by this target.
vrx/wave_gazebo/CMakeFiles/wave_gazebo__xacro_auto_generate_to_devel_space_.dir/build: wave_gazebo__xacro_auto_generate_to_devel_space_

.PHONY : vrx/wave_gazebo/CMakeFiles/wave_gazebo__xacro_auto_generate_to_devel_space_.dir/build

vrx/wave_gazebo/CMakeFiles/wave_gazebo__xacro_auto_generate_to_devel_space_.dir/clean:
	cd /home/shaede/vrx_amore/build/vrx/wave_gazebo && $(CMAKE_COMMAND) -P CMakeFiles/wave_gazebo__xacro_auto_generate_to_devel_space_.dir/cmake_clean.cmake
.PHONY : vrx/wave_gazebo/CMakeFiles/wave_gazebo__xacro_auto_generate_to_devel_space_.dir/clean

vrx/wave_gazebo/CMakeFiles/wave_gazebo__xacro_auto_generate_to_devel_space_.dir/depend:
	cd /home/shaede/vrx_amore/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/shaede/vrx_amore/src /home/shaede/vrx_amore/src/vrx/wave_gazebo /home/shaede/vrx_amore/build /home/shaede/vrx_amore/build/vrx/wave_gazebo /home/shaede/vrx_amore/build/vrx/wave_gazebo/CMakeFiles/wave_gazebo__xacro_auto_generate_to_devel_space_.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : vrx/wave_gazebo/CMakeFiles/wave_gazebo__xacro_auto_generate_to_devel_space_.dir/depend

