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

# Utility rule file for wamv_description__xacro_auto_generate.

# Include the progress variables for this target.
include vrx/wamv_description/CMakeFiles/wamv_description__xacro_auto_generate.dir/progress.make

vrx/wamv_description/CMakeFiles/wamv_description__xacro_auto_generate: vrx/wamv_description/urdf/wamv_base.urdf


vrx/wamv_description/urdf/wamv_base.urdf: /home/shaede/vrx_amore/src/vrx/wamv_description/urdf/wamv_base.urdf.xacro
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/shaede/vrx_amore/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "xacro: generating urdf/wamv_base.urdf from urdf/wamv_base.urdf.xacro"
	cd /home/shaede/vrx_amore/src/vrx/wamv_description && /home/shaede/vrx_amore/build/catkin_generated/env_cached.sh xacro -o /home/shaede/vrx_amore/build/vrx/wamv_description/urdf/wamv_base.urdf urdf/wamv_base.urdf.xacro

wamv_description__xacro_auto_generate: vrx/wamv_description/CMakeFiles/wamv_description__xacro_auto_generate
wamv_description__xacro_auto_generate: vrx/wamv_description/urdf/wamv_base.urdf
wamv_description__xacro_auto_generate: vrx/wamv_description/CMakeFiles/wamv_description__xacro_auto_generate.dir/build.make

.PHONY : wamv_description__xacro_auto_generate

# Rule to build all files generated by this target.
vrx/wamv_description/CMakeFiles/wamv_description__xacro_auto_generate.dir/build: wamv_description__xacro_auto_generate

.PHONY : vrx/wamv_description/CMakeFiles/wamv_description__xacro_auto_generate.dir/build

vrx/wamv_description/CMakeFiles/wamv_description__xacro_auto_generate.dir/clean:
	cd /home/shaede/vrx_amore/build/vrx/wamv_description && $(CMAKE_COMMAND) -P CMakeFiles/wamv_description__xacro_auto_generate.dir/cmake_clean.cmake
.PHONY : vrx/wamv_description/CMakeFiles/wamv_description__xacro_auto_generate.dir/clean

vrx/wamv_description/CMakeFiles/wamv_description__xacro_auto_generate.dir/depend:
	cd /home/shaede/vrx_amore/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/shaede/vrx_amore/src /home/shaede/vrx_amore/src/vrx/wamv_description /home/shaede/vrx_amore/build /home/shaede/vrx_amore/build/vrx/wamv_description /home/shaede/vrx_amore/build/vrx/wamv_description/CMakeFiles/wamv_description__xacro_auto_generate.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : vrx/wamv_description/CMakeFiles/wamv_description__xacro_auto_generate.dir/depend

