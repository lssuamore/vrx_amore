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

# Utility rule file for dock_erb_generation.

# Include the progress variables for this target.
include vrx/vrx_gazebo/CMakeFiles/dock_erb_generation.dir/progress.make

vrx/vrx_gazebo/CMakeFiles/dock_erb_generation: /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2016/model.sdf
vrx/vrx_gazebo/CMakeFiles/dock_erb_generation: /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2018/model.sdf
vrx/vrx_gazebo/CMakeFiles/dock_erb_generation: /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2022/model.sdf
vrx/vrx_gazebo/CMakeFiles/dock_erb_generation: /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2016_dynamic/model.sdf
vrx/vrx_gazebo/CMakeFiles/dock_erb_generation: /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2018_dynamic/model.sdf
vrx/vrx_gazebo/CMakeFiles/dock_erb_generation: /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2022_dynamic/model.sdf


/home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2016/model.sdf: /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2016/model.sdf.erb
/home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2016/model.sdf: /home/shaede/vrx_amore/src/vrx/vrx_gazebo/dock_generator.erb
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/shaede/vrx_amore/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2016/model.sdf"
	cd /home/shaede/vrx_amore/src/vrx/vrx_gazebo && /usr/bin/erb models/dock_2016/model.sdf.erb > /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2016/model.sdf

/home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2018/model.sdf: /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2018/model.sdf.erb
/home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2018/model.sdf: /home/shaede/vrx_amore/src/vrx/vrx_gazebo/dock_generator.erb
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/shaede/vrx_amore/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2018/model.sdf"
	cd /home/shaede/vrx_amore/src/vrx/vrx_gazebo && /usr/bin/erb models/dock_2018/model.sdf.erb > /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2018/model.sdf

/home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2022/model.sdf: /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2022/model.sdf.erb
/home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2022/model.sdf: /home/shaede/vrx_amore/src/vrx/vrx_gazebo/dock_generator.erb
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/shaede/vrx_amore/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2022/model.sdf"
	cd /home/shaede/vrx_amore/src/vrx/vrx_gazebo && /usr/bin/erb models/dock_2022/model.sdf.erb > /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2022/model.sdf

/home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2016_dynamic/model.sdf: /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2016_dynamic/model.sdf.erb
/home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2016_dynamic/model.sdf: /home/shaede/vrx_amore/src/vrx/vrx_gazebo/dock_generator.erb
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/shaede/vrx_amore/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Generating /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2016_dynamic/model.sdf"
	cd /home/shaede/vrx_amore/src/vrx/vrx_gazebo && /usr/bin/erb models/dock_2016_dynamic/model.sdf.erb > /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2016_dynamic/model.sdf

/home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2018_dynamic/model.sdf: /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2018_dynamic/model.sdf.erb
/home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2018_dynamic/model.sdf: /home/shaede/vrx_amore/src/vrx/vrx_gazebo/dock_generator.erb
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/shaede/vrx_amore/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Generating /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2018_dynamic/model.sdf"
	cd /home/shaede/vrx_amore/src/vrx/vrx_gazebo && /usr/bin/erb models/dock_2018_dynamic/model.sdf.erb > /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2018_dynamic/model.sdf

/home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2022_dynamic/model.sdf: /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2022_dynamic/model.sdf.erb
/home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2022_dynamic/model.sdf: /home/shaede/vrx_amore/src/vrx/vrx_gazebo/dock_generator.erb
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/shaede/vrx_amore/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Generating /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2022_dynamic/model.sdf"
	cd /home/shaede/vrx_amore/src/vrx/vrx_gazebo && /usr/bin/erb models/dock_2022_dynamic/model.sdf.erb > /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2022_dynamic/model.sdf

dock_erb_generation: vrx/vrx_gazebo/CMakeFiles/dock_erb_generation
dock_erb_generation: /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2016/model.sdf
dock_erb_generation: /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2018/model.sdf
dock_erb_generation: /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2022/model.sdf
dock_erb_generation: /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2016_dynamic/model.sdf
dock_erb_generation: /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2018_dynamic/model.sdf
dock_erb_generation: /home/shaede/vrx_amore/src/vrx/vrx_gazebo/models/dock_2022_dynamic/model.sdf
dock_erb_generation: vrx/vrx_gazebo/CMakeFiles/dock_erb_generation.dir/build.make

.PHONY : dock_erb_generation

# Rule to build all files generated by this target.
vrx/vrx_gazebo/CMakeFiles/dock_erb_generation.dir/build: dock_erb_generation

.PHONY : vrx/vrx_gazebo/CMakeFiles/dock_erb_generation.dir/build

vrx/vrx_gazebo/CMakeFiles/dock_erb_generation.dir/clean:
	cd /home/shaede/vrx_amore/build/vrx/vrx_gazebo && $(CMAKE_COMMAND) -P CMakeFiles/dock_erb_generation.dir/cmake_clean.cmake
.PHONY : vrx/vrx_gazebo/CMakeFiles/dock_erb_generation.dir/clean

vrx/vrx_gazebo/CMakeFiles/dock_erb_generation.dir/depend:
	cd /home/shaede/vrx_amore/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/shaede/vrx_amore/src /home/shaede/vrx_amore/src/vrx/vrx_gazebo /home/shaede/vrx_amore/build /home/shaede/vrx_amore/build/vrx/vrx_gazebo /home/shaede/vrx_amore/build/vrx/vrx_gazebo/CMakeFiles/dock_erb_generation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : vrx/vrx_gazebo/CMakeFiles/dock_erb_generation.dir/depend

