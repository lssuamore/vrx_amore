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
CMAKE_SOURCE_DIR = /home/brad/vrx_amore/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/brad/vrx_amore/build

# Include any dependencies generated for this target.
include vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/depend.make

# Include the progress variables for this target.
include vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/progress.make

# Include the compile flags for this target's objects.
include vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/flags.make

vrx/vrx_gazebo/msgs/light_buoy_colors.pb.h: /home/brad/vrx_amore/src/vrx/vrx_gazebo/msgs/light_buoy_colors.proto
vrx/vrx_gazebo/msgs/light_buoy_colors.pb.h: /usr/bin/protoc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/brad/vrx_amore/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Running cpp protocol buffer compiler on light_buoy_colors.proto"
	cd /home/brad/vrx_amore/build/vrx/vrx_gazebo/msgs && /usr/bin/protoc --cpp_out /home/brad/vrx_amore/build/vrx/vrx_gazebo/msgs -I /home/brad/vrx_amore/src/vrx/vrx_gazebo/msgs -I /usr/include/gazebo-11/gazebo/msgs/proto /home/brad/vrx_amore/src/vrx/vrx_gazebo/msgs/light_buoy_colors.proto

vrx/vrx_gazebo/msgs/light_buoy_colors.pb.cc: vrx/vrx_gazebo/msgs/light_buoy_colors.pb.h
	@$(CMAKE_COMMAND) -E touch_nocreate vrx/vrx_gazebo/msgs/light_buoy_colors.pb.cc

vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/light_buoy_colors_msgs_autogen/mocs_compilation.cpp.o: vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/flags.make
vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/light_buoy_colors_msgs_autogen/mocs_compilation.cpp.o: vrx/vrx_gazebo/msgs/light_buoy_colors_msgs_autogen/mocs_compilation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/brad/vrx_amore/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/light_buoy_colors_msgs_autogen/mocs_compilation.cpp.o"
	cd /home/brad/vrx_amore/build/vrx/vrx_gazebo/msgs && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/light_buoy_colors_msgs.dir/light_buoy_colors_msgs_autogen/mocs_compilation.cpp.o -c /home/brad/vrx_amore/build/vrx/vrx_gazebo/msgs/light_buoy_colors_msgs_autogen/mocs_compilation.cpp

vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/light_buoy_colors_msgs_autogen/mocs_compilation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/light_buoy_colors_msgs.dir/light_buoy_colors_msgs_autogen/mocs_compilation.cpp.i"
	cd /home/brad/vrx_amore/build/vrx/vrx_gazebo/msgs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/brad/vrx_amore/build/vrx/vrx_gazebo/msgs/light_buoy_colors_msgs_autogen/mocs_compilation.cpp > CMakeFiles/light_buoy_colors_msgs.dir/light_buoy_colors_msgs_autogen/mocs_compilation.cpp.i

vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/light_buoy_colors_msgs_autogen/mocs_compilation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/light_buoy_colors_msgs.dir/light_buoy_colors_msgs_autogen/mocs_compilation.cpp.s"
	cd /home/brad/vrx_amore/build/vrx/vrx_gazebo/msgs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/brad/vrx_amore/build/vrx/vrx_gazebo/msgs/light_buoy_colors_msgs_autogen/mocs_compilation.cpp -o CMakeFiles/light_buoy_colors_msgs.dir/light_buoy_colors_msgs_autogen/mocs_compilation.cpp.s

vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/light_buoy_colors.pb.cc.o: vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/flags.make
vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/light_buoy_colors.pb.cc.o: vrx/vrx_gazebo/msgs/light_buoy_colors.pb.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/brad/vrx_amore/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/light_buoy_colors.pb.cc.o"
	cd /home/brad/vrx_amore/build/vrx/vrx_gazebo/msgs && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/light_buoy_colors_msgs.dir/light_buoy_colors.pb.cc.o -c /home/brad/vrx_amore/build/vrx/vrx_gazebo/msgs/light_buoy_colors.pb.cc

vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/light_buoy_colors.pb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/light_buoy_colors_msgs.dir/light_buoy_colors.pb.cc.i"
	cd /home/brad/vrx_amore/build/vrx/vrx_gazebo/msgs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/brad/vrx_amore/build/vrx/vrx_gazebo/msgs/light_buoy_colors.pb.cc > CMakeFiles/light_buoy_colors_msgs.dir/light_buoy_colors.pb.cc.i

vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/light_buoy_colors.pb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/light_buoy_colors_msgs.dir/light_buoy_colors.pb.cc.s"
	cd /home/brad/vrx_amore/build/vrx/vrx_gazebo/msgs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/brad/vrx_amore/build/vrx/vrx_gazebo/msgs/light_buoy_colors.pb.cc -o CMakeFiles/light_buoy_colors_msgs.dir/light_buoy_colors.pb.cc.s

# Object files for target light_buoy_colors_msgs
light_buoy_colors_msgs_OBJECTS = \
"CMakeFiles/light_buoy_colors_msgs.dir/light_buoy_colors_msgs_autogen/mocs_compilation.cpp.o" \
"CMakeFiles/light_buoy_colors_msgs.dir/light_buoy_colors.pb.cc.o"

# External object files for target light_buoy_colors_msgs
light_buoy_colors_msgs_EXTERNAL_OBJECTS =

/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/light_buoy_colors_msgs_autogen/mocs_compilation.cpp.o
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/light_buoy_colors.pb.cc.o
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/build.make
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libSimTKsimbody.so.3.6
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libdart.so.6.9.2
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libsdformat9.so.9.8.0
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libignition-common3-graphics.so.3.14.2
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libSimTKmath.so.3.6
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libSimTKcommon.so.3.6
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libblas.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/liblapack.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libblas.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/liblapack.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libdart-external-odelcpsolver.so.6.9.2
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libccd.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libfcl.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libassimp.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/liboctomap.so.1.9.3
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/liboctomath.so.1.9.3
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.71.0
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libignition-transport8.so.8.3.0
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libignition-fuel_tools4.so.4.6.0
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libignition-msgs5.so.5.10.0
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libignition-math6.so.6.13.0
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libignition-common3.so.3.14.2
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so: vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/brad/vrx_amore/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library /home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so"
	cd /home/brad/vrx_amore/build/vrx/vrx_gazebo/msgs && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/light_buoy_colors_msgs.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/build: /home/brad/vrx_amore/devel/lib/liblight_buoy_colors_msgs.so

.PHONY : vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/build

vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/clean:
	cd /home/brad/vrx_amore/build/vrx/vrx_gazebo/msgs && $(CMAKE_COMMAND) -P CMakeFiles/light_buoy_colors_msgs.dir/cmake_clean.cmake
.PHONY : vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/clean

vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/depend: vrx/vrx_gazebo/msgs/light_buoy_colors.pb.h
vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/depend: vrx/vrx_gazebo/msgs/light_buoy_colors.pb.cc
	cd /home/brad/vrx_amore/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/brad/vrx_amore/src /home/brad/vrx_amore/src/vrx/vrx_gazebo/msgs /home/brad/vrx_amore/build /home/brad/vrx_amore/build/vrx/vrx_gazebo/msgs /home/brad/vrx_amore/build/vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : vrx/vrx_gazebo/msgs/CMakeFiles/light_buoy_colors_msgs.dir/depend

