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

# Include any dependencies generated for this target.
include vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/depend.make

# Include the progress variables for this target.
include vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/progress.make

# Include the compile flags for this target's objects.
include vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/flags.make

vrx/vrx_gazebo/msgs/dock_placard.pb.h: /home/shaede/vrx_amore/src/vrx/vrx_gazebo/msgs/dock_placard.proto
vrx/vrx_gazebo/msgs/dock_placard.pb.h: /usr/bin/protoc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/shaede/vrx_amore/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Running cpp protocol buffer compiler on dock_placard.proto"
	cd /home/shaede/vrx_amore/build/vrx/vrx_gazebo/msgs && /usr/bin/protoc --cpp_out /home/shaede/vrx_amore/build/vrx/vrx_gazebo/msgs -I /home/shaede/vrx_amore/src/vrx/vrx_gazebo/msgs -I /usr/include/gazebo-11/gazebo/msgs/proto /home/shaede/vrx_amore/src/vrx/vrx_gazebo/msgs/dock_placard.proto

vrx/vrx_gazebo/msgs/dock_placard.pb.cc: vrx/vrx_gazebo/msgs/dock_placard.pb.h
	@$(CMAKE_COMMAND) -E touch_nocreate vrx/vrx_gazebo/msgs/dock_placard.pb.cc

vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/dock_placard_msgs_autogen/mocs_compilation.cpp.o: vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/flags.make
vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/dock_placard_msgs_autogen/mocs_compilation.cpp.o: vrx/vrx_gazebo/msgs/dock_placard_msgs_autogen/mocs_compilation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shaede/vrx_amore/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/dock_placard_msgs_autogen/mocs_compilation.cpp.o"
	cd /home/shaede/vrx_amore/build/vrx/vrx_gazebo/msgs && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dock_placard_msgs.dir/dock_placard_msgs_autogen/mocs_compilation.cpp.o -c /home/shaede/vrx_amore/build/vrx/vrx_gazebo/msgs/dock_placard_msgs_autogen/mocs_compilation.cpp

vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/dock_placard_msgs_autogen/mocs_compilation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dock_placard_msgs.dir/dock_placard_msgs_autogen/mocs_compilation.cpp.i"
	cd /home/shaede/vrx_amore/build/vrx/vrx_gazebo/msgs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shaede/vrx_amore/build/vrx/vrx_gazebo/msgs/dock_placard_msgs_autogen/mocs_compilation.cpp > CMakeFiles/dock_placard_msgs.dir/dock_placard_msgs_autogen/mocs_compilation.cpp.i

vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/dock_placard_msgs_autogen/mocs_compilation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dock_placard_msgs.dir/dock_placard_msgs_autogen/mocs_compilation.cpp.s"
	cd /home/shaede/vrx_amore/build/vrx/vrx_gazebo/msgs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shaede/vrx_amore/build/vrx/vrx_gazebo/msgs/dock_placard_msgs_autogen/mocs_compilation.cpp -o CMakeFiles/dock_placard_msgs.dir/dock_placard_msgs_autogen/mocs_compilation.cpp.s

vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/dock_placard.pb.cc.o: vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/flags.make
vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/dock_placard.pb.cc.o: vrx/vrx_gazebo/msgs/dock_placard.pb.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shaede/vrx_amore/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/dock_placard.pb.cc.o"
	cd /home/shaede/vrx_amore/build/vrx/vrx_gazebo/msgs && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dock_placard_msgs.dir/dock_placard.pb.cc.o -c /home/shaede/vrx_amore/build/vrx/vrx_gazebo/msgs/dock_placard.pb.cc

vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/dock_placard.pb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dock_placard_msgs.dir/dock_placard.pb.cc.i"
	cd /home/shaede/vrx_amore/build/vrx/vrx_gazebo/msgs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shaede/vrx_amore/build/vrx/vrx_gazebo/msgs/dock_placard.pb.cc > CMakeFiles/dock_placard_msgs.dir/dock_placard.pb.cc.i

vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/dock_placard.pb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dock_placard_msgs.dir/dock_placard.pb.cc.s"
	cd /home/shaede/vrx_amore/build/vrx/vrx_gazebo/msgs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shaede/vrx_amore/build/vrx/vrx_gazebo/msgs/dock_placard.pb.cc -o CMakeFiles/dock_placard_msgs.dir/dock_placard.pb.cc.s

# Object files for target dock_placard_msgs
dock_placard_msgs_OBJECTS = \
"CMakeFiles/dock_placard_msgs.dir/dock_placard_msgs_autogen/mocs_compilation.cpp.o" \
"CMakeFiles/dock_placard_msgs.dir/dock_placard.pb.cc.o"

# External object files for target dock_placard_msgs
dock_placard_msgs_EXTERNAL_OBJECTS =

/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/dock_placard_msgs_autogen/mocs_compilation.cpp.o
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/dock_placard.pb.cc.o
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/build.make
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libSimTKsimbody.so.3.6
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libdart.so.6.9.2
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libsdformat9.so.9.7.0
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libignition-common3-graphics.so.3.14.0
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libSimTKmath.so.3.6
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libSimTKcommon.so.3.6
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libblas.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/liblapack.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libblas.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/liblapack.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libdart-external-odelcpsolver.so.6.9.2
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libccd.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libfcl.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libassimp.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/liboctomap.so.1.9.3
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/liboctomath.so.1.9.3
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.71.0
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libignition-transport8.so.8.2.1
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libignition-fuel_tools4.so.4.4.0
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libignition-msgs5.so.5.9.0
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libignition-math6.so.6.10.0
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libignition-common3.so.3.14.0
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so: vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/shaede/vrx_amore/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library /home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so"
	cd /home/shaede/vrx_amore/build/vrx/vrx_gazebo/msgs && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/dock_placard_msgs.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/build: /home/shaede/vrx_amore/devel/lib/libdock_placard_msgs.so

.PHONY : vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/build

vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/clean:
	cd /home/shaede/vrx_amore/build/vrx/vrx_gazebo/msgs && $(CMAKE_COMMAND) -P CMakeFiles/dock_placard_msgs.dir/cmake_clean.cmake
.PHONY : vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/clean

vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/depend: vrx/vrx_gazebo/msgs/dock_placard.pb.h
vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/depend: vrx/vrx_gazebo/msgs/dock_placard.pb.cc
	cd /home/shaede/vrx_amore/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/shaede/vrx_amore/src /home/shaede/vrx_amore/src/vrx/vrx_gazebo/msgs /home/shaede/vrx_amore/build /home/shaede/vrx_amore/build/vrx/vrx_gazebo/msgs /home/shaede/vrx_amore/build/vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : vrx/vrx_gazebo/msgs/CMakeFiles/dock_placard_msgs.dir/depend

