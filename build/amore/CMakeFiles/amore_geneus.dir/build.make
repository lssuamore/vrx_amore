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

# Utility rule file for amore_geneus.

# Include the progress variables for this target.
include amore/CMakeFiles/amore_geneus.dir/progress.make

amore_geneus: amore/CMakeFiles/amore_geneus.dir/build.make

.PHONY : amore_geneus

# Rule to build all files generated by this target.
amore/CMakeFiles/amore_geneus.dir/build: amore_geneus

.PHONY : amore/CMakeFiles/amore_geneus.dir/build

amore/CMakeFiles/amore_geneus.dir/clean:
	cd /home/brad/vrx_amore/build/amore && $(CMAKE_COMMAND) -P CMakeFiles/amore_geneus.dir/cmake_clean.cmake
.PHONY : amore/CMakeFiles/amore_geneus.dir/clean

amore/CMakeFiles/amore_geneus.dir/depend:
	cd /home/brad/vrx_amore/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/brad/vrx_amore/src /home/brad/vrx_amore/src/amore /home/brad/vrx_amore/build /home/brad/vrx_amore/build/amore /home/brad/vrx_amore/build/amore/CMakeFiles/amore_geneus.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : amore/CMakeFiles/amore_geneus.dir/depend

