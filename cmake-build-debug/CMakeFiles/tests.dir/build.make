# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

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
CMAKE_COMMAND = /home/ya/clion-2020.2.5/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/ya/clion-2020.2.5/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ya/lili-om-modify_ws/src/LiLi-OM-ROT

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ya/lili-om-modify_ws/src/LiLi-OM-ROT/cmake-build-debug

# Utility rule file for tests.

# Include the progress variables for this target.
include CMakeFiles/tests.dir/progress.make

tests: CMakeFiles/tests.dir/build.make

.PHONY : tests

# Rule to build all files generated by this target.
CMakeFiles/tests.dir/build: tests

.PHONY : CMakeFiles/tests.dir/build

CMakeFiles/tests.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/tests.dir/cmake_clean.cmake
.PHONY : CMakeFiles/tests.dir/clean

CMakeFiles/tests.dir/depend:
	cd /home/ya/lili-om-modify_ws/src/LiLi-OM-ROT/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ya/lili-om-modify_ws/src/LiLi-OM-ROT /home/ya/lili-om-modify_ws/src/LiLi-OM-ROT /home/ya/lili-om-modify_ws/src/LiLi-OM-ROT/cmake-build-debug /home/ya/lili-om-modify_ws/src/LiLi-OM-ROT/cmake-build-debug /home/ya/lili-om-modify_ws/src/LiLi-OM-ROT/cmake-build-debug/CMakeFiles/tests.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/tests.dir/depend

