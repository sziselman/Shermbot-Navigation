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
CMAKE_SOURCE_DIR = /home/sarah/Documents/ME_495/ws/src/repository/rigid2d

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sarah/Documents/ME_495/ws/src/repository/rigid2d/build

# Include any dependencies generated for this target.
include CMakeFiles/rigid_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rigid_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rigid_test.dir/flags.make

CMakeFiles/rigid_test.dir/tests/tests.cpp.o: CMakeFiles/rigid_test.dir/flags.make
CMakeFiles/rigid_test.dir/tests/tests.cpp.o: ../tests/tests.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sarah/Documents/ME_495/ws/src/repository/rigid2d/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rigid_test.dir/tests/tests.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rigid_test.dir/tests/tests.cpp.o -c /home/sarah/Documents/ME_495/ws/src/repository/rigid2d/tests/tests.cpp

CMakeFiles/rigid_test.dir/tests/tests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rigid_test.dir/tests/tests.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sarah/Documents/ME_495/ws/src/repository/rigid2d/tests/tests.cpp > CMakeFiles/rigid_test.dir/tests/tests.cpp.i

CMakeFiles/rigid_test.dir/tests/tests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rigid_test.dir/tests/tests.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sarah/Documents/ME_495/ws/src/repository/rigid2d/tests/tests.cpp -o CMakeFiles/rigid_test.dir/tests/tests.cpp.s

# Object files for target rigid_test
rigid_test_OBJECTS = \
"CMakeFiles/rigid_test.dir/tests/tests.cpp.o"

# External object files for target rigid_test
rigid_test_EXTERNAL_OBJECTS =

rigid_test: CMakeFiles/rigid_test.dir/tests/tests.cpp.o
rigid_test: CMakeFiles/rigid_test.dir/build.make
rigid_test: librigid2d.a
rigid_test: CMakeFiles/rigid_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sarah/Documents/ME_495/ws/src/repository/rigid2d/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable rigid_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rigid_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rigid_test.dir/build: rigid_test

.PHONY : CMakeFiles/rigid_test.dir/build

CMakeFiles/rigid_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rigid_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rigid_test.dir/clean

CMakeFiles/rigid_test.dir/depend:
	cd /home/sarah/Documents/ME_495/ws/src/repository/rigid2d/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sarah/Documents/ME_495/ws/src/repository/rigid2d /home/sarah/Documents/ME_495/ws/src/repository/rigid2d /home/sarah/Documents/ME_495/ws/src/repository/rigid2d/build /home/sarah/Documents/ME_495/ws/src/repository/rigid2d/build /home/sarah/Documents/ME_495/ws/src/repository/rigid2d/build/CMakeFiles/rigid_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rigid_test.dir/depend
