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
include CMakeFiles/rigid2d_main.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rigid2d_main.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rigid2d_main.dir/flags.make

CMakeFiles/rigid2d_main.dir/src/main.cpp.o: CMakeFiles/rigid2d_main.dir/flags.make
CMakeFiles/rigid2d_main.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sarah/Documents/ME_495/ws/src/repository/rigid2d/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rigid2d_main.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rigid2d_main.dir/src/main.cpp.o -c /home/sarah/Documents/ME_495/ws/src/repository/rigid2d/src/main.cpp

CMakeFiles/rigid2d_main.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rigid2d_main.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sarah/Documents/ME_495/ws/src/repository/rigid2d/src/main.cpp > CMakeFiles/rigid2d_main.dir/src/main.cpp.i

CMakeFiles/rigid2d_main.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rigid2d_main.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sarah/Documents/ME_495/ws/src/repository/rigid2d/src/main.cpp -o CMakeFiles/rigid2d_main.dir/src/main.cpp.s

# Object files for target rigid2d_main
rigid2d_main_OBJECTS = \
"CMakeFiles/rigid2d_main.dir/src/main.cpp.o"

# External object files for target rigid2d_main
rigid2d_main_EXTERNAL_OBJECTS =

rigid2d_main: CMakeFiles/rigid2d_main.dir/src/main.cpp.o
rigid2d_main: CMakeFiles/rigid2d_main.dir/build.make
rigid2d_main: librigid2d.a
rigid2d_main: CMakeFiles/rigid2d_main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sarah/Documents/ME_495/ws/src/repository/rigid2d/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable rigid2d_main"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rigid2d_main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rigid2d_main.dir/build: rigid2d_main

.PHONY : CMakeFiles/rigid2d_main.dir/build

CMakeFiles/rigid2d_main.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rigid2d_main.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rigid2d_main.dir/clean

CMakeFiles/rigid2d_main.dir/depend:
	cd /home/sarah/Documents/ME_495/ws/src/repository/rigid2d/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sarah/Documents/ME_495/ws/src/repository/rigid2d /home/sarah/Documents/ME_495/ws/src/repository/rigid2d /home/sarah/Documents/ME_495/ws/src/repository/rigid2d/build /home/sarah/Documents/ME_495/ws/src/repository/rigid2d/build /home/sarah/Documents/ME_495/ws/src/repository/rigid2d/build/CMakeFiles/rigid2d_main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rigid2d_main.dir/depend

