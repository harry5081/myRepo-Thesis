# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/kimko/KIMKO

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kimko/KIMKO/build

# Include any dependencies generated for this target.
include pid/CMakeFiles/pid.dir/depend.make

# Include the progress variables for this target.
include pid/CMakeFiles/pid.dir/progress.make

# Include the compile flags for this target's objects.
include pid/CMakeFiles/pid.dir/flags.make

pid/CMakeFiles/pid.dir/pid.cpp.o: pid/CMakeFiles/pid.dir/flags.make
pid/CMakeFiles/pid.dir/pid.cpp.o: ../pid/pid.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kimko/KIMKO/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object pid/CMakeFiles/pid.dir/pid.cpp.o"
	cd /home/kimko/KIMKO/build/pid && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pid.dir/pid.cpp.o -c /home/kimko/KIMKO/pid/pid.cpp

pid/CMakeFiles/pid.dir/pid.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pid.dir/pid.cpp.i"
	cd /home/kimko/KIMKO/build/pid && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kimko/KIMKO/pid/pid.cpp > CMakeFiles/pid.dir/pid.cpp.i

pid/CMakeFiles/pid.dir/pid.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pid.dir/pid.cpp.s"
	cd /home/kimko/KIMKO/build/pid && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kimko/KIMKO/pid/pid.cpp -o CMakeFiles/pid.dir/pid.cpp.s

pid/CMakeFiles/pid.dir/pid.cpp.o.requires:

.PHONY : pid/CMakeFiles/pid.dir/pid.cpp.o.requires

pid/CMakeFiles/pid.dir/pid.cpp.o.provides: pid/CMakeFiles/pid.dir/pid.cpp.o.requires
	$(MAKE) -f pid/CMakeFiles/pid.dir/build.make pid/CMakeFiles/pid.dir/pid.cpp.o.provides.build
.PHONY : pid/CMakeFiles/pid.dir/pid.cpp.o.provides

pid/CMakeFiles/pid.dir/pid.cpp.o.provides.build: pid/CMakeFiles/pid.dir/pid.cpp.o


# Object files for target pid
pid_OBJECTS = \
"CMakeFiles/pid.dir/pid.cpp.o"

# External object files for target pid
pid_EXTERNAL_OBJECTS =

pid/libpid.a: pid/CMakeFiles/pid.dir/pid.cpp.o
pid/libpid.a: pid/CMakeFiles/pid.dir/build.make
pid/libpid.a: pid/CMakeFiles/pid.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kimko/KIMKO/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libpid.a"
	cd /home/kimko/KIMKO/build/pid && $(CMAKE_COMMAND) -P CMakeFiles/pid.dir/cmake_clean_target.cmake
	cd /home/kimko/KIMKO/build/pid && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pid.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
pid/CMakeFiles/pid.dir/build: pid/libpid.a

.PHONY : pid/CMakeFiles/pid.dir/build

pid/CMakeFiles/pid.dir/requires: pid/CMakeFiles/pid.dir/pid.cpp.o.requires

.PHONY : pid/CMakeFiles/pid.dir/requires

pid/CMakeFiles/pid.dir/clean:
	cd /home/kimko/KIMKO/build/pid && $(CMAKE_COMMAND) -P CMakeFiles/pid.dir/cmake_clean.cmake
.PHONY : pid/CMakeFiles/pid.dir/clean

pid/CMakeFiles/pid.dir/depend:
	cd /home/kimko/KIMKO/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kimko/KIMKO /home/kimko/KIMKO/pid /home/kimko/KIMKO/build /home/kimko/KIMKO/build/pid /home/kimko/KIMKO/build/pid/CMakeFiles/pid.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : pid/CMakeFiles/pid.dir/depend

