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
CMAKE_SOURCE_DIR = /home/kimko/myRepo-Thesis/KIMKO

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kimko/myRepo-Thesis/KIMKO/build

# Include any dependencies generated for this target.
include CMakeFiles/kimko.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/kimko.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/kimko.dir/flags.make

CMakeFiles/kimko.dir/main.cpp.o: CMakeFiles/kimko.dir/flags.make
CMakeFiles/kimko.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kimko/myRepo-Thesis/KIMKO/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/kimko.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kimko.dir/main.cpp.o -c /home/kimko/myRepo-Thesis/KIMKO/main.cpp

CMakeFiles/kimko.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kimko.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kimko/myRepo-Thesis/KIMKO/main.cpp > CMakeFiles/kimko.dir/main.cpp.i

CMakeFiles/kimko.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kimko.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kimko/myRepo-Thesis/KIMKO/main.cpp -o CMakeFiles/kimko.dir/main.cpp.s

CMakeFiles/kimko.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/kimko.dir/main.cpp.o.requires

CMakeFiles/kimko.dir/main.cpp.o.provides: CMakeFiles/kimko.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/kimko.dir/build.make CMakeFiles/kimko.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/kimko.dir/main.cpp.o.provides

CMakeFiles/kimko.dir/main.cpp.o.provides.build: CMakeFiles/kimko.dir/main.cpp.o


CMakeFiles/kimko.dir/run.cpp.o: CMakeFiles/kimko.dir/flags.make
CMakeFiles/kimko.dir/run.cpp.o: ../run.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kimko/myRepo-Thesis/KIMKO/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/kimko.dir/run.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kimko.dir/run.cpp.o -c /home/kimko/myRepo-Thesis/KIMKO/run.cpp

CMakeFiles/kimko.dir/run.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kimko.dir/run.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kimko/myRepo-Thesis/KIMKO/run.cpp > CMakeFiles/kimko.dir/run.cpp.i

CMakeFiles/kimko.dir/run.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kimko.dir/run.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kimko/myRepo-Thesis/KIMKO/run.cpp -o CMakeFiles/kimko.dir/run.cpp.s

CMakeFiles/kimko.dir/run.cpp.o.requires:

.PHONY : CMakeFiles/kimko.dir/run.cpp.o.requires

CMakeFiles/kimko.dir/run.cpp.o.provides: CMakeFiles/kimko.dir/run.cpp.o.requires
	$(MAKE) -f CMakeFiles/kimko.dir/build.make CMakeFiles/kimko.dir/run.cpp.o.provides.build
.PHONY : CMakeFiles/kimko.dir/run.cpp.o.provides

CMakeFiles/kimko.dir/run.cpp.o.provides.build: CMakeFiles/kimko.dir/run.cpp.o


# Object files for target kimko
kimko_OBJECTS = \
"CMakeFiles/kimko.dir/main.cpp.o" \
"CMakeFiles/kimko.dir/run.cpp.o"

# External object files for target kimko
kimko_EXTERNAL_OBJECTS =

kimko: CMakeFiles/kimko.dir/main.cpp.o
kimko: CMakeFiles/kimko.dir/run.cpp.o
kimko: CMakeFiles/kimko.dir/build.make
kimko: /usr/lib/x86_64-linux-gnu/libpython3.6m.so
kimko: plot/libplot.a
kimko: pid/libpid.a
kimko: controlPlant/libplant.a
kimko: mpc/libmpc.a
kimko: leader/libleader.a
kimko: traj_plan/libtraj_plan.a
kimko: /usr/lib/libpcanbasic.so
kimko: /lib/x86_64-linux-gnu/libpthread.so.0
kimko: /usr/lib/x86_64-linux-gnu/libpython3.6m.so
kimko: CMakeFiles/kimko.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kimko/myRepo-Thesis/KIMKO/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable kimko"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kimko.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/kimko.dir/build: kimko

.PHONY : CMakeFiles/kimko.dir/build

CMakeFiles/kimko.dir/requires: CMakeFiles/kimko.dir/main.cpp.o.requires
CMakeFiles/kimko.dir/requires: CMakeFiles/kimko.dir/run.cpp.o.requires

.PHONY : CMakeFiles/kimko.dir/requires

CMakeFiles/kimko.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/kimko.dir/cmake_clean.cmake
.PHONY : CMakeFiles/kimko.dir/clean

CMakeFiles/kimko.dir/depend:
	cd /home/kimko/myRepo-Thesis/KIMKO/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kimko/myRepo-Thesis/KIMKO /home/kimko/myRepo-Thesis/KIMKO /home/kimko/myRepo-Thesis/KIMKO/build /home/kimko/myRepo-Thesis/KIMKO/build /home/kimko/myRepo-Thesis/KIMKO/build/CMakeFiles/kimko.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/kimko.dir/depend

