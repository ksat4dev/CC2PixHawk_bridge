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
CMAKE_SOURCE_DIR = /home/kash14sat/Desktop/zmq

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kash14sat/Desktop/zmq/build

# Include any dependencies generated for this target.
include CMakeFiles/zmq_sub.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/zmq_sub.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/zmq_sub.dir/flags.make

CMakeFiles/zmq_sub.dir/src/subscriber.cpp.o: CMakeFiles/zmq_sub.dir/flags.make
CMakeFiles/zmq_sub.dir/src/subscriber.cpp.o: ../src/subscriber.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kash14sat/Desktop/zmq/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/zmq_sub.dir/src/subscriber.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/zmq_sub.dir/src/subscriber.cpp.o -c /home/kash14sat/Desktop/zmq/src/subscriber.cpp

CMakeFiles/zmq_sub.dir/src/subscriber.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/zmq_sub.dir/src/subscriber.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kash14sat/Desktop/zmq/src/subscriber.cpp > CMakeFiles/zmq_sub.dir/src/subscriber.cpp.i

CMakeFiles/zmq_sub.dir/src/subscriber.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/zmq_sub.dir/src/subscriber.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kash14sat/Desktop/zmq/src/subscriber.cpp -o CMakeFiles/zmq_sub.dir/src/subscriber.cpp.s

CMakeFiles/zmq_sub.dir/src/subscriber.cpp.o.requires:

.PHONY : CMakeFiles/zmq_sub.dir/src/subscriber.cpp.o.requires

CMakeFiles/zmq_sub.dir/src/subscriber.cpp.o.provides: CMakeFiles/zmq_sub.dir/src/subscriber.cpp.o.requires
	$(MAKE) -f CMakeFiles/zmq_sub.dir/build.make CMakeFiles/zmq_sub.dir/src/subscriber.cpp.o.provides.build
.PHONY : CMakeFiles/zmq_sub.dir/src/subscriber.cpp.o.provides

CMakeFiles/zmq_sub.dir/src/subscriber.cpp.o.provides.build: CMakeFiles/zmq_sub.dir/src/subscriber.cpp.o


# Object files for target zmq_sub
zmq_sub_OBJECTS = \
"CMakeFiles/zmq_sub.dir/src/subscriber.cpp.o"

# External object files for target zmq_sub
zmq_sub_EXTERNAL_OBJECTS =

zmq_sub: CMakeFiles/zmq_sub.dir/src/subscriber.cpp.o
zmq_sub: CMakeFiles/zmq_sub.dir/build.make
zmq_sub: /usr/lib/x86_64-linux-gnu/libzmq.so
zmq_sub: CMakeFiles/zmq_sub.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kash14sat/Desktop/zmq/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable zmq_sub"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/zmq_sub.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/zmq_sub.dir/build: zmq_sub

.PHONY : CMakeFiles/zmq_sub.dir/build

CMakeFiles/zmq_sub.dir/requires: CMakeFiles/zmq_sub.dir/src/subscriber.cpp.o.requires

.PHONY : CMakeFiles/zmq_sub.dir/requires

CMakeFiles/zmq_sub.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/zmq_sub.dir/cmake_clean.cmake
.PHONY : CMakeFiles/zmq_sub.dir/clean

CMakeFiles/zmq_sub.dir/depend:
	cd /home/kash14sat/Desktop/zmq/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kash14sat/Desktop/zmq /home/kash14sat/Desktop/zmq /home/kash14sat/Desktop/zmq/build /home/kash14sat/Desktop/zmq/build /home/kash14sat/Desktop/zmq/build/CMakeFiles/zmq_sub.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/zmq_sub.dir/depend

