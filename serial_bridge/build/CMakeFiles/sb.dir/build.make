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
CMAKE_SOURCE_DIR = "/home/kash14sat/Desktop/ROS Stuff/workspace/litefw/serial_bridge"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/kash14sat/Desktop/ROS Stuff/workspace/litefw/serial_bridge/build"

# Include any dependencies generated for this target.
include CMakeFiles/sb.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sb.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sb.dir/flags.make

CMakeFiles/sb.dir/src/serial_create.cpp.o: CMakeFiles/sb.dir/flags.make
CMakeFiles/sb.dir/src/serial_create.cpp.o: ../src/serial_create.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/kash14sat/Desktop/ROS Stuff/workspace/litefw/serial_bridge/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sb.dir/src/serial_create.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sb.dir/src/serial_create.cpp.o -c "/home/kash14sat/Desktop/ROS Stuff/workspace/litefw/serial_bridge/src/serial_create.cpp"

CMakeFiles/sb.dir/src/serial_create.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sb.dir/src/serial_create.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/kash14sat/Desktop/ROS Stuff/workspace/litefw/serial_bridge/src/serial_create.cpp" > CMakeFiles/sb.dir/src/serial_create.cpp.i

CMakeFiles/sb.dir/src/serial_create.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sb.dir/src/serial_create.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/kash14sat/Desktop/ROS Stuff/workspace/litefw/serial_bridge/src/serial_create.cpp" -o CMakeFiles/sb.dir/src/serial_create.cpp.s

CMakeFiles/sb.dir/src/serial_create.cpp.o.requires:

.PHONY : CMakeFiles/sb.dir/src/serial_create.cpp.o.requires

CMakeFiles/sb.dir/src/serial_create.cpp.o.provides: CMakeFiles/sb.dir/src/serial_create.cpp.o.requires
	$(MAKE) -f CMakeFiles/sb.dir/build.make CMakeFiles/sb.dir/src/serial_create.cpp.o.provides.build
.PHONY : CMakeFiles/sb.dir/src/serial_create.cpp.o.provides

CMakeFiles/sb.dir/src/serial_create.cpp.o.provides.build: CMakeFiles/sb.dir/src/serial_create.cpp.o


CMakeFiles/sb.dir/src/SerialPort.cpp.o: CMakeFiles/sb.dir/flags.make
CMakeFiles/sb.dir/src/SerialPort.cpp.o: ../src/SerialPort.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/kash14sat/Desktop/ROS Stuff/workspace/litefw/serial_bridge/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/sb.dir/src/SerialPort.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sb.dir/src/SerialPort.cpp.o -c "/home/kash14sat/Desktop/ROS Stuff/workspace/litefw/serial_bridge/src/SerialPort.cpp"

CMakeFiles/sb.dir/src/SerialPort.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sb.dir/src/SerialPort.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/kash14sat/Desktop/ROS Stuff/workspace/litefw/serial_bridge/src/SerialPort.cpp" > CMakeFiles/sb.dir/src/SerialPort.cpp.i

CMakeFiles/sb.dir/src/SerialPort.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sb.dir/src/SerialPort.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/kash14sat/Desktop/ROS Stuff/workspace/litefw/serial_bridge/src/SerialPort.cpp" -o CMakeFiles/sb.dir/src/SerialPort.cpp.s

CMakeFiles/sb.dir/src/SerialPort.cpp.o.requires:

.PHONY : CMakeFiles/sb.dir/src/SerialPort.cpp.o.requires

CMakeFiles/sb.dir/src/SerialPort.cpp.o.provides: CMakeFiles/sb.dir/src/SerialPort.cpp.o.requires
	$(MAKE) -f CMakeFiles/sb.dir/build.make CMakeFiles/sb.dir/src/SerialPort.cpp.o.provides.build
.PHONY : CMakeFiles/sb.dir/src/SerialPort.cpp.o.provides

CMakeFiles/sb.dir/src/SerialPort.cpp.o.provides.build: CMakeFiles/sb.dir/src/SerialPort.cpp.o


# Object files for target sb
sb_OBJECTS = \
"CMakeFiles/sb.dir/src/serial_create.cpp.o" \
"CMakeFiles/sb.dir/src/SerialPort.cpp.o"

# External object files for target sb
sb_EXTERNAL_OBJECTS =

sb: CMakeFiles/sb.dir/src/serial_create.cpp.o
sb: CMakeFiles/sb.dir/src/SerialPort.cpp.o
sb: CMakeFiles/sb.dir/build.make
sb: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
sb: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
sb: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
sb: /usr/lib/x86_64-linux-gnu/libboost_regex.so
sb: /usr/lib/x86_64-linux-gnu/libboost_system.so
sb: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
sb: /usr/lib/x86_64-linux-gnu/libboost_thread.so
sb: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
sb: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
sb: /usr/lib/x86_64-linux-gnu/libpthread.so
sb: /usr/lib/x86_64-linux-gnu/libzmq.so
sb: CMakeFiles/sb.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/kash14sat/Desktop/ROS Stuff/workspace/litefw/serial_bridge/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable sb"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sb.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sb.dir/build: sb

.PHONY : CMakeFiles/sb.dir/build

CMakeFiles/sb.dir/requires: CMakeFiles/sb.dir/src/serial_create.cpp.o.requires
CMakeFiles/sb.dir/requires: CMakeFiles/sb.dir/src/SerialPort.cpp.o.requires

.PHONY : CMakeFiles/sb.dir/requires

CMakeFiles/sb.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sb.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sb.dir/clean

CMakeFiles/sb.dir/depend:
	cd "/home/kash14sat/Desktop/ROS Stuff/workspace/litefw/serial_bridge/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/kash14sat/Desktop/ROS Stuff/workspace/litefw/serial_bridge" "/home/kash14sat/Desktop/ROS Stuff/workspace/litefw/serial_bridge" "/home/kash14sat/Desktop/ROS Stuff/workspace/litefw/serial_bridge/build" "/home/kash14sat/Desktop/ROS Stuff/workspace/litefw/serial_bridge/build" "/home/kash14sat/Desktop/ROS Stuff/workspace/litefw/serial_bridge/build/CMakeFiles/sb.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/sb.dir/depend

