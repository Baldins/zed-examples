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
CMAKE_SOURCE_DIR = "/home/fbaldini/Desktop/robot_perception/tutorials/tutorial 6 - object detection/cpp"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/fbaldini/Desktop/robot_perception/tutorials/tutorial 6 - object detection/cpp/build"

# Include any dependencies generated for this target.
include CMakeFiles/ZED_Tutorial_6.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ZED_Tutorial_6.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ZED_Tutorial_6.dir/flags.make

CMakeFiles/ZED_Tutorial_6.dir/main.o: CMakeFiles/ZED_Tutorial_6.dir/flags.make
CMakeFiles/ZED_Tutorial_6.dir/main.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/fbaldini/Desktop/robot_perception/tutorials/tutorial 6 - object detection/cpp/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ZED_Tutorial_6.dir/main.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ZED_Tutorial_6.dir/main.o -c "/home/fbaldini/Desktop/robot_perception/tutorials/tutorial 6 - object detection/cpp/main.cpp"

CMakeFiles/ZED_Tutorial_6.dir/main.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ZED_Tutorial_6.dir/main.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/fbaldini/Desktop/robot_perception/tutorials/tutorial 6 - object detection/cpp/main.cpp" > CMakeFiles/ZED_Tutorial_6.dir/main.i

CMakeFiles/ZED_Tutorial_6.dir/main.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ZED_Tutorial_6.dir/main.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/fbaldini/Desktop/robot_perception/tutorials/tutorial 6 - object detection/cpp/main.cpp" -o CMakeFiles/ZED_Tutorial_6.dir/main.s

# Object files for target ZED_Tutorial_6
ZED_Tutorial_6_OBJECTS = \
"CMakeFiles/ZED_Tutorial_6.dir/main.o"

# External object files for target ZED_Tutorial_6
ZED_Tutorial_6_EXTERNAL_OBJECTS =

ZED_Tutorial_6: CMakeFiles/ZED_Tutorial_6.dir/main.o
ZED_Tutorial_6: CMakeFiles/ZED_Tutorial_6.dir/build.make
ZED_Tutorial_6: /usr/local/zed/lib/libsl_zed.so
ZED_Tutorial_6: /usr/lib/x86_64-linux-gnu/libopenblas.so
ZED_Tutorial_6: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
ZED_Tutorial_6: /usr/lib/x86_64-linux-gnu/libcuda.so
ZED_Tutorial_6: /usr/local/cuda-11.0/lib64/libcudart.so
ZED_Tutorial_6: CMakeFiles/ZED_Tutorial_6.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/fbaldini/Desktop/robot_perception/tutorials/tutorial 6 - object detection/cpp/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ZED_Tutorial_6"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ZED_Tutorial_6.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ZED_Tutorial_6.dir/build: ZED_Tutorial_6

.PHONY : CMakeFiles/ZED_Tutorial_6.dir/build

CMakeFiles/ZED_Tutorial_6.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ZED_Tutorial_6.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ZED_Tutorial_6.dir/clean

CMakeFiles/ZED_Tutorial_6.dir/depend:
	cd "/home/fbaldini/Desktop/robot_perception/tutorials/tutorial 6 - object detection/cpp/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/fbaldini/Desktop/robot_perception/tutorials/tutorial 6 - object detection/cpp" "/home/fbaldini/Desktop/robot_perception/tutorials/tutorial 6 - object detection/cpp" "/home/fbaldini/Desktop/robot_perception/tutorials/tutorial 6 - object detection/cpp/build" "/home/fbaldini/Desktop/robot_perception/tutorials/tutorial 6 - object detection/cpp/build" "/home/fbaldini/Desktop/robot_perception/tutorials/tutorial 6 - object detection/cpp/build/CMakeFiles/ZED_Tutorial_6.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/ZED_Tutorial_6.dir/depend

