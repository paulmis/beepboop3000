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
CMAKE_SOURCE_DIR = /home/jesuschrist/projects/beepboop3000/ros2_ws/src/bb3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jesuschrist/projects/beepboop3000/ros2_ws/build/bb3

# Include any dependencies generated for this target.
include CMakeFiles/bb3.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/bb3.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/bb3.dir/flags.make

CMakeFiles/bb3.dir/src/main.cpp.o: CMakeFiles/bb3.dir/flags.make
CMakeFiles/bb3.dir/src/main.cpp.o: /home/jesuschrist/projects/beepboop3000/ros2_ws/src/bb3/src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jesuschrist/projects/beepboop3000/ros2_ws/build/bb3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/bb3.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bb3.dir/src/main.cpp.o -c /home/jesuschrist/projects/beepboop3000/ros2_ws/src/bb3/src/main.cpp

CMakeFiles/bb3.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bb3.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jesuschrist/projects/beepboop3000/ros2_ws/src/bb3/src/main.cpp > CMakeFiles/bb3.dir/src/main.cpp.i

CMakeFiles/bb3.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bb3.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jesuschrist/projects/beepboop3000/ros2_ws/src/bb3/src/main.cpp -o CMakeFiles/bb3.dir/src/main.cpp.s

# Object files for target bb3
bb3_OBJECTS = \
"CMakeFiles/bb3.dir/src/main.cpp.o"

# External object files for target bb3
bb3_EXTERNAL_OBJECTS =

bb3: CMakeFiles/bb3.dir/src/main.cpp.o
bb3: CMakeFiles/bb3.dir/build.make
bb3: /opt/ros/foxy/lib/librclcpp.so
bb3: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
bb3: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_c.so
bb3: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
bb3: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
bb3: /opt/ros/foxy/lib/liblibstatistics_collector.so
bb3: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_c.so
bb3: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_generator_c.so
bb3: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_c.so
bb3: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_cpp.so
bb3: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_cpp.so
bb3: /opt/ros/foxy/lib/librcl.so
bb3: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
bb3: /opt/ros/foxy/lib/librcl_interfaces__rosidl_generator_c.so
bb3: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_c.so
bb3: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
bb3: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_cpp.so
bb3: /opt/ros/foxy/lib/librmw_implementation.so
bb3: /opt/ros/foxy/lib/librmw.so
bb3: /opt/ros/foxy/lib/librcl_logging_spdlog.so
bb3: /usr/lib/x86_64-linux-gnu/libspdlog.so.1.5.0
bb3: /opt/ros/foxy/lib/librcl_yaml_param_parser.so
bb3: /opt/ros/foxy/lib/libyaml.so
bb3: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
bb3: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_generator_c.so
bb3: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_c.so
bb3: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
bb3: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
bb3: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
bb3: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_generator_c.so
bb3: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_c.so
bb3: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
bb3: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
bb3: /opt/ros/foxy/lib/libtracetools.so
bb3: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_generator_c.so
bb3: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
bb3: /opt/ros/foxy/lib/libstd_msgs__rosidl_generator_c.so
bb3: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_c.so
bb3: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
bb3: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_cpp.so
bb3: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
bb3: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
bb3: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
bb3: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
bb3: /opt/ros/foxy/lib/librosidl_typesupport_introspection_cpp.so
bb3: /opt/ros/foxy/lib/librosidl_typesupport_introspection_c.so
bb3: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
bb3: /opt/ros/foxy/lib/librosidl_typesupport_cpp.so
bb3: /opt/ros/foxy/lib/librosidl_typesupport_c.so
bb3: /opt/ros/foxy/lib/librcpputils.so
bb3: /opt/ros/foxy/lib/librosidl_runtime_c.so
bb3: /opt/ros/foxy/lib/librcutils.so
bb3: CMakeFiles/bb3.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jesuschrist/projects/beepboop3000/ros2_ws/build/bb3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bb3"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/bb3.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/bb3.dir/build: bb3

.PHONY : CMakeFiles/bb3.dir/build

CMakeFiles/bb3.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/bb3.dir/cmake_clean.cmake
.PHONY : CMakeFiles/bb3.dir/clean

CMakeFiles/bb3.dir/depend:
	cd /home/jesuschrist/projects/beepboop3000/ros2_ws/build/bb3 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jesuschrist/projects/beepboop3000/ros2_ws/src/bb3 /home/jesuschrist/projects/beepboop3000/ros2_ws/src/bb3 /home/jesuschrist/projects/beepboop3000/ros2_ws/build/bb3 /home/jesuschrist/projects/beepboop3000/ros2_ws/build/bb3 /home/jesuschrist/projects/beepboop3000/ros2_ws/build/bb3/CMakeFiles/bb3.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/bb3.dir/depend

