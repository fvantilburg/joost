# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/floris/ros_groovy/joost

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/floris/ros_groovy/joost/build

# Include any dependencies generated for this target.
include CMakeFiles/joost_bin.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/joost_bin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/joost_bin.dir/flags.make

CMakeFiles/joost_bin.dir/src/joost.cpp.o: CMakeFiles/joost_bin.dir/flags.make
CMakeFiles/joost_bin.dir/src/joost.cpp.o: ../src/joost.cpp
CMakeFiles/joost_bin.dir/src/joost.cpp.o: ../manifest.xml
CMakeFiles/joost_bin.dir/src/joost.cpp.o: /opt/ros/groovy/share/cpp_common/package.xml
CMakeFiles/joost_bin.dir/src/joost.cpp.o: /opt/ros/groovy/share/rostime/package.xml
CMakeFiles/joost_bin.dir/src/joost.cpp.o: /opt/ros/groovy/share/roscpp_traits/package.xml
CMakeFiles/joost_bin.dir/src/joost.cpp.o: /opt/ros/groovy/share/roscpp_serialization/package.xml
CMakeFiles/joost_bin.dir/src/joost.cpp.o: /opt/ros/groovy/share/genmsg/package.xml
CMakeFiles/joost_bin.dir/src/joost.cpp.o: /opt/ros/groovy/share/genpy/package.xml
CMakeFiles/joost_bin.dir/src/joost.cpp.o: /opt/ros/groovy/share/message_runtime/package.xml
CMakeFiles/joost_bin.dir/src/joost.cpp.o: /opt/ros/groovy/share/rosconsole/package.xml
CMakeFiles/joost_bin.dir/src/joost.cpp.o: /opt/ros/groovy/share/std_msgs/package.xml
CMakeFiles/joost_bin.dir/src/joost.cpp.o: /opt/ros/groovy/share/rosgraph_msgs/package.xml
CMakeFiles/joost_bin.dir/src/joost.cpp.o: /opt/ros/groovy/share/xmlrpcpp/package.xml
CMakeFiles/joost_bin.dir/src/joost.cpp.o: /opt/ros/groovy/share/roscpp/package.xml
CMakeFiles/joost_bin.dir/src/joost.cpp.o: /home/floris/ros_groovy/dbl-ros-pkg-dev/dbl_repos/manifest.xml
CMakeFiles/joost_bin.dir/src/joost.cpp.o: /home/floris/ros_groovy/dbl-ros-pkg-dev/drivers/shared_serial/manifest.xml
CMakeFiles/joost_bin.dir/src/joost.cpp.o: /home/floris/ros_groovy/dbl-ros-pkg-dev/drivers/threemxl/manifest.xml
CMakeFiles/joost_bin.dir/src/joost.cpp.o: /home/floris/ros_groovy/dbl-ros-pkg-dev/drivers/shared_serial/msg_gen/generated
CMakeFiles/joost_bin.dir/src/joost.cpp.o: /home/floris/ros_groovy/dbl-ros-pkg-dev/drivers/shared_serial/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/floris/ros_groovy/joost/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/joost_bin.dir/src/joost.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/joost_bin.dir/src/joost.cpp.o -c /home/floris/ros_groovy/joost/src/joost.cpp

CMakeFiles/joost_bin.dir/src/joost.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joost_bin.dir/src/joost.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/floris/ros_groovy/joost/src/joost.cpp > CMakeFiles/joost_bin.dir/src/joost.cpp.i

CMakeFiles/joost_bin.dir/src/joost.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joost_bin.dir/src/joost.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/floris/ros_groovy/joost/src/joost.cpp -o CMakeFiles/joost_bin.dir/src/joost.cpp.s

CMakeFiles/joost_bin.dir/src/joost.cpp.o.requires:
.PHONY : CMakeFiles/joost_bin.dir/src/joost.cpp.o.requires

CMakeFiles/joost_bin.dir/src/joost.cpp.o.provides: CMakeFiles/joost_bin.dir/src/joost.cpp.o.requires
	$(MAKE) -f CMakeFiles/joost_bin.dir/build.make CMakeFiles/joost_bin.dir/src/joost.cpp.o.provides.build
.PHONY : CMakeFiles/joost_bin.dir/src/joost.cpp.o.provides

CMakeFiles/joost_bin.dir/src/joost.cpp.o.provides.build: CMakeFiles/joost_bin.dir/src/joost.cpp.o

CMakeFiles/joost_bin.dir/src/Arm.cpp.o: CMakeFiles/joost_bin.dir/flags.make
CMakeFiles/joost_bin.dir/src/Arm.cpp.o: ../src/Arm.cpp
CMakeFiles/joost_bin.dir/src/Arm.cpp.o: ../manifest.xml
CMakeFiles/joost_bin.dir/src/Arm.cpp.o: /opt/ros/groovy/share/cpp_common/package.xml
CMakeFiles/joost_bin.dir/src/Arm.cpp.o: /opt/ros/groovy/share/rostime/package.xml
CMakeFiles/joost_bin.dir/src/Arm.cpp.o: /opt/ros/groovy/share/roscpp_traits/package.xml
CMakeFiles/joost_bin.dir/src/Arm.cpp.o: /opt/ros/groovy/share/roscpp_serialization/package.xml
CMakeFiles/joost_bin.dir/src/Arm.cpp.o: /opt/ros/groovy/share/genmsg/package.xml
CMakeFiles/joost_bin.dir/src/Arm.cpp.o: /opt/ros/groovy/share/genpy/package.xml
CMakeFiles/joost_bin.dir/src/Arm.cpp.o: /opt/ros/groovy/share/message_runtime/package.xml
CMakeFiles/joost_bin.dir/src/Arm.cpp.o: /opt/ros/groovy/share/rosconsole/package.xml
CMakeFiles/joost_bin.dir/src/Arm.cpp.o: /opt/ros/groovy/share/std_msgs/package.xml
CMakeFiles/joost_bin.dir/src/Arm.cpp.o: /opt/ros/groovy/share/rosgraph_msgs/package.xml
CMakeFiles/joost_bin.dir/src/Arm.cpp.o: /opt/ros/groovy/share/xmlrpcpp/package.xml
CMakeFiles/joost_bin.dir/src/Arm.cpp.o: /opt/ros/groovy/share/roscpp/package.xml
CMakeFiles/joost_bin.dir/src/Arm.cpp.o: /home/floris/ros_groovy/dbl-ros-pkg-dev/dbl_repos/manifest.xml
CMakeFiles/joost_bin.dir/src/Arm.cpp.o: /home/floris/ros_groovy/dbl-ros-pkg-dev/drivers/shared_serial/manifest.xml
CMakeFiles/joost_bin.dir/src/Arm.cpp.o: /home/floris/ros_groovy/dbl-ros-pkg-dev/drivers/threemxl/manifest.xml
CMakeFiles/joost_bin.dir/src/Arm.cpp.o: /home/floris/ros_groovy/dbl-ros-pkg-dev/drivers/shared_serial/msg_gen/generated
CMakeFiles/joost_bin.dir/src/Arm.cpp.o: /home/floris/ros_groovy/dbl-ros-pkg-dev/drivers/shared_serial/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/floris/ros_groovy/joost/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/joost_bin.dir/src/Arm.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/joost_bin.dir/src/Arm.cpp.o -c /home/floris/ros_groovy/joost/src/Arm.cpp

CMakeFiles/joost_bin.dir/src/Arm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joost_bin.dir/src/Arm.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/floris/ros_groovy/joost/src/Arm.cpp > CMakeFiles/joost_bin.dir/src/Arm.cpp.i

CMakeFiles/joost_bin.dir/src/Arm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joost_bin.dir/src/Arm.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/floris/ros_groovy/joost/src/Arm.cpp -o CMakeFiles/joost_bin.dir/src/Arm.cpp.s

CMakeFiles/joost_bin.dir/src/Arm.cpp.o.requires:
.PHONY : CMakeFiles/joost_bin.dir/src/Arm.cpp.o.requires

CMakeFiles/joost_bin.dir/src/Arm.cpp.o.provides: CMakeFiles/joost_bin.dir/src/Arm.cpp.o.requires
	$(MAKE) -f CMakeFiles/joost_bin.dir/build.make CMakeFiles/joost_bin.dir/src/Arm.cpp.o.provides.build
.PHONY : CMakeFiles/joost_bin.dir/src/Arm.cpp.o.provides

CMakeFiles/joost_bin.dir/src/Arm.cpp.o.provides.build: CMakeFiles/joost_bin.dir/src/Arm.cpp.o

# Object files for target joost_bin
joost_bin_OBJECTS = \
"CMakeFiles/joost_bin.dir/src/joost.cpp.o" \
"CMakeFiles/joost_bin.dir/src/Arm.cpp.o"

# External object files for target joost_bin
joost_bin_EXTERNAL_OBJECTS =

../bin/joost_bin: CMakeFiles/joost_bin.dir/src/joost.cpp.o
../bin/joost_bin: CMakeFiles/joost_bin.dir/src/Arm.cpp.o
../bin/joost_bin: CMakeFiles/joost_bin.dir/build.make
../bin/joost_bin: CMakeFiles/joost_bin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/joost_bin"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/joost_bin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/joost_bin.dir/build: ../bin/joost_bin
.PHONY : CMakeFiles/joost_bin.dir/build

CMakeFiles/joost_bin.dir/requires: CMakeFiles/joost_bin.dir/src/joost.cpp.o.requires
CMakeFiles/joost_bin.dir/requires: CMakeFiles/joost_bin.dir/src/Arm.cpp.o.requires
.PHONY : CMakeFiles/joost_bin.dir/requires

CMakeFiles/joost_bin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/joost_bin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/joost_bin.dir/clean

CMakeFiles/joost_bin.dir/depend:
	cd /home/floris/ros_groovy/joost/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/floris/ros_groovy/joost /home/floris/ros_groovy/joost /home/floris/ros_groovy/joost/build /home/floris/ros_groovy/joost/build /home/floris/ros_groovy/joost/build/CMakeFiles/joost_bin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/joost_bin.dir/depend

