# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_COMMAND = /opt/clion-2019.2/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /opt/clion-2019.2/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/johannes/Devel/snmpc

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/johannes/Devel/snmpc/cmake-build-debug

# Include any dependencies generated for this target.
include ThirdParty/polympc/examples/CMakeFiles/kite_control_test.dir/depend.make

# Include the progress variables for this target.
include ThirdParty/polympc/examples/CMakeFiles/kite_control_test.dir/progress.make

# Include the compile flags for this target's objects.
include ThirdParty/polympc/examples/CMakeFiles/kite_control_test.dir/flags.make

ThirdParty/polympc/examples/CMakeFiles/kite_control_test.dir/kite_control_test.cpp.o: ThirdParty/polympc/examples/CMakeFiles/kite_control_test.dir/flags.make
ThirdParty/polympc/examples/CMakeFiles/kite_control_test.dir/kite_control_test.cpp.o: ../ThirdParty/polympc/examples/kite_control_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/johannes/Devel/snmpc/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object ThirdParty/polympc/examples/CMakeFiles/kite_control_test.dir/kite_control_test.cpp.o"
	cd /home/johannes/Devel/snmpc/cmake-build-debug/ThirdParty/polympc/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kite_control_test.dir/kite_control_test.cpp.o -c /home/johannes/Devel/snmpc/ThirdParty/polympc/examples/kite_control_test.cpp

ThirdParty/polympc/examples/CMakeFiles/kite_control_test.dir/kite_control_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kite_control_test.dir/kite_control_test.cpp.i"
	cd /home/johannes/Devel/snmpc/cmake-build-debug/ThirdParty/polympc/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/johannes/Devel/snmpc/ThirdParty/polympc/examples/kite_control_test.cpp > CMakeFiles/kite_control_test.dir/kite_control_test.cpp.i

ThirdParty/polympc/examples/CMakeFiles/kite_control_test.dir/kite_control_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kite_control_test.dir/kite_control_test.cpp.s"
	cd /home/johannes/Devel/snmpc/cmake-build-debug/ThirdParty/polympc/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/johannes/Devel/snmpc/ThirdParty/polympc/examples/kite_control_test.cpp -o CMakeFiles/kite_control_test.dir/kite_control_test.cpp.s

# Object files for target kite_control_test
kite_control_test_OBJECTS = \
"CMakeFiles/kite_control_test.dir/kite_control_test.cpp.o"

# External object files for target kite_control_test
kite_control_test_EXTERNAL_OBJECTS =

ThirdParty/polympc/examples/kite_control_test: ThirdParty/polympc/examples/CMakeFiles/kite_control_test.dir/kite_control_test.cpp.o
ThirdParty/polympc/examples/kite_control_test: ThirdParty/polympc/examples/CMakeFiles/kite_control_test.dir/build.make
ThirdParty/polympc/examples/kite_control_test: ThirdParty/polympc/examples/libkite.a
ThirdParty/polympc/examples/kite_control_test: ThirdParty/polympc/cmake/libpolymath.a
ThirdParty/polympc/examples/kite_control_test: /usr/local/lib/libcasadi.so.3.5
ThirdParty/polympc/examples/kite_control_test: /usr/local/lib/libcasadi.so
ThirdParty/polympc/examples/kite_control_test: ThirdParty/polympc/examples/CMakeFiles/kite_control_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/johannes/Devel/snmpc/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable kite_control_test"
	cd /home/johannes/Devel/snmpc/cmake-build-debug/ThirdParty/polympc/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kite_control_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ThirdParty/polympc/examples/CMakeFiles/kite_control_test.dir/build: ThirdParty/polympc/examples/kite_control_test

.PHONY : ThirdParty/polympc/examples/CMakeFiles/kite_control_test.dir/build

ThirdParty/polympc/examples/CMakeFiles/kite_control_test.dir/clean:
	cd /home/johannes/Devel/snmpc/cmake-build-debug/ThirdParty/polympc/examples && $(CMAKE_COMMAND) -P CMakeFiles/kite_control_test.dir/cmake_clean.cmake
.PHONY : ThirdParty/polympc/examples/CMakeFiles/kite_control_test.dir/clean

ThirdParty/polympc/examples/CMakeFiles/kite_control_test.dir/depend:
	cd /home/johannes/Devel/snmpc/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/johannes/Devel/snmpc /home/johannes/Devel/snmpc/ThirdParty/polympc/examples /home/johannes/Devel/snmpc/cmake-build-debug /home/johannes/Devel/snmpc/cmake-build-debug/ThirdParty/polympc/examples /home/johannes/Devel/snmpc/cmake-build-debug/ThirdParty/polympc/examples/CMakeFiles/kite_control_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ThirdParty/polympc/examples/CMakeFiles/kite_control_test.dir/depend

