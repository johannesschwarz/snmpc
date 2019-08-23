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
CMAKE_SOURCE_DIR = /home/johannes/Devel/johannes_mpc

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/johannes/Devel/johannes_mpc/build

# Include any dependencies generated for this target.
include ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/depend.make

# Include the progress variables for this target.
include ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/progress.make

# Include the compile flags for this target's objects.
include ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/flags.make

ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/polymath.cpp.o: ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/flags.make
ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/polymath.cpp.o: ../ThirdParty/polympc/src/polymath.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/johannes/Devel/johannes_mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/polymath.cpp.o"
	cd /home/johannes/Devel/johannes_mpc/build/ThirdParty/polympc/cmake && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/polymath.dir/polymath.cpp.o -c /home/johannes/Devel/johannes_mpc/ThirdParty/polympc/src/polymath.cpp

ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/polymath.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/polymath.dir/polymath.cpp.i"
	cd /home/johannes/Devel/johannes_mpc/build/ThirdParty/polympc/cmake && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/johannes/Devel/johannes_mpc/ThirdParty/polympc/src/polymath.cpp > CMakeFiles/polymath.dir/polymath.cpp.i

ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/polymath.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/polymath.dir/polymath.cpp.s"
	cd /home/johannes/Devel/johannes_mpc/build/ThirdParty/polympc/cmake && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/johannes/Devel/johannes_mpc/ThirdParty/polympc/src/polymath.cpp -o CMakeFiles/polymath.dir/polymath.cpp.s

ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/polymath.cpp.o.requires:

.PHONY : ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/polymath.cpp.o.requires

ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/polymath.cpp.o.provides: ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/polymath.cpp.o.requires
	$(MAKE) -f ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/build.make ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/polymath.cpp.o.provides.build
.PHONY : ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/polymath.cpp.o.provides

ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/polymath.cpp.o.provides.build: ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/polymath.cpp.o


# Object files for target polymath
polymath_OBJECTS = \
"CMakeFiles/polymath.dir/polymath.cpp.o"

# External object files for target polymath
polymath_EXTERNAL_OBJECTS =

ThirdParty/polympc/cmake/libpolymath.a: ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/polymath.cpp.o
ThirdParty/polympc/cmake/libpolymath.a: ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/build.make
ThirdParty/polympc/cmake/libpolymath.a: ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/johannes/Devel/johannes_mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libpolymath.a"
	cd /home/johannes/Devel/johannes_mpc/build/ThirdParty/polympc/cmake && $(CMAKE_COMMAND) -P CMakeFiles/polymath.dir/cmake_clean_target.cmake
	cd /home/johannes/Devel/johannes_mpc/build/ThirdParty/polympc/cmake && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/polymath.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/build: ThirdParty/polympc/cmake/libpolymath.a

.PHONY : ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/build

ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/requires: ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/polymath.cpp.o.requires

.PHONY : ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/requires

ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/clean:
	cd /home/johannes/Devel/johannes_mpc/build/ThirdParty/polympc/cmake && $(CMAKE_COMMAND) -P CMakeFiles/polymath.dir/cmake_clean.cmake
.PHONY : ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/clean

ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/depend:
	cd /home/johannes/Devel/johannes_mpc/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/johannes/Devel/johannes_mpc /home/johannes/Devel/johannes_mpc/ThirdParty/polympc/src /home/johannes/Devel/johannes_mpc/build /home/johannes/Devel/johannes_mpc/build/ThirdParty/polympc/cmake /home/johannes/Devel/johannes_mpc/build/ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ThirdParty/polympc/cmake/CMakeFiles/polymath.dir/depend

