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
CMAKE_SOURCE_DIR = /home/hafizh/rosreal/tutor/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hafizh/rosreal/tutor/build

# Include any dependencies generated for this target.
include rs/CMakeFiles/rs.dir/depend.make

# Include the progress variables for this target.
include rs/CMakeFiles/rs.dir/progress.make

# Include the compile flags for this target's objects.
include rs/CMakeFiles/rs.dir/flags.make

rs/CMakeFiles/rs.dir/src/test.cpp.o: rs/CMakeFiles/rs.dir/flags.make
rs/CMakeFiles/rs.dir/src/test.cpp.o: /home/hafizh/rosreal/tutor/src/rs/src/test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hafizh/rosreal/tutor/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object rs/CMakeFiles/rs.dir/src/test.cpp.o"
	cd /home/hafizh/rosreal/tutor/build/rs && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rs.dir/src/test.cpp.o -c /home/hafizh/rosreal/tutor/src/rs/src/test.cpp

rs/CMakeFiles/rs.dir/src/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rs.dir/src/test.cpp.i"
	cd /home/hafizh/rosreal/tutor/build/rs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hafizh/rosreal/tutor/src/rs/src/test.cpp > CMakeFiles/rs.dir/src/test.cpp.i

rs/CMakeFiles/rs.dir/src/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rs.dir/src/test.cpp.s"
	cd /home/hafizh/rosreal/tutor/build/rs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hafizh/rosreal/tutor/src/rs/src/test.cpp -o CMakeFiles/rs.dir/src/test.cpp.s

# Object files for target rs
rs_OBJECTS = \
"CMakeFiles/rs.dir/src/test.cpp.o"

# External object files for target rs
rs_EXTERNAL_OBJECTS =

/home/hafizh/rosreal/tutor/devel/lib/librs.so: rs/CMakeFiles/rs.dir/src/test.cpp.o
/home/hafizh/rosreal/tutor/devel/lib/librs.so: rs/CMakeFiles/rs.dir/build.make
/home/hafizh/rosreal/tutor/devel/lib/librs.so: rs/CMakeFiles/rs.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hafizh/rosreal/tutor/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/hafizh/rosreal/tutor/devel/lib/librs.so"
	cd /home/hafizh/rosreal/tutor/build/rs && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rs.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
rs/CMakeFiles/rs.dir/build: /home/hafizh/rosreal/tutor/devel/lib/librs.so

.PHONY : rs/CMakeFiles/rs.dir/build

rs/CMakeFiles/rs.dir/clean:
	cd /home/hafizh/rosreal/tutor/build/rs && $(CMAKE_COMMAND) -P CMakeFiles/rs.dir/cmake_clean.cmake
.PHONY : rs/CMakeFiles/rs.dir/clean

rs/CMakeFiles/rs.dir/depend:
	cd /home/hafizh/rosreal/tutor/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hafizh/rosreal/tutor/src /home/hafizh/rosreal/tutor/src/rs /home/hafizh/rosreal/tutor/build /home/hafizh/rosreal/tutor/build/rs /home/hafizh/rosreal/tutor/build/rs/CMakeFiles/rs.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : rs/CMakeFiles/rs.dir/depend

