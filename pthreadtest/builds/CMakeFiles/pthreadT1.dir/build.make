# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = "/home/bmbed/Documents/work spade/pthreadtest"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/bmbed/Documents/work spade/pthreadtest/builds"

# Include any dependencies generated for this target.
include CMakeFiles/pthreadT1.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pthreadT1.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pthreadT1.dir/flags.make

CMakeFiles/pthreadT1.dir/pthreadT1.cpp.o: CMakeFiles/pthreadT1.dir/flags.make
CMakeFiles/pthreadT1.dir/pthreadT1.cpp.o: ../pthreadT1.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/bmbed/Documents/work spade/pthreadtest/builds/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/pthreadT1.dir/pthreadT1.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pthreadT1.dir/pthreadT1.cpp.o -c "/home/bmbed/Documents/work spade/pthreadtest/pthreadT1.cpp"

CMakeFiles/pthreadT1.dir/pthreadT1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pthreadT1.dir/pthreadT1.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/bmbed/Documents/work spade/pthreadtest/pthreadT1.cpp" > CMakeFiles/pthreadT1.dir/pthreadT1.cpp.i

CMakeFiles/pthreadT1.dir/pthreadT1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pthreadT1.dir/pthreadT1.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/bmbed/Documents/work spade/pthreadtest/pthreadT1.cpp" -o CMakeFiles/pthreadT1.dir/pthreadT1.cpp.s

CMakeFiles/pthreadT1.dir/pthreadT1.cpp.o.requires:

.PHONY : CMakeFiles/pthreadT1.dir/pthreadT1.cpp.o.requires

CMakeFiles/pthreadT1.dir/pthreadT1.cpp.o.provides: CMakeFiles/pthreadT1.dir/pthreadT1.cpp.o.requires
	$(MAKE) -f CMakeFiles/pthreadT1.dir/build.make CMakeFiles/pthreadT1.dir/pthreadT1.cpp.o.provides.build
.PHONY : CMakeFiles/pthreadT1.dir/pthreadT1.cpp.o.provides

CMakeFiles/pthreadT1.dir/pthreadT1.cpp.o.provides.build: CMakeFiles/pthreadT1.dir/pthreadT1.cpp.o


# Object files for target pthreadT1
pthreadT1_OBJECTS = \
"CMakeFiles/pthreadT1.dir/pthreadT1.cpp.o"

# External object files for target pthreadT1
pthreadT1_EXTERNAL_OBJECTS =

pthreadT1: CMakeFiles/pthreadT1.dir/pthreadT1.cpp.o
pthreadT1: CMakeFiles/pthreadT1.dir/build.make
pthreadT1: CMakeFiles/pthreadT1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/bmbed/Documents/work spade/pthreadtest/builds/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable pthreadT1"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pthreadT1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pthreadT1.dir/build: pthreadT1

.PHONY : CMakeFiles/pthreadT1.dir/build

CMakeFiles/pthreadT1.dir/requires: CMakeFiles/pthreadT1.dir/pthreadT1.cpp.o.requires

.PHONY : CMakeFiles/pthreadT1.dir/requires

CMakeFiles/pthreadT1.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pthreadT1.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pthreadT1.dir/clean

CMakeFiles/pthreadT1.dir/depend:
	cd "/home/bmbed/Documents/work spade/pthreadtest/builds" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/bmbed/Documents/work spade/pthreadtest" "/home/bmbed/Documents/work spade/pthreadtest" "/home/bmbed/Documents/work spade/pthreadtest/builds" "/home/bmbed/Documents/work spade/pthreadtest/builds" "/home/bmbed/Documents/work spade/pthreadtest/builds/CMakeFiles/pthreadT1.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/pthreadT1.dir/depend

