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
CMAKE_COMMAND = /opt/local/bin/cmake

# The command to remove a file.
RM = /opt/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /opt/local/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts"

# Include any dependencies generated for this target.
include common/CMakeFiles/heighttool.dir/depend.make

# Include the progress variables for this target.
include common/CMakeFiles/heighttool.dir/progress.make

# Include the compile flags for this target's objects.
include common/CMakeFiles/heighttool.dir/flags.make

common/CMakeFiles/heighttool.dir/heighttool.cpp.o: common/CMakeFiles/heighttool.dir/flags.make
common/CMakeFiles/heighttool.dir/heighttool.cpp.o: common/heighttool.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts/CMakeFiles" $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object common/CMakeFiles/heighttool.dir/heighttool.cpp.o"
	cd "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts/common" && /opt/local/bin/gcc   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/heighttool.dir/heighttool.cpp.o -c "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts/common/heighttool.cpp"

common/CMakeFiles/heighttool.dir/heighttool.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/heighttool.dir/heighttool.cpp.i"
	cd "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts/common" && /opt/local/bin/gcc  $(CXX_DEFINES) $(CXX_FLAGS) -E "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts/common/heighttool.cpp" > CMakeFiles/heighttool.dir/heighttool.cpp.i

common/CMakeFiles/heighttool.dir/heighttool.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/heighttool.dir/heighttool.cpp.s"
	cd "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts/common" && /opt/local/bin/gcc  $(CXX_DEFINES) $(CXX_FLAGS) -S "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts/common/heighttool.cpp" -o CMakeFiles/heighttool.dir/heighttool.cpp.s

common/CMakeFiles/heighttool.dir/heighttool.cpp.o.requires:
.PHONY : common/CMakeFiles/heighttool.dir/heighttool.cpp.o.requires

common/CMakeFiles/heighttool.dir/heighttool.cpp.o.provides: common/CMakeFiles/heighttool.dir/heighttool.cpp.o.requires
	$(MAKE) -f common/CMakeFiles/heighttool.dir/build.make common/CMakeFiles/heighttool.dir/heighttool.cpp.o.provides.build
.PHONY : common/CMakeFiles/heighttool.dir/heighttool.cpp.o.provides

common/CMakeFiles/heighttool.dir/heighttool.cpp.o.provides.build: common/CMakeFiles/heighttool.dir/heighttool.cpp.o

# Object files for target heighttool
heighttool_OBJECTS = \
"CMakeFiles/heighttool.dir/heighttool.cpp.o"

# External object files for target heighttool
heighttool_EXTERNAL_OBJECTS =

common/heighttool: common/CMakeFiles/heighttool.dir/heighttool.cpp.o
common/heighttool: common/CMakeFiles/heighttool.dir/build.make
common/heighttool: common/libcommon.a
common/heighttool: common/libcommon_rgba.a
common/heighttool: /opt/local/lib/libboost_program_options-mt.dylib
common/heighttool: common/libcommon.a
common/heighttool: /opt/local/lib/libboost_serialization-mt.dylib
common/heighttool: common/CMakeFiles/heighttool.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable heighttool"
	cd "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts/common" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/heighttool.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
common/CMakeFiles/heighttool.dir/build: common/heighttool
.PHONY : common/CMakeFiles/heighttool.dir/build

common/CMakeFiles/heighttool.dir/requires: common/CMakeFiles/heighttool.dir/heighttool.cpp.o.requires
.PHONY : common/CMakeFiles/heighttool.dir/requires

common/CMakeFiles/heighttool.dir/clean:
	cd "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts/common" && $(CMAKE_COMMAND) -P CMakeFiles/heighttool.dir/cmake_clean.cmake
.PHONY : common/CMakeFiles/heighttool.dir/clean

common/CMakeFiles/heighttool.dir/depend:
	cd "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts" "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts/common" "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts" "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts/common" "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts/common/CMakeFiles/heighttool.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : common/CMakeFiles/heighttool.dir/depend
