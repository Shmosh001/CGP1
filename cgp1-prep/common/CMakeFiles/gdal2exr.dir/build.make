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
include common/CMakeFiles/gdal2exr.dir/depend.make

# Include the progress variables for this target.
include common/CMakeFiles/gdal2exr.dir/progress.make

# Include the compile flags for this target's objects.
include common/CMakeFiles/gdal2exr.dir/flags.make

common/CMakeFiles/gdal2exr.dir/gdal2exr.cpp.o: common/CMakeFiles/gdal2exr.dir/flags.make
common/CMakeFiles/gdal2exr.dir/gdal2exr.cpp.o: common/gdal2exr.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts/CMakeFiles" $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object common/CMakeFiles/gdal2exr.dir/gdal2exr.cpp.o"
	cd "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts/common" && /opt/local/bin/gcc   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/gdal2exr.dir/gdal2exr.cpp.o -c "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts/common/gdal2exr.cpp"

common/CMakeFiles/gdal2exr.dir/gdal2exr.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gdal2exr.dir/gdal2exr.cpp.i"
	cd "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts/common" && /opt/local/bin/gcc  $(CXX_DEFINES) $(CXX_FLAGS) -E "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts/common/gdal2exr.cpp" > CMakeFiles/gdal2exr.dir/gdal2exr.cpp.i

common/CMakeFiles/gdal2exr.dir/gdal2exr.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gdal2exr.dir/gdal2exr.cpp.s"
	cd "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts/common" && /opt/local/bin/gcc  $(CXX_DEFINES) $(CXX_FLAGS) -S "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts/common/gdal2exr.cpp" -o CMakeFiles/gdal2exr.dir/gdal2exr.cpp.s

common/CMakeFiles/gdal2exr.dir/gdal2exr.cpp.o.requires:
.PHONY : common/CMakeFiles/gdal2exr.dir/gdal2exr.cpp.o.requires

common/CMakeFiles/gdal2exr.dir/gdal2exr.cpp.o.provides: common/CMakeFiles/gdal2exr.dir/gdal2exr.cpp.o.requires
	$(MAKE) -f common/CMakeFiles/gdal2exr.dir/build.make common/CMakeFiles/gdal2exr.dir/gdal2exr.cpp.o.provides.build
.PHONY : common/CMakeFiles/gdal2exr.dir/gdal2exr.cpp.o.provides

common/CMakeFiles/gdal2exr.dir/gdal2exr.cpp.o.provides.build: common/CMakeFiles/gdal2exr.dir/gdal2exr.cpp.o

# Object files for target gdal2exr
gdal2exr_OBJECTS = \
"CMakeFiles/gdal2exr.dir/gdal2exr.cpp.o"

# External object files for target gdal2exr
gdal2exr_EXTERNAL_OBJECTS =

common/gdal2exr: common/CMakeFiles/gdal2exr.dir/gdal2exr.cpp.o
common/gdal2exr: common/CMakeFiles/gdal2exr.dir/build.make
common/gdal2exr: common/libcommon.a
common/gdal2exr: /opt/local/lib/libboost_program_options-mt.dylib
common/gdal2exr: /usr/local/Cellar/gdal/1.10.1/lib/libgdal.dylib
common/gdal2exr: /opt/local/lib/libboost_serialization-mt.dylib
common/gdal2exr: common/CMakeFiles/gdal2exr.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable gdal2exr"
	cd "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts/common" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gdal2exr.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
common/CMakeFiles/gdal2exr.dir/build: common/gdal2exr
.PHONY : common/CMakeFiles/gdal2exr.dir/build

common/CMakeFiles/gdal2exr.dir/requires: common/CMakeFiles/gdal2exr.dir/gdal2exr.cpp.o.requires
.PHONY : common/CMakeFiles/gdal2exr.dir/requires

common/CMakeFiles/gdal2exr.dir/clean:
	cd "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts/common" && $(CMAKE_COMMAND) -P CMakeFiles/gdal2exr.dir/cmake_clean.cmake
.PHONY : common/CMakeFiles/gdal2exr.dir/clean

common/CMakeFiles/gdal2exr.dir/depend:
	cd "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts" "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts/common" "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts" "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts/common" "/Volumes/1Tb Macintosh HD/Users/Jed/Desktop/PROGRAMMING/uts/common/CMakeFiles/gdal2exr.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : common/CMakeFiles/gdal2exr.dir/depend
