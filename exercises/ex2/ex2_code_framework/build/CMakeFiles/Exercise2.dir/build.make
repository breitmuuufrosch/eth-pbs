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

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ax/master/2017_01/PhysicallyBasedSimulation/exercises/ex2/ex2_code_framework

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ax/master/2017_01/PhysicallyBasedSimulation/exercises/ex2/ex2_code_framework/build

# Include any dependencies generated for this target.
include CMakeFiles/Exercise2.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Exercise2.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Exercise2.dir/flags.make

CMakeFiles/Exercise2.dir/SimpleFEM.cpp.o: CMakeFiles/Exercise2.dir/flags.make
CMakeFiles/Exercise2.dir/SimpleFEM.cpp.o: ../SimpleFEM.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ax/master/2017_01/PhysicallyBasedSimulation/exercises/ex2/ex2_code_framework/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Exercise2.dir/SimpleFEM.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Exercise2.dir/SimpleFEM.cpp.o -c /home/ax/master/2017_01/PhysicallyBasedSimulation/exercises/ex2/ex2_code_framework/SimpleFEM.cpp

CMakeFiles/Exercise2.dir/SimpleFEM.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Exercise2.dir/SimpleFEM.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ax/master/2017_01/PhysicallyBasedSimulation/exercises/ex2/ex2_code_framework/SimpleFEM.cpp > CMakeFiles/Exercise2.dir/SimpleFEM.cpp.i

CMakeFiles/Exercise2.dir/SimpleFEM.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Exercise2.dir/SimpleFEM.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ax/master/2017_01/PhysicallyBasedSimulation/exercises/ex2/ex2_code_framework/SimpleFEM.cpp -o CMakeFiles/Exercise2.dir/SimpleFEM.cpp.s

CMakeFiles/Exercise2.dir/SimpleFEM.cpp.o.requires:
.PHONY : CMakeFiles/Exercise2.dir/SimpleFEM.cpp.o.requires

CMakeFiles/Exercise2.dir/SimpleFEM.cpp.o.provides: CMakeFiles/Exercise2.dir/SimpleFEM.cpp.o.requires
	$(MAKE) -f CMakeFiles/Exercise2.dir/build.make CMakeFiles/Exercise2.dir/SimpleFEM.cpp.o.provides.build
.PHONY : CMakeFiles/Exercise2.dir/SimpleFEM.cpp.o.provides

CMakeFiles/Exercise2.dir/SimpleFEM.cpp.o.provides.build: CMakeFiles/Exercise2.dir/SimpleFEM.cpp.o

CMakeFiles/Exercise2.dir/MeshViewer.cpp.o: CMakeFiles/Exercise2.dir/flags.make
CMakeFiles/Exercise2.dir/MeshViewer.cpp.o: ../MeshViewer.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ax/master/2017_01/PhysicallyBasedSimulation/exercises/ex2/ex2_code_framework/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Exercise2.dir/MeshViewer.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Exercise2.dir/MeshViewer.cpp.o -c /home/ax/master/2017_01/PhysicallyBasedSimulation/exercises/ex2/ex2_code_framework/MeshViewer.cpp

CMakeFiles/Exercise2.dir/MeshViewer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Exercise2.dir/MeshViewer.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ax/master/2017_01/PhysicallyBasedSimulation/exercises/ex2/ex2_code_framework/MeshViewer.cpp > CMakeFiles/Exercise2.dir/MeshViewer.cpp.i

CMakeFiles/Exercise2.dir/MeshViewer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Exercise2.dir/MeshViewer.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ax/master/2017_01/PhysicallyBasedSimulation/exercises/ex2/ex2_code_framework/MeshViewer.cpp -o CMakeFiles/Exercise2.dir/MeshViewer.cpp.s

CMakeFiles/Exercise2.dir/MeshViewer.cpp.o.requires:
.PHONY : CMakeFiles/Exercise2.dir/MeshViewer.cpp.o.requires

CMakeFiles/Exercise2.dir/MeshViewer.cpp.o.provides: CMakeFiles/Exercise2.dir/MeshViewer.cpp.o.requires
	$(MAKE) -f CMakeFiles/Exercise2.dir/build.make CMakeFiles/Exercise2.dir/MeshViewer.cpp.o.provides.build
.PHONY : CMakeFiles/Exercise2.dir/MeshViewer.cpp.o.provides

CMakeFiles/Exercise2.dir/MeshViewer.cpp.o.provides.build: CMakeFiles/Exercise2.dir/MeshViewer.cpp.o

CMakeFiles/Exercise2.dir/FEMElementTri.cpp.o: CMakeFiles/Exercise2.dir/flags.make
CMakeFiles/Exercise2.dir/FEMElementTri.cpp.o: ../FEMElementTri.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ax/master/2017_01/PhysicallyBasedSimulation/exercises/ex2/ex2_code_framework/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Exercise2.dir/FEMElementTri.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Exercise2.dir/FEMElementTri.cpp.o -c /home/ax/master/2017_01/PhysicallyBasedSimulation/exercises/ex2/ex2_code_framework/FEMElementTri.cpp

CMakeFiles/Exercise2.dir/FEMElementTri.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Exercise2.dir/FEMElementTri.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ax/master/2017_01/PhysicallyBasedSimulation/exercises/ex2/ex2_code_framework/FEMElementTri.cpp > CMakeFiles/Exercise2.dir/FEMElementTri.cpp.i

CMakeFiles/Exercise2.dir/FEMElementTri.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Exercise2.dir/FEMElementTri.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ax/master/2017_01/PhysicallyBasedSimulation/exercises/ex2/ex2_code_framework/FEMElementTri.cpp -o CMakeFiles/Exercise2.dir/FEMElementTri.cpp.s

CMakeFiles/Exercise2.dir/FEMElementTri.cpp.o.requires:
.PHONY : CMakeFiles/Exercise2.dir/FEMElementTri.cpp.o.requires

CMakeFiles/Exercise2.dir/FEMElementTri.cpp.o.provides: CMakeFiles/Exercise2.dir/FEMElementTri.cpp.o.requires
	$(MAKE) -f CMakeFiles/Exercise2.dir/build.make CMakeFiles/Exercise2.dir/FEMElementTri.cpp.o.provides.build
.PHONY : CMakeFiles/Exercise2.dir/FEMElementTri.cpp.o.provides

CMakeFiles/Exercise2.dir/FEMElementTri.cpp.o.provides.build: CMakeFiles/Exercise2.dir/FEMElementTri.cpp.o

# Object files for target Exercise2
Exercise2_OBJECTS = \
"CMakeFiles/Exercise2.dir/SimpleFEM.cpp.o" \
"CMakeFiles/Exercise2.dir/MeshViewer.cpp.o" \
"CMakeFiles/Exercise2.dir/FEMElementTri.cpp.o"

# External object files for target Exercise2
Exercise2_EXTERNAL_OBJECTS =

Exercise2: CMakeFiles/Exercise2.dir/SimpleFEM.cpp.o
Exercise2: CMakeFiles/Exercise2.dir/MeshViewer.cpp.o
Exercise2: CMakeFiles/Exercise2.dir/FEMElementTri.cpp.o
Exercise2: CMakeFiles/Exercise2.dir/build.make
Exercise2: /usr/lib/x86_64-linux-gnu/libGLU.so
Exercise2: /usr/lib/x86_64-linux-gnu/libGL.so
Exercise2: /usr/lib/x86_64-linux-gnu/libSM.so
Exercise2: /usr/lib/x86_64-linux-gnu/libICE.so
Exercise2: /usr/lib/x86_64-linux-gnu/libX11.so
Exercise2: /usr/lib/x86_64-linux-gnu/libXext.so
Exercise2: /usr/lib/x86_64-linux-gnu/libglut.so
Exercise2: /usr/lib/x86_64-linux-gnu/libXmu.so
Exercise2: /usr/lib/x86_64-linux-gnu/libXi.so
Exercise2: CMakeFiles/Exercise2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable Exercise2"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Exercise2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Exercise2.dir/build: Exercise2
.PHONY : CMakeFiles/Exercise2.dir/build

CMakeFiles/Exercise2.dir/requires: CMakeFiles/Exercise2.dir/SimpleFEM.cpp.o.requires
CMakeFiles/Exercise2.dir/requires: CMakeFiles/Exercise2.dir/MeshViewer.cpp.o.requires
CMakeFiles/Exercise2.dir/requires: CMakeFiles/Exercise2.dir/FEMElementTri.cpp.o.requires
.PHONY : CMakeFiles/Exercise2.dir/requires

CMakeFiles/Exercise2.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Exercise2.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Exercise2.dir/clean

CMakeFiles/Exercise2.dir/depend:
	cd /home/ax/master/2017_01/PhysicallyBasedSimulation/exercises/ex2/ex2_code_framework/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ax/master/2017_01/PhysicallyBasedSimulation/exercises/ex2/ex2_code_framework /home/ax/master/2017_01/PhysicallyBasedSimulation/exercises/ex2/ex2_code_framework /home/ax/master/2017_01/PhysicallyBasedSimulation/exercises/ex2/ex2_code_framework/build /home/ax/master/2017_01/PhysicallyBasedSimulation/exercises/ex2/ex2_code_framework/build /home/ax/master/2017_01/PhysicallyBasedSimulation/exercises/ex2/ex2_code_framework/build/CMakeFiles/Exercise2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Exercise2.dir/depend
