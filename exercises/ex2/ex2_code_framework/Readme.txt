================================================================================
=============   Physically-based Simulation in Computer Graphics   =============
================================================================================

This framework uses CMake to create Makefiles and project files for various
compilers and IDEs. It can be run using the CMake-GUI or the appropriate
generateBuild* file in this folder. These files can also be adjusted to for 
different compilers.

Default:
  - generateBuildLinux.sh: Unix Makefiles
  - generateBuildOSX.sh: XCode
  - generateBuildWindows.bat: Visual Studio 2015

================================================================================
=== Troubleshooting ============================================================

== Windows ==

- CMake is installed, but cannot be found:
Make sure the CMake directory is part of the PATH system variable.

- CMake cannot be installed:
Download the CMake binaries from https://cmake.org/download/, extract them
somewhere, then modify the generateBuildWindows.bat by replacing the 'cmake'
call with the full path to the CMake binaries you just extracted.
Alternatively, you can run CMake by using the CMake GUI from the bin folder.

- Visual Studio 2015 cannot be installed:
Change the CMake generator settings in generateBuildWindows.bat to a version
of Visual Studio that you have. For example, replace "Visual Studio 14 2015" by
"Visual Studio 12 2013" to create a solution for Visual Studio 2013.
Alternatively, you can run CMake by using the CMake GUI from the bin folder,
and specify there which compiler you would like to use.

== Linux ==

- CMake cannot find the OpenGL libraries
Make sure you have the OpenGL dev environment installed. The specific packages
for your system might vary, but as an example, on Debian-based systems, the
packages should be called 'mesa-common-dev' and 'libgl1-mesa-dev'.

- CMake cannot find the GLUT library
Try installing freeglut through your favorite package manager. As an example,
on Debian-based systems, the necessary packages are freeglut-dev, libXi-dev, 
and libXmu-dev.
