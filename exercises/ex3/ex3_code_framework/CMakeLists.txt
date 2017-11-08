cmake_minimum_required(VERSION 2.8)

project(Exercise3)

# OpenGL
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

# GLUT
find_package(GLUT)
if(NOT GLUT_FOUND)
	set(GLUT_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/GL)
	find_library(GLUT_LIBRARY glut32 PATHS ${CMAKE_CURRENT_LIST_DIR}/GL)
	set(GLUT_LIBRARIES ${GLUT_LIBRARY})
endif(NOT GLUT_FOUND)
include_directories(${GLUT_INCLUDE_DIRS})

if(UNIX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif(UNIX)

file(GLOB ex3_files
		${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/*.h
		${CMAKE_CURRENT_SOURCE_DIR}/Utilities/*.h
		${CMAKE_CURRENT_SOURCE_DIR}/LodePNG/*.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/LodePNG/*.h
	)
	
add_executable(Exercise3 ${ex3_files})

target_link_libraries(Exercise3 ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})

# Set startup project for Visual Studio (only possible with CMake version >= 3.6)
if (WIN32 AND (CMAKE_MAJOR_VERSION GREATER 3 OR (CMAKE_MAJOR_VERSION GREATER 2 AND CMAKE_MINOR_VERSION GREATER 5)))
	set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT Exercise3)
endif()
