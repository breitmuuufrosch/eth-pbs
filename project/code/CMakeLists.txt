PROJECT(PBS17_group11_project)

CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
SET(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "add a postfix for Debug mode, usually d on windows")

# check c++11 compiler-flags
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
CHECK_CXX_COMPILER_FLAG("-std=gnu++11" COMPILER_SUPPORTS_GNUXX11)

IF(COMPILER_SUPPORTS_CXX11)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSEIF(COMPILER_SUPPORTS_GNUXX11)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
ELSE()
        MESSAGE(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

# set cmake policy
IF(COMMAND CMAKE_POLICY)
	CMAKE_POLICY(SET CMP0003 NEW)
ENDIF(COMMAND CMAKE_POLICY)

IF(WIN32)
	IF(MSVC)
		ADD_DEFINITIONS(-D_SCL_SECURE_NO_WARNINGS)
		ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)
	ENDIF(MSVC)
ELSE(WIN32)
	SET(CMAKE_CXX_FLAGS "-W -Wall -Wno-unused")
ENDIF(WIN32)

# OpenGL
FIND_PACKAGE(OpenGL REQUIRED)
INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR})
LINK_DIRECTORIES(${OPENGL_INCLUDE_DIR})

# GLUT
FIND_PACKAGE(GLUT)
IF(NOT GLUT_FOUND)
	SET(GLUT_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/libraries/GL)
	FIND_LIBRARY(GLUT_LIBRARY glut32 PATHS ${CMAKE_CURRENT_LIST_DIR}/libraries/GL)
	SET(GLUT_LIBRARIES ${GLUT_LIBRARY})
ENDIF(NOT GLUT_FOUND)
INCLUDE_DIRECTORIES(${GLUT_INCLUDE_DIRS})
LINK_DIRECTORIES(${GLUT_INCLUDE_DIRS})

# find include directory by looking for certain header file
FIND_PATH(OPENSCENEGRAPH_INCLUDE_DIR osg/Referenced
	PATHS
	$ENV{OSG_ROOT}/include
	/usr/include
	/usr/local/include
)

# find library directory by looking for certain library file
FIND_PATH(OPENSCENEGRAPH_LIB_DIR libso.so osg.lib
	PATHS
	$ENV{OSG_ROOT}/lib
	/usr/lib
	/usr/local/lib
	/usr/lib/x86_64-linux-gnu
)

# apply them to following projects
INCLUDE_DIRECTORIES(${OPENSCENEGRAPH_INCLUDE_DIR})
LINK_DIRECTORIES(${OPENSCENEGRAPH_LIB_DIR})

MACRO(START_PROJECT)

    INCLUDE_DIRECTORIES(${EXTERNAL_INCLUDE_DIR})
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_LIST_DIR}/libraries/eigen)
    ADD_EXECUTABLE(${EXAMPLE_NAME} ${SOURCES} ${COMMON_FILES})
    SET_TARGET_PROPERTIES(${EXAMPLE_NAME} PROPERTIES
		DEBUG_POSTFIX "${CMAKE_DEBUG_POSTFIX}"
		CXX_STANDARD 11
		CXX_STANDARD_REQUIRED YES
	)
	
    TARGET_LINK_LIBRARIES(${EXAMPLE_NAME}
        debug osg${CMAKE_DEBUG_POSTFIX}          optimized osg
        debug osgAnimation${CMAKE_DEBUG_POSTFIX} optimized osgAnimation
        debug osgParticle${CMAKE_DEBUG_POSTFIX}  optimized osgParticle
        debug osgDB${CMAKE_DEBUG_POSTFIX}        optimized osgDB
        debug osgGA${CMAKE_DEBUG_POSTFIX}        optimized osgGA
        debug osgText${CMAKE_DEBUG_POSTFIX}      optimized osgText
        debug osgUtil${CMAKE_DEBUG_POSTFIX}      optimized osgUtil
        debug osgViewer${CMAKE_DEBUG_POSTFIX}    optimized osgViewer
        debug OpenThreads${CMAKE_DEBUG_POSTFIX}  optimized OpenThreads
		${OPENGL_LIBRARIES}
		${GLUT_LIBRARY}
	)
    INSTALL(TARGETS ${EXAMPLE_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

ENDMACRO(START_PROJECT)

# Project
SET(EXAMPLE_NAME asteroid_field)
SET(SOURCES "")

set(DATA_MODEL_PATH ${CMAKE_CURRENT_LIST_DIR}/data)
configure_file(precompile/config.h.in ${CMAKE_CURRENT_LIST_DIR}/config.h)

FILE(GLOB main_SRCS "./*.cpp")
FILE(GLOB main_HDRS "./*.h")
SOURCE_GROUP(Core FILES ${main_SRCS} ${main_HDRS})
LIST(APPEND SOURCES ${main_SRCS} ${main_HDRS})

FILE(GLOB scene_SRCS "./scene/*.cpp")
FILE(GLOB scene_HDRS "./scene/*.h")
SOURCE_GROUP(Scene FILES ${scene_SRCS} ${scene_HDRS})
LIST(APPEND SOURCES ${scene_SRCS} ${scene_HDRS})

FILE(GLOB osg_SRCS "./osg/*.cpp")
FILE(GLOB osg_HDRS "./osg/*.h")
SOURCE_GROUP(Osg FILES ${osg_SRCS} ${osg_HDRS})
LIST(APPEND SOURCES ${osg_SRCS} ${osg_HDRS})

FILE(GLOB physics_SRCS "./physics/*.cpp")
FILE(GLOB physics_HDRS "./physics/*.h")
SOURCE_GROUP(Physics FILES ${physics_SRCS} ${physics_HDRS})
LIST(APPEND SOURCES ${physics_SRCS} ${physics_HDRS})

START_PROJECT()