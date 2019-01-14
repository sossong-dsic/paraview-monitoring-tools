# Author: Bill Archerd
# Date: February 07, 2016
####################################################################################
cmake_minimum_required (VERSION 2.8)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
project (geometry_plugin)

#------------------------------------------------------------------------------
# Setup DSIC cmake modules directory
#------------------------------------------------------------------------------
find_path(DSIC_CMAKE_MODULE_PATH dsicCMakeModules.cmake PATHS /opt/dsc/lib/cmake DOC "Location of DSIC CMake Modules")
if(NOT DSIC_CMAKE_MODULE_PATH)
  set(DSIC_CMAKE_MODULE_PATH "${CMAKE_INSTALL_PREFIX}/lib/cmake" CACHE PATH "Location of DSIC CMake Modules")
endif()
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${DSIC_CMAKE_MODULE_PATH})
include(dsicCMakeModules)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMakeModules")
#------------------------------------------------------------------------------
# END Setup DSIC cmake modules directory
#------------------------------------------------------------------------------

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMakeModules")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=gnu++0x -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=gnu++0x -Wall -O0 -g")

#Printouts to aid in development and checking
############################################################################################
message(STATUS "The CXX flags: ${CMAKE_CXX_FLAGS}")
message(STATUS "The install dir: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "The CXX compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
############################################################################################
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib:$$ORIGIN/../lib")

#These are the include and link directories.
#These will be the -l and -L command line equivelents.
############################################################################################

set(TARGET_LIBRARIES) # just append lib calls here as we go

find_path(DSC_INCLUDE dsclib PATHS ${CMAKE_INSTALL_PREFIX}/include DOC "DSIC software include location")
message(STATUS "DSC_INCLUDE: ${DSC_INCLUDE}")
include_directories(${DSC_INCLUDE})

find_library(MTLIB_LIBRARY mtlib ${CMAKE_INSTALL_PREFIX}/lib REQUIRED)
message(STATUS "MLIB_LIBRARY: ${MTLIB_LIBRARY}")
list(APPEND TARGET_LIBRARIES ${MTLIB_LIBRARY})

find_package(ParaView)
include(${PARAVIEW_USE_FILE})
if (NOT PARAVIEW_BUILD_QT_GUI)
  message(FATAL_ERROR
    "Paraview plugins require PARAVIEW_BUILD_QT_GUI to be enabled. "
    "Please rebuild ParaView (or point to a different build of ParaView) "
    "with PARAVIEW_BUILD_QT_GUI set to TRUE")
endif()
# include(dsicCheckVTK)
# list(APPEND TARGET_LIBRARIES ${VTK_LIBRARIES})
# find_package(VTK)
include(${VTK_USE_FILE}) # paraview already loads vtk

message(STATUS "VTK_LIBRARIES: ${VTK_LIBRARIES}")


SET(Boost_USE_STATIC_LIBS       OFF)
SET(Boost_USE_MULTITHREADED      ON)
SET(Boost_USE_STATIC_RUNTIME    OFF)
SET(Boost_NO_BOOST_CMAKE ON)
FIND_PACKAGE( Boost COMPONENTS system filesystem python REQUIRED)
IF(Boost_FOUND)
    INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
ENDIF()
list( APPEND TARGET_LIBRARIES ${Boost_LIBRARIES} )
message(STATUS "BOOST: ${Boost_LIBRARIES}")
#
# BUILD src and header lists
###########
set(PLUGIN_NAME GeometryInfoIOBinDTS)
set(PLUGIN_VERSION "1.0")

set (SM_XML
  src/GeometryDriftTubeSection.xml
  )

set (Module_SRCS
  src/GeometryDriftTubeSection.cxx
 )
 
set (Module_MOC_HDRS
  src/GeometryDriftTubeSection.h
 )

include_directories( . src ${DSC_INCLUDE} )


ADD_PARAVIEW_PLUGIN(${PLUGIN_NAME} ${PLUGIN_VERSION}
  REQUIRED_ON_SERVER
  SERVER_MANAGER_XML ${SM_XML}
  SERVER_MANAGER_SOURCES ${Module_SRCS}
  )

target_link_libraries(${PLUGIN_NAME} LINK_PRIVATE ${TARGET_LIBRARIES})

install(TARGETS ${PLUGIN_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
