cmake_minimum_required (VERSION 2.8)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
project (paraview_scan_time_vis_plugin)

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
list( APPEND TARGET_LIBRARIES ${MTLIB_LIBRARY} )

find_package(ParaView)
include(${PARAVIEW_USE_FILE})
if (NOT PARAVIEW_BUILD_QT_GUI)
  message(FATAL_ERROR
    "mtview example requires PARAVIEW_BUILD_QT_GUI to be enabled. "
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
find_package(ZeroMQ REQUIRED)
message(STATUS "ZeroMQ_INCLUDE_DIR: ${ZeroMQ_INCLUDE_DIR}")
message(STATUS "ZeroMQ_LIBRARY: ${ZeroMQ_LIBRARY}")

## We also need 0mq cpp support
find_path(ZeroMQ_INCLUDE_DIR zmq.hpp PATHS ZeroMQ_INCLUDE_DIR DOC "Require zmq.hpp" REQUIRED)
include_directories(${ZeroMQ_INCLUDE_DIR})

list( APPEND TARGET_LIBRARIES ${ZeroMQ_LIBRARY})

## BEGIN find package for icomm ####
find_library(ICOMM_LIB icomm ${CMAKE_INSTALL_PREFIX}/lib REQUIRED)
find_path(ICOMM_INCLUDE_DIR dsc-zhelpers.h PATHS ${DSC_INCLUDE}/icomm DOC "icomm include directory" REQUIRED)
include_directories(${ICOMM_INCLUDE_DIR})
include_directories(${ICOMM_INCLUDE_DIR}/marshal/buffer)
message(STATUS "ICOMM_LIB " ${ICOMM_LIB})
message(STATUS "ICOMM_INCLUDE_DIR " ${ICOMM_INCLUDE_DIR})
list( APPEND TARGET_LIBRARIES ${ICOMM_LIB})
## END find package for icomm ####

### BEGIN find package for das/avro-types ###
find_path(DAS_AVRO_TYPES_INCLUDE_DIR PacketHeader.hh PATHS ${DSC_INCLUDE}/das-avro-types DOC "das/avro-types include directory" REQUIRED)
include_directories(${DAS_AVRO_TYPES_INCLUDE_DIR})
### END find package for das/avro-types ###

find_package(Avro)
include_directories(${AVRO_INCLUDE_DIRS})
message(STATUS "AVRO_INCLUDE_DIRS: ${AVRO_INCLUDE_DIRS}")
message(STATUS "AVRO_LIBRARIES: ${AVRO_LIBRARIES}")
list( APPEND TARGET_LIBRARIES ${AVRO_LIBRARIES} )

find_program(QMAKE_EXECUTABLE NAMES qmake HINTS ${QTDIR} ENV QTDIR PATH_SUFFIXES bin)
execute_process(COMMAND ${QMAKE_EXECUTABLE} -query QT_VERSION OUTPUT_VARIABLE QT_VERSION)
message(STATUS "QT_VERSION=" ${QT_VERSION})

include(mmpdsQtSetup)
set(QT_VERSION ${Qt5Core_VERSION_STRING})
message(STATUS "QT_VERSION=" ${QT_VERSION})

if(QT_VERSION GREATER 5.0.0)
  set(Qt5_FIND_COMPONENTS Concurrent)
else()
  set(Qt4_FIND_COMPONENTS Concurrent)
endif()

set (MMPDS_ENABLE_QT_SUPPORT True)
set (QT_REQUIRED TRUE)
include (mmpdsQtSetup)
set(MMPDS_QT_QMAKE_EXECUTABLE ${QT_QMAKE_EXECUTABLE})
list( APPEND TARGET_LIBRARIES ${QT_LIBRARIES} )
set(CMAKE_AUTOMOC OFF)

set(gui_sources)
set(gui_interfaces)
set(servermanager_sources)
set(servermanager_xml)
set(server_deps)


include_directories( . src ../Common/src ${DSC_INCLUDE} )

if(PARAVIEW_BUILD_QT_GUI)
  if(QT_VERSION GREATER 5.0.0)
    QT5_WRAP_CPP(moc_srcs 
      ../Common/src/AutoReadKafkaPropertyWidget.h
      ../Common/src/UpdateRenderPropertyWidget.h
      )
  else()
    QT4_WRAP_CPP(moc_srcs 
      ../Common/src/AutoReadKafkaPropertyWidget.h
      ../Common/src/UpdateRenderPropertyWidget.h
      )
  endif()

  message(STATUS "Adding Paraview Property Widget")
  # add a property widget for the auto update timer and render update
  ADD_PARAVIEW_PROPERTY_WIDGET(gui_interfaces_auto gui_srcs_auto
    TYPE "auto_read_widget_type"
    CLASS_NAME AutoReadKafkaPropertyWidget)
  ADD_PARAVIEW_PROPERTY_WIDGET(gui_interfaces_update gui_srcs_update
    TYPE "update_widget_type"
    CLASS_NAME UpdateRenderPropertyWidget)
  
endif()

list(APPEND servermanager_sources
  src/DssVtpScanTime.cxx
  )

list(APPEND servermanager_xml
  src/xml/DssVtpScanTime.xml
  )

set(PLUGIN_NAME DssVtpScanTime)
set(PLUGIN_VERSION "1.0")

ADD_PARAVIEW_PLUGIN(${PLUGIN_NAME} ${PLUGIN_VERSION}
  SERVER_MANAGER_XML ${servermanager_xml}
  SERVER_MANAGER_SOURCES ${servermanager_sources}
  GUI_INTERFACES ${gui_interfaces_auto} ${gui_interfaces_update}
  SOURCES ${gui_srcs_auto} ${gui_srcs_update}
          ${moc_srcs}
          ../Common/src/AutoReadKafkaPropertyWidget.cxx
          ../Common/src/UpdateRenderPropertyWidget.cxx
  )

message ( STATUS "TARGET_LIBRARIES: ${TARGET_LIBRARIES}")

target_link_libraries(${PLUGIN_NAME} LINK_PRIVATE ${TARGET_LIBRARIES})

install(TARGETS ${PLUGIN_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
