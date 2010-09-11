cmake_minimum_required(VERSION 2.6)

PROJECT( Transport )

INCLUDE(FindPkgConfig)
PKG_SEARCH_MODULE(RAPI REQUIRED autolab-rapi)
PKG_SEARCH_MODULE(RAPICB REQUIRED autolab-rapi-chatterbox)
PKG_SEARCH_MODULE(RAPISTAGE REQUIRED autolab-rapi-stage)
PKG_SEARCH_MODULE(ND REQUIRED autolab-nd)
PKG_SEARCH_MODULE(TRACKER REQUIRED autolab-rapi-autolabtracker)

INCLUDE_DIRECTORIES(
  ${RAPI_INCLUDE_DIRS}
  ${RAPISTAGE_INCLUDE_DIRS}
  ${RAPICB_INCLUDE_DIRS}
  ${ND_INCLUDE_DIRS}
  ${TRACKER_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
  ${RAPI_LIBRARY_DIRS}
  ${RAPISTAGE_LIBRARY_DIRS}
  ${RAPICB_LIBRARY_DIRS}
  ${ND_LIBRARY_DIRS}
  ${TRACKER_LIBRARY_DIRS}
)

# get rid of warnings about jsonrpc class initialization
ADD_DEFINITIONS( -g -Wall )

#ADD_EXECUTABLE( transport
ADD_LIBRARY( transport MODULE
  chatterbox.cpp
  chatterboxctrl.cpp
  waypoint.cpp
)

TARGET_LINK_LIBRARIES( transport
  ${RAPI_LIBRARIES}
  ${RAPISTAGE_LIBRARIES}
  ${RAPICB_LIBRARIES}
  ${ND_LIBRARIES}
  ${TRACKER_LIBRARIES}
)

# delete the "lib" prefix from the plugin libraries
SET_TARGET_PROPERTIES( transport PROPERTIES PREFIX "" )

#MESSAGE(STATUS "Remember to add the module path to the STAGEPATH variable" )
