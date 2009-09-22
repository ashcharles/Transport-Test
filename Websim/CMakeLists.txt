cmake_minimum_required( VERSION 2.6 )

include( FindPkgConfig )
pkg_search_module( WEBSIM REQUIRED websim )
pkg_search_module( RAPI REQUIRED autolab-rapi )
pkg_search_module( RAPISTAGE REQUIRED autolab-rapi-stage )

include_directories(
  ${WEBSIM_INCLUDE_DIRS}
  ${RAPI_INCLUDE_DIRS}
  ${RAPISTAGE_INCLUDE_DIRS}
)
link_directories(
  ${WEBSIM_LIBRARY_DIRS}
  ${RAPI_LIBRARY_DIRS}
  ${RAPISTAGE_LIBRARY_DIRS}
)
add_executable( rapiwebsim
  rapiwebsim.cpp
)
target_link_libraries( rapiwebsim
  ${WEBSIM_LIBRARIES}
  ${RAPI_LIBRARIES}
  ${RAPISTAGE_LIBRARIES}
)
add_definitions( -g -Wall )
