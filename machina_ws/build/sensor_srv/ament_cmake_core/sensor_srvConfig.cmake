# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_sensor_srv_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED sensor_srv_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(sensor_srv_FOUND FALSE)
  elseif(NOT sensor_srv_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(sensor_srv_FOUND FALSE)
  endif()
  return()
endif()
set(_sensor_srv_CONFIG_INCLUDED TRUE)

# output package information
if(NOT sensor_srv_FIND_QUIETLY)
  message(STATUS "Found sensor_srv: 0.0.0 (${sensor_srv_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'sensor_srv' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${sensor_srv_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(sensor_srv_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${sensor_srv_DIR}/${_extra}")
endforeach()
