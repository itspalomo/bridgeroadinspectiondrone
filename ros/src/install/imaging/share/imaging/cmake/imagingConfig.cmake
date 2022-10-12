# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_imaging_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED imaging_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(imaging_FOUND FALSE)
  elseif(NOT imaging_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(imaging_FOUND FALSE)
  endif()
  return()
endif()
set(_imaging_CONFIG_INCLUDED TRUE)

# output package information
if(NOT imaging_FIND_QUIETLY)
  message(STATUS "Found imaging: 0.0.0 (${imaging_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'imaging' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${imaging_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(imaging_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${imaging_DIR}/${_extra}")
endforeach()
