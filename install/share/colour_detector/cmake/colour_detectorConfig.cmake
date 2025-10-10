# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_colour_detector_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED colour_detector_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(colour_detector_FOUND FALSE)
  elseif(NOT colour_detector_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(colour_detector_FOUND FALSE)
  endif()
  return()
endif()
set(_colour_detector_CONFIG_INCLUDED TRUE)

# output package information
if(NOT colour_detector_FIND_QUIETLY)
  message(STATUS "Found colour_detector: 0.0.1 (${colour_detector_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'colour_detector' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT colour_detector_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(colour_detector_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${colour_detector_DIR}/${_extra}")
endforeach()
