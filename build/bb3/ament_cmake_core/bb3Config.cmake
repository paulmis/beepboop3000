# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_bb3_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED bb3_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(bb3_FOUND FALSE)
  elseif(NOT bb3_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(bb3_FOUND FALSE)
  endif()
  return()
endif()
set(_bb3_CONFIG_INCLUDED TRUE)

# output package information
if(NOT bb3_FIND_QUIETLY)
  message(STATUS "Found bb3: 0.0.1 (${bb3_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'bb3' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${bb3_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(bb3_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${bb3_DIR}/${_extra}")
endforeach()
