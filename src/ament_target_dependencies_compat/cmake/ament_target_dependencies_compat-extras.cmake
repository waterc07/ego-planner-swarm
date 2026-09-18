# Compatibility shim for ROS 2 Lyrical, where the classic
# ament_target_dependencies() macro was removed from ament_cmake.
#
# This replicates the upstream implementation (ament_cmake_target_dependencies):
# for each dependency prefer the modern CMake targets (${dep}_TARGETS,
# expanded recursively), and fall back to the classic
# ${dep}_INCLUDE_DIRS / ${dep}_LIBRARIES / ${dep}_DEFINITIONS /
# ${dep}_LINK_FLAGS variables for packages without modern targets.

# PCL pulls in VTK, and Ubuntu's VTK config (VTK-targets.cmake) links against
# the JsonCpp::JsonCpp imported target without ever calling
# find_package(jsoncpp) for it. Configuration then fails with:
#   The link interface of target "VTK::jsoncpp" contains: JsonCpp::JsonCpp
#   but the target was not found.
# Every package here finds this shim before PCL, so defining the target here
# keeps VTK (and therefore PCL) configurable.
if(NOT TARGET JsonCpp::JsonCpp)
  find_package(jsoncpp QUIET)
endif()

# Memoized replacement for ament_get_recursive_properties().
#
# The upstream version (ament_cmake_target_dependencies) walks
# INTERFACE_LINK_LIBRARIES recursively without remembering which targets it
# has already visited. On a densely connected imported-target graph that is
# exponential, and configuration never terminates - VTK's graph, reached
# through pcl_ros, is big enough to hang CMake indefinitely.
#
# This walks the same graph breadth-first, visiting every target exactly once,
# and produces the same include directories and libraries the recursive
# version would.
function(ament_get_recursive_properties_memoized var_include_dirs var_libraries)
  set(_queue ${ARGN})
  set(_visited "")
  set(_all_include_dirs "")
  set(_all_libraries "")

  while(_queue)
    list(POP_FRONT _queue _target)
    if(NOT TARGET "${_target}")
      continue()
    endif()
    if(_target IN_LIST _visited)
      continue()
    endif()
    list(APPEND _visited "${_target}")

    get_target_property(_include_dirs ${_target} INTERFACE_INCLUDE_DIRECTORIES)
    if(_include_dirs)
      list(APPEND _all_include_dirs ${_include_dirs})
    endif()

    get_target_property(_link_libraries ${_target} INTERFACE_LINK_LIBRARIES)
    if(_link_libraries)
      foreach(_link_library ${_link_libraries})
        if(TARGET "${_link_library}")
          list(APPEND _queue "${_link_library}")
        else()
          list(APPEND _all_libraries "${_link_library}")
        endif()
      endforeach()
    endif()

    get_target_property(_imported_configurations ${_target} IMPORTED_CONFIGURATIONS)
    if(_imported_configurations)
      foreach(_imported_config ${_imported_configurations})
        get_target_property(_imported_implib ${_target} IMPORTED_IMPLIB_${_imported_config})
        if(_imported_implib)
          list(APPEND _all_libraries "${_imported_implib}")
        else()
          get_target_property(_imported_location ${_target} IMPORTED_LOCATION_${_imported_config})
          if(_imported_location)
            list(APPEND _all_libraries "${_imported_location}")
          endif()
        endif()
      endforeach()
    endif()
  endwhile()

  ament_include_directories_order(_ordered_include_dirs ${_all_include_dirs})
  ament_libraries_deduplicate(_unique_libraries ${_all_libraries})

  set(${var_include_dirs} ${_ordered_include_dirs} PARENT_SCOPE)
  set(${var_libraries} ${_unique_libraries} PARENT_SCOPE)
endfunction()

macro(ament_target_dependencies target)
  set(_targets "")
  foreach(_dep ${ARGN})
    if(NOT ${_dep}_FOUND)
      find_package(${_dep} QUIET)
    endif()
    if(${_dep}_TARGETS)
      foreach(_target ${${_dep}_TARGETS})
        if(TARGET ${_target})
          get_target_property(_is_imported ${_target} IMPORTED)
          if(_is_imported)
            set(_recursive_include_dirs "")
            set(_recursive_libraries "")
            ament_get_recursive_properties_memoized(_recursive_include_dirs _recursive_libraries ${_target})
            if(_recursive_include_dirs)
              target_include_directories(${target} SYSTEM PUBLIC ${_recursive_include_dirs})
            endif()
            if(_recursive_libraries)
              target_link_libraries(${target} ${_recursive_libraries})
            endif()
          else()
            list(APPEND _targets ${_target})
          endif()
        else()
          list(APPEND _targets ${_target})
        endif()
      endforeach()
    else()
      # classic variables fallback for packages without modern CMake targets
      if(${_dep}_INCLUDE_DIRS)
        target_include_directories(${target} SYSTEM PUBLIC ${${_dep}_INCLUDE_DIRS})
      endif()
      if(${_dep}_LIBRARIES)
        target_link_libraries(${target} ${${_dep}_LIBRARIES})
      endif()
      if(${_dep}_DEFINITIONS)
        target_compile_definitions(${target} PUBLIC ${${_dep}_DEFINITIONS})
      endif()
      if(${_dep}_LINK_FLAGS)
        target_link_options(${target} PUBLIC ${${_dep}_LINK_FLAGS})
      endif()
    endif()
  endforeach()
  if(_targets)
    target_link_libraries(${target} ${_targets})
  endif()
endmacro()
