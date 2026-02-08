function(get_all_targets var)
  set(targets)
  get_all_targets_recursive(targets ${CMAKE_CURRENT_SOURCE_DIR})
  set(${var}
      ${targets}
      PARENT_SCOPE)
endfunction()

function(get_all_installable_targets var)
  set(targets)
  get_all_targets(targets)
  foreach(_target ${targets})
    get_target_property(_target_type ${_target} TYPE)
    if(NOT
       ${_target_type}
       MATCHES
       ".*LIBRARY|EXECUTABLE")
      list(REMOVE_ITEM targets ${_target})
    endif()
  endforeach()
  set(${var}
      ${targets}
      PARENT_SCOPE)
endfunction()

macro(get_all_targets_recursive targets dir)
  get_property(
    subdirectories
    DIRECTORY ${dir}
    PROPERTY SUBDIRECTORIES)
  foreach(subdir ${subdirectories})
    get_all_targets_recursive(${targets} ${subdir})
  endforeach()

  get_property(
    current_targets
    DIRECTORY ${dir}
    PROPERTY BUILDSYSTEM_TARGETS)
  list(APPEND ${targets} ${current_targets})
endmacro()
