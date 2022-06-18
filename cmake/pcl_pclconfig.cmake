
set(PCL_SUBSYSTEMS_MODULES ${PCL_SUBSYSTEMS})
list(REMOVE_ITEM PCL_SUBSYSTEMS_MODULES tools cuda_apps global_tests proctor examples)


file(GLOB PCLCONFIG_FIND_MODULES "${PCL_SOURCE_DIR}/cmake/Modules/*.cmake")
list(REMOVE_ITEM PCLCONFIG_FIND_MODULES "${PCL_SOURCE_DIR}/cmake/Modules/FindGTestSource.cmake")
install(FILES ${PCLCONFIG_FIND_MODULES} COMPONENT pclconfig DESTINATION ${PCLCONFIG_INSTALL_DIR}/Modules)

set(PCLCONFIG_AVAILABLE_COMPONENTS)
set(PCLCONFIG_AVAILABLE_COMPONENTS_LIST)
set(PCLCONFIG_INTERNAL_DEPENDENCIES)
set(PCLCONFIG_EXTERNAL_DEPENDENCIES)
set(PCLCONFIG_OPTIONAL_DEPENDENCIES)
set(PCLCONFIG_SSE_DEFINITIONS "${SSE_DEFINITIONS}")
set(PCLCONFIG_SSE_COMPILE_OPTIONS ${SSE_FLAGS})
set(PCLCONFIG_AVX_COMPILE_OPTIONS ${AVX_FLAGS})

foreach(_ss ${PCL_SUBSYSTEMS_MODULES})
  PCL_GET_SUBSYS_STATUS(_status ${_ss})

  # do not include test targets
  string(REGEX MATCH "^tests_" _is_test ${_ss})

  if(_status AND NOT _is_test)
    string(APPEND PCLCONFIG_AVAILABLE_COMPONENTS " ${_ss}")
    string(APPEND PCLCONFIG_AVAILABLE_COMPONENTS_LIST "\n# - ${_ss}")
    GET_IN_MAP(_deps PCL_SUBSYS_DEPS ${_ss})
    if(_deps)
      string(APPEND PCLCONFIG_INTERNAL_DEPENDENCIES "set(pcl_${_ss}_int_dep ")
      foreach(_dep ${_deps})
        string(APPEND PCLCONFIG_INTERNAL_DEPENDENCIES "${_dep} ")
      endforeach()
      string(APPEND PCLCONFIG_INTERNAL_DEPENDENCIES ")\n")
    endif()
    GET_IN_MAP(_ext_deps PCL_SUBSYS_EXT_DEPS ${_ss})
    if(_ext_deps)
      string(APPEND PCLCONFIG_EXTERNAL_DEPENDENCIES "set(pcl_${_ss}_ext_dep ")
      foreach(_ext_dep ${_ext_deps})
        string(APPEND PCLCONFIG_EXTERNAL_DEPENDENCIES "${_ext_dep} ")
      endforeach()
      string(APPEND PCLCONFIG_EXTERNAL_DEPENDENCIES ")\n")
    endif()
    GET_IN_MAP(_opt_deps PCL_SUBSYS_OPT_DEPS ${_ss})
    if(_opt_deps)
      string(APPEND PCLCONFIG_OPTIONAL_DEPENDENCIES "set(pcl_${_ss}_opt_dep ")
      foreach(_opt_dep ${_opt_deps})
        string(TOUPPER "WITH_${_opt_dep}" _tmp)
        string(REGEX REPLACE "-(.*)" "" _condition ${_tmp}) #libusb-1.0 case
        if(${_condition})
          string(APPEND PCLCONFIG_OPTIONAL_DEPENDENCIES "${_opt_dep} ")
        endif()
      endforeach()
      string(APPEND PCLCONFIG_OPTIONAL_DEPENDENCIES ")\n")
    endif()

    #look for subsystems
    string(TOUPPER "PCL_${_ss}_SUBSYS" PCL_SUBSYS_SUBSYS)
    if(${PCL_SUBSYS_SUBSYS})
      string(TOUPPER "PCL_${_ss}_SUBSYS_STATUS" PCL_SUBSYS_SUBSYS_STATUS)
      foreach(_sub ${${PCL_SUBSYS_SUBSYS}})
        PCL_GET_SUBSUBSYS_STATUS(_sub_status ${_ss} ${_sub})
        if(_sub_status)
          string(APPEND PCLCONFIG_AVAILABLE_COMPONENTS " ${_sub}")
          string(APPEND PCLCONFIG_AVAILABLE_COMPONENTS_LIST "\n# - ${_sub}")
          GET_IN_MAP(_deps PCL_SUBSYS_DEPS ${_ss}_${sub})
          if(_deps)
            string(APPEND PCLCONFIG_INTERNAL_DEPENDENCIES "set(pcl_${_sub}_int_dep ")
            foreach(_dep ${_deps})
              string(APPEND PCLCONFIG_INTERNAL_DEPENDENCIES "${_dep} ")
            endforeach()
            string(APPEND PCLCONFIG_INTERNAL_DEPENDENCIES ")\n")
          endif()
        endif()
      endforeach()
    endif()
  endif()
endforeach()

#Boost modules
set(PCLCONFIG_AVAILABLE_BOOST_MODULES "system filesystem date_time iostreams")
if(Boost_SERIALIZATION_FOUND)
  string(APPEND PCLCONFIG_AVAILABLE_BOOST_MODULES " serialization")
endif()
if(Boost_CHRONO_FOUND)
  string(APPEND PCLCONFIG_AVAILABLE_BOOST_MODULES " chrono")
endif()

configure_file("${PCL_SOURCE_DIR}/PCLConfig.cmake.in"
               "${PCL_BINARY_DIR}/PCLConfig.cmake" @ONLY)
configure_file("${PCL_SOURCE_DIR}/PCLConfigVersion.cmake.in"
               "${PCL_BINARY_DIR}/PCLConfigVersion.cmake" @ONLY)
install(FILES
        "${PCL_BINARY_DIR}/PCLConfig.cmake"
        "${PCL_BINARY_DIR}/PCLConfigVersion.cmake"
        COMPONENT pclconfig
        DESTINATION ${PCLCONFIG_INSTALL_DIR})