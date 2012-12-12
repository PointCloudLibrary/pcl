# Package creation using CPack

###############################################################################
#find available package generators

# RPM (disabled until RedHat/Fedora users/developers need this)
#find_program(RPM_PROGRAM rpm)
#if(EXISTS ${RPM_PROGRAM})
#  list(APPEND CPACK_GENERATOR "RPM")
#endif(EXISTS ${RPM_PROGRAM})

set(CPACK_PACKAGE_VERSION "${PCL_VERSION}")
set(CPACK_PACKAGE_VERSION_MAJOR "${PCL_MAJOR_VERSION}")
set(CPACK_PACKAGE_VERSION_MINOR "${PCL_MINOR_VERSION}")
set(CPACK_PACKAGE_VERSION_PATCH "${PCL_REVISION_VERSION}")
set(CPACK_PACKAGE_CONFIG_INSTALL_DIR ${PCLCONFIG_INSTALL_DIR})

# DEB
if("${CMAKE_SYSTEM}" MATCHES "Linux")
  find_program(DPKG_PROGRAM dpkg)
  if(EXISTS ${DPKG_PROGRAM})
    list(APPEND CPACK_GENERATOR "DEB")
  endif(EXISTS ${DPKG_PROGRAM})
endif()

# NSIS
if(WIN32)
  list(APPEND CPACK_GENERATOR "NSIS")
  if(CMAKE_CL_64)
    set(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES64")
    set(win_system_name win64)
  else(CMAKE_CL_64)
    set(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES32")
    set(win_system_name win32)
  endif(CMAKE_CL_64)
  set(CPACK_NSIS_PACKAGE_NAME "${PROJECT_NAME}-${PCL_VERSION}")
  if(BUILD_all_in_one_installer)
    set(CPACK_NSIS_PACKAGE_NAME "${PROJECT_NAME}-${PCL_VERSION}-AllInOne")
  endif(BUILD_all_in_one_installer)
  if(MSVC10)
    set(CPACK_NSIS_PACKAGE_NAME "${CPACK_NSIS_PACKAGE_NAME}-msvc2010-${win_system_name}")
  else(MSVC10)
    set(CPACK_NSIS_PACKAGE_NAME "${CPACK_NSIS_PACKAGE_NAME}-${win_system_name}")
  endif(MSVC10)
  set(CPACK_PACKAGE_FILE_NAME ${CPACK_NSIS_PACKAGE_NAME})
  # force CPACK_PACKAGE_INSTALL_REGISTRY_KEY because of a known limitation in cmake/cpack to be fixed in next releases
  # http://public.kitware.com/Bug/view.php?id=9094
  # This is to allow a 32bit and a 64bit of PCL to get installed on one system
  set(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "${PROJECT_NAME} ${PCL_VERSION} ${win_system_name}" )
endif()

# dpkg
if(APPLE)
  find_program(PACKAGE_MAKER_PROGRAM PackageMaker
               HINTS /Developer/Applications/Utilities)
  if(EXISTS ${PACKAGE_MAKER_PROGRAM})
    list(APPEND CPACK_GENERATOR "PackageMaker")
  endif(EXISTS ${PACKAGE_MAKER_PROGRAM})
endif()

# By default, do not warn when built on machines using only VS Express:
if(NOT DEFINED CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS)
  set(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS ON)
endif()
include(InstallRequiredSystemLibraries)

set(PCL_CPACK_CFG_FILE "${PCL_BINARY_DIR}/cpack_options.cmake")

###############################################################################
# Make the CPack input file.
macro(PCL_MAKE_CPACK_INPUT)
    set(_cpack_cfg_in "${PCL_SOURCE_DIR}/cmake/cpack_options.cmake.in")
    set(${_var} "${${_var}}\nset(CPACK_COMPONENT_GROUP_PCL_DESCRIPTION \"PCL headers and librairies\")\n")

    # Prepare the components list
    set(PCL_CPACK_COMPONENTS)
    PCL_CPACK_MAKE_COMPS_OPTS(PCL_CPACK_COMPONENTS "${_comps}")

    # add documentation
    if(BUILD_documentation)
        set(CPACK_COMPONENTS_ALL "${CPACK_COMPONENTS_ALL} doc")
        set(PCL_CPACK_COMPONENTS "${PCL_CPACK_COMPONENTS}\nset(CPACK_COMPONENT_DOC_GROUP \"PCL\")\n")
        set(PCL_CPACK_COMPONENTS "${PCL_CPACK_COMPONENTS}\nset(CPACK_COMPONENT_DOC_DISPLAY_NAME \"Documentation\")\n")
        set(PCL_CPACK_COMPONENTS "${PCL_CPACK_COMPONENTS}\nset(CPACK_COMPONENT_DOC_DESCRIPTION \"API documentation and tutorials\")\n")	
    endif(BUILD_documentation)
    # add PCLConfig
    set(CPACK_COMPONENTS_ALL "${CPACK_COMPONENTS_ALL} pclconfig")
    set(PCL_CPACK_COMPONENTS "${PCL_CPACK_COMPONENTS}\nset(CPACK_COMPONENT_PCLCONFIG_GROUP \"PCL\")\n")
    set(PCL_CPACK_COMPONENTS "${PCL_CPACK_COMPONENTS}\nset(CPACK_COMPONENT_PCLCONFIG_DISPLAY_NAME \"PCLConfig\")\n")
    set(PCL_CPACK_COMPONENTS "${PCL_CPACK_COMPONENTS}\nset(CPACK_COMPONENT_PCLCONFIG_DESCRIPTION \"Helper cmake configuration scripts used by find_package(PCL)\")\n")	

    # add 3rdParty libs
    if(BUILD_all_in_one_installer)
        set(PCL_CPACK_COMPONENTS "${PCL_CPACK_COMPONENTS}\nset(CPACK_COMPONENT_GROUP_THIRDPARTY_DISPLAY_NAME \"3rd Party Libraries\")")
        set(PCL_CPACK_COMPONENTS "${PCL_CPACK_COMPONENTS}\nset(CPACK_COMPONENT_GROUP_THIRDPARTY_DESCRIPTION \"3rd Party Libraries\")")
        foreach(dep ${PCL_3RDPARTY_COMPONENTS})
            string(TOUPPER ${dep} DEP)
            set(PCL_CPACK_COMPONENTS "${PCL_CPACK_COMPONENTS}\nset(CPACK_COMPONENT_${DEP}_GROUP \"ThirdParty\")")
            set(CPACK_COMPONENTS_ALL "${CPACK_COMPONENTS_ALL} ${dep}")
        endforeach(dep)    
    endif(BUILD_all_in_one_installer)

    set(PCL_CPACK_COMPONENTS "${PCL_CPACK_COMPONENTS}\nset(CPACK_COMPONENTS_ALL${CPACK_COMPONENTS_ALL})\n")
    configure_file(${_cpack_cfg_in} ${PCL_CPACK_CFG_FILE} @ONLY)
endmacro(PCL_MAKE_CPACK_INPUT)


macro(PCL_CPACK_MAKE_COMPS_OPTS _var _current)
    set(_comps_list)
    set(PCL_CPACK_SUBSYSTEMS ${PCL_SUBSYSTEMS})
    list(REMOVE_ITEM PCL_CPACK_SUBSYSTEMS global_tests examples)
    foreach(_ss ${PCL_CPACK_SUBSYSTEMS})
        PCL_GET_SUBSYS_STATUS(_status ${_ss})
        if(_status)
            set(_comps_list "${_comps_list} pcl_${_ss}")
            PCL_CPACK_ADD_COMP_INFO(${_var} ${_ss})
        endif(_status)
    endforeach(_ss)
    set(CPACK_COMPONENTS_ALL ${_comps_list})
endmacro(PCL_CPACK_MAKE_COMPS_OPTS)


macro(PCL_CPACK_ADD_COMP_INFO _var _ss)
    string(TOUPPER "${_ss}" _comp_name)
    set(${_var}
        "${${_var}}set(CPACK_COMPONENT_PCL_${_comp_name}_DISPLAY_NAME \"${_ss}\")\n")
    GET_IN_MAP(_desc PCL_SUBSYS_DESC ${_ss})
    set(${_var}
        "${${_var}}set(CPACK_COMPONENT_PCL_${_comp_name}_DESCRIPTION \"${_desc}\")\n")
    set(_deps_str)
    GET_IN_MAP(_deps PCL_SUBSYS_DEPS ${_ss})
    foreach(_dep ${_deps})
        set(_deps_str "${_deps_str} pcl_${_dep}")
    endforeach(_dep)
    set(${_var}
        "${${_var}}set(CPACK_COMPONENT_PCL_${_comp_name}_DEPENDS ${_deps_str})\n")
    set(${_var} 
        "${${_var}}set(CPACK_COMPONENT_PCL_${_comp_name}_GROUP \"PCL\")\n")
endmacro(PCL_CPACK_ADD_COMP_INFO)

