# Package creation using CPack

###############################################################################
#find available package generators

# RPM
find_program(RPM_PROGRAM rpm)
if(EXISTS ${RPM_PROGRAM})
  list(APPEND CPACK_GENERATOR "RPM")
  list(APPEND PACKAGE_EXTENSION ".rpm")
endif(EXISTS ${RPM_PROGRAM})
# DEB
find_program(DPKG_PROGRAM dpkg)
if(EXISTS ${DPKG_PROGRAM})
  list(APPEND CPACK_GENERATOR "DEB")
  list(APPEND PACKAGE_EXTENSION ".deb")
endif(EXISTS ${DPKG_PROGRAM})
# NSIS
find_program(NSIS_PROGRAM makensis MakeNSIS)
if(EXISTS ${NSIS_PROGRAM})
  list(APPEND CPACK_GENERATOR "NSIS")
  list(APPEND PACKAGE_EXTENSION ".exe;")
endif(EXISTS ${NSIS_PROGRAM})
# dpkg
find_program(PACKAGE_MAKER_PROGRAM PackageMaker)
if(EXISTS ${PACKAGE_MAKER_PROGRAM})
  list(APPEND CPACK_GENERATOR "PackageMaker")
  list(APPEND PACKAGE_EXTENSION ".dmg;")
endif(EXISTS ${PACKAGE_MAKER_PROGRAM})

list(LENGTH PACKAGE_EXTENSION nb_extensions)
if(nb_extensions GREATER 0)
  list(GET PACKAGE_EXTENSION 0 ext)
  set(COPY_PACKAGE_COMMAND "cp *${ext} ../../")
  math(EXPR before_last "${nb_extensions} - 1")
  if(${before_last} GREATER 0)
    foreach(ext RANGE 1 ${before_last})
      list(GET PACKAGE_EXTENSION ${ext} ext_name)
      set(COPY_PACKAGE_COMMAND "${COPY_PACKAGE_COMMAND} && \\
  cp *${ext_name} ../../")
    endforeach(ext)
  endif(${before_last} GREATER 0)
  #now configure all the 3rdpaty Makefile
  file(GLOB parties ${PCL_SOURCE_DIR}/3rdparty/*)
  foreach(party ${parties})
    if(EXISTS ${party}/Makefile.in)
      configure_file(${party}/Makefile.in ${party}/Makefile @ONLY)
    endif(EXISTS ${party}/Makefile.in)
  endforeach(party)
endif(nb_extensions GREATER 0)


include(InstallRequiredSystemLibraries)

set(PCL_CPACK_CFG_FILE "${PCL_BINARY_DIR}/cpack_options.cmake")

###############################################################################
# Make the CPack input file.
macro(PCL_MAKE_CPACK_INPUT)
    set(_cpack_cfg_in "${PCL_SOURCE_DIR}/cmake/cpack_options.cmake.in")

    # Prepare the components list
    set(PCL_CPACK_COMPONENTS)
    PCL_CPACK_MAKE_COMPS_OPTS(PCL_CPACK_COMPONENTS "${_comps}")

    configure_file(${_cpack_cfg_in} ${PCL_CPACK_CFG_FILE} @ONLY)
endmacro(PCL_MAKE_CPACK_INPUT)


macro(PCL_CPACK_MAKE_COMPS_OPTS _var _current)
    set(_comps_list)
    foreach(_ss ${PCL_SUBSYSTEMS})
        PCL_GET_SUBSYS_STATUS(_status ${_ss})
        if(_status)
            set(_comps_list "${_comps_list} ${_ss}")
            PCL_CPACK_ADD_COMP_INFO(${_var} ${_ss})
        endif(_status)
    endforeach(_ss)
    set(${_var} "${${_var}}\nset(CPACK_COMPONENTS_ALL${_comps_list})\n")
endmacro(PCL_CPACK_MAKE_COMPS_OPTS)


macro(PCL_CPACK_ADD_COMP_INFO _var _ss)
    string(TOUPPER "${_ss}" _comp_name)
    set(${_var}
        "${${_var}}set(CPACK_COMPONENT_${_comp_name}_DISPLAY_NAME \"${_ss}\")\n")
    GET_IN_MAP(_desc PCL_SUBSYS_DESC ${_ss})
    set(${_var}
        "${${_var}}set(CPACK_COMPONENT_${_comp_name}_DESCRIPTION \"${_desc}\")\n")
    set(_deps_str)
    GET_IN_MAP(_deps PCL_SUBSYS_DEPS ${_ss})
    foreach(_dep ${_deps})
        set(_deps_str "${_deps_str} ${_dep}")
    endforeach(_dep)
    set(${_var}
        "${${_var}}set(CPACK_COMPONENT_${_comp_name}_DEPENDS ${_deps_str})\n")
endmacro(PCL_CPACK_ADD_COMP_INFO)

