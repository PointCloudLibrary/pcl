
if(WIN32)
    option(BUILD_all_in_one_installer "Build an all-in-one NSIS installer" OFF)
endif(WIN32)

if(BUILD_all_in_one_installer)
    get_filename_component(BOOST_ROOT "${Boost_INCLUDE_DIR}" PATH)
    get_filename_component(EIGEN_ROOT "${EIGEN_INCLUDE_DIRS}" PATH)
    get_filename_component(QHULL_ROOT "${QHULL_INCLUDE_DIRS}" PATH)
    get_filename_component(FLANN_ROOT "${FLANN_INCLUDE_DIRS}" PATH)
    get_filename_component(VTK_ROOT "${VTK_DIR}" PATH)
    get_filename_component(VTK_ROOT "${VTK_ROOT}" PATH)
    set(PCL_3RDPARTY_COMPONENTS)
    foreach(dep Eigen Boost Qhull Flann VTK)
	    string(TOUPPER ${dep} DEP)
        install(
            DIRECTORY "${${DEP}_ROOT}"
            DESTINATION 3rdParty
            COMPONENT ${dep}
            PATTERN "*/Uninstall.exe" EXCLUDE
        )
		list(APPEND PCL_3RDPARTY_COMPONENTS ${dep})
    endforeach(dep)
endif(BUILD_all_in_one_installer)
