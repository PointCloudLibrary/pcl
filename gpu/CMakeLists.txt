set(SUBSYS_NAME gpu)
set(SUBSYS_DESC "Point cloud GPU libraries")
set(SUBSYS_DEPS )

OPTION(BUILD_GPU "Build the GPU-related subsystems" ${DEFAULT})

if(BUILD_GPU AND CUDA_FOUND)
    		
	if(CMAKE_COMPILER_IS_GNUCXX)
		string(REPLACE "-Wold-style-cast" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
		string(REPLACE "-Wno-invalid-offsetof" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-conversion -Wno-unused-parameter -Wno-unused-variable -Wno-unused-function")
    if (GCC_VERSION VERSION_GREATER 4.3)
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-but-set-variable")
    endif()
	endif()
	       
	collect_subproject_directory_names("${CMAKE_CURRENT_SOURCE_DIR}" "CMakeLists.txt" PCL_GPU_MODULES_NAMES PCL_GPU_MODULES_DIRS)
	set(PCL_GPU_MODULES_NAMES_UNSORTED ${PCL_GPU_MODULES_NAMES})
	topological_sort(PCL_GPU_MODULES_NAMES PCL_ _DEPENDS)
	sort_relative(PCL_GPU_MODULES_NAMES_UNSORTED PCL_GPU_MODULES_NAMES PCL_GPU_MODULES_DIRS)
	foreach(subdir ${PCL_GPU_MODULES_DIRS})
		add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/${subdir}")
	endforeach(subdir)    
endif()

