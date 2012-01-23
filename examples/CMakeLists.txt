set(SUBSYS_NAME examples)
set(SUBSYS_DESC "PCL examples")
#!!! Add here the internal dependencies to all the libraries you need !!!
set(SUBSYS_DEPS common)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

if (build)
  include_directories(${PCL_INCLUDE_DIRS})
	#!!! This looks for all examples/XXX/CMakeLists.txt
	file (GLOB examples_sbudirs */CMakeLists.txt)
	#!!! extract only relevant XXX and append it to PCL_EXAMPLES_SUBDIRS
	foreach(subdir ${examples_sbudirs})
		get_filename_component (dir ${subdir} PATH)
		list(APPEND PCL_EXAMPLES_SUBDIRS ${dir})
	endforeach(subdir)
	#!!! display the found subdirectories
	message (STATUS "PCL_EXAMPLES_SUBDIRS ${PCL_EXAMPLES_SUBDIRS}")
	#!!! add the subdirectories
	foreach(subdir ${PCL_EXAMPLES_SUBDIRS})
    add_subdirectory(${subdir})
	endforeach(subdir)
endif (build)

#!!! David: remove my comments ASAP
