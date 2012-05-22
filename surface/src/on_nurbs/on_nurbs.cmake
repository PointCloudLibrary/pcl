
set(ON_NURBS_INCLUDES
	include/pcl/${SUBSYS_NAME}/on_nurbs/closing_boundary.h
	include/pcl/${SUBSYS_NAME}/on_nurbs/fitting_curve_2d_pdm.h
	include/pcl/${SUBSYS_NAME}/on_nurbs/fitting_curve_2d_sdm.h
	include/pcl/${SUBSYS_NAME}/on_nurbs/fitting_curve_2d_tdm.h
	include/pcl/${SUBSYS_NAME}/on_nurbs/fitting_curve_pdm.h
	include/pcl/${SUBSYS_NAME}/on_nurbs/fitting_cylinder_pdm.h
	include/pcl/${SUBSYS_NAME}/on_nurbs/fitting_surface_pdm.h
	include/pcl/${SUBSYS_NAME}/on_nurbs/fitting_surface_tdm.h
	include/pcl/${SUBSYS_NAME}/on_nurbs/global_optimization_pdm.h
	include/pcl/${SUBSYS_NAME}/on_nurbs/global_optimization_tdm.h
	include/pcl/${SUBSYS_NAME}/on_nurbs/nurbs_data.h
	include/pcl/${SUBSYS_NAME}/on_nurbs/nurbs_solve.h
	include/pcl/${SUBSYS_NAME}/on_nurbs/nurbs_tools.h
	include/pcl/${SUBSYS_NAME}/on_nurbs/sequential_fitter.h
	include/pcl/${SUBSYS_NAME}/on_nurbs/sparse_mat.h)

set(ON_NURBS_SOURCES
	src/on_nurbs/closing_boundary.cpp
	src/on_nurbs/fitting_curve_2d_pdm.cpp
	src/on_nurbs/fitting_curve_2d_sdm.cpp
	src/on_nurbs/fitting_curve_2d_tdm.cpp
	src/on_nurbs/fitting_curve_pdm.cpp
	src/on_nurbs/fitting_cylinder_pdm.cpp
	src/on_nurbs/fitting_surface_pdm.cpp
	src/on_nurbs/fitting_surface_tdm.cpp
	src/on_nurbs/global_optimization_pdm.cpp
	src/on_nurbs/global_optimization_tdm.cpp
	src/on_nurbs/nurbs_tools.cpp
	src/on_nurbs/sequential_fitter.cpp
	src/on_nurbs/sparse_mat.cpp)
	
if(true)
  set(ON_NURBS_SOURCES ${ON_NURBS_SOURCES} src/on_nurbs/nurbs_solve_eigen.cpp)
else()
	set(ON_NURBS_SOURCES ${ON_NURBS_SOURCES} src/on_nurbs/nurbs_solve_umfpack.cpp)
	set(ON_NURBS_LIBRARIES ${ON_NURBS_LIBRARIES} cholmod umfpack)
endif()

