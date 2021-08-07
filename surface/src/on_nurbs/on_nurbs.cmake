
set(ON_NURBS_INCLUDES
  include/pcl/${SUBSYS_NAME}/on_nurbs/closing_boundary.h
  include/pcl/${SUBSYS_NAME}/on_nurbs/fitting_curve_2d_apdm.h
  include/pcl/${SUBSYS_NAME}/on_nurbs/fitting_curve_2d_asdm.h
  include/pcl/${SUBSYS_NAME}/on_nurbs/fitting_curve_2d_atdm.h
  include/pcl/${SUBSYS_NAME}/on_nurbs/fitting_curve_2d_pdm.h
  include/pcl/${SUBSYS_NAME}/on_nurbs/fitting_curve_2d_sdm.h
  include/pcl/${SUBSYS_NAME}/on_nurbs/fitting_curve_2d_tdm.h
  include/pcl/${SUBSYS_NAME}/on_nurbs/fitting_curve_2d.h
  include/pcl/${SUBSYS_NAME}/on_nurbs/fitting_curve_pdm.h
  include/pcl/${SUBSYS_NAME}/on_nurbs/fitting_cylinder_pdm.h
  include/pcl/${SUBSYS_NAME}/on_nurbs/fitting_sphere_pdm.h
  include/pcl/${SUBSYS_NAME}/on_nurbs/fitting_surface_im.h
  include/pcl/${SUBSYS_NAME}/on_nurbs/fitting_surface_pdm.h
  include/pcl/${SUBSYS_NAME}/on_nurbs/fitting_surface_tdm.h
  include/pcl/${SUBSYS_NAME}/on_nurbs/global_optimization_pdm.h
  include/pcl/${SUBSYS_NAME}/on_nurbs/global_optimization_tdm.h
  include/pcl/${SUBSYS_NAME}/on_nurbs/nurbs_data.h
  include/pcl/${SUBSYS_NAME}/on_nurbs/nurbs_solve.h
  include/pcl/${SUBSYS_NAME}/on_nurbs/nurbs_tools.h
  include/pcl/${SUBSYS_NAME}/on_nurbs/sequential_fitter.h
  include/pcl/${SUBSYS_NAME}/on_nurbs/sparse_mat.h
  include/pcl/${SUBSYS_NAME}/on_nurbs/triangulation.h
)

set(ON_NURBS_SOURCES
  src/on_nurbs/closing_boundary.cpp
  src/on_nurbs/fitting_curve_2d_apdm.cpp
  src/on_nurbs/fitting_curve_2d_asdm.cpp
  src/on_nurbs/fitting_curve_2d_atdm.cpp
  src/on_nurbs/fitting_curve_2d_pdm.cpp
  src/on_nurbs/fitting_curve_2d_sdm.cpp
  src/on_nurbs/fitting_curve_2d_tdm.cpp
  src/on_nurbs/fitting_curve_2d.cpp
  src/on_nurbs/fitting_curve_pdm.cpp
  src/on_nurbs/fitting_cylinder_pdm.cpp
  src/on_nurbs/fitting_sphere_pdm.cpp
  src/on_nurbs/fitting_surface_im.cpp
  src/on_nurbs/fitting_surface_pdm.cpp
  src/on_nurbs/fitting_surface_tdm.cpp
  src/on_nurbs/global_optimization_pdm.cpp
  src/on_nurbs/global_optimization_tdm.cpp
  src/on_nurbs/nurbs_tools.cpp
  src/on_nurbs/sequential_fitter.cpp
  src/on_nurbs/sparse_mat.cpp
  src/on_nurbs/triangulation.cpp
)

set(USE_UMFPACK 0 CACHE BOOL "Use UmfPack for solving sparse systems of equations (e.g. in surface/on_nurbs)")
if(USE_UMFPACK)
  set(ON_NURBS_SOURCES ${ON_NURBS_SOURCES} src/on_nurbs/nurbs_solve_umfpack.cpp)
  set(ON_NURBS_LIBRARIES ${ON_NURBS_LIBRARIES} cholmod umfpack)
else()
  set(ON_NURBS_SOURCES ${ON_NURBS_SOURCES} src/on_nurbs/nurbs_solve_eigen.cpp)
endif()

