set(SUBSYS_NAME gpu_people)
set(SUBSYS_PATH gpu/people)
set(SUBSYS_DESC "Point cloud GPU people library")
set(SUBSYS_DEPS common geometry io filters sample_consensus segmentation visualization kdtree features surface octree registration keypoints tracking search people)

# OpenNI found?
if(NOT OPENNI_FOUND)
    set(DEFAULT FALSE)
    set(REASON "OpenNI was not found.")
else(NOT OPENNI_FOUND)
    set(DEFAULT TRUE)
    set(REASON)
endif(NOT OPENNI_FOUND)

PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} EXT_DEPS openni)

if(build)
  add_subdirectory(apps)
endif(build)
