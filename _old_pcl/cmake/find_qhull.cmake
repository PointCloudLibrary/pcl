if(WIN32)
    set(QHULL_INCLUDE_DIR "/usr" CACHE PATH "Location of the qhull includes.")
else(WIN32)
    set(QHULL_PREFIX "/usr" CACHE PATH "Location of the qhull install.")
endif(WIN32)

