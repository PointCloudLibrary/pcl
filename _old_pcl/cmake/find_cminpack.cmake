if(WIN32)
    set(CMINPACK_INCLUDE_DIR "/usr" CACHE PATH "Location of the cminpack includes.")
else(WIN32)
    set(CMINPACK_PREFIX "/usr" CACHE PATH "Location of the cminpack install.")
endif(WIN32)

