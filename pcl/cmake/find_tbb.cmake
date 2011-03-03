if(WIN32)
    set(TBB_INCLUDE_DIR "/usr" CACHE PATH "Location of the TBB includes.")
    set(TBB_LIB_DIR "/usr" CACHE PATH "Location of the TBB libraries.")
else(WIN32)
    include(CheckIncludeFileCXX)
    check_include_file_cxx(tbb/tbb.h HAVE_TBB)
endif(WIN32)

