# Example management

# Add an option so the user can turn tests off
option(BUILD_EXAMPLES "Build the library examples" ${DEFAULT})

# Print a suitable status message
if(NOT ${DEFAULT} AND NOT ${BUILD_EXAMPLES})
    if(REASON)
        message(STATUS "Examples will not be built: ${REASON}")
    else(REASON)
        message(STATUS "Examples will not be built: Disabled manually")
    endif(REASON)
elseif(NOT ${BUILD_EXAMPLES})
    message(STATUS "Examples will not be built: Disabled manually")
else(NOT ${DEFAULT} AND NOT ${BUILD_EXAMPLES})
    message(STATUS "Examples will be built")
endif(NOT ${DEFAULT} AND NOT ${BUILD_EXAMPLES})
