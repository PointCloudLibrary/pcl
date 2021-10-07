# Template to make a tool
function(define_composer_tool TOOL_NAME TOOL_SOURCES TOOL_HEADERS DEPS)

  project(pcl_cc_tool_${TOOL_NAME})

  set(TOOL_TARGET pcl_cc_tool_${TOOL_NAME})
  PCL_ADD_LIBRARY(${TOOL_TARGET} COMPONENT ${SUBSYS_NAME} SOURCES ${TOOL_SOURCES} ${TOOL_HEADERS})
  
  if(WIN32)
    set_target_properties (${TOOL_TARGET} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CLOUD_COMPOSER_PLUGIN_DIR}
                                                    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CLOUD_COMPOSER_PLUGIN_DIR})
  else()
    set_target_properties (${TOOL_TARGET} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CLOUD_COMPOSER_PLUGIN_DIR})
  endif()
  
  add_definitions(${QT_DEFINITIONS})
  add_definitions(-DQT_PLUGIN)
  add_definitions(-DQT_SHARED)

  target_link_libraries(${TOOL_TARGET} pcl_cc_tool_interface pcl_common pcl_io ${DEPS} ${QTX}::Widgets)

  if(APPLE)
    set_target_properties(${TOOL_TARGET} PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
  endif()

endfunction()
