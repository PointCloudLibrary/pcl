# Template to make a tool
macro(define_composer_tool name TOOL_SOURCES TOOL_HEADERS DEPS)
    
  project(cc_tool_${name})

    source_group("Src" FILES ${TOOL_SOURCES} ${TOOL_HEADERS})
    
    QT4_WRAP_CPP(TOOL_HEADERS_MOC ${TOOL_HEADERS})
    set(TOOL_TARGET "pcl_cc_tool_${name}")
    PCL_ADD_LIBRARY(${TOOL_TARGET} ${SUBSYS_NAME} ${TOOL_SOURCES} ${TOOL_HEADERS_MOC})
    
   ADD_DEFINITIONS(${QT_DEFINITIONS})
   ADD_DEFINITIONS(-DQT_PLUGIN)
   ADD_DEFINITIONS(-DQT_NO_DEBUG)
   ADD_DEFINITIONS(-DQT_SHARED)
   
   add_dependencies(${TOOL_TARGET} pcl_cc_tool_interface ${DEPS})
   target_link_libraries(${TOOL_TARGET} pcl_cc_tool_interface pcl_common pcl_io ${DEPS} ${QT_LIBRARIES}) 
   
endmacro() 
