# Try to Find OpenGL and GLUT silently
# In addition sets two flags if the found versions are Apple frameworks
# OPENGL_IS_A_FRAMEWORK
# GLUT_IS_A_FRAMEWORK

if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()

find_package(OpenGL QUIET REQUIRED)

if(APPLE AND OPENGL_FOUND)
  if("${OPENGL_INCLUDE_DIR}" MATCHES "\\.framework")
    set(OPENGL_IS_A_FRAMEWORK TRUE)
  endif()
endif()

find_package(GLUT QUIET)

if(APPLE AND GLUT_FOUND)
  if("${GLUT_INCLUDE_DIR}" MATCHES "\\.framework")
    set(GLUT_IS_A_FRAMEWORK TRUE)
  endif()
endif()
