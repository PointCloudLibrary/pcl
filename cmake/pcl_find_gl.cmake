# Try to Find OpenGL and GLUT silently
# In addition sets two flags if the found versions are Apple frameworks
# OPENGL_IS_A_FRAMEWORK
# GLUT_IS_A_FRAMEWORK

find_package(OpenGL QUIET REQUIRED)

if(APPLE AND OPENGL_FOUND)
  if ("${OPENGL_INCLUDE_DIR}" MATCHES "\\.framework")
    set(OPENGL_IS_A_FRAMEWORK TRUE)
  endif ("${OPENGL_INCLUDE_DIR}" MATCHES "\\.framework")
endif(APPLE AND OPENGL_FOUND)

find_package(GLUT QUIET)

if(APPLE AND GLUT_FOUND)
  if ("${GLUT_INCLUDE_DIR}" MATCHES "\\.framework")
    set(GLUT_IS_A_FRAMEWORK TRUE)
  endif ("${GLUT_INCLUDE_DIR}" MATCHES "\\.framework")
endif(APPLE AND GLUT_FOUND)
