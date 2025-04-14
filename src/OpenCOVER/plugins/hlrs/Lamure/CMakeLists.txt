USING(Lamure)
USING(Freeimage)

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR})

set(SCHISM_INCLUDE_DIRS
    $ENV{EXTERNLIBS}/../../schism
    $ENV{EXTERNLIBS}/../../schism/scm_core/src
    $ENV{EXTERNLIBS}/../../schism/scm_gl_core/src
    $ENV{EXTERNLIBS}/../../schism/scm_gl_util/src
    $ENV{EXTERNLIBS}/../../schism/scm_input/src
    $ENV{EXTERNLIBS}/../../schism/scm_cl_core/src
)

include_directories(
    ${SCHISM_INCLUDE_DIRS}
)

SET(HEADERS
  Lamure.h
  LamureDrawable.h
  management.h
  measurement.h
  renderer.h
  utils.h
)
SET(SOURCES
  Lamure.cpp
  LamureDrawable.cpp
  management.cpp
  measurement.cpp
  renderer.cpp
  utils.cpp
)
cover_add_plugin(Lamure)
covise_wnoerror(Lamure)
TARGET_LINK_LIBRARIES(Lamure)
