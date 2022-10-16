#OpenCOVER plugins calit2 PointCloud
USING(E57 optional)

SET(HEADERS
  PointCloud.h
  PointCloudGeometry.h
  PointCloudInteractor.h
  Point.h	
)

SET(HEADERS
)
SET(SOURCES
  PointLOD/PointLOD.cpp
)

ADD_COVISE_EXECUTABLE(PointLOD)
COVISE_WNOERROR(PointLOD)

TARGET_LINK_LIBRARIES(PointLOD ${EXTRA_LIBS} ${OPENSCENEGRAPH_LIBRARIES} )

IF(UNIX OR MINGW)
      SET(WITH_OPENMP "TRUE")
      IF(MINGW)
         SET(WITH_OPENMP "FALSE")
      ENDIF()
      IF(APPLE)
          IF (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
            SET(WITH_OPENMP "FALSE")
         ENDIF()
      ENDIF(APPLE)

      IF(WITH_OPENMP)
         ADD_COVISE_COMPILE_FLAGS(PointLOD "-fopenmp")
         ADD_COVISE_LINK_FLAGS(PointLOD "-fopenmp")
      ENDIF()
ENDIF(UNIX OR MINGW)

IF(MSVC)
      ADD_COVISE_COMPILE_FLAGS(PointLOD "/openmp")
ENDIF(MSVC)


if (NOT E57_FOUND)
    return()
endif()
