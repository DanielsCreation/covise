USING(LamurePointCloud)

SET(HEADERS
  LamurePointCloud.h

)
SET(SOURCES
  LamurePointCloud.cpp
)

include_directories(${REND_INCLUDE_DIR}
                    ${COMMON_INCLUDE_DIR}
                    ${PVS_COMMON_INCLUDE_DIR}
			        ${FREEGLUT_INCLUDE_DIR}
                    ${GLUT_INCLUDE_DIR}
		            ${GLM_INCLUDE_DIR}
                    ${FREEIMAGE_INCLUDE_DIR}
	                ${LAMURE_INCLUDE_DIR}
                    ${SCHISM_INCLUDE_DIRS}
                    ${GLEW_INCLUDE_DIR}
                    ${Boost_INCLUDE_DIR}
)



cover_add_plugin(LamurePointCloud)
covise_wnoerror(LamurePointCloud)
TARGET_LINK_LIBRARIES(LamurePointCloud)