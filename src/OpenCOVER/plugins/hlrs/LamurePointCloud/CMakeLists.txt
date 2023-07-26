set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR})

INCLUDE(${SRC_DIR}/pcl_makros.cmake)

USING(LamurePointCloud)


set(LAMURE_INCLUDE_DIRS
    $ENV{EXTERNLIBS}/lamure/include
    $ENV{EXTERNLIBS}/lamure/include/virtual_texturing/include
    $ENV{EXTERNLIBS}/lamure/include/rendering/include
    $ENV{EXTERNLIBS}/lamure/include/pvs_common/include
    $ENV{EXTERNLIBS}/lamure/include/common/include
    $ENV{EXTERNLIBS}/lamure/include/pvs_preprocessing/include
    $ENV{EXTERNLIBS}/lamure/include/provenance/include
    $ENV{EXTERNLIBS}/lamure/include/quality_measurement/include
)


#pcl_project_files(SOURCE_FILES      ${SRC_DIR}/virtual_texturing/src                        *.cpp)
#pcl_project_files(SOURCE_FILES      ${SRC_DIR}/virtual_texturing/src/..                     *.cpp)
#pcl_project_files(HEADER_FILES      ${SRC_DIR}/virtual_texturing/include/lamure/vt          *.h)
#pcl_project_files(HEADER_FILES      ${SRC_DIR}/virtual_texturing/include/lamure/vt/..       *.h)

#pcl_project_files(SOURCE_FILES      ${SRC_DIR}/rendering/src                                *.cpp)
#pcl_project_files(HEADER_FILES      ${SRC_DIR}/rendering/include/lamure/ren                 *.h)

#pcl_project_files(SOURCE_FILES      ${SRC_DIR}/pvs_common/src                               *.cpp)
#pcl_project_files(HEADER_FILES      ${SRC_DIR}/pvs_common/include/lamure/pvs                *.h)

#pcl_project_files(SOURCE_FILES      ${SRC_DIR}/common/src                                   *.cpp)
#pcl_project_files(HEADER_FILES      ${SRC_DIR}/common/include/lamure                        *.h)

#pcl_project_files(SOURCE_FILES      ${SRC_DIR}/pvs_preprocessing/src                        *.cpp)
#pcl_project_files(HEADER_FILES      ${SRC_DIR}/pvs_preprocessing/include/lamure/pvs         *.h)

#pcl_project_files(SOURCE_FILES      ${SRC_DIR}/provenance/src                               *.cpp)
#pcl_project_files(HEADER_FILES      ${SRC_DIR}/provenance/include/lamure/prov               *.h)

#pcl_project_files(SOURCE_FILES      ${SRC_DIR}/quality_measurement/src                      *.cpp)
#pcl_project_files(HEADER_FILES      ${SRC_DIR}/quality_measurement/include/lamure/qm        *.h)

pcl_project_files(SOURCE_FILES      ${SRC_DIR}/lamure                                       *.cpp)
pcl_project_files(HEADER_FILES      ${SRC_DIR}/lamure                                       *.h)

SET(HEADERS 
  ${HEADER_FILES}
  LamurePointCloud.h
  LamureGeometry.h
  LamureDrawable.h
  LamureContext.h
  LamureDevice.h
)

SET(SOURCES 
  ${SOURCE_FILES}
  LamurePointCloud.cpp
  LamureGeometry.cpp
  LamureDrawable.cpp
  #LamureContext.cpp
  #LamureDevice.cpp
)

set(SCHISM_INCLUDE_DIRS
    $ENV{EXTERNLIBS}/schism/include
    $ENV{EXTERNLIBS}/schism/include/scm_core/src
    $ENV{EXTERNLIBS}/schism/include/scm_gl_core/src
    $ENV{EXTERNLIBS}/schism/include/scm_gl_util/src
    $ENV{EXTERNLIBS}/schism/include/scm_input/src
    $ENV{EXTERNLIBS}/schism/include/scm_cl_core/src
)

#set(FREEGLUT_INCLUDE_DIR $ENV{EXTERNLIBS}/freeglut/include)
#set(FREEGLUT_LIBRARY $ENV{EXTERNLIBS}/freeglut/lib/x64/freeglut.lib)
#set(FREEIMAGE_INCLUDE_DIR $ENV{EXTERNLIBS}/FreeImage/include)
#set(FREEIMAGE_LIBRARY $ENV{EXTERNLIBS}/FreeImage/lib/FreeImage.lib)
#set(FREEIMAGE_LIBRARY_DEBUG $ENV{EXTERNLIBS}/FreeImage/lib/FreeImaged.lib)
#set(FREEIMAGEPLUS_LIBRARY $ENV{EXTERNLIBS}/FreeImage/lib/FreeImagePlus.lib)
#set(FREEIMAGEPLUS_LIBRARY_DEBUG $ENV{EXTERNLIBS}/FreeImage/lib/FreeImagePlusd.lib)
#set(FREETYPE_INCLUDE_DIR $ENV{EXTERNLIBS}/freetype/include)
#set(FREETYPE_LIBRARY_RELEASE $ENV{EXTERNLIBS}/freetype/lib/freetype.lib)
#set(FREETYPE_LIBRARY_DEBUG $ENV{EXTERNLIBS}/freetype/lib/freetyped.lib)

set(SCHISM_CORE_LIBRARY $ENV{EXTERNLIBS}/schism/lib/scm_core.lib)
set(SCHISM_CORE_LIBRARY_DEBUG $ENV{EXTERNLIBS}/schism/lib/scm_core-gd.lib)
set(SCHISM_GL_CORE_LIBRARY $ENV{EXTERNLIBS}/schism/lib/scm_gl_core.lib)
set(SCHISM_GL_CORE_LIBRARY_DEBUG $ENV{EXTERNLIBS}/schism/lib/scm_gl_core-gd.lib)
set(SCHISM_GL_UTIL_LIBRARY $ENV{EXTERNLIBS}/schism/lib/scm_gl_util.lib)
set(SCHISM_GL_UTIL_LIBRARY_DEBUG $ENV{EXTERNLIBS}/schism/lib/scm_gl_util-gd.lib)

set(GLEW_INCLUDE_DIR $ENV{EXTERNLIBS}/glew/include)
set(GLEW_LIBRARY $ENV{EXTERNLIBS}/glew/lib/glew32.lib)
set(GLFW_INCLUDE_DIR $ENV{EXTERNLIBS}/glfw/include)
set(GLFW_LIBRARY_RELEASE $ENV{EXTERNLIBS}/glfw/lib/glfw3.lib)
set(GLFW_LIBRARY_DEBUG $ENV{EXTERNLIBS}/glfw/lib/glfw3d.lib)


include_directories(
#${REND_INCLUDE_DIR}
#${COMMON_INCLUDE_DIR}
#${PVS_COMMON_INCLUDE_DIR}

${FREEGLUT_INCLUDE_DIR}
${GLM_INCLUDE_DIR}
${FREEIMAGE_INCLUDE_DIR}
${LAMURE_INCLUDE_DIRS}
${SCHISM_INCLUDE_DIRS}
${Boost_INCLUDE_DIR}
${GLEW_INCLUDE_DIR}
${GLFW_INCLUDE_DIR}
${FREETYPE_INCLUDE_DIR}
)


cover_add_plugin(LamurePointCloud)
covise_wnoerror(LamurePointCloud)

target_link_libraries(LamurePointCloud
#${FREEGLUT_LIBRARY}
optimized ${LAMURE_RENDERING_LIBRARY} 
debug ${LAMURE_RENDERING_LIBRARY_DEBUG}
optimized ${LAMURE_COMMON_LIBRARY} 
debug ${LAMURE_COMMON_LIBRARY_DEBUG}
optimized ${LAMURE_PVS_COMMON_LIBRARY} 
debug ${LAMURE_PVS_COMMON_LIBRARY_DEBUG}
optimized ${LAMUREVT_LIBRARY} 
debug ${LAMUREVT_LIBRARY_DEBUG}
optimized ${LAMURE_QUALITY_MEASUREMENT_LIBRARY} 
debug ${LAMURE_QUALITY_MEASUREMENT_LIBRARY_DEBUG}
optimized ${LAMURE_PVS_PREPROCESSING_LIBRARY} 
debug ${LAMURE_PVS_PREPROCESSING_LIBRARY_DEBUG}
optimized ${LAMURE_PREPROCESSING_LIBRARY} 
debug ${LAMURE_PREPROCESSING_LIBRARY_DEBUG}
optimized ${FREEIMAGEPLUS_LIBRARY} 
debug ${FREEIMAGEPLUS_LIBRARY_DEBUG}
optimized ${GLEW_LIBRARY} 
debug ${GLEW_LIBRARY}
optimized ${FREETYPE_LIBRARY_RELEASE} 
debug ${FREETYPE_LIBRARY_DEBUG}
optimized ${GLFW_LIBRARY_RELEASE} 
debug ${GLFW_LIBRARY_DEBUG}
optimized ${SCHISM_CORE_LIBRARY} 
debug ${SCHISM_CORE_LIBRARY_DEBUG}
optimized ${SCHISM_GL_CORE_LIBRARY} 
debug ${SCHISM_GL_CORE_LIBRARY_DEBUG}
optimized ${SCHISM_GL_UTIL_LIBRARY} 
debug ${SCHISM_GL_UTIL_LIBRARY_DEBUG}
optimized ${Boost_TIMER_LIBRARY_RELEASE} 
debug ${Boost_TIMER_LIBRARY_DEBUG}
optimized ${FREEIMAGE_LIBRARY} 
debug ${FREEIMAGE_LIBRARY_DEBUG}
)

