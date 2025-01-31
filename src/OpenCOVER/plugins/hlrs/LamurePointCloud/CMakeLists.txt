USING(LamurePointCloud)

SET(HEADERS
  LamurePointCloud.h
  LamureGeometry.h
  LamureDrawable.h
  LamureContext.h
  LamureDevice.h
  bvh.h
  bvh_stream.h
  cache.h
  cache_index.h
  cache_queue.h
  camera.h
  config.h
  controller.h
  cut.h
  cut_database.h
  cut_database_record.h
  cut_update_index.h
  cut_update_pool.h
  cut_update_queue.h
  data_provenance.h
  dataset.h
  gpu_access.h
  gpu_cache.h
  gpu_context.h
  item_provenance.h
  lod_stream.h
  model_database.h
  ooc_cache.h
  ooc_pool.h
  platform.h
  policy.h
  provenance_stream.h
  ray.h
  trackball.h
  atomic_counter.h
  bounding_box.h
  memory.h
  memory_status.h
  semaphore.h
  sphere.h
  types.h
  utils.h
  version.h
)

SET(SOURCES
  LamurePointCloud.cpp
  LamureGeometry.cpp
  LamureDrawable.cpp
  LamureContext.cpp
  LamureDevice.cpp
  bvh.cpp
  bvh_stream.cpp
  cache.cpp
  cache_index.cpp
  cache_queue.cpp
  camera.cpp
  controller.cpp
  cut.cpp
  cut_database.cpp
  cut_database_record.cpp
  cut_update_index.cpp
  cut_update_pool.cpp
  cut_update_queue.cpp
  dataset.cpp
  gpu_access.cpp
  gpu_cache.cpp
  gpu_context.cpp
  lod_stream.cpp
  model_database.cpp
  ooc_cache.cpp
  ooc_pool.cpp
  policy.cpp
  provenance_stream.cpp
  ray.cpp
  trackball.cpp
  bounding_box.cpp
  memory.cpp
  memory_status.cpp
  semaphore.cpp
  sphere.cpp
  utils.cpp
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

include_directories(SYSTEM 
  ${SCHISM_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR})


cover_add_plugin(LamurePointCloud)
covise_wnoerror(LamurePointCloud)

TARGET_LINK_LIBRARIES(LamurePointCloud
    optimized ${LAMURE_RENDERING_LIBRARY} debug ${LAMURE_RENDERING_LIBRARY_DEBUG}
    optimized ${LAMURE_COMMON_LIBRARY} debug ${LAMURE_COMMON_LIBRARY_DEBUG}
    optimized ${LAMURE_PVS_COMMON_LIBRARY} debug ${LAMURE_PVS_COMMON_LIBRARY_DEBUG}
    optimized ${LAMUREVT_LIBRARY} debug ${LAMUREVT_LIBRARY_DEBUG}
    optimized ${LAMURE_QUALITY_MEASUREMENT_LIBRARY} debug ${LAMURE_QUALITY_MEASUREMENT_LIBRARY_DEBUG}
    optimized ${LAMURE_PVS_PREPROCESSING_LIBRARY} debug ${LAMURE_PVS_PREPROCESSING_LIBRARY_DEBUG}
    optimized ${LAMURE_PREPROCESSING_LIBRARY} debug ${LAMURE_PREPROCESSING_LIBRARY_DEBUG}
    optimized ${SCHISM_CORE_LIBRARY} debug ${SCHISM_CORE_LIBRARY_DEBUG}
    optimized ${SCHISM_GL_CORE_LIBRARY} debug ${SCHISM_GL_CORE_LIBRARY_DEBUG}
    optimized ${SCHISM_GL_UTIL_LIBRARY} debug ${SCHISM_GL_UTIL_LIBRARY_DEBUG}
    optimized ${Boost_TIMER_LIBRARY_RELEASE} debug ${Boost_TIMER_LIBRARY_DEBUG}
)