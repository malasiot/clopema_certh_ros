SET ( ALG_SOURCES 
    recognition/ComputeDescriptors.cpp
    recognition/findLargestContour.cpp
    recognition/Matching.cpp
    recognition/IDSC_ShapeMatcher.cpp
    recognition/Prototypes.cpp
    recognition/ScanDir.cpp
)

SET ( ALG_HEADERS 
	${CMAKE_SOURCE_DIR}/include/certh_libs/ShapeMatcher.h
)

#linking 

rosbuild_add_library(certh_libs_alg ${ALG_SOURCES} ${ALG_HEADERS})
target_link_libraries(certh_libs_alg ${OpenCV_LIBRARIES})


