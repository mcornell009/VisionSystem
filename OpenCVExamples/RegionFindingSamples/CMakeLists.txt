cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(Disparity_Comparison)

find_package(OpenCV REQUIRED)

add_executable (trackbar trackbar.cpp)
target_link_libraries (trackbar ${OpenCV_LIBS})

add_executable (findcontours findcontours.cpp)
target_link_libraries (findcontours ${OpenCV_LIBS})

add_executable (linefinding_sobel linefinding_sobel.cpp)
target_link_libraries (linefinding_sobel ${OpenCV_LIBS})

add_executable (linefinding_laplace linefinding_laplace.cpp)
target_link_libraries (linefinding_laplace ${OpenCV_LIBS})

add_executable (linefinding_canny linefinding_canny.cpp)
target_link_libraries (linefinding_canny ${OpenCV_LIBS})

add_executable (calcBackProject_Demo1 calcBackProject_Demo1.cpp)
target_link_libraries (calcBackProject_Demo1 ${OpenCV_LIBS})

add_executable (calcBackProject_Demo2 calcBackProject_Demo2.cpp)
target_link_libraries (calcBackProject_Demo2 ${OpenCV_LIBS})

add_executable (camshiftdemo camshiftdemo.cpp)
target_link_libraries (camshiftdemo ${OpenCV_LIBS})

add_executable (meanshift_segmentation meanshift_segmentation.cpp)
target_link_libraries (meanshift_segmentation ${OpenCV_LIBS})

add_executable (pyramid_segmentation pyramid_segmentation.c)
target_link_libraries (pyramid_segmentation ${OpenCV_LIBS})

add_executable (segment_objects segment_objects.cpp)
target_link_libraries (segment_objects ${OpenCV_LIBS})

add_executable (cornerDetector_Demo cornerDetector_Demo.cpp)
target_link_libraries (cornerDetector_Demo ${OpenCV_LIBS})

add_executable (cornerHarris_Demo cornerHarris_Demo.cpp)
target_link_libraries (cornerHarris_Demo ${OpenCV_LIBS})

add_executable (cornerSubPix_Demo cornerSubPix_Demo.cpp)
target_link_libraries (cornerSubPix_Demo ${OpenCV_LIBS})

add_executable (goodFeaturesToTrack_Demo goodFeaturesToTrack_Demo.cpp)
target_link_libraries (goodFeaturesToTrack_Demo ${OpenCV_LIBS})


add_executable (brox_optical_flow brox_optical_flow.cpp)
target_link_libraries (brox_optical_flow ${OpenCV_LIBS})

add_executable (farneback_optical_flow farneback_optical_flow.cpp)
target_link_libraries (farneback_optical_flow ${OpenCV_LIBS})

add_executable (pyrlk_optical_flow pyrlk_optical_flow.cpp)
target_link_libraries (pyrlk_optical_flow ${OpenCV_LIBS})

add_executable (mser_sample mser_sample.cpp)
target_link_libraries (mser_sample ${OpenCV_LIBS})

add_executable (ffilldemo ffilldemo.cpp)
target_link_libraries (ffilldemo ${OpenCV_LIBS})

add_executable (moments_demo moments_demo.cpp)
target_link_libraries (moments_demo ${OpenCV_LIBS})
