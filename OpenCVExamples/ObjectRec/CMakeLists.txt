cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(Disparity_Comparison)

find_package(OpenCV REQUIRED)

add_executable (descriptor_extractor_matcher descriptor_extractor_matcher.cpp)
target_link_libraries (descriptor_extractor_matcher ${OpenCV_LIBS})

add_executable (detection_based_tracker_sample detection_based_tracker_sample.cpp)
target_link_libraries (detection_based_tracker_sample ${OpenCV_LIBS})

add_executable (detector_descriptor_evaluation detector_descriptor_evaluation.cpp)
target_link_libraries (detector_descriptor_evaluation ${OpenCV_LIBS})

add_executable (detector_descriptor_matcher_evaluation detector_descriptor_matcher_evaluation.cpp)
target_link_libraries (detector_descriptor_matcher_evaluation ${OpenCV_LIBS})

add_executable (generic_descriptor_match generic_descriptor_match.cpp)
target_link_libraries (generic_descriptor_match ${OpenCV_LIBS})

add_executable (matching_to_many_images matching_to_many_images.cpp)
target_link_libraries (matching_to_many_images ${OpenCV_LIBS})

add_executable (peopledetect peopledetect.cpp)
target_link_libraries (peopledetect ${OpenCV_LIBS})

#add_executable (SURF_FlannMatcher SURF_FlannMatcher.cpp)
#target_link_libraries (SURF_FlannMatcher ${OpenCV_LIBS})

#add_executable (SURF_Homography SURF_Homography.cpp)
#target_link_libraries (SURF_Homography ${OpenCV_LIBS})



