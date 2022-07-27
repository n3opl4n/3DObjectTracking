cmake_minimum_required(VERSION 3.10)

# Define sources
# =============================================================================
set(SOURCES
        common.cpp body.cpp renderer_geometry.cpp renderer.cpp
        normal_renderer.cpp basic_depth_renderer.cpp model.cpp region_model.cpp
        depth_model.cpp camera.cpp loader_camera.cpp viewer.cpp
        image_viewer.cpp normal_viewer.cpp modality.cpp region_modality.cpp
        depth_modality.cpp optimizer.cpp detector.cpp static_detector.cpp
        manual_detector.cpp refiner.cpp publisher.cpp tracker.cpp
        generator.cpp ros_camera.cpp)

if (USE_AZURE_KINECT)
    list(APPEND SOURCES azure_kinect_camera.cpp)
endif ()
if (USE_REALSENSE)
    list(APPEND SOURCES realsense_camera.cpp)
endif ()

# Define libraries
# =============================================================================
set(LIBRARIES Eigen3::Eigen OpenGL::GL GLEW::GLEW glfw ${OpenCV_LIBRARIES})
# set(LIBRARIES Eigen3::Eigen OpenGL::GL GLEW::GLEW glfw rclcpp::rclcpp sensor_msgs ${OpenCV_LIBRARIES})
if (USE_AZURE_KINECT)
    list(APPEND LIBRARIES k4a::k4a)
endif ()
if (USE_REALSENSE)
    list(APPEND LIBRARIES ${realsense2_LIBRARY})
endif ()
if(UNIX)
    list(APPEND LIBRARIES pthread stdc++fs)
endif()

# Define target
# =============================================================================
add_library(icg STATIC ${SOURCES})
ament_target_dependencies(icg 
    ament_cmake
    rclcpp
    sensor_msgs
    cv_bridge
    camera_utils_msgs
#     Eigen3
#     glfw3
#     GLEW
#     OpenGL
#     realsense2
#     OpenCV
)

target_link_libraries(icg ${LIBRARIES}) #PUBLIC
