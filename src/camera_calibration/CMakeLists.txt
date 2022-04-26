cmake_minimum_required(VERSION 3.8)
project(camera_calibration)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)

#add_executable(CameraCalibrationNode src/CameraCalibrationNode.cpp)
ament_auto_add_library(CameraCalibrationNode SHARED
        include/camera_calibration/CameraCalibrationNode.hpp
        src/CameraCalibrationNode.cpp
        )

include_directories(
        include
        SYSTEM
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(CameraCalibrationNode ${OpenCV_LIBS})

target_include_directories(CameraCalibrationNode PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(CameraCalibrationNode PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS CameraCalibrationNode
        DESTINATION lib/${PROJECT_NAME})

rclcpp_components_register_node(CameraCalibrationNode
        PLUGIN "CameraCalibration::CameraCalibrationNode"
        EXECUTABLE camera_calibration_node_exe
        )

ament_auto_package(
        INSTALL_TO_SHARE
        launch
        param
)
