cmake_minimum_required(VERSION 3.10)

# 프로젝트 이름
project(CameraInterfaceTest)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# OpenCV 라이브러리 찾기
find_package(OpenCV REQUIRED)

# 실행 파일 이름 및 소스 파일 설정
add_executable(camera_test main.cc csi2_camera.cc v4l2_camera.cc)

# camera_interface 헤더 파일 경로 추가
target_include_directories(camera_test PRIVATE ${CMAKE_SOURCE_DIR})

# OpenCV 라이브러리 연결
target_link_libraries(camera_test PRIVATE ${OpenCV_LIBS} pthread)

# 컴파일 옵션 (필요 시 추가)
target_compile_options(camera_test PRIVATE -Wall -Wextra -Wpedantic)
