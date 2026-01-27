cmake_minimum_required(VERSION 3.10)
project(ethernet_ptp_test)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

# 컴파일 옵션
add_compile_options(-Wall -Wextra)

# 실행 파일 생성
add_executable(trigger_test main_trigger.cc )
target_link_libraries(trigger_test PRIVATE Threads::Threads)
install(TARGETS trigger_test DESTINATION bin)

# For Raspberry PI4B only
if(EXISTS "/usr/include/pigpio.h")
  message("pigpio.h found. build RPI4Btarget.")
  add_executable(
    test_rpi4 main_rpi4b.cc)
  target_link_libraries(test_rpi4 PRIVATE Threads::Threads pigpio)
  install(TARGETS test_rpi4 DESTINATION bin)
endif()

# For Raspberry PI5 only
if(EXISTS "/usr/include/lgpio.h")
  message("lgpio.h found. build RPI5 target.")
  add_executable(
    test_rpi5 main_rpi5.cc)
    target_link_libraries(test_rpi5 PRIVATE Threads::Threads lgpio)
    install(TARGETS test_rpi5 DESTINATION bin)
endif()
