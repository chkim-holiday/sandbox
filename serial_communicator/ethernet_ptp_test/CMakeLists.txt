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

# For Raspberry PI only
if(NOT EXISTS "/usr/include/pigpio.h")
  message(WARNING "pigpio.h not found. Skip RPibuild target.")
  return()
endif()
add_executable(
  test_rpi4 main_rpi4b.cc)
target_link_libraries(test_rpi4 PRIVATE Threads::Threads pigpio)
install(TARGETS test_rpi4 DESTINATION bin)

# For Raspberry PI only
if(NOT EXISTS "/usr/include/lgpio.h")
  message(WARNING "lgpio.h not found. Skip RPibuild target.")
  return()
endif()
add_executable(
  test_rpi5 main_rpi5.cc)
target_link_libraries(test_rpi5 PRIVATE Threads::Threads lgpio)
install(TARGETS test_rpi5 DESTINATION bin)
