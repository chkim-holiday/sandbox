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


add_executable(
  test_rpi5 main_rpi5.cc)
target_link_libraries(test_rpi5 PRIVATE Threads::Threads lgpio)
install(TARGETS test_rpi5 DESTINATION bin)

# For Raspberry PI only
if(NOT EXISTS "/usr/include/pigpio.h")
  message(WARNING "pigpio.h not found. Skip RPibuild target.")
  return()
endif()
add_executable(
  serial_comm_improved_rpi4b main_improved_rpi4b.cc crc8_table.h
                             packet_serializer.h serial_communicator.h)
target_link_libraries(serial_comm_improved_rpi4b PRIVATE Threads::Threads
                                                         pigpio crc8_lib)
install(TARGETS serial_comm_improved_rpi4b DESTINATION bin)
