cmake_minimum_required(VERSION 3.10)
project(serial_communicator)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

# 컴파일 옵션
add_compile_options(-Wall -Wextra)

# CRC8 소스 파일 생성 (헤더 전용이 아닌 경우)
add_library(crc8_lib INTERFACE)
target_include_directories(crc8_lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

# 실행 파일 생성
add_executable(serial_comm main.cc crc8_table.h packet_parser.h
                           serial_communicator.h)
target_link_libraries(serial_comm PRIVATE Threads::Threads crc8_lib)
install(TARGETS serial_comm DESTINATION bin)

add_executable(serial_comm_improved main_improved.cc crc8_table.h
                                    packet_serializer.h serial_communicator.h)
target_link_libraries(serial_comm_improved PRIVATE Threads::Threads crc8_lib)
install(TARGETS serial_comm_improved DESTINATION bin)

# For Raspberry PI only
if(NOT EXISTS "/usr/include/pigpio.h")
  message(
    WARNING "pigpio.h not found. Skipping Raspberry Pi specific build target.")
  return()
endif()
add_executable(
  serial_comm_improved_rpi4b main_improved_rpi4b.cc crc8_table.h
                             packet_serializer.h serial_communicator.h)
target_link_libraries(serial_comm_improved_rpi4b PRIVATE Threads::Threads
                                                         pigpio crc8_lib)
install(TARGETS serial_comm_improved_rpi4b DESTINATION bin)
