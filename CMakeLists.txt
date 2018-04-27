cmake_minimum_required(VERSION 2.8)

set (TARGET "RS485_Master")


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_executable(${TARGET} src/${TARGET}.cpp )




