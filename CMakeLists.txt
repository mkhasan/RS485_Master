cmake_minimum_required(VERSION 2.8)

set (TARGET "RS485_Master")


set(ACE_DIR $ENV{ACE_ROOT})

set (SRC src/${TARGET}.cpp src/USBSERIAL_Linux.cpp)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories(${ACE_DIR})

link_directories (${ACE_DIR}/lib)

#message ("link dir: ${ACE_DIR}/lib")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb -Wall -fpermissive -Wno-narrowing")

add_executable(${TARGET} ${SRC})

target_link_libraries(${TARGET} ACE pthread)




