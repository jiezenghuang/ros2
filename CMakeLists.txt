cmake_minimum_required(VERSION 3.2.0)

PROJECT(smart_car VERSION 0.0.1)

INCLUDE_DIRECTORIES(
    raspberrypi_car/include
)

#AUX_SOURCE_DIRECTORY(raspberrypi_car/src SRCS_DIR)

#ADD_EXECUTABLE(${PROJECT_NAME} ${SRCS_DIR})
ADD_EXECUTABLE(${PROJECT_NAME} main.cpp raspberrypi_car/src/smart_car.cpp)

TARGET_LINK_LIBRARIES(
    ${PROJECT_NAME}
    wiringPi
    pthread
)
