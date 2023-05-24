cmake_minimum_required(VERSION 3.14)

project(canlib LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core)
find_package(Qt6 REQUIRED COMPONENTS SerialPort)

add_library(canlib SHARED
  deviceprotocol/deviceprotocol.hpp
  canlib_global.h
  passthrudef.h
  passthru.cpp
  passthru.h
  driver.h
  driver.cpp
  canlib_global.cpp
  circularbuffer.hpp
)

target_link_libraries(canlib PRIVATE
  Qt${QT_VERSION_MAJOR}::Core
  Qt${QT_VERSION_MAJOR}::SerialPort)

target_compile_definitions(canlib PRIVATE CANLIB_LIBRARY)
