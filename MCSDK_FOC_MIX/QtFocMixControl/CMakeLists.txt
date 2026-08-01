cmake_minimum_required(VERSION 3.21)

project(QtFocMixControl VERSION 2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets)

qt_add_executable(QtFocMixControl WIN32
    src/main.cpp
    src/mainwindow.cpp
    src/mainwindow.h
    src/positiondial.cpp
    src/positiondial.h
    src/serialtransport.cpp
    src/serialtransport.h
    src/aspepprotocol.cpp
    src/aspepprotocol.h
)

target_link_libraries(QtFocMixControl PRIVATE Qt6::Widgets)
target_compile_definitions(QtFocMixControl PRIVATE
    QT_NO_CAST_FROM_ASCII
    QT_NO_CAST_TO_ASCII
)

if(WIN32)
    target_compile_definitions(QtFocMixControl PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
endif()
