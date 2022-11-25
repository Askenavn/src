cmake_minimum_required(VERSION 3.0)

project(video) # название проекта

find_package( OpenCV REQUIRED ) # поиcк зависимостей

include_directories( ${OpenCV_INCLUDE_DIRS} ) # включать в дирректории опенсв

add_executable(new main.cpp) # название файлов

target_link_libraries(new  PUBLIC ${OpenCV_LIBS})