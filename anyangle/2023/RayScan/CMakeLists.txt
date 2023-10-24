cmake_minimum_required(VERSION 3.13)

project( RayScan  VERSION 1.0  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

add_executable(run
	main.cpp
	Entry.cpp
	ScenarioLoader.cpp
	Timer.cpp)
target_include_directories(run PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

add_subdirectory(rayscan)
