project(startkit)
cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-pedantic -Wall -Wno-strict-aliasing -Wno-long-long -Wno-deprecated -Wno-deprecated-declarations")
set(DEV_CXX_FLAGS "-g -ggdb -O0 -fno-omit-frame-pointer")
set(RELEASE_CXX_FLAGS "-O3 -DNDEBUG")
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_FLAGS_DEBUG "${DEV_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "${RELEASE_CXX_FLAGS}")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -L../meshConverter/lib/lib_APPLE -lfade2d")

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/../)
file(GLOB SOURCES
        "meshConverter/fade2d/*.h"
        "meshConverter/fadeutils/*.h"
        "meshConverter/fadeutils/*.cpp"
        "meshConverter/*.h"
        "Polyanya/*.h"
        "Polyanya/structs/*.cpp"
        "Polyanya/helpers/*.cpp"
        "Polyanya/search/*.cpp"
        "Polyanya/structs/*.h"
        "Polyanya/helpers/*.h"
        "Polyanya/search/*.h"
        "validator/*.hpp"
        )


if(UNIX AND NOT APPLE)
    message(STATUS ">>> Linux")
#    file(GLOB LIBRARIES "meshConverter/lib/ubuntu16.10_x86_64/libfade2d.so")
    file(GLOB LIBRARIES "meshConverter/lib/lib_ubuntu18.04_x86_64/libfade2d.so")
endif ()

if(APPLE)
    message(STATUS ">>> Apple")
    file(GLOB LIBRARIES "meshConverter/lib/lib_APPLE/libfade2d.dylib")
endif()


message("LIBRARIES = ${LIBRARIES}")

include_directories("meshConverter")
include_directories("meshConverter/include_fade2d")
include_directories("meshConverter/fadeutils")
include_directories("Polyanya")
include_directories("Polyanya/search")
include_directories("Polyanya/structs")
include_directories("Polyanya/helpers")

set(SRC
        Astar.h
        Entry.h
        Entry.cpp
        GPPC.h
        ScenarioLoader.cpp
        ScenarioLoader.h
        Timer.cpp
        Timer.h
        )

add_executable(run ${SRC} ${SOURCES} main.cpp)
target_link_libraries(run ${LIBRARIES})
