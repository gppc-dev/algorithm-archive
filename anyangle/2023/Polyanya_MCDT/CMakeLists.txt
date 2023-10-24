project(startkit)
cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w -pedantic -Wall -Wno-strict-aliasing -Wno-long-long -Wno-deprecated -Wno-deprecated-declarations")
set(DEV_CXX_FLAGS "-g -ggdb -O0 -fno-omit-frame-pointer")
set(RELEASE_CXX_FLAGS "-O3 -DNDEBUG")
#set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_FLAGS_DEBUG "${DEV_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "${RELEASE_CXX_FLAGS}")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -L../meshConverter/lib/lib_APPLE -lfade2d")

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/../)
file(GLOB SOURCES
        "meshConverter/include_cdt/*.h"
        "meshConverter/fade2d/*.h"
        "meshConverter/CDTutils/*.h"
        "meshConverter/CDTutils/*.cpp"
        "meshConverter/fadeutils/*.h"
        "meshConverter/fadeutils/*.cpp"
        "meshConverter/*.h"
        "End_Point_Search/*.h"
        "End_Point_Search/structs/*.cpp"
        "End_Point_Search/helpers/*.cpp"
        "End_Point_Search/search/*.cpp"
        "End_Point_Search/structs/*.h"
        "End_Point_Search/helpers/*.h"
        "End_Point_Search/search/*.h"
        "validator/*.hpp"
        )


if(UNIX AND NOT APPLE)
    message(STATUS ">>> Linux")
    #    file(GLOB LIBRARIES "meshConverter/lib/ubuntu16.10_x86_64/libfade2d.so")
    file(GLOB LIBRARIES "meshConverter/lib/lib_ubuntu18.04_x86_64/libfade2d.so")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/usr/include -fopenmp")
endif ()

if(APPLE)
    message(STATUS ">>> Apple")
    file(GLOB LIBRARIES "meshConverter/lib/lib_APPLE/libfade2d.dylib")
    find_library(omp_lib omp "/usr/local/opt")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Xpreprocessor -fopenmp")
    include_directories("/usr/local/Cellar/libomp/10.0.0/include")
endif()



message("LIBRARIES = ${LIBRARIES}")

include_directories("meshConverter")
include_directories("meshConverter/include_cdt")
include_directories("meshConverter/include_fade2d")
include_directories("meshConverter/fadeutils")
include_directories("meshConverter/CDTutils")
include_directories("End_Point_Search")
include_directories("End_Point_Search/search")
include_directories("End_Point_Search/structs")
include_directories("End_Point_Search/helpers")

set(SRC
        ThetaStar.h
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
if (omp_lib)
    target_link_libraries(run ${omp_lib})
endif()