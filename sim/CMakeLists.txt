cmake_minimum_required(VERSION 3.5)
project(sim)



SET(CMAKE_BUILD_TYPE Release CACHE STRING
        "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
        #	FORCE
        )

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

add_compile_options(-DHAVE_CSTDDEF)
include_directories(${DART_INCLUDE_DIRS})
#include_directories(${TinyXML_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})

file(GLOB srcs "*.hpp" "*.cpp")

add_library(sim ${srcs} BVH.cpp BVH.hpp Character.cpp Character.hpp)
target_link_libraries(sim ${DART_LIBRARIES} ${Boost_LIBRARIES} #[[${TinyXML_LIBRARIES}]] ${PYTHON_LIBRARIES})