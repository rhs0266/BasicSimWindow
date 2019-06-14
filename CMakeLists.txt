cmake_minimum_required(VERSION 3.5)
add_compile_options(-std=gnu++11)
add_compile_options(-Wdeprecated-declarations)
project(editor)

SET(CMAKE_BUILD_TYPE Release CACHE STRING
        "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
        # FORCE
        )
add_definitions(-DMSS_ROOT_DIR="${CMAKE_HOME_DIRECTORY}")

include(FindOpenMP)

find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR})

find_package(DART REQUIRED COMPONENTS gui optimizer-ipopt CONFIG)
add_compile_options(-DHAVE_CSTDDEF)
include_directories(${DART_INCLUDE_DIRS})


file(GLOB srcs
        "main.cpp"
#        "functions/*.h" "functions/*.cpp"
#        "generator/*.h" "generator/*.cpp"
#        "generator/motionAdjust/*.h" "generator/motionAdjust/*.cpp"
#        "model/*.h" "model/*.cpp"
        "render/*.hpp" "render/*.cpp"
#        "world/*.h" "world/*.cpp"
#        "simpleMotion/*.h" "simpleMotion/*.cpp"
#        "visualROM/*.h" "visualROM/*.cpp"
#        "editor/*.h" "editor/*.cpp"
#        "romEditor/*.h" "romEditor/*.cpp"
#        "APTOpt/*.h" "APTOpt/*.cpp")
        )

add_executable(editor ${srcs} render/GLUTWindow.cpp render/GLUTWindow.hpp render/Camera.cpp render/Camera.hpp render/SimWindow.cpp render/SimWindow.hpp)

target_link_libraries(editor ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} ${DART_LIBRARIES} /usr/lib/x86_64-linux-gnu/libtinyxml.so stdc++fs)
