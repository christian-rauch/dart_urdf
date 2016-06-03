cmake_minimum_required(VERSION 2.8)

project(dart_urdf)

# enable C++ 11
add_definitions(-std=c++11)

## find dependendies
find_package(urdfdom REQUIRED)          # reading URDF models
find_package(CUDA REQUIRED)             # CUDA types used by DART
find_package(Boost REQUIRED filesystem) # traversing paths to find meshes

set( ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${DART_INSTALL_ROOT}/lib/pkgconfig" )

find_package(PkgConfig)
pkg_check_modules(dart REQUIRED dart)

if(NOT dart_FOUND)
    message(STATUS
        "You can specify the path to your DART installation by cmake variable DART_INSTALL_ROOT, "
        "e.g. 'cmake -DDART_INSTALL_ROOT=/path/to/dart/install/dir ..'"
        )
endif()


include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${CUDA_TOOLKIT_ROOT_DIR}/samples/common/inc)

include_directories(${dart_INCLUDE_DIRS})
# TODO: we need to add "dart" subdirectory because dart's header files use absolute paths
# e.g. it is using '#include "model/model.h"' instead of '#include <dart/model/model.h>'
include_directories(${dart_INCLUDE_DIRS}/dart)

include_directories(${urdfdom_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(include)

link_directories(${dart_LIBRARY_DIRS})
link_directories(${urdfdom_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

## define sources and headers

set(SRC_LIST
    src/read_model_urdf.cpp
    )

set(HDR_LIST
    include/read_model_urdf.h
    )

## define and link library
add_library(dart_urdf SHARED ${SRC_LIST})

target_link_libraries(dart_urdf ${dart_LIBRARIES})
target_link_libraries(dart_urdf ${urdfdom_LIBRARIES})
target_link_libraries(dart_urdf ${Boost_LIBRARIES})

## install
install(TARGETS dart_urdf LIBRARY DESTINATION lib)  # shared library
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
