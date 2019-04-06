cmake_minimum_required(VERSION 2.8)

project(dart_urdf)

# enable C++ 11
add_definitions(-std=c++11)

add_definitions ("-Wall")

## find dependendies
find_package(urdfdom REQUIRED)          # reading URDF models
find_package(Boost REQUIRED filesystem) # traversing paths to find meshes
find_package(Doxygen)

find_package(dart REQUIRED)

if(DOXYGEN_FOUND)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_target(doc
    ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen" VERBATIM
    )
endif()

include_directories(${urdfdom_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(include)

link_directories(${urdfdom_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

## define and link library
add_library(dart_urdf SHARED src/read_model_urdf.cpp)

target_link_libraries(dart_urdf dart ${urdfdom_LIBRARIES} ${Boost_LIBRARIES})

include(GNUInstallDirs)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>/${PROJECT_NAME}")

set_target_properties(${PROJECT_NAME} PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_INSTALL_LIBDIR}
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_INSTALL_LIBDIR}
)

## install
install(TARGETS dart_urdf EXPORT dart_urdf LIBRARY DESTINATION lib)  # shared library
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(EXPORT dart_urdf
    DESTINATION share/${PROJECT_NAME}/cmake
    FILE ${PROJECT_NAME}Config.cmake
)

# setup pkg-config files
SET(PKG_CONFIG_REQUIRES "dart urdfdom")
SET(PKG_CONFIG_LIBDIR       "\${prefix}/lib" )
SET(PKG_CONFIG_INCLUDEDIR   "\${prefix}/include" )
SET(PKG_CONFIG_LIBS         "-L\${libdir} -ldart_urdf" )
SET(PKG_CONFIG_CFLAGS       "-I\${includedir}" )

CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/pkg-config.pc.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/dart_urdf.pc"
)

INSTALL(FILES "${CMAKE_BINARY_DIR}/dart_urdf.pc"
        DESTINATION lib/pkgconfig)
