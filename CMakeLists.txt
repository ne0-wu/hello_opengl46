cmake_minimum_required(VERSION 3.25)

project(HelloOpenGL46)

set(CMAKE_CXX_STANDARD 17)

# OpenMesh
set(OPENMESH_DOCS OFF)
add_subdirectory(external/openmesh)
add_compile_definitions(_USE_MATH_DEFINES)

# Eigen
set(EIGEN_BUILD_DOC OFF)
set(BUILD_TESTING OFF)
add_subdirectory(external/eigen)

# MyGL
add_subdirectory(MyGL)

# Project
set(HEADERS
    Mesh.hh
    Dijkstra.hh
    MeshToGL.hh
)

set(SOURCES
    main.cc
    Dijkstra.cc
)

add_executable(${PROJECT_NAME}
    ${SOURCES}
    ${HEADERS}
)

target_link_libraries(${PROJECT_NAME}
    OpenMeshCore
    Eigen3::Eigen
    MyGL
)

set(DATA_DIR "${CMAKE_SOURCE_DIR}/data")
if(EXISTS ${DATA_DIR} AND IS_DIRECTORY ${DATA_DIR})
    add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${DATA_DIR}"
            "$<TARGET_FILE_DIR:${PROJECT_NAME}>/data"
        COMMENT "Copying data directory to output directory"
    )
endif()