cmake_minimum_required (VERSION 3.15)

project ("JoyShockLibrary" CXX)

find_package (PkgConfig QUIET REQUIRED)
pkg_search_module (HIDAPI REQUIRED IMPORTED_TARGET hidapi-hidraw)

add_library (
    ${PROJECT_NAME}
    JoyShockLibrary/JoyShockLibrary.cpp    JoyShockLibrary/JoyShockLibrary.h
    JoyShockLibrary/InputHelpers.cpp
    JoyShockLibrary/JoyShock.cpp
    JoyShockLibrary/tools.cpp
)

target_include_directories (
    ${PROJECT_NAME} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_compile_definitions (
    ${PROJECT_NAME} PRIVATE
    -D$<UPPER_CASE:${BINARY_NAME}>_LIB
    -DSOURCE_DIR="${PROJECT_SOURCE_DIR}/JoyShockLibrary"
    -DBINARY_DIR="${PROJECT_BINARY_DIR}/JoyShockLibrary"
)

target_link_libraries (
    ${PROJECT_NAME} PRIVATE
)

target_link_libraries (
    ${PROJECT_NAME} PUBLIC
    PkgConfig::HIDAPI
)

