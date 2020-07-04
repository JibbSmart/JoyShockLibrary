cmake_minimum_required (VERSION 3.15)

project ("JoyShockLibrary" CXX)

find_package (PkgConfig QUIET REQUIRED)
pkg_search_module (HIDAPI REQUIRED IMPORTED_TARGET hidapi-hidraw)

add_library (
    ${PROJECT_NAME}
    JoyShockLibrary/JoyShockLibrary.cpp    JoyShockLibrary/JoyShockLibrary.h
#    JoyShockLibrary/InputHelpers.cpp
#    JoyShockLibrary/JoyShock.cpp
#    JoyShockLibrary/tools.cpp
)

target_link_libraries (
    ${PROJECT_NAME} PRIVATE
)

target_link_libraries (
    ${PROJECT_NAME} PUBLIC
    PkgConfig::HIDAPI
)

