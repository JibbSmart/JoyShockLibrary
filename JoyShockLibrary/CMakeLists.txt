add_library (
    ${PROJECT_NAME}
    JoyShockLibrary.cpp    JoyShockLibrary.h
)

target_include_directories (
    ${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries (
    ${PROJECT_NAME} ${JSL_PLATFORM_DEPENDENCY_VISIBILITY}
    JSL_Platform::Dependencies
)
