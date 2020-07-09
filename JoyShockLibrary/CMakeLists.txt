add_library (
    ${PROJECT_NAME} SHARED
    JoyShockLibrary.cpp    JoyShockLibrary.h
)

target_include_directories (
    ${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries (
    ${PROJECT_NAME} PRIVATE
)

if (WIN32)
    target_include_directories (
        ${PROJECT_NAME} PRIVATE
        hidapi
    )

    target_compile_definitions (
        ${PROJECT_NAME} PRIVATE
        -DGYROCONTROLLERLIBRARY_EXPORTS
        -D_WINDOWS
        -D_USRDLL
    )

    if (CMAKE_GENERATOR_PLATFORM STREQUAL "Win32")
        target_link_libraries (
            ${PROJECT_NAME} PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/hidapi/objs/x86/hid.obj
        )
    else ()
        target_link_libraries (
            ${PROJECT_NAME} PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/hidapi/objs/x64/hid.obj
        )
    endif ()
endif ()

if (UNIX AND NOT APPLE)
    find_package (PkgConfig QUIET REQUIRED)
    pkg_search_module (HIDAPI REQUIRED IMPORTED_TARGET hidapi-hidraw)

    target_link_libraries (
        ${PROJECT_NAME} PUBLIC
        PkgConfig::HIDAPI
    )
endif ()
