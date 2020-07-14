if (WIN32)
	enable_language (C)

    set (WINDOWS ON)
    set_property (GLOBAL PROPERTY USE_FOLDERS ON)

    include (cmake/CPM.cmake)

    if (MSVC)
        # Statically link the runtime libraries
		set (CMAKE_C_FLAGS "/DWIN32 /D_WINDOWS /W3")
		set (CMAKE_C_FLAGS_DEBUG "/MTd /Zi /Ob0 /Od /RTC1")
		set (CMAKE_C_FLAGS_RELEASE "/MT /O2 /Ob2 /DNDEBUG")

        set (
            MSVC_COMPILE_FLAGS
            CMAKE_CXX_FLAGS
            CMAKE_CXX_FLAGS_DEBUG
            CMAKE_CXX_FLAGS_RELEASE
            CMAKE_C_FLAGS
            CMAKE_C_FLAGS_DEBUG
            CMAKE_C_FLAGS_RELEASE
        )
        foreach (FLAG ${MSVC_COMPILE_FLAGS})
            string (REPLACE "/MD" "/MT" ${FLAG} "${${FLAG}}")
        endforeach ()
    endif ()

    add_library (
        jsl_platform_dependencies INTERFACE
    )

    target_compile_definitions (
        jsl_platform_dependencies INTERFACE
        -DGYROCONTROLLERLIBRARY_EXPORTS
        -DUNICODE
        -D_UNICODE
        -DNOMINMAX
        -DWIN32_LEAN_AND_MEAN
        -D_CRT_SECURE_NO_WARNINGS
        -D_WINDOWS
        -D_USRDLL
    )

    CPMAddPackage (
        NAME hidapi
        GITHUB_REPOSITORY signal11/hidapi
        VERSION 0
        GIT_TAG a6a622ffb680c55da0de787ff93b80280498330f
        DOWNLOAD_ONLY YES
    )

    if (hidapi_ADDED)
        add_library (
            hidapi STATIC
            ${hidapi_SOURCE_DIR}/windows/hid.c
        )

        target_include_directories (
            hidapi PUBLIC
            $<BUILD_INTERFACE:${hidapi_SOURCE_DIR}/hidapi>
        )

        target_link_libraries (
            hidapi PUBLIC
            setupapi
        )
    endif()

    target_link_libraries (
        jsl_platform_dependencies INTERFACE
        hidapi
    )

    add_library (JSL_Platform::Dependencies ALIAS jsl_platform_dependencies)
endif ()
