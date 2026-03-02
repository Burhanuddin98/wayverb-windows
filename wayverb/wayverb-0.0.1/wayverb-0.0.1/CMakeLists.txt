cmake_minimum_required(VERSION 3.5)

if(POLICY CMP0063)
    cmake_policy(SET CMP0063 NEW)
endif()

project(WAYVERB VERSION 0.1 LANGUAGES CXX)

set(ONLY_BUILD_DOCS false CACHE BOOL "skip configuring the build and just build docs")

if(ONLY_BUILD_DOCS)
    find_package(Doxygen)
    if(DOXYGEN_FOUND)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config/Doxyfile.in
            ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
        add_custom_target(doc ALL
            ${DOXYGEN_EXECUTABLE}
            ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
            WORKING_DIRECTORY
            ${CMAKE_CURRENT_BINARY_DIR})
    endif(DOXYGEN_FOUND)
else()
    set(CMAKE_VERBOSE_MAKEFILE OFF)

    include(${CMAKE_CURRENT_SOURCE_DIR}/config/dependencies.cmake)

    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE Release)
    endif()

    if(NOT MSVC)
        set(CMAKE_CXX_VISIBILITY_PRESET hidden)
        set(CMAKE_VISIBILITY_INLINES_HIDDEN true)
        add_compile_options(-ftemplate-depth=1024)
    endif()

    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

    #set(CMAKE_CXX_VISIBILITY_PRESET hidden)
    #set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

    enable_testing()
    add_subdirectory(src)
    add_subdirectory(bin)
endif()
