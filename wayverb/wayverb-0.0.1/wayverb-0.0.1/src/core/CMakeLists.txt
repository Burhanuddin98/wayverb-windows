set(outfile ${CMAKE_CURRENT_BINARY_DIR}/hrtf_entries.h)
file(GLOB hrtf_wav_files "${CMAKE_CURRENT_SOURCE_DIR}/data/IRC_1002_C/*.wav")
if(hrtf_wav_files)
    # Real IRCAM data present — generate the header from it
    add_custom_command(OUTPUT ${outfile}
        COMMAND write_hrtf "${CMAKE_CURRENT_SOURCE_DIR}/data/IRC_1002_C" > ${outfile}
        DEPENDS write_hrtf
    )
else()
    # No data files — use the flat stub so the build can proceed
    message(STATUS "HRTF data not found in data/IRC_1002_C — using flat stub.")
    message(STATUS "Download IRC_1002_C from https://sofacoustics.org/data/database/ircam/ and rebuild for real HRTFs.")
    add_custom_command(OUTPUT ${outfile}
        COMMAND ${CMAKE_COMMAND} -E copy
            "${CMAKE_CURRENT_SOURCE_DIR}/hrtf_entries_stub.h"
            "${outfile}"
    )
endif()

file(GLOB_RECURSE sources "src/*.cpp")
file(GLOB_RECURSE public_headers "include/*.h")
file(GLOB_RECURSE private_headers "src/*.h")

add_library(core STATIC ${sources} ${public_headers} ${private_headers} ${outfile})

target_compile_definitions(core PUBLIC GLM_ENABLE_EXPERIMENTAL)

add_dependencies(core glm_external opencl_cpp_external cereal_external)

target_include_directories(core
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${WAYVERB_SOURCE_DIR}/include
    PRIVATE
    src
    ${DEPENDENCY_INSTALL_PREFIX}/include
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(core
    ${OpenCL_LIBRARIES}
    assimp
    zlibstatic 
    fftwf 
    audio_file 
    frequency_domain
    utilities
    hrtf
)

add_subdirectory(tests)
