cmake_minimum_required(VERSION 3.10)
project(wayverb)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Paths ─────────────────────────────────────────────────────────────────────
set(ROOT_DIR   "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(BUILD_DIR  "${ROOT_DIR}/build_win")
set(MGU_DIR    "${ROOT_DIR}/modern_gl_utils")

# ── Sources ────────────────────────────────────────────────────────────────────
set(JUCE_SRCS
    JuceLibraryCode/BinaryData.cpp
    JuceLibraryCode/juce_audio_basics.cpp
    JuceLibraryCode/juce_audio_devices.cpp
    JuceLibraryCode/juce_audio_formats.cpp
    JuceLibraryCode/juce_audio_processors.cpp
    JuceLibraryCode/juce_core.cpp
    JuceLibraryCode/juce_cryptography.cpp
    JuceLibraryCode/juce_data_structures.cpp
    JuceLibraryCode/juce_events.cpp
    JuceLibraryCode/juce_graphics.cpp
    JuceLibraryCode/juce_gui_basics.cpp
    JuceLibraryCode/juce_gui_extra.cpp
    JuceLibraryCode/juce_opengl.cpp
    JuceLibraryCode/juce_video.cpp
)

file(GLOB_RECURSE APP_SRCS "Source/*.cpp")

add_executable(wayverb WIN32
    ${JUCE_SRCS}
    ${APP_SRCS}
)

# ── Compile definitions ───────────────────────────────────────────────────────
target_compile_definitions(wayverb PRIVATE
    JUCE_GLOBAL_MODULE_SETTINGS_INCLUDED=1
    JUCE_STANDALONE_APPLICATION=1
    JUCE_APP_VERSION="1.1.0"
    JUCE_APP_VERSION_HEX=0x10100
    UNICODE=1
    _UNICODE=1
    GLEW_STATIC
    GLM_ENABLE_EXPERIMENTAL
    GLM_FORCE_CTOR_INIT
    # Disable features that need extra dependencies we haven't set up
    JUCE_DIRECTSHOW=0
    JUCE_MEDIAFOUNDATION=0
    JUCE_QUICKTIME=0
    JUCE_USE_CAMERA=0
    JUCE_WEB_BROWSER=0
    JUCE_USE_CURL=0
    JUCE_INCLUDE_ZLIB_CODE=1
    JUCE_USE_OGGVORBIS=0
    JUCE_USE_FLAC=0
    JUCE_USE_MP3AUDIOFORMAT=0
)

# ── Include directories ───────────────────────────────────────────────────────
target_include_directories(wayverb PRIVATE
    # JUCE
    "${CMAKE_CURRENT_SOURCE_DIR}/JuceLibraryCode"
    "${CMAKE_CURRENT_SOURCE_DIR}/JuceLibraryCode/modules"
    # Source subdirectories that include each other with relative paths
    "${CMAKE_CURRENT_SOURCE_DIR}/Source"
    "${CMAKE_CURRENT_SOURCE_DIR}/Source/UtilityComponents"
    "${CMAKE_CURRENT_SOURCE_DIR}/Source/3d_objects"
    "${CMAKE_CURRENT_SOURCE_DIR}/Source/left_bar"
    "${CMAKE_CURRENT_SOURCE_DIR}/Source/output"
    "${CMAKE_CURRENT_SOURCE_DIR}/Source/scene"
    "${CMAKE_CURRENT_SOURCE_DIR}/Source/results"
    # Wayverb libraries (source include dirs)
    "${ROOT_DIR}/src/utilities/include"
    "${ROOT_DIR}/src/core/include"
    "${ROOT_DIR}/src/audio_file/include"
    "${ROOT_DIR}/src/frequency_domain/include"
    "${ROOT_DIR}/src/hrtf/lib/include"
    "${ROOT_DIR}/src/raytracer/include"
    "${ROOT_DIR}/src/waveguide/include"
    "${ROOT_DIR}/src/waveguide/itpp_compat"
    "${ROOT_DIR}/src/combined/include"
    # Generated headers (hrtf_entries.h, etc.)
    "${BUILD_DIR}/src/core"
    # modern_gl_utils
    "${MGU_DIR}/include"
    # System / package headers
    "C:/msys64/mingw64/include"
)

# ── Link libraries ─────────────────────────────────────────────────────────────
target_link_libraries(wayverb PRIVATE
    # Wayverb static libraries (in correct link order)
    "${BUILD_DIR}/lib/libcombined.a"
    "${BUILD_DIR}/lib/libraytracer.a"
    "${BUILD_DIR}/lib/libwaveguide.a"
    "${BUILD_DIR}/lib/libcore.a"
    "${BUILD_DIR}/lib/libfrequency_domain.a"
    "${BUILD_DIR}/lib/libcompensation_signal.a"
    "${BUILD_DIR}/lib/libaudio_file.a"
    "${BUILD_DIR}/lib/libutilities.a"
    # modern_gl_utils
    "${MGU_DIR}/build_win/libmodern_gl_utils.a"
    # GLEW (static)
    "C:/msys64/mingw64/lib/libglew32.a"
    # External libraries
    "C:/msys64/mingw64/lib/libassimp.dll.a"
    "C:/msys64/mingw64/lib/libsndfile.a"
    "C:/msys64/mingw64/lib/libsamplerate.a"
    "C:/msys64/mingw64/lib/libfftw3f.a"
    "C:/msys64/mingw64/lib/libOpenCL.dll.a"
    # sndfile dependencies
    "C:/msys64/mingw64/lib/libFLAC.a"
    "C:/msys64/mingw64/lib/libvorbisenc.a"
    "C:/msys64/mingw64/lib/libvorbis.a"
    "C:/msys64/mingw64/lib/libogg.a"
    "C:/msys64/mingw64/lib/libmpg123.dll.a"
    "C:/msys64/mingw64/lib/libmp3lame.a"
    "C:/msys64/mingw64/lib/libopus.a"
    # Windows system libraries required by JUCE
    wininet
    opengl32
    gdi32
    gdiplus
    user32
    kernel32
    shell32
    winmm
    comdlg32
    ole32
    oleaut32
    uuid
    ws2_32
    imm32
    version
    shlwapi
    dwmapi
    setupapi
)

# ── Compiler flags ─────────────────────────────────────────────────────────────
if(MINGW)
    target_compile_options(wayverb PRIVATE
        -Wno-ignored-attributes
        -Wno-attributes
        -Wno-deprecated-declarations
    )
endif()
