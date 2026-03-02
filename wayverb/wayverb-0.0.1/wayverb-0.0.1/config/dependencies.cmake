# Windows / MinGW64 build: use pacman-installed system packages
# instead of ExternalProject_Add + autotools.
# All deps are under C:/msys64/mingw64/

find_package(Git REQUIRED)
find_package(PkgConfig REQUIRED)

set(MINGW64_PREFIX "C:/msys64/mingw64")
set(DEPENDENCY_INSTALL_PREFIX "${MINGW64_PREFIX}")
list(APPEND CMAKE_PREFIX_PATH "${MINGW64_PREFIX}")

# glm (header-only) ############################################################

find_package(glm CONFIG REQUIRED)
add_custom_target(glm_external)   # dummy – used only for add_dependencies ordering

# assimp #######################################################################

find_package(assimp CONFIG REQUIRED)
# Create a plain target named "assimp" that the sub-CMakeLists expect
add_library(assimp INTERFACE)
target_link_libraries(assimp INTERFACE assimp::assimp)

# zlibstatic: the pacman assimp is a shared lib and handles zlib internally
add_library(zlibstatic INTERFACE)

# fftwf (single-precision) #####################################################

pkg_check_modules(FFTWF REQUIRED IMPORTED_TARGET fftw3f)
add_library(fftwf INTERFACE)
target_link_libraries(fftwf INTERFACE PkgConfig::FFTWF)

# fftw (double-precision) – kept as a dummy; itpp is replaced by our shim ######

add_library(fftw INTERFACE)

# sndfile ######################################################################

find_package(SndFile CONFIG REQUIRED)
add_library(sndfile INTERFACE)
target_link_libraries(sndfile INTERFACE SndFile::sndfile)

# samplerate ###################################################################

find_package(SampleRate CONFIG REQUIRED)
add_library(samplerate INTERFACE)
target_link_libraries(samplerate INTERFACE SampleRate::samplerate)

# gtest ########################################################################

find_package(GTest CONFIG REQUIRED)
add_library(gtest INTERFACE)
target_link_libraries(gtest INTERFACE GTest::gtest GTest::gtest_main)

# cereal (header-only) #########################################################

find_package(cereal CONFIG REQUIRED)
add_custom_target(cereal_external)

# ITPP replacement #############################################################
# wayverb uses only itpp::yulewalk from itpp/signal/filter_design.h.
# A self-contained replacement lives in src/waveguide/itpp_compat/.
# No linking required.
set(ITPP_LIBRARIES "")
add_custom_target(itpp_external)

# OpenCL #######################################################################

add_custom_target(opencl_cpp_external)   # dummy – headers already installed
find_package(OpenCL REQUIRED)

################################################################################
# APP STUFF (JUCE GUI) #########################################################
################################################################################

# modern_gl_utils – built separately; stub target for now so the library
# CMakeLists (which add_dependencies on it) don't fail.
add_custom_target(modern_gl_utils_external)

if(EXISTS "${MINGW64_PREFIX}/lib/libmodern_gl_utils.a")
    add_library(modern_gl_utils STATIC IMPORTED)
    set_property(TARGET modern_gl_utils PROPERTY
        IMPORTED_LOCATION "${MINGW64_PREFIX}/lib/libmodern_gl_utils.a")
else()
    add_library(modern_gl_utils INTERFACE)
    message(STATUS "modern_gl_utils not found – GUI will need it later")
endif()
