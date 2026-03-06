#!/bin/bash
export PATH="/c/msys64/mingw64/bin:/c/msys64/usr/bin:/usr/bin"
export PKG_CONFIG_PATH="/c/msys64/mingw64/lib/pkgconfig"
export TEMP="C:/Users/bsaka/AppData/Local/Temp"
export TMP="C:/Users/bsaka/AppData/Local/Temp"

ROOT="/c/RoomGUI/wayverb/wayverb-0.0.1/wayverb-0.0.1"
MGU_BUILD="$ROOT/build_mgu"
mkdir -p "$MGU_BUILD"

cmake -G "MinGW Makefiles" \
    -DCMAKE_MAKE_PROGRAM="/c/msys64/mingw64/bin/mingw32-make.exe" \
    -DCMAKE_C_COMPILER="/c/msys64/mingw64/bin/gcc.exe" \
    -DCMAKE_CXX_COMPILER="/c/msys64/mingw64/bin/g++.exe" \
    -DCMAKE_CXX_STANDARD=17 \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="/c/msys64/mingw64" \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -S "$ROOT/modern_gl_utils" \
    -B "$MGU_BUILD" 2>&1

mingw32-make -C "$MGU_BUILD" -j4 2>&1
