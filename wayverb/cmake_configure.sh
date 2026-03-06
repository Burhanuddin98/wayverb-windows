#!/usr/bin/bash
export PATH="/c/msys64/mingw64/bin:/c/msys64/usr/bin:/usr/bin"
export PKG_CONFIG_PATH="/c/msys64/mingw64/lib/pkgconfig"
export TEMP="/c/Users/bsaka/AppData/Local/Temp"
export TMP="/c/Users/bsaka/AppData/Local/Temp"

BUILD_DIR="/c/RoomGUI/wayverb/wayverb-0.0.1/wayverb-0.0.1/build_win"
SRC_DIR="/c/RoomGUI/wayverb/wayverb-0.0.1/wayverb-0.0.1"

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake \
  -G "MinGW Makefiles" \
  -DCMAKE_MAKE_PROGRAM="/c/msys64/mingw64/bin/mingw32-make.exe" \
  -DCMAKE_C_COMPILER="/c/msys64/mingw64/bin/gcc.exe" \
  -DCMAKE_CXX_COMPILER="/c/msys64/mingw64/bin/g++.exe" \
  -DCMAKE_BUILD_TYPE=Release \
  "$SRC_DIR"
