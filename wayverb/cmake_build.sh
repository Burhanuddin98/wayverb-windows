#!/usr/bin/bash
export PATH="/c/msys64/mingw64/bin:/c/msys64/usr/bin:/usr/bin"
export PKG_CONFIG_PATH="/c/msys64/mingw64/lib/pkgconfig"
export TEMP="/c/Users/bsaka/AppData/Local/Temp"
export TMP="/c/Users/bsaka/AppData/Local/Temp"

cd "/c/RoomGUI/wayverb/wayverb-0.0.1/wayverb-0.0.1/build_win"
mingw32-make -j4 2>&1
