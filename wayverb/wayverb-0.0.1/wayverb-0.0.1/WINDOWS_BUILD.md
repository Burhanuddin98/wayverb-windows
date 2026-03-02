# Building Wayverb on Windows 11

This guide documents how to build and run Wayverb on Windows 11 using MSYS2 / MinGW-w64.
The original project was macOS-only; the patches in this repository make it work on Windows
with an NVIDIA GPU.

---

## Requirements

- Windows 10 or 11 (64-bit)
- An NVIDIA GPU (NVIDIA's OpenCL driver is required; AMD should also work but is untested)
- [MSYS2](https://www.msys2.org/) — provides the MinGW-w64 toolchain and all dependencies
- ~4 GB of free disk space for packages and build output

---

## Step 1 — Install MSYS2

1. Download the MSYS2 installer from https://www.msys2.org/ and run it.
2. Accept the default install path `C:\msys64`.
3. After installation, open the **MSYS2 MinGW64** shell (not the MSYS shell).

---

## Step 2 — Install the toolchain and dependencies

Open the **MSYS2 MinGW64** shell and run:

```bash
pacman -Syu
```

Close and reopen the shell, then install everything in one go:

```bash
pacman -S --needed \
  mingw-w64-x86_64-gcc \
  mingw-w64-x86_64-cmake \
  mingw-w64-x86_64-ninja \
  mingw-w64-x86_64-pkg-config \
  mingw-w64-x86_64-glm \
  mingw-w64-x86_64-glew \
  mingw-w64-x86_64-assimp \
  mingw-w64-x86_64-fftw \
  mingw-w64-x86_64-libsndfile \
  mingw-w64-x86_64-libsamplerate \
  mingw-w64-x86_64-gtest \
  mingw-w64-x86_64-cereal \
  mingw-w64-x86_64-opencl-icd \
  mingw-w64-x86_64-opencl-headers \
  git
```

OpenCL will use your GPU's driver (NVIDIA or AMD) at runtime via the ICD loader — no
separate driver install is needed beyond your normal GPU driver.

---

## Step 3 — Build modern_gl_utils

Wayverb's GUI depends on a small OpenGL utility library that must be compiled separately.

```bash
cd /c/RoomGUI/wayverb/wayverb-0.0.1/wayverb-0.0.1/modern_gl_utils
mkdir -p build_win && cd build_win
cmake .. -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=C:/msys64/mingw64
ninja
```

This produces `modern_gl_utils/build_win/libmodern_gl_utils.a`.

---

## Step 4 — Build the Wayverb libraries

```bash
cd /c/RoomGUI/wayverb/wayverb-0.0.1/wayverb-0.0.1
mkdir -p build_win && cd build_win
cmake .. -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=C:/msys64/mingw64
ninja
```

This builds all static libraries (`libcombined.a`, `libraytracer.a`, `libwaveguide.a`,
`libcore.a`, etc.) into `build_win/lib/`.

---

## Step 5 — Build the GUI application

```bash
cd /c/RoomGUI/wayverb/wayverb-0.0.1/wayverb-0.0.1/wayverb
mkdir -p build_mgu && cd build_mgu
cmake .. -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=C:/msys64/mingw64
ninja
```

The executable is written to:

```
wayverb-0.0.1/bin/wayverb.exe
```

---

## Step 6 — Copy runtime DLLs

The exe needs several MinGW DLLs alongside it. Copy them from MSYS2:

```bash
cd /c/RoomGUI/wayverb/wayverb-0.0.1/wayverb-0.0.1/bin

cp /c/msys64/mingw64/bin/libgcc_s_seh-1.dll .
cp /c/msys64/mingw64/bin/libstdc++-6.dll .
cp /c/msys64/mingw64/bin/libwinpthread-1.dll .
cp /c/msys64/mingw64/bin/libassimp*.dll .
cp /c/msys64/mingw64/bin/libsndfile*.dll .
cp /c/msys64/mingw64/bin/libsamplerate*.dll .
cp /c/msys64/mingw64/bin/libfftw3f*.dll .
cp /c/msys64/mingw64/bin/libOpenCL.dll .
cp /c/msys64/mingw64/bin/libmpg123*.dll .
```

---

## Step 7 — Run Wayverb

Double-click `bin/wayverb.exe`, or run it from an MSYS2 shell:

```bash
/c/RoomGUI/wayverb/wayverb-0.0.1/wayverb-0.0.1/bin/wayverb.exe
```

---

## Loading a scene

1. Go to **File → Open** and load a `.obj` model file.
   - The bundled example scenes are in `demo/assets/test_models/`.
   - Start with `bedroom.obj` — it is small (88 triangles) and renders in a few minutes.
2. Place the source and receiver inside the room using the 3D viewport.
3. Click **Render** and wait. Progress is shown in the status bar.
4. The output `.wav` file is written to the folder you selected.

> **Scene size warning:** Wayverb was designed for small architectural rooms
> (roughly under 500 m³ and under a few thousand triangles). Very large or
> geometrically complex meshes will cause slow voxelisation and potential GPU
> out-of-memory errors. The patches in this repo automatically cap the ray count
> to 50 000 and the image-source order to 1 to prevent crashes on larger scenes,
> but small scenes will always give the best results.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Black window on launch | GLM not initialising matrices to identity | Already patched via `GLM_FORCE_CTOR_INIT` |
| `clBuildProgram` error in render output | NVIDIA driver rejected kernel | Already patched — check your GPU driver is up to date |
| Render crashes immediately | Scene too large | Use a smaller `.obj` file |
| Missing DLL on launch | Runtime DLLs not copied | Repeat Step 6 |
| `Source is outside mesh` error | Source/receiver placed outside the model | Move them inside the room in the viewport |

---

## What was patched vs. the original source

The original Wayverb 0.0.1 was macOS-only. The following changes were made to run on Windows:

1. **`src/core/src/program_wrapper.cpp`** — Removed `-Werror` from OpenCL builds; added build log output on failure.
2. **`src/waveguide/src/boundary_coefficient_program.cpp`** — Fixed implicit `int3 × float3` multiplication rejected by NVIDIA's OpenCL compiler; replaced with `convert_float3()`.
3. **`src/combined/src/engine.cpp`** — Capped ray count to 50 000 and image-source order to 1 to prevent GPU out-of-memory on large scenes; added diagnostic logging.
4. **`src/combined/src/threaded_engine.cpp`** — Added `cl::Error` catch with error code reporting; added stage-by-stage logging throughout the engine.
5. **`src/raytracer/include/raytracer/raytracer.h`** — Added diagnostic logging in the ray-tracing loop.
6. **`src/raytracer/include/raytracer/stochastic/postprocessing.h`** — Fixed `std::uniform_real_distribution{1.0, 0.0}` (invalid `a > b`, asserted by GCC 15); replaced with the correct exponential inverse-CDF formula.
7. **`wayverb/Source/3d_objects/reflections_object.cpp`** — Fixed an off-by-one bug in the reflection index loop that caused an out-of-bounds vector access.
8. **`wayverb/Source/scene/` and `UtilityComponents/generic_renderer.h`** — Added `GLM_FORCE_CTOR_INIT` and fixed GLSL attribute/uniform name mismatches that caused a black viewport on MSYS2.
