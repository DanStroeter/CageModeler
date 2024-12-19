# CageModeler

CageModeler enables the deformation of meshes by cage-based deformation.
This tool has been developed for a recent Eurographics [state of the art report](https://onlinelibrary.wiley.com/doi/full/10.1111/cgf.15060) about cage-based deformation of 3D models.
It unifies many of the most relevant approaches in cage-based deformation and offers a simple GUI for modeling. 
Users can load models and corresponding cages. 
The navigation enables to rotate (`LMB` + `Alt`), to drad view (`LMB` + `Shift`) and to zoom (`RMB`).
For modeling by cage control, the user can select vertices, edges and faces of the cage.
Whenever the user deforms the cage, the model is adjusted to the cage: 

![Cage-based deformatio](https://hessenbox.tu-darmstadt.de/dl/fiDFC1aVrUS22fAuGU48ULsE/Cactus_QGC.gif)

## Getting started

CageModeler utilizes CMake for project file generation, vcpkg for managing external dependencies, and CMake presets 
to streamline configurations across major platforms - Windows, Linux, and macOS. For Linux, ensure the presence of the following packages:
- `zip` (for vcpkg functionality)
- `libomp-dev`
- `libtool`
- `autoconf`
- `gfortran`
- `libxrandr-dev`
- `gtk-3.0`

Set the environment variable `CUDA_HOME` to point to the CUDA installation directory (e.g. `/usr/local/cuda`).
Ensure that `LD_LIBRARY_PATH` contains `$CUDA_HOME/lib64:$CUDA_HOME/extras/CUPTI/lib64` and `PATH` includes `$CUDA_HOME/bin`
for the CMake scripts to locate the CUDA installation.
The build has been tested on Windows 10, Mac OS, and Ubuntu 24.04 LTS.

## Cloning

As the models in this repository are maintained as lfs, ensure you have `git-lfs` installed before cloning.
CageModeler will load models on startup.
Cloning the repository necessitates a recursive fetch of all submodules:

```
git clone https://github.com/DanStroeter/CageModeler.git
cd CageModeler
git submodule update --init --recursive
```

## Available presets

The following presets are defined and can be utilized either on a build server or a local machine corresponding to the operating system:

- `windows-msvc-debug` - <strong>Windows</strong> using <strong>MSVC</strong> in debug mode.
- `windows-msvc-release` - <strong>Windows</strong> using <strong>MSVC</strong> in release mode.
- `unix-clang-debug` - <strong>Linux</strong> using <strong>clang</strong> in debug mode.
- `unix-clang-release` - <strong>Linux</strong> using <strong>clang</strong> in release mode.
- `unix-gcc-debug` - <strong>Linux</strong> using <strong>gcc</strong> in debug mode.
- `unix-gcc-release` - <strong>Linux</strong> using <strong>gcc</strong> in release mode.
- `macos-debug` - <strong>macOS</strong> in debug mode using <strong>clang</strong> which is bundled with Xcode and the command line tools.
- `macos-release` - <strong>macOS</strong> in release mode using <strong>clang</strong> which is bundled with Xcode and the command line tools.

## Generating the project files

Generating project files is straightforward:
```
mkdir build
cd build
cmake .. --preset=<Your Preset Name>
```
