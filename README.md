# CageModeler

## Getting started

CageModeler utilizes CMake for project file generation, vcpkg for managing external dependencies, and CMake presets 
to streamline configurations across major platforms - Windows, Linux, and macOS. For Linux, ensure the presence of
`zip` for vcpkg functionality and `libomp-dev` as an external dependency. On macOS, `libomp` can be installed via 
Homebrew. 

Set the environment variable `CUDA_HOME` to point to the CUDA installation directory (e.g. `/usr/local/cuda`).
Ensure that `LD_LIBRARY_PATH` contains `$CUDA_HOME/lib64:$CUDA_HOME/extras/CUPTI/lib64` and `PATH` includes `$CUDA_HOME/bin`
for the CMake scripts to locate the CUDA installation.

## Cloning

Cloning the repository necessitates a recursive fetch of all submodules:

```
git clone git@github.com:DanStroeter/CageModeler.git
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
