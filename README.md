# CageModeler

## Getting started

The project uses `CMake` to generate the project files, `vcpkg` to manage external dependencies and CMake presets to supply pre-made configurations for the 3 major platforms - <strong>Windows</strong>, <strong>Linux</strong> and <strong>macOS</strong>.
On <strong>Linux</strong> you also need to install `zip` if you don't have it already for `vcpkg` to work properly and `libomp-dev` as an external dependency. On <strong>macOS</strong> you need to install `libomp` using `Homebrew`.
Additionally if you are compiling on <strong>Linux</strong> and you have CUDA installed, you will require the following libraries to compile Somigliana:

- `gfortran` (Fortran compiler)
- `libcgal-dev` (CGAL)
- `libblas3-dev` (BLAS)
- `liblapack-dev` (LAPACK)

It's important that you have the environment variable `CUDA_HOME` pointing to the CUDA installation directory (e.g. `/usr/local/cuda`). The
environment variable `LD_LIBRARY_PATH` contain `$CUDA_HOME/lib64:$CUDA_HOME/extras/CUPTI/lib64` and `PATH` should have `$CUDA_HOME/bin`. That way the CMake scripts
will be able to find the CUDA installation.

## Cloning

Cloning the repository requires recursive fetch of all submodules as well:

```
git clone git@git.rwth-aachen.de:daniel_jan.stroeter/cagemodeler.git
git submodule update --init --recursive
```

## Available presets

The following presets are defined and can be used either on a build server or on a local machine running the given operating system:

- `windows-msvc-debug` - <strong>Windows</strong> using <strong>MSVC</strong> in debug mode.
- `windows-msvc-release` - <strong>Windows</strong> using <strong>MSVC</strong> in release mode.
- `unix-clang-debug` - <strong>Linux</strong> using <strong>clang</strong> in debug mode.
- `unix-clang-release` - <strong>Linux</strong> using <strong>clang</strong> in release mode.
- `unix-gcc-debug` - <strong>Linux</strong> using <strong>gcc</strong> in debug mode.
- `unix-gcc-release` - <strong>Linux</strong> using <strong>gcc</strong> in release mode.
- `macos-debug` - <strong>macOS</strong> in debug mode using <strong>clang</strong> which is bundled with Xcode and the command line tools.
- `macos-release` - <strong>macOS</strong> in release mode using <strong>clang</strong> which is bundled with Xcode and the command line tools.

## Generating the project files

Generating the project files is as simple as:
```
mkdir build
cd build
cmake .. --preset=<Your Preset Name>
```
