# Advance Path Tracing with NVIDIA RTX
## Abstract
Hardware accelerated ray tracing is now possible with the NVIDIA RTX and the
newly released AMD RDNA 2 GPUs.
This master thesis implements common and advanced path tracing techniques
while utilizing the new ray tracing hardware acceleration, and evaluating their
performance and quality. The evaluated techniques are Next Event Estimation,
Multiple Importance Sampling, Irradiance Caching, Adjoint-drive Russian Roulette
and Splitting, and Parallax-aware Path Guiding.
Also included is a wide range of visualization modes that can be enabled interactively
to better understand the inner workings of the implemented techniques. This, in
combination with the way higher performance than CPU based path tracer and
resulting fast convergence rate, allows for interactive comparisons between the
different methods.

## Images
### Path tracing of a dielectric element in a Cornell box
![DielectricPathTracing](https://github.com/FelixFifi/rtx-pathtracer/blob/master/resources/dielectric_PT.png?raw=true)
### Irradiance Cache enabled
![IC](https://github.com/FelixFifi/rtx-pathtracer/blob/master/resources/dielectric_IC.png?raw=true)
### Visualization of Irradiance Cache values
![IC_visu](https://github.com/FelixFifi/rtx-pathtracer/blob/master/resources/IC_visu.png?raw=true)
### UI of the path tracer, where most settings can be changed at runtime to switch between different methods
![CompleteUI](https://github.com/FelixFifi/rtx-pathtracer/blob/master/resources/Ui_Complete.png?raw=true)
### Visualization of the generated guiding information as spherical distributions in the scene
![GuidingVisualization](https://github.com/FelixFifi/rtx-pathtracer/blob/master/resources/Visu_guiding_all.png?raw=true)
![GuidingVisualizationSingle](https://github.com/FelixFifi/rtx-pathtracer/blob/master/resources/Visu_veach_guidingMove2.png?raw=true)



## Requirements
* A graphics card with ray tracing support
* Vulkan SDK 1.2.154 https://vulkan.lunarg.com/sdk/home as well as compatible drivers
* SDL2 `sudo apt-get install libsdl2-dev`
* OpenEXR (recommended to install from source, so that CMake can easily find OpenEXR) 
* CMake `sudo apt-get install cmake`
* GLM

`sudo apt-get install libsdl2-dev libopenexr-dev cmake build-essential libglm-dev`

## Build steps

`cmake -DCMAKE_BUILD_TYPE=Debug -S . -B cmake-build-debug`

`cmake -DCMAKE_BUILD_TYPE=Release -S . -B cmake-build-release`

`cd cmake-build-release`

`make -j8`

Compile the shaders with
`./shaders/compile.sh`

## Running the application
`./rtx_raytracer WIDTH HEIGHT IC_SIZE GUIDING_SPLITS Scenes ...`
IC_SIZE is the maximum number of Irradiance Cache entries.
GUIDING_SPLITS is the initial number of bounding box splits for the path-guiding regions
Scenes are paths to scene definitions, relative to the scenes folder

For example:
`./rtx_raytracer 1280 720 10000 8 cornell-dielectric/cornell-dielectric.xml veachMIS/veachMIS.xml`

## Vulkan Version
This project was created prior to the finalization of the Vulkan Ray Tracing extension and therefore depends on the beta version of the extension, i.e. Vulkan SDK 1.2.154 or earlier.
