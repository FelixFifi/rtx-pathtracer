#!/bin/bash
rm *.spv
glslc --target-env=vulkan1.2 post.vert -o post.vert.spv
glslc --target-env=vulkan1.2 post.frag -o post.frag.spv
glslc --target-env=vulkan1.2 raytrace.rchit -O -o raytrace.rchit.spv
glslc --target-env=vulkan1.2 raytrace.rahit -O -o raytrace.rahit.spv
glslc --target-env=vulkan1.2 raytrace.rgen -O -o raytrace.rgen.spv
glslc --target-env=vulkan1.2 raytrace.rmiss -O -o raytrace.rmiss.spv
glslc --target-env=vulkan1.2 raytrace.shadow.rmiss -O -o raytrace.shadow.rmiss.spv
glslc --target-env=vulkan1.2 raytrace.irradiance.rint -O -o raytrace.irradiance.rint.spv
glslc --target-env=vulkan1.2 raytrace.irradiance.rahit -O -o raytrace.irradiance.rahit.spv
glslc --target-env=vulkan1.2 raytrace.irradiance.visualize.rahit -O -o raytrace.irradiance.visualize.rahit.spv
glslc --target-env=vulkan1.2 raytrace.sphere.rint -O -o raytrace.sphere.rint.spv
glslc --target-env=vulkan1.2 raytrace.sphere.rchit -O -o raytrace.sphere.rchit.spv