#!/bin/bash
rm *.spv
glslc post.vert -o post.vert.spv
glslc post.frag -o post.frag.spv
glslc raytrace.rchit -O -o raytrace.rchit.spv
glslc raytrace.rahit -O -o raytrace.rahit.spv
glslc raytrace.rgen -O -o raytrace.rgen.spv
glslc raytrace.rmiss -O -o raytrace.rmiss.spv
glslc raytrace.shadow.rmiss -O -o raytrace.shadow.rmiss.spv
glslc raytrace.sphere.rint -O -o raytrace.sphere.rint.spv
glslc raytrace.sphere.rchit -O -o raytrace.sphere.rchit.spv