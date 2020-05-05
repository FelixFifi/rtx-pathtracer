#!/bin/bash
glslc shader.vert -o vert.spv
glslc shader.frag -o frag.spv
glslc post.vert -o post.vert.spv
glslc post.frag -o post.frag.spv
glslc raytrace.rchit -o raytrace.rchit.spv
glslc raytrace.rgen -o raytrace.rgen.spv
glslc raytrace.rmiss -o raytrace.rmiss.spv
