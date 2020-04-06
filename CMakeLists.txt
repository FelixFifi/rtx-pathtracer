cmake_minimum_required(VERSION 3.15)
project(rtx_raytracer)

set(CMAKE_CXX_STANDARD 20)

add_executable(rtx_raytracer src/main.cpp src/vec3.h src/ray.h src/camera.h src/hittable.h src/sphere.h src/hittable_list.h src/utility.h src/material.h src/tri.h src/obj_reader.h src/tri_mesh.h)