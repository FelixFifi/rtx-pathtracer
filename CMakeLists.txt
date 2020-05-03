cmake_minimum_required(VERSION 3.15)
project(rtx_raytracer)

set(CMAKE_CXX_STANDARD 20)

find_package(Vulkan REQUIRED)
find_package(SDL2 REQUIRED)

include_directories(${Vulkan_INCLUDE_DIR} ${SDL2_INCLUDE_DIRS} "../nvidia_shared_sources")


link_directories(${Vulkan_LIBRARY} ${SDL2_LIBRARIES})
add_executable(rtx_raytracer src/main.cpp src/vec3.h src/ray.h src/camera.h src/hittable.h src/sphere.h src/hittable_list.h src/utility.h src/material.h src/tri.h src/obj_reader.h src/tri_mesh.h src/AABB.h src/model_instance.h src/RayTracingApp.h)
target_link_libraries(rtx_raytracer ${Vulkan_LIBRARY} ${SDL2_LIBRARIES} ${CMAKE_DL_LIBS})