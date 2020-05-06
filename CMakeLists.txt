cmake_minimum_required(VERSION 3.15)
project(rtx_raytracer)

set(CMAKE_CXX_STANDARD 20)

find_package(Vulkan REQUIRED)
find_package(SDL2 REQUIRED)

include_directories(${Vulkan_INCLUDE_DIR} ${SDL2_INCLUDE_DIRS} "../nvidia_shared_sources")


link_directories(${Vulkan_LIBRARY} ${SDL2_LIBRARIES})
add_executable(rtx_raytracer src/main.cpp src/RayTracingApp.h src/PostProcessing.h src/VulkanOps.h src/VulkanOps.cpp src/PostProcessing.cpp src/CommonOps.h src/VulkanWindow.cpp src/VulkanWindow.h src/VulkanLoader.h src/VulkanLoader.cpp src/RayTracingApp.cpp)
target_link_libraries(rtx_raytracer ${Vulkan_LIBRARY} ${SDL2_LIBRARIES} ${CMAKE_DL_LIBS})