cmake_minimum_required(VERSION 3.15)
project(rtx_raytracer)

set(CMAKE_CXX_STANDARD 20)

find_package(Vulkan REQUIRED)
find_package(SDL2 REQUIRED)

include_directories(${Vulkan_INCLUDE_DIR} ${SDL2_INCLUDE_DIRS} "../nvidia_shared_sources" "external")

set(IMGUI_CPP
        external/imgui/imgui.cpp
        external/imgui/imgui_demo.cpp
        external/imgui/imgui_draw.cpp
        external/imgui/imgui_impl_sdl.cpp
        external/imgui/imgui_impl_vulkan.cpp
        external/imgui/imgui_widgets.cpp)

link_directories(${Vulkan_LIBRARY} ${SDL2_LIBRARIES})
add_executable(rtx_raytracer
        src/main.cpp
        src/VulkanOps.cpp
        src/PostProcessing.cpp
        src/VulkanWindow.cpp
        src/VulkanLoader.cpp
        src/RayTracingApp.cpp
        src/Model.cpp
        src/CameraController.cpp
        src/SceneLoader.cpp
        src/WeightedSampler.cpp
        ${IMGUI_CPP})

target_link_libraries(rtx_raytracer ${Vulkan_LIBRARY} ${SDL2_LIBRARIES} ${CMAKE_DL_LIBS})
