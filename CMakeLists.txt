cmake_minimum_required(VERSION 3.15)
project(rtx_raytracer)

set(CMAKE_CXX_STANDARD 20)

find_package(Vulkan REQUIRED)
find_package(SDL2 REQUIRED)

include_directories(${Vulkan_INCLUDE_DIR} ${SDL2_INCLUDE_DIRS} "../nvidia_shared_sources" "vendor")

set(IMGUI_CPP
        vendor/imgui/imgui.cpp
        vendor/imgui/imgui_demo.cpp
        vendor/imgui/imgui_draw.cpp
        vendor/imgui/imgui_impl_sdl.cpp
        vendor/imgui/imgui_impl_vulkan.cpp
        vendor/imgui/imgui_widgets.cpp
        )

link_directories(${Vulkan_LIBRARY} ${SDL2_LIBRARIES})
add_executable(rtx_raytracer
        src/main.cpp
        src/VulkanOps.cpp
        src/PostProcessing.cpp
        src/VulkanWindow.cpp
        src/VulkanLoader.cpp
        src/RayTracingApp.cpp
        src/Model.cpp
        ${IMGUI_CPP}
        src/CameraController.cpp
        src/ModelLoader.cpp)

target_link_libraries(rtx_raytracer ${Vulkan_LIBRARY} ${SDL2_LIBRARIES} ${CMAKE_DL_LIBS})