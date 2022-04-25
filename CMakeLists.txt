cmake_minimum_required(VERSION 3.0.0)
project(MOESim VERSION 0.1.0)

include(FetchContent) 
FetchContent_Declare(moe 
                     GIT_REPOSITORY https://github.com/mahilab/moe.git
                     GIT_TAG origin/dynamics-separation) 
FetchContent_MakeAvailable(moe)

FetchContent_Declare(mahi-gui GIT_REPOSITORY https://github.com/mahilab/mahi-gui.git) 
FetchContent_MakeAvailable(mahi-gui)

add_executable(test src/test.cpp)
target_link_libraries(test moe)

# virtual - working with the MOE c++ library
add_library(virtual_moe SHARED src/dll_virtual.cpp)
target_link_libraries(virtual_moe moe)

add_custom_command(TARGET virtual_moe POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different
                   "${PROJECT_SOURCE_DIR}/build/Release/virtual_moe.dll"
                   "${PROJECT_SOURCE_DIR}/unity/MOE_sim")