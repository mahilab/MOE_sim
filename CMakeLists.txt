cmake_minimum_required(VERSION 3.0.0)
project(MOESim VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent) 
FetchContent_Declare(mahi-com GIT_REPOSITORY https://github.com/mahilab/mahi-com.git) 
FetchContent_MakeAvailable(mahi-com)
FetchContent_Declare(mahi-robo GIT_REPOSITORY https://github.com/mahilab/mahi-robo.git) 
FetchContent_MakeAvailable(mahi-robo)
FetchContent_Declare(mahi-gui GIT_REPOSITORY https://github.com/mahilab/mahi-gui.git) 
FetchContent_MakeAvailable(mahi-gui)

include_directories(include)

add_executable(test src/test.cpp include/MoeModel.hpp src/MoeModel.cpp)
target_link_libraries(test mahi::util mahi::robo)

add_executable(tuner_gui src/tuner.cpp)
target_link_libraries(tuner_gui mahi::com mahi::gui)

# tuner - working with PD gains and such
add_library(tuner_moe SHARED include/MoeModel.hpp src/MoeModel.cpp src/dll.cpp include/dll.hpp)
target_link_libraries(tuner_moe mahi::robo mahi::com)

add_custom_command(TARGET tuner_moe POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different
                   "${PROJECT_SOURCE_DIR}/build/Release/tuner_moe.dll"
                   "${PROJECT_SOURCE_DIR}/unity/MOE_sim")

# virtual - working with the MOE c++ library
add_library(virtual_moe SHARED include/MoeModel.hpp src/MoeModel.cpp src/dll_virtual.cpp)
target_link_libraries(virtual_moe mahi::robo mahi::com)

add_custom_command(TARGET virtual_moe POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different
                   "${PROJECT_SOURCE_DIR}/build/Release/virtual_moe.dll"
                   "${PROJECT_SOURCE_DIR}/unity/MOE_sim")