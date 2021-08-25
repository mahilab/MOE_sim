cmake_minimum_required(VERSION 3.0.0)
project(MeiiSim VERSION 0.1.0)

include(FetchContent) 
FetchContent_Declare(mahi-com GIT_REPOSITORY https://github.com/mahilab/mahi-com.git) 
FetchContent_MakeAvailable(mahi-com)
FetchContent_Declare(mahi-robo GIT_REPOSITORY https://github.com/mahilab/mahi-robo.git) 
FetchContent_MakeAvailable(mahi-robo)
FetchContent_Declare(mahi-gui GIT_REPOSITORY https://github.com/mahilab/mahi-gui.git) 
FetchContent_MakeAvailable(mahi-gui)

include_directories(include)

# add_executable(test src/test.cpp)
# target_link_libraries(test mahi::util mahi::robo)

add_library(meii_model SHARED include/MeiiModel.hpp src/MeiiModel.cpp src/dll.cpp include/dll.hpp)
target_link_libraries(meii_model mahi::robo mahi::com)

add_library(virtual_meii SHARED include/MeiiModel.hpp src/MeiiModel.cpp src/dll_virtual.cpp)
target_link_libraries(virtual_meii mahi::robo mahi::com)

add_executable(tuner src/tuner.cpp)
target_link_libraries(tuner mahi::gui mahi::com)
