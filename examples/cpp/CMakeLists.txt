# AeroSimX C++ Examples CMake Configuration

# Basic simulation example
add_executable(basic_sim basic_sim.cpp)
target_link_libraries(basic_sim PRIVATE aerosimx_core)

# Vehicle control example
add_executable(vehicle_control vehicle_control.cpp)
target_link_libraries(vehicle_control PRIVATE aerosimx_core)
