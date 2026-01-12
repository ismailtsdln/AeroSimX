# AeroSimX C++ Tests CMake Configuration

# Find GoogleTest
find_package(GTest REQUIRED)

# Integration tests
add_executable(test_integration test_integration.cpp)
target_link_libraries(test_integration 
    PRIVATE 
        aerosimx_core
        GTest::gtest_main
)
gtest_discover_tests(test_integration)

# Performance benchmarks
add_executable(benchmark_physics benchmark_physics.cpp)
target_link_libraries(benchmark_physics 
    PRIVATE 
        aerosimx_core
)

# Individual unit tests (stubs)
# add_executable(test_physics test_physics.cpp)
# target_link_libraries(test_physics PRIVATE aerosimx_core GTest::gtest_main)
# gtest_discover_tests(test_physics)
