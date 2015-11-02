cmake_minimum_required(VERSION 2.8)
project(Laberinto_final)

find_package(qibuild)

# Create a executable named Laberinto_final
# with the source file: main.cpp
qi_create_bin(Laberinto_final "main.cpp")

# Add a simple test:
enable_testing()
qi_create_test(test_Laberinto_final "test.cpp")

qi_use_lib(Laberinto_final ALCOMMON ALPROXIES)
qi_use_lib(Laberinto_final OPENCV2_CORE OPENCV2_HIGHGUI OPENCV2_IMGPROC)
