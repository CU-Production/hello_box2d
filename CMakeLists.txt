cmake_minimum_required(VERSION 3.26)
project(hello_box2d)

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(3rd_party)

add_subdirectory(src/hellobox2d)
add_subdirectory(src/hellobox2d_draw)
add_subdirectory(src/hellobox2d_draw_mt)
add_subdirectory(src/circles)
add_subdirectory(src/car)
add_subdirectory(src/ragdoll)
add_subdirectory(src/cloth)
