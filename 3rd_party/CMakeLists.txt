# box2d
option(BOX2D_BUILD_UNIT_TESTS OFF)
option(BOX2D_BUILD_TESTBED OFF)
option(BOX2D_BUILD_DOCS OFF)
option(BOX2D_USER_SETTINGS OFF)
add_subdirectory(box2d)

# sokol
add_library(sokol INTERFACE)
target_include_directories(sokol INTERFACE sokol)
