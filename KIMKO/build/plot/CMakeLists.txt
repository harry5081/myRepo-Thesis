add_library(plot readWritePlot.cpp readWritePlot.h)

target_include_directories(plot PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")