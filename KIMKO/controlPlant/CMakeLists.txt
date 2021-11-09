add_library(plant controlPlant.h controlPlant.cpp )

target_include_directories(pid PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")