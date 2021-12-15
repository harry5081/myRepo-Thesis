cmake_minimum_required(VERSION 2.6)
project(kimko)

add_executable(kimko main.cpp run.cpp)



find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
target_link_libraries(kimko ${PYTHON_LIBRARIES})

find_package(pybind11 CONFIG)
include_directories(${pybind11_INCLUDE_DIRS})
target_link_libraries(kimko PRIVATE pybind11::embed)


add_subdirectory(plot)
target_link_libraries(kimko PRIVATE plot)

add_subdirectory(pid)
target_link_libraries(kimko PRIVATE pid)

add_subdirectory(controlPlant)
target_link_libraries(kimko PRIVATE plant)

add_subdirectory(mpc)
target_link_libraries(kimko PRIVATE mpc)






target_link_libraries(kimko /usr/lib/libpcanbasic.so)
target_link_libraries(kimko /lib/x86_64-linux-gnu/libpthread.so.0)

install(TARGETS kimko RUNTIME DESTINATION bin)
