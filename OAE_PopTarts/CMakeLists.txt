cmake_minimum_required(VERSION 3.20)

project("OAE_PopTarts" C CXX ASM)

include(cmake/st-project.cmake)
include_directories("../common/CMSIS-DSP/Include")

add_executable(${PROJECT_NAME})
add_st_target_properties(${PROJECT_NAME})
