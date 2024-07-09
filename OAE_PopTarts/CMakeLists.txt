cmake_minimum_required(VERSION 3.20)

project("OAE_PopTarts" C CXX ASM)


# Add CMSIS-DSP source files
file(GLOB CMSIS_DSP_SRC "../common/CMSIS-DSP/Source/*.c")

include(cmake/st-project.cmake)
include_directories(
    "../common/CMSIS-DSP/Include"
    "app"
    )



set(SOURCE_FILES
app/oae_algorithm.c)

add_executable(
${PROJECT_NAME} 
app/oae_algorithm.c
app/app_main.c
app/app_lib.c
app/button.c
app/dual_dma.c
app/TLV320ADC3120.c
Core/Src/main.c
Core/Src/stm32l4xx_hal_msp.c
Core/Src/stm32l4xx_it.c
Core/Src/syscalls.c
Core/Src/sysmem.c
Core/Src/system_stm32l4xx.c
Core/Startup/startup_stm32l475rctx.s

${SOURCE_FILES} 
${CMSIS_DSP_SRC}
)

add_st_target_properties(${PROJECT_NAME})
