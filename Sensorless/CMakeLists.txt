# Sensorless library
# 无传感器控制核心算法库，只包含算法实现和私有函数

# 添加源文件
set(Sensorless_SOURCES
    # ${CMAKE_CURRENT_SOURCE_DIR}/Src/flux_observer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/hf_injection.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/flying.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/leso.c
)

if(NUMERIC_BACKEND STREQUAL "IQMATH")
    add_library(Sensorless STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/leso_iq.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/hf_injection_iq.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/flying_iq.c
    )
else()
    add_library(Sensorless STATIC
        ${Sensorless_SOURCES}
    )
endif()

# 添加头文件目录  
if(NUMERIC_BACKEND STREQUAL "IQMATH")
    target_include_directories(Sensorless
        PUBLIC Inc
        PRIVATE ${CMAKE_SOURCE_DIR}/Application/Inc
    )
else()
    target_include_directories(Sensorless PUBLIC Inc)
endif()

# 链接依赖库
if(NUMERIC_BACKEND STREQUAL "IQMATH")
    target_link_libraries(Sensorless PUBLIC MCU Utils)
else()
    target_link_libraries(Sensorless PUBLIC MCU Utils)
endif()

# 编译选项
target_compile_options(Sensorless PRIVATE -Wall -Wextra -O2)
if(NUMERIC_BACKEND STREQUAL "IQMATH")
    target_compile_options(Sensorless PRIVATE -fstack-usage)
endif()
