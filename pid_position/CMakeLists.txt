cmake_minimum_required(VERSION 3.10)

# 定义项目名称
project(pid_incremental)

# 包含源文件
file(GLOB_RECURSE SOURCES "${PROJECT_SOURCE_DIR}/src/**/*.cpp"
    "${PROJECT_SOURCE_DIR}/src/**/*.hpp"
    "${PROJECT_SOURCE_DIR}/src/**/*.h"
    "${PROJECT_SOURCE_DIR}/src/**/*.c")

if(WIN32)
    add_definitions(-D_WINDOWS)
    if(MSVC)
        add_definitions(/DWIN32_LEAN_AND_MEAN)  # 在这里统一定义
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /W4 /WX")
    endif()
endif()
# 启用 C++11
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# 动态包含 src 目录下的所有子目录
file(GLOB_RECURSE SUBDIRS ${PROJECT_SOURCE_DIR}/src/*)

foreach(DIR IN LISTS SUBDIRS)
    get_filename_component(PATH ${DIR} DIRECTORY)
    include_directories(${PATH})
endforeach()

# 声明可执行目标
add_executable(${CMAKE_PROJECT_NAME} src/main.c src/ctrl_pid_position.c ${SOURCES})