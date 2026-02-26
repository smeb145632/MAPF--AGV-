cmake_minimum_required(VERSION 3.16)
project(MAPF_AGV VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置编码为UTF-8和兼容性选项
if(MSVC)
    add_compile_options(/utf-8)
    add_compile_definitions(_UNICODE UNICODE _USE_MATH_DEFINES)
    # Qt 5.9.0 兼容性：允许某些 constexpr 函数在运行时计算
    add_compile_options(/Zc:__cplusplus)
    # 如果仍有 constexpr 问题，可以尝试：
    # add_compile_options(/permissive-)
endif()

# Windows编译器检测和提示
if(WIN32)
    if(NOT CMAKE_CXX_COMPILER)
        message(FATAL_ERROR 
            "\n"
            "========================================\n"
            "ERROR: C++ Compiler not found!\n"
            "========================================\n"
            "\n"
            "You need to install a C++ compiler:\n"
            "\n"
            "Option 1: Install Visual Studio (Recommended)\n"
            "  - Download Visual Studio Community (free):\n"
            "    https://visualstudio.microsoft.com/downloads/\n"
            "  - During installation, select:\n"
            "    * Desktop development with C++\n"
            "    * MSVC v143 or v142 compiler toolset\n"
            "\n"
            "Option 2: Install Visual C++ Build Tools\n"
            "  - Download Build Tools for Visual Studio:\n"
            "    https://visualstudio.microsoft.com/downloads/\n"
            "  - Select 'C++ build tools' workload\n"
            "\n"
            "Option 3: Use MinGW (if you have Qt MinGW version)\n"
            "  - Install MinGW-w64\n"
            "  - Set CMAKE_C_COMPILER and CMAKE_CXX_COMPILER\n"
            "\n"
            "After installation, restart your terminal and try again.\n"
            "========================================\n"
        )
    endif()
endif()

# 自动查找Qt路径（Windows常见路径）
if(WIN32 AND NOT CMAKE_PREFIX_PATH)
    # 尝试常见的Qt安装路径
    set(QT_POSSIBLE_PATHS
        "C:/Qt/6.7.0/msvc2019_64"
        "C:/Qt/6.6.0/msvc2019_64"
        "C:/Qt/6.5.0/msvc2019_64"
        "C:/Qt/6.4.0/msvc2019_64"
        "C:/Qt/5.15.2/msvc2019_64"
        "C:/Qt/5.15.0/msvc2019_64"
        "C:/Qt/5.14.2/msvc2019_64"
        "C:/Qt/5.9.0/msvc2019_64"
        "C:/Qt/5.9.0/msvc2017_64"
        "C:/Qt/5.9.0/msvc2015_64"
        "C:/Qt/Qt5.9.0/5.9/msvc2017_64"
        "C:/Qt/Qt5.9.0/5.9/msvc2015_64"
        "C:/Qt/Qt5.9.0/5.9/msvc2013_64"
	"E:/qt-env/5.9/msvc2017_64"
    )
    
    foreach(QT_PATH ${QT_POSSIBLE_PATHS})
        if(EXISTS "${QT_PATH}")
            set(CMAKE_PREFIX_PATH "${QT_PATH}")
            message(STATUS "Found Qt at: ${CMAKE_PREFIX_PATH}")
            break()
        endif()
    endforeach()
endif()

# Find Qt6 or Qt5
find_package(QT NAMES Qt6 Qt5 COMPONENTS Core Widgets)
if(NOT QT_FOUND)
    message(FATAL_ERROR "Qt not found! Please set CMAKE_PREFIX_PATH to your Qt installation directory.\n"
                        "Example: cmake .. -DCMAKE_PREFIX_PATH=\"C:/Qt/5.9.0/msvc2017_64\"")
endif()

find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Source files
set(SOURCES
    src/main.cpp
    src/ui/mainwindow.cpp
    src/ui/mapwidget.cpp
    src/ui/mapeditor.cpp
    src/core/vehicle.cpp
    src/core/map.cpp
    src/core/mapdata.cpp
    src/control/controller.cpp
)

set(HEADERS
    src/ui/mainwindow.h
    src/ui/mapwidget.h
    src/ui/mapeditor.h
    src/core/vehicle.h
    src/core/map.h
    src/core/mapdata.h
    src/control/controller.h
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Link Qt libraries
target_link_libraries(${PROJECT_NAME} 
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Widgets
)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE src)

