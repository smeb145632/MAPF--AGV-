@echo off
chcp 65001 >nul
REM AGV项目构建脚本

echo ========================================
echo AGV调度控制界面 - 构建脚本
echo ========================================
echo.

REM 检查是否有清理参数
if "%1"=="clean" (
    echo [信息] 清理构建目录...
    if exist build (
        rmdir /s /q build
        echo [信息] 构建目录已清理
    ) else (
        echo [信息] 构建目录不存在，无需清理
    )
    echo.
)

REM 检查CMake是否安装
where cmake >nul 2>&1
if %errorlevel% neq 0 (
    echo [错误] 未找到CMake！
    echo.
    echo 请先安装CMake:
    echo 1. 访问 https://cmake.org/download/
    echo 2. 下载Windows安装程序
    echo 3. 安装时选择"Add CMake to system PATH"
    echo.
    pause
    exit /b 1
)

echo [信息] 找到CMake
cmake --version
echo.

REM 尝试自动查找Qt
set QT_PATH=
if exist "C:\Qt\Qt5.9.0\5.9\msvc2017_64" (
    set QT_PATH=C:\Qt\Qt5.9.0\5.9\msvc2017_64
) else if exist "C:\Qt\Qt5.9.0\5.9\msvc2015_64" (
    set QT_PATH=C:\Qt\Qt5.9.0\5.9\msvc2015_64
) else if exist "C:\Qt\5.9.0\msvc2017_64" (
    set QT_PATH=C:\Qt\5.9.0\msvc2017_64
) else if exist "C:\Qt\5.9.0\msvc2015_64" (
    set QT_PATH=C:\Qt\5.9.0\msvc2015_64
)

if "%QT_PATH%"=="" (
    echo [警告] 未自动找到Qt路径
    echo 请手动设置Qt路径，例如:
    echo   set QT_PATH=C:\Qt\Qt5.9.0\5.9\msvc2017_64
    echo.
    set /p QT_PATH="请输入Qt路径 (或按回车跳过，让CMake自动查找): "
    if "%QT_PATH%"=="" set "QT_PATH="
)

REM 检查编译器 - 使用更可靠的方法
echo [信息] 检查C++编译器...
where cl.exe >nul 2>&1
if %errorlevel% neq 0 (
    echo [警告] 未找到MSVC编译器，尝试自动设置...
    echo.
    
    REM 尝试自动查找并设置Visual Studio环境
    set "VS_SETUP=0"
    
    REM 检查VS2026 (版本18)
    if exist "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" (
        echo [信息] 找到Visual Studio 2026，正在设置环境...
        call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
        set "VS_SETUP=1"
    ) else if exist "C:\Program Files\Microsoft Visual Studio\18\Professional\VC\Auxiliary\Build\vcvars64.bat" (
        echo [信息] 找到Visual Studio 2026，正在设置环境...
        call "C:\Program Files\Microsoft Visual Studio\18\Professional\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
        set "VS_SETUP=1"
    ) else if exist "C:\Program Files\Microsoft Visual Studio\18\Enterprise\VC\Auxiliary\Build\vcvars64.bat" (
        echo [信息] 找到Visual Studio 2026，正在设置环境...
        call "C:\Program Files\Microsoft Visual Studio\18\Enterprise\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
        set "VS_SETUP=1"
    REM 检查VS2022
    ) else if exist "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" (
        echo [信息] 找到Visual Studio 2022，正在设置环境...
        call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
        set "VS_SETUP=1"
    ) else if exist "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvars64.bat" (
        echo [信息] 找到Visual Studio 2019，正在设置环境...
        call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
        set "VS_SETUP=1"
    ) else if exist "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvars64.bat" (
        echo [信息] 找到Visual Studio 2017，正在设置环境...
        call "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
        set "VS_SETUP=1"
    )
    
    REM 再次检查编译器
    where cl.exe >nul 2>&1
    if %errorlevel% neq 0 (
        if "%VS_SETUP%"=="0" (
            echo [错误] 未找到Visual Studio
            echo.
            echo 请安装Visual Studio或使用Developer Command Prompt运行此脚本
            echo.
            pause
            exit /b 1
        ) else (
            echo [错误] 设置编译环境失败
            echo 请使用Developer Command Prompt for VS运行此脚本
            pause
            exit /b 1
        )
    )
)

echo [信息] MSVC编译器已就绪
echo.

REM 创建build目录
if not exist build mkdir build
cd build

echo.
echo [信息] 开始配置项目...
echo.

if "%QT_PATH%"=="" (
    cmake ..
) else (
    echo [信息] 使用Qt路径: %QT_PATH%
    cmake .. -DCMAKE_PREFIX_PATH="%QT_PATH%"
)

if %errorlevel% neq 0 (
    echo.
    echo [错误] CMake配置失败！
    echo.
    echo 可能的原因:
    echo 1. Qt未安装或路径不正确
    echo 2. 需要设置CMAKE_PREFIX_PATH
    echo.
    echo 解决方法:
    echo 手动运行: cmake .. -DCMAKE_PREFIX_PATH="你的Qt路径"
    echo 例如: cmake .. -DCMAKE_PREFIX_PATH="C:\Qt\Qt5.9.0\5.9\msvc2017_64"
    echo.
    pause
    exit /b 1
)

echo.
echo [信息] 开始编译...
echo.

cmake --build . --config Release

if %errorlevel% neq 0 (
    echo.
    echo [错误] 编译失败！
    pause
    exit /b 1
)

echo.
echo [成功] 编译完成！
echo 可执行文件位置: build\Release\MAPF_AGV.exe
echo.
echo 使用说明:
echo   - 运行程序: build\Release\MAPF_AGV.exe
echo   - 清理构建: build.bat clean
echo.
pause


