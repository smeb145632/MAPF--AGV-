# AGV项目构建脚本 (PowerShell版本)
# 设置输出编码为UTF-8以正确显示中文
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
$OutputEncoding = [System.Text.Encoding]::UTF8

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "AGV调度控制界面 - 构建脚本" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# 检查清理参数
if ($args.Count -gt 0 -and $args[0] -eq "clean") {
    Write-Host "[信息] 清理构建目录..." -ForegroundColor Yellow
    if (Test-Path "build") {
        try {
            Remove-Item -Recurse -Force "build" -ErrorAction Stop
            Write-Host "[信息] 构建目录已清理" -ForegroundColor Green
        } catch {
            Write-Host "[警告] 清理失败，可能文件正在使用中" -ForegroundColor Yellow
            Write-Host "请关闭所有相关程序后重试" -ForegroundColor Yellow
        }
    } else {
        Write-Host "[信息] 构建目录不存在，无需清理" -ForegroundColor Yellow
    }
    Write-Host ""
    # 清理后继续执行构建
}

# 检查CMake是否安装
$cmakePath = Get-Command cmake -ErrorAction SilentlyContinue
if (-not $cmakePath) {
    Write-Host "[错误] 未找到CMake！" -ForegroundColor Red
    Write-Host ""
    Write-Host "请先安装CMake:" -ForegroundColor Yellow
    Write-Host "1. 访问 https://cmake.org/download/"
    Write-Host "2. 下载Windows安装程序"
    Write-Host "3. 安装时选择'Add CMake to system PATH'"
    Write-Host ""
    Read-Host "按回车键退出"
    exit 1
}

Write-Host "[信息] 找到CMake" -ForegroundColor Green
cmake --version
Write-Host ""

# 尝试自动查找Qt
$qtPath = $null
$possiblePaths = @(
    "C:\Qt\Qt5.9.0\5.9\msvc2017_64",
    "C:\Qt\Qt5.9.0\5.9\msvc2015_64",
    "C:\Qt\Qt5.9.0\5.9\msvc2013_64",
    "C:\Qt\5.9.0\msvc2017_64",
    "C:\Qt\5.9.0\msvc2015_64",
    "C:\Qt\5.9.0\msvc2013_64",
    "C:\Qt\6.7.0\msvc2019_64",
    "C:\Qt\6.6.0\msvc2019_64",
    "C:\Qt\6.5.0\msvc2019_64",
    "C:\Qt\6.4.0\msvc2019_64",
    "C:\Qt\5.15.2\msvc2019_64",
    "C:\Qt\5.15.0\msvc2019_64",
    "C:\Qt\5.14.2\msvc2019_64"
)

foreach ($path in $possiblePaths) {
    if (Test-Path $path) {
        $qtPath = $path
        Write-Host "[信息] 自动找到Qt: $qtPath" -ForegroundColor Green
        break
    }
}

if (-not $qtPath) {
    Write-Host "[警告] 未自动找到Qt路径" -ForegroundColor Yellow
    Write-Host "请手动输入Qt路径，例如: C:\Qt\Qt5.9.0\5.9\msvc2017_64" -ForegroundColor Yellow
    $qtPath = Read-Host "请输入Qt路径 (或按回车让CMake自动查找)"
    if ([string]::IsNullOrWhiteSpace($qtPath)) {
        $qtPath = $null
    }
}

# 检查编译器
Write-Host "[信息] 检查C++编译器..." -ForegroundColor Green
$clPath = Get-Command cl -ErrorAction SilentlyContinue
if (-not $clPath) {
    Write-Host "[警告] 未找到MSVC编译器，尝试自动设置..." -ForegroundColor Yellow
    Write-Host ""
    
    # 尝试自动查找并设置Visual Studio环境
    $vsSetup = $false
    $vsPaths = @(
        "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat",
        "C:\Program Files\Microsoft Visual Studio\18\Professional\VC\Auxiliary\Build\vcvars64.bat",
        "C:\Program Files\Microsoft Visual Studio\18\Enterprise\VC\Auxiliary\Build\vcvars64.bat",
        "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat",
        "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat",
        "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvars64.bat",
        "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvars64.bat"
    )
    
    foreach ($vsPath in $vsPaths) {
        if (Test-Path $vsPath) {
            $vsVersion = "Unknown"
            if ($vsPath -match "\\18\\") { $vsVersion = "2026" }
            elseif ($vsPath -match "\\2022\\") { $vsVersion = "2022" }
            elseif ($vsPath -match "\\2019\\") { $vsVersion = "2019" }
            elseif ($vsPath -match "\\2017\\") { $vsVersion = "2017" }
            
            Write-Host "[信息] 找到Visual Studio $vsVersion，正在设置环境..." -ForegroundColor Green
            try {
                & cmd /c "`"$vsPath`" && set" | ForEach-Object {
                    if ($_ -match "^(.+?)=(.*)$") {
                        [System.Environment]::SetEnvironmentVariable($matches[1], $matches[2], "Process")
                    }
                }
                $vsSetup = $true
                break
            } catch {
                Write-Host "[警告] 设置环境时出错: $_" -ForegroundColor Yellow
            }
        }
    }
    
    # 再次检查编译器
    $clPath = Get-Command cl -ErrorAction SilentlyContinue
    if (-not $clPath) {
        if (-not $vsSetup) {
            Write-Host "[错误] 未找到Visual Studio或Visual C++ Build Tools" -ForegroundColor Red
            Write-Host ""
            Write-Host "请安装Visual Studio:" -ForegroundColor Yellow
            Write-Host "1. 访问 https://visualstudio.microsoft.com/downloads/"
            Write-Host "2. 下载 Visual Studio Community (免费)"
            Write-Host "3. 安装时选择 '使用C++的桌面开发'"
            Write-Host ""
            Write-Host "或者使用 'Developer Command Prompt for VS' 运行此脚本" -ForegroundColor Yellow
            Write-Host ""
            Read-Host "按回车键退出"
            exit 1
        } else {
            Write-Host "[错误] 设置编译环境失败" -ForegroundColor Red
            Write-Host "请使用 'Developer Command Prompt for VS' 运行此脚本" -ForegroundColor Yellow
            Read-Host "按回车键退出"
            exit 1
        }
    }
}

Write-Host "[信息] 找到MSVC编译器" -ForegroundColor Green
Write-Host ""

# 创建build目录
if (-not (Test-Path "build")) {
    New-Item -ItemType Directory -Path "build" | Out-Null
    Write-Host "[信息] 创建构建目录" -ForegroundColor Green
}

# 保存当前目录
$originalLocation = Get-Location

try {
    Set-Location build

Write-Host ""
Write-Host "[信息] 开始配置项目..." -ForegroundColor Green
Write-Host ""

if ($qtPath) {
    Write-Host "[信息] 使用Qt路径: $qtPath" -ForegroundColor Green
    cmake .. -DCMAKE_PREFIX_PATH="$qtPath"
} else {
    cmake ..
}

if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "[错误] CMake配置失败！" -ForegroundColor Red
    Write-Host ""
    Write-Host "可能的原因:" -ForegroundColor Yellow
    Write-Host "1. Qt未安装或路径不正确"
    Write-Host "2. 需要设置CMAKE_PREFIX_PATH"
    Write-Host "3. 编译器环境未正确设置"
    Write-Host ""
    Write-Host "解决方法:" -ForegroundColor Yellow
    if ($qtPath) {
        Write-Host "当前Qt路径: $qtPath" -ForegroundColor Gray
    }
    Write-Host '手动运行: cmake .. -DCMAKE_PREFIX_PATH="你的Qt路径"'
    Write-Host '例如: cmake .. -DCMAKE_PREFIX_PATH="C:\Qt\Qt5.9.0\5.9\msvc2017_64"'
    Write-Host ""
    Set-Location $originalLocation
    Read-Host "按回车键退出"
    exit 1
}

       Write-Host ""
       Write-Host "[信息] 检查是否有程序正在运行..." -ForegroundColor Yellow
       $process = Get-Process -Name "MAPF_AGV" -ErrorAction SilentlyContinue
       if ($process) {
           Write-Host "[警告] 检测到 MAPF_AGV.exe 正在运行，正在关闭..." -ForegroundColor Yellow
           Stop-Process -Name "MAPF_AGV" -Force -ErrorAction SilentlyContinue
           Start-Sleep -Seconds 1
       }
       
       Write-Host "[信息] 开始编译..." -ForegroundColor Green
       Write-Host ""

       cmake --build . --config Release

if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "[错误] 编译失败！" -ForegroundColor Red
    Write-Host "请检查上面的错误信息" -ForegroundColor Yellow
    Set-Location $originalLocation
    Read-Host "按回车键退出"
    exit 1
}

Write-Host ""
Write-Host "[成功] 编译完成！" -ForegroundColor Green
Write-Host "可执行文件位置: build\Release\MAPF_AGV.exe" -ForegroundColor Cyan
Write-Host ""

# 部署 Qt DLL（解决找不到 Qt5Gui.dll 等问题）
$exePath = Join-Path $originalLocation "build\Release\MAPF_AGV.exe"
if (Test-Path $exePath) {
    $windeployqt = $null
    if ($qtPath) {
        $wdp = Join-Path $qtPath "bin\windeployqt.exe"
        if (Test-Path $wdp) { $windeployqt = $wdp }
    }
    if (-not $windeployqt) {
        $windeployqt = Get-Command windeployqt -ErrorAction SilentlyContinue
        if ($windeployqt) { $windeployqt = $windeployqt.Source }
    }
    if ($windeployqt) {
        Write-Host "[信息] 正在部署 Qt 依赖库 (windeployqt)..." -ForegroundColor Green
        & $windeployqt --release --no-translations $exePath
        if ($LASTEXITCODE -eq 0) {
            Write-Host "[成功] Qt DLL 已复制到 exe 目录，可直接运行" -ForegroundColor Green
        } else {
            Write-Host "[警告] windeployqt 执行失败，请手动部署" -ForegroundColor Yellow
        }
    } else {
        Write-Host "[警告] 未找到 windeployqt，Qt DLL 需手动部署" -ForegroundColor Yellow
        Write-Host "  方法: 将 Qt 安装目录下的 bin\windeployqt.exe 添加到 PATH" -ForegroundColor Gray
        Write-Host "  或运行: <Qt的bin目录>\windeployqt.exe --release build\Release\MAPF_AGV.exe" -ForegroundColor Gray
    }
    Write-Host ""
}

# 检查可执行文件是否存在
if (Test-Path $exePath) {
    $fileInfo = Get-Item $exePath
    Write-Host "文件大小: $([math]::Round($fileInfo.Length / 1MB, 2)) MB" -ForegroundColor Gray
    Write-Host "修改时间: $($fileInfo.LastWriteTime)" -ForegroundColor Gray
    Write-Host ""
}

Write-Host "使用说明:" -ForegroundColor Yellow
Write-Host "  - 运行程序: .\build\Release\MAPF_AGV.exe" -ForegroundColor White
Write-Host "  - 清理构建: .\build.ps1 clean" -ForegroundColor White
Write-Host ""

} finally {
    # 确保恢复原始目录
    Set-Location $originalLocation
}

Read-Host "按回车键退出"


