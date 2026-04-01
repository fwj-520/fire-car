@echo off
chcp 65001 >nul
title 导航功能启动脚本

echo ============================================
echo 🚗 导航功能启动脚本
echo ============================================

cd /d "%~dp0"

if "%ROS_DISTRO%" == "" (
    echo ❌ 未检测到 ROS 2 环境
    echo 请先运行: call C:\dev\ros2\humble\local_setup.bat
    echo.
    pause
    exit /b 1
)

echo ✅ ROS 2 环境已配置 (%ROS_DISTRO%)
echo.

cd /d "%~dp0\.."

if not exist "install\setup.bat" (
    echo ❌ 工作区未编译
    echo 请先运行: colcon build --packages-select yahboomcar_nav
    echo.
    pause
    exit /b 1
)

call install\local_setup.bat

echo ✅ 工作区已准备就绪
echo.

echo ============================================
echo 步骤 1: 启动导航功能
echo ============================================

echo 正在启动导航功能...
ros2 launch yahboomcar_nav navigation_launch.py

echo.
echo ============================================
echo ❌ 导航功能已停止
echo ============================================

pause