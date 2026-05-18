@echo off
REM ============================================================
REM   打包 ModbusHMI 为单文件 exe
REM ------------------------------------------------------------
REM   当前环境打包 (跟当前 Python 架构/版本绑定)
REM     双击本脚本 → 生成 dist\ModbusHMI.exe
REM
REM   要目标 Win7 + 32-bit 的用户：
REM     1. 装 Python 3.8.10 Windows x86 (32-bit)
REM        https://www.python.org/ftp/python/3.8.10/python-3.8.10.exe
REM     2. 用那个 Python 的 pip 装:
REM          py -3.8-32 -m pip install pyinstaller==5.13.2 pyserial
REM     3. 改本脚本第一行的 PYTHON_EXE 为 "py -3.8-32"
REM     4. 双击本脚本
REM ============================================================

set PYTHON_EXE=python
REM set PYTHON_EXE=py -3.8-32

echo.
echo === 清理旧构建 ===
rd /s /q build 2>nul
rd /s /q dist  2>nul
del /f /q ModbusHMI.spec 2>nul

echo.
echo === 打包 ModbusHMI.exe ===
%PYTHON_EXE% -m PyInstaller ^
    --onefile ^
    --noconsole ^
    --clean ^
    --name ModbusHMI ^
    --collect-all tkinter ^
    --collect-all serial ^
    modbus_hmi.py

if errorlevel 1 (
    echo.
    echo [错误] 打包失败，看上面日志
    pause
    exit /b 1
)

echo.
echo === 构建完成 ===
dir dist\ModbusHMI.exe
echo.
echo 拷贝 dist\ModbusHMI.exe 到目标机器即可，无需 Python
pause
