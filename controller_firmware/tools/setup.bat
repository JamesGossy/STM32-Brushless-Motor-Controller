@echo off
rem Installs prerequisites, builds and tests the simulator (and firmware if STM32CubeCLT is installed).
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0setup.ps1" %*
pause
