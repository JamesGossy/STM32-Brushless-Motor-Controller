@echo off
rem Starts the simulated motor and the dashboard, then opens it in the browser.
cd /d "%~dp0.."
where py >nul 2>nul && (set PY=py) || (set PY=python)
start "" /b cmd /c "timeout /t 3 >nul && start http://localhost:8988"
%PY% tools\dashboard\plot_motor.py --sim %*
pause
