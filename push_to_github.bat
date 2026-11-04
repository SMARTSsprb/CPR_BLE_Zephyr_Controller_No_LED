@echo off
cd /d "%~dp0"
echo Pushing CPR_BLE_Zephyr_Controller_No_LED to GitHub...
echo Repository: https://github.com/SMARTSsprb/CPR_BLE_Zephyr_Controller_No_LED.git
echo.
git push -u origin main
if %errorlevel% equ 0 (
    echo.
    echo ====================================
    echo SUCCESS! Repository uploaded to GitHub.
    echo ====================================
) else (
    echo.
    echo Push failed or was cancelled.
)
echo.
pause

