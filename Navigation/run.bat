@echo off
setlocal

set "ROOT_DIR=%~dp0"
where py >nul 2>nul
if not errorlevel 1 (
    py -3 -c "import sys" >nul 2>nul
    if not errorlevel 1 (
        py -3 "%ROOT_DIR%python\scripts\run.py" %*
        exit /b %errorlevel%
    )
)

python "%ROOT_DIR%python\scripts\run.py" %*
exit /b %errorlevel%
