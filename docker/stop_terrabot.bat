@echo off

set CONTAINER_NAME=terrabot_container

REM Check if the container exists
docker ps --format "{{.Names}}" | findstr /i "^%CONTAINER_NAME%" >nul
if %ERRORLEVEL%==0 (
    echo Stopping %CONTAINER_NAME%
    docker container stop %CONTAINER_NAME%
    docker container rm %CONTAINER_NAME%
) else (
    echo %CONTAINER_NAME% not currently running
)
pause
