@echo off

set USER_DIR="."
if not "%1"=="" set USER_DIR=%1

set CONTAINER_NAME=terrabot_container_ros2
set IMAGE_NAME=terrabot_image_ros2
set PORT=5901

REM Check if the container exists
docker ps --format "{{.Names}}" | findstr /i "^%CONTAINER_NAME%" >nul
if %ERRORLEVEL%==0 (
    echo Starting existing container: %CONTAINER_NAME%
    docker exec -it %CONTAINER_NAME% bash
) else (
    echo Starting new container: %CONTAINER_NAME%
    docker run -it --rm --name %CONTAINER_NAME% -p %PORT%:%PORT% ^
               --volume %USER_DIR%:/home/robotanist/User %IMAGE_NAME% 
)
pause
