@echo off
setlocal enabledelayedexpansion

set XAUTH=%TEMP%\.docker.xauth

if not exist %XAUTH% (
    for /f "delims=" %%i in ('xauth nlist %DISPLAY%') do (
        set "xauth_line=%%i"
        set "xauth_line=ffff!xauth_line:~4!"
        echo !xauth_line! | xauth -f %XAUTH% nmerge -
    )
    if not defined xauth_line (
        type nul > %XAUTH%
    )
    icacls %XAUTH% /grant Everyone:R
)

docker run -it ^
    --rm ^
    --name orca4 ^
    -e DISPLAY ^
    -e QT_X11_NO_MITSHM=1 ^
    -e XAUTHORITY=%XAUTH% ^
    -e NVIDIA_VISIBLE_DEVICES=all ^
    -e NVIDIA_DRIVER_CAPABILITIES=all ^
    -v "%XAUTH%:%XAUTH%" ^
    -v "/tmp/.X11-unix:/tmp/.X11-unix" ^
    -v "/etc/localtime:/etc/localtime:ro" ^
    -v "/dev/input:/dev/input" ^
    --privileged ^
    --security-opt seccomp=unconfined ^
    --gpus all ^
    orca4:latest

endlocal
