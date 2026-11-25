@echo off
rem Build ADAMEm with the Visual C++ compiler (cl.exe), 64-bit.
rem Needs: Build Tools for Visual Studio (C++ workload) and the SDL2 VC dev package.
setlocal
cd /d "%~dp0"
if "%SDL2_DIR%"=="" set "SDL2_DIR=C:\SDL2-2.0.12"

where cl >nul 2>nul
if not errorlevel 1 goto build

set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" goto novs
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSDIR=%%i"
if "%VSDIR%"=="" goto novs
call "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul

:build
if not exist "%SDL2_DIR%\include\SDL.h" goto nosdl
if not exist build mkdir build
cl /nologo /O2 /MD /W3 /wd4996 /I IDE /I "%SDL2_DIR%\include" ^
 /D WIN32 /D _CRT_SECURE_NO_WARNINGS /D LSB_FIRST /D SDL /D SOUND /D SOUND_PSG ^
 /D IDEHD /D DEBUG /D TEXT80 /D WINDOWS_HOST ^
 ADAMEm.c AdamemSDL.c AdamNet.c FujiCart.c F18A.c AdamSDLSound_2.c Bitmap.c Coleco.c ^
 IDE\HarddiskIDE.c sms_ntsc.c Sound.c Z80.c Z80Debug.c ^
 /Fo:build\ /Fe:build\adamem.exe ^
 /link /SUBSYSTEM:CONSOLE /LIBPATH:"%SDL2_DIR%\lib\x64" SDL2.lib SDL2main.lib ws2_32.lib shell32.lib
if errorlevel 1 exit /b 1
copy /y "%SDL2_DIR%\lib\x64\SDL2.dll" build\ >nul
copy /y OS7.rom build\ >nul
copy /y EOS.rom build\ >nul
copy /y WP.rom build\ >nul
echo Built build\adamem.exe
exit /b 0

:novs
echo Visual C++ compiler not found. Install "Build Tools for Visual Studio" with the C++ workload.
exit /b 1

:nosdl
echo SDL2 not found in %SDL2_DIR%. Unzip SDL2-devel-2.x.x-VC.zip there, or set SDL2_DIR.
exit /b 1
