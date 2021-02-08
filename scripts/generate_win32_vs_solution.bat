@echo off

cd ..
REM Get project directory name
for %%a in (.) do set projectDir=%%~na

cd ..
mkdir build-jsl-win32
cd build-jsl-win32
cmake ../%projectDir% -G "Visual Studio 16 2019" -A Win32 -DBUILD_SHARED_LIBS=1

echo Open the generated Visual Studio solution located at: %cd%\JoyShockLibrary.sln

pause
