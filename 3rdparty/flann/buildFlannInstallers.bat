@echo off

setlocal

set PACKAGE_NAME=flann
set PACKAGE_VERSION=1.6.9
set PACKAGE_ARCHIVE_BASENAME=flann-%PACKAGE_VERSION%-src
set PACKAGE_ARCHIVE_EXT=flann-%PACKAGE_VERSION%-src.zip
set PACKAGE_URL=http://people.cs.ubc.ca/%%7Emariusm/uploads/FLANN/%PACKAGE_ARCHIVE_EXT%
set PACKAGE_CMAKE_ARGS=-DBUILD_C_BINDINGS=true -DBUILD_PYTHON_BINDINGS=false -DBUILD_MATLAB_BINDINGS=false 

set VS2010VCVARSALL=%VS100COMNTOOLS%\..\..\VC\vcvarsall.bat
set VS2008VCVARSALL=%VS90COMNTOOLS%\..\..\VC\vcvarsall.bat

set CMAKE_COMMAND=c:\Program Files (x86)\CMake 2.8\bin\cmake.exe
set NSIS_EXE=c:\Program Files (x86)\NSIS\NSIS.exe
REM get URL2FILE from http://www.chamisplace.com/asp/redir.asp?i=DL_U2F_W_ZIP
set URL2FILE=URL2FILE.EXE

REM ********** download and patch ******************

echo Building %PACKAGE_NAME% Installers
echo Downloading %PACKAGE_NAME% %PACKAGE_VERSION% ...
%URL2FILE% %PACKAGE_URL% %PACKAGE_ARCHIVE_EXT%
echo Unpacking %PACKAGE_NAME% %PACKAGE_VERSION% ...
"%CMAKE_COMMAND%" -E tar xzf %PACKAGE_ARCHIVE_EXT%
echo Patching %PACKAGE_NAME% %PACKAGE_VERSION% ...
TortoiseMerge /diff:patch-1.6.9-msvc.patch /patchpath:%PACKAGE_ARCHIVE_BASENAME%

REM ********* VC 2010 - x86 ************************

echo Configuring cmake %PACKAGE_NAME% %PACKAGE_VERSION%  VC10 32bit...
if not exist "%VS2010VCVARSALL%" goto missing
call "%VS2010VCVARSALL%" x86
@mkdir build-vc10-x86
cd build-vc10-x86
"%CMAKE_COMMAND%" ..\%PACKAGE_ARCHIVE_BASENAME% -G "Visual Studio 10" -DCPACK_GENERATOR=NSIS -DNSIS_PROGRAM="%NSIS_EXE%" %PACKAGE_CMAKE_ARGS%
devenv %PACKAGE_NAME%.sln /build Release /project PACKAGE 
cd ..

REM ********* VC 2010 - x64 ************************

echo Configuring cmake %PACKAGE_NAME% %PACKAGE_VERSION%  VC10 64bit...
if not exist "%VS2010VCVARSALL%" goto missing
call "%VS2010VCVARSALL%" amd64
@mkdir build-vc10-amd64
cd build-vc10-amd64
"%CMAKE_COMMAND%" ..\%PACKAGE_ARCHIVE_BASENAME% -G "Visual Studio 10 Win64" -DCPACK_GENERATOR=NSIS  -DNSIS_PROGRAM="%NSIS_EXE%" %PACKAGE_CMAKE_ARGS%
devenv %PACKAGE_NAME%.sln /build Release /project PACKAGE 
cd ..

REM ******** VC 2008 - x86 *************************

REM echo Configuring cmake %PACKAGE_NAME% %PACKAGE_VERSION%  VC9 32bit...
REM if not exist "%VS2008VCVARSALL%" goto missing
REM call "%VS2008VCVARSALL%" x86
REM @mkdir build-vc9-x86
REM cd build-vc9-x86
REM "%CMAKE_COMMAND%" ..\%PACKAGE_ARCHIVE_BASENAME% -G "Visual Studio 9 2008" -DCPACK_GENERATOR=NSIS  -DNSIS_PROGRAM="%NSIS_EXE%" %PACKAGE_CMAKE_ARGS%
REM devenv %PACKAGE_NAME%.sln /build Release /project PACKAGE 
REM cd ..

REM ******** VC 2008 - x64 *************************

REM echo Configuring cmake %PACKAGE_NAME% %PACKAGE_VERSION%  VC9 64bit...
REM if not exist "%VS2008VCVARSALL%" goto missing
REM call "%VS2008VCVARSALL%" amd64
REM @mkdir build-vc9-amd64
REM cd build-vc9-amd64
REM "%CMAKE_COMMAND%" ..\%PACKAGE_ARCHIVE_BASENAME% -G "Visual Studio 9 2008 Win64" -DCPACK_GENERATOR=NSIS  -DNSIS_PROGRAM="%NSIS_EXE%" %PACKAGE_CMAKE_ARGS%
REM devenv %PACKAGE_NAME%.sln /build Release /project PACKAGE 
REM cd ..

goto end

:missing
echo The specified configuration type is missing.  The tools for the
echo configuration might not be installed.
:end
endlocal

