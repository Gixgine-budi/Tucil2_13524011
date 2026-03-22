@echo off
setlocal
cd /d "%~dp0"

if /i "%~1"=="clean" goto clean
if /i "%~1"=="run" goto run
if "%~1"=="" goto build
if /i "%~1"=="build" goto build

echo Usage: %~nx0 [build ^| run ^| clean]
exit /b 1

:build
if not exist bin mkdir bin
echo Building ttov...
go build -o bin\ttov.exe .\src\ttov
exit /b %ERRORLEVEL%

:run
if not exist bin mkdir bin
echo Building ttov...
go build -o bin\ttov.exe .\src\ttov
if errorlevel 1 exit /b 1
bin\ttov.exe
exit /b %ERRORLEVEL%

:clean
if exist bin rmdir /s /q bin
exit /b 0
