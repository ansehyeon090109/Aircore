@echo off
set "PATH=C:\Program Files\Microsoft Visual Studio\18\Community\VC\Tools\MSVC\14.50.35717\bin\Hostx64\x64;%PATH%"
lib /nologo /machine:x64 /def:validation\msvcrt.def /out:validation\msvcrt.lib
if errorlevel 1 exit /b 1
cl /nologo /W4 /WX /GS- /Zl /TC validation\firmware_test.c /Fe:validation\firmware_test.exe /Fo:validation\firmware_test.obj /link /nodefaultlib /entry:main /subsystem:console validation\msvcrt.lib
if errorlevel 1 exit /b 1
validation\firmware_test.exe
