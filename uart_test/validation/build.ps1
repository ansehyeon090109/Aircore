$ErrorActionPreference = 'Stop'
$taskIde = 'C:/ST/STM32CubeIDE_1.19.0/STM32CubeIDE'
$taskMake = (Get-ChildItem "$taskIde/plugins" -Recurse -Filter make.exe | Select-Object -First 1).FullName
$taskCompiler = (Get-ChildItem "$taskIde/plugins" -Recurse -Filter arm-none-eabi-gcc.exe | Select-Object -First 1).DirectoryName
if (!$taskMake -or !$taskCompiler) { throw 'STM32CubeIDE toolchain not found' }
$env:PATH = "$taskCompiler;$(Split-Path $taskMake);$env:PATH"
& $taskMake -C Debug -B -j4 all 2>&1 | Tee-Object -FilePath validation/build-debug.log
if ($LASTEXITCODE -ne 0) { throw "Build failed: $LASTEXITCODE" }
