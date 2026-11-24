$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot
$ide = 'C:/ST/STM32CubeIDE_1.19.0/STM32CubeIDE/plugins'
$gcc = (Get-ChildItem $ide -Recurse -Filter arm-none-eabi-gcc.exe | Select-Object -First 1).FullName
if (!$gcc) { throw 'STM32 ARM GCC not found' }

Push-Location $root
try {
  New-Item -ItemType Directory -Force Debug | Out-Null
  $sources = Get-ChildItem Core/Src, Core/Startup, Drivers/STM32F1xx_HAL_Driver/Src -File |
    Where-Object { $_.Extension -in '.c', '.s' } |
    ForEach-Object { Resolve-Path -Relative $_.FullName }
  $options = @(
    '-mcpu=cortex-m3', '-mthumb', '-mfloat-abi=soft', '-std=gnu11',
    '-g3', '-O0', '-Wall', '-ffunction-sections', '-fdata-sections',
    '-DUSE_HAL_DRIVER', '-DSTM32F103xB', '-ICore/Inc',
    '-IDrivers/STM32F1xx_HAL_Driver/Inc', '-IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy',
    '-IDrivers/CMSIS/Device/ST/STM32F1xx/Include', '-IDrivers/CMSIS/Include',
    '-TSTM32F103RBTX_FLASH.ld', '--specs=nano.specs', '--specs=nosys.specs',
    '-Wl,--gc-sections,-Map=Debug/firmware.map', '-o', 'Debug/firmware.elf'
  )
  cmd /c ('"' + $gcc + '" ' + (($options + $sources) -join ' ') + ' > validation\build-debug.log 2>&1')
  $result = $LASTEXITCODE
  Get-Content validation/build-debug.log
  if ($result -ne 0) { throw "Firmware build failed ($result)" }
  & (Join-Path (Split-Path $gcc) 'arm-none-eabi-size.exe') Debug/firmware.elf
  & (Join-Path (Split-Path $gcc) 'arm-none-eabi-objcopy.exe') -O ihex Debug/firmware.elf Debug/firmware.hex
  if ($LASTEXITCODE -ne 0) { throw 'HEX export failed' }
} finally {
  Pop-Location
}
