$ErrorActionPreference = 'Stop'
function Assert-Setting($condition, $message) {
    if (!$condition) { throw $message }
}
$taskDb = 'C:/ST/STM32CubeIDE_1.19.0/STM32CubeIDE/plugins/com.st.stm32cube.common.mx_6.15.0.202507011659/db/mcu'
[xml]$taskMcu = Get-Content -LiteralPath "$taskDb/STM32F407V(E-G)Tx.xml" -Raw
[xml]$taskProject = Get-Content .project -Raw
[xml]$taskCproject = Get-Content .cproject -Raw
[xml]$taskLaunch = Get-Content 'uart_test Debug.launch' -Raw
$taskIoc = @{}
Get-Content uart_test.ioc | ForEach-Object {
    if ($_ -match '^([^#=]+)=(.*)$') { $taskIoc[$matches[1].Replace('\ ', ' ')] = $matches[2] }
}
$taskPins = @{'PB4'='TIM3_CH1';'PB5'='TIM3_CH2';'PA0-WKUP'='TIM5_CH1';'PA1'='TIM5_CH2';'PA15'='TIM2_CH1';'PB3'='TIM2_CH2';'PD8'='USART3_TX';'PD9'='USART3_RX'}
foreach ($taskPin in $taskPins.Keys) {
    $taskNode = $taskMcu.SelectSingleNode("//*[local-name()='Pin' and @Name='$taskPin']")
    Assert-Setting ($null -ne $taskNode) "MCU pin missing: $taskPin"
    $taskSignal = $taskPins[$taskPin]
    Assert-Setting ($null -ne $taskNode.SelectSingleNode("*[local-name()='Signal' and @Name='$taskSignal']")) "Unsupported pin function: $taskPin/$taskSignal"
}
Assert-Setting ($taskProject.projectDescription.name -eq 'uart_test') 'Project name mismatch'
foreach ($taskConfiguration in $taskCproject.SelectNodes('//cconfiguration')) {
    $taskName = $taskConfiguration.SelectSingleNode('storageModule').GetAttribute('name')
    $taskMcuOptions = $taskConfiguration.SelectNodes(".//option[contains(@superClass,'option.target_mcu')]")
    foreach ($taskOption in $taskMcuOptions) { Assert-Setting ($taskOption.value -eq 'STM32F407VGTx') "$taskName MCU mismatch" }
    $taskSources = @($taskConfiguration.SelectNodes('.//sourceEntries/entry') | ForEach-Object {$_.name})
    Assert-Setting (($taskSources -join ',') -eq 'USB_HOST,Core,Middlewares,Drivers') "$taskName source roots mismatch"
    $taskLinker = $taskConfiguration.SelectSingleNode(".//option[contains(@superClass,'c.linker.option.script')]")
    Assert-Setting ($taskLinker.value -like '*STM32F407VGTX_FLASH.ld*') "$taskName linker mismatch"
}
$taskAttributes = @{}
foreach ($taskAttribute in $taskLaunch.launchConfiguration.ChildNodes) {
    if ($taskAttribute.HasAttribute('key') -and $taskAttribute.HasAttribute('value')) { $taskAttributes[$taskAttribute.key] = $taskAttribute.value }
}
Assert-Setting ($taskAttributes['com.st.stm32cube.ide.mcu.debug.launch.swd_mode'] -eq 'true') 'Debug interface is not SWD'
Assert-Setting ($taskAttributes['com.st.stm32cube.ide.mcu.debug.launch.enable_swv'] -eq 'false') 'SWV conflicts with encoder PB3'
Assert-Setting ($taskAttributes['org.eclipse.cdt.launch.PROGRAM_NAME'] -eq 'Debug/uart_test.elf') 'Debug image mismatch'
Assert-Setting ($taskAttributes['org.eclipse.cdt.launch.PROJECT_ATTR'] -eq 'uart_test') 'Debug project mismatch'
$taskMain = Get-Content Core/Src/main.c -Raw
$taskHal = Get-Content Core/Inc/stm32f4xx_hal_conf.h -Raw
$taskLink = Get-Content STM32F407VGTX_FLASH.ld -Raw
foreach ($taskExpected in @('PLLM = 8;', 'PLLN = 336;', 'PLLP = RCC_PLLP_DIV2;', 'PLLQ = 7;', 'APB1CLKDivider = RCC_HCLK_DIV4;', 'APB2CLKDivider = RCC_HCLK_DIV2;')) {
    Assert-Setting ($taskMain.Contains($taskExpected)) "Clock mismatch: $taskExpected"
}
Assert-Setting ($taskHal -match '#define HSE_VALUE\s+8000000U') 'HAL HSE mismatch'
Assert-Setting ($taskIoc['RCC.SYSCLKFreq_VALUE'] -eq '168000000') 'IOC SYSCLK mismatch'
Assert-Setting ($taskIoc['RCC.APB1TimFreq_Value'] -eq '84000000') 'IOC APB1 timer clock mismatch'
Assert-Setting ($taskLink -match 'FLASH\s+\(rx\)\s*:\s*ORIGIN = 0x8000000,\s*LENGTH = 1024K') 'Flash memory mismatch'
Assert-Setting ($taskLink -match 'RAM\s+\(xrw\)\s*:\s*ORIGIN = 0x20000000,\s*LENGTH = 128K') 'RAM mismatch'
foreach ($taskModule in @('TIM','UART','GPIO','RCC','CORTEX')) { Assert-Setting ($taskHal -match "(?m)^#define HAL_${taskModule}_MODULE_ENABLED") "HAL module disabled: $taskModule" }
$taskStartup = Get-Content Core/Startup/startup_stm32f407vgtx.s -Raw
Assert-Setting ($taskStartup.Contains('.word     USART3_IRQHandler') -or $taskStartup -match '\.word\s+USART3_IRQHandler') 'UART vector missing'
$taskFunctionList = $taskIoc['ProjectManager.functionlistsort']
Assert-Setting ($taskFunctionList -match 'MX_TIM3_Init-TIM3') 'TIM3 missing from code generation order'
Assert-Setting ($taskFunctionList -notmatch 'MX_I2C1_Init') 'I2C1 remains in code generation order'
$taskMsp = Get-Content Core/Src/stm32f4xx_hal_msp.c -Raw
$taskUserOne = [regex]::Match($taskMsp,'(?s)/\* USER CODE BEGIN 1 \*/(.*?)/\* USER CODE END 1 \*/').Groups[1].Value
Assert-Setting ($taskUserOne -notmatch 'void HAL_TIM_.*Msp') 'Generated MSP functions inside USER CODE would duplicate after regeneration'
Write-Output 'PASS MCU database pin functions, Debug/Release target and source roots, linker memory, clock tree, HAL modules, IRQ vector, SWD/SWV and regeneration structure'
