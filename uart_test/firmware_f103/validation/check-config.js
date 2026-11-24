const assert = require('node:assert/strict');
const fs = require('node:fs');

const iocText = fs.readFileSync('firmware.ioc', 'utf8');
const ioc = Object.fromEntries(iocText.split(/\r?\n/).filter(line => line.includes('='))
  .map(line => [line.slice(0, line.indexOf('=')), line.slice(line.indexOf('=') + 1)]));
const main = fs.readFileSync('Core/Src/main.c', 'utf8');
const msp = fs.readFileSync('Core/Src/stm32f1xx_hal_msp.c', 'utf8');
const irq = fs.readFileSync('Core/Src/stm32f1xx_it.c', 'utf8');

assert.equal(ioc['Mcu.CPN'], 'STM32F103RBT6');
assert.equal(ioc.board, 'NUCLEO-F103RB');
const pins = Array.from({length: +ioc['Mcu.PinsNb']}, (_, i) => ioc['Mcu.Pin' + i]);
assert(!pins.includes(undefined));
assert.equal(new Set(pins).size, pins.length);
for (const [pin, signal] of Object.entries({
  'PA0-WKUP': 'S_TIM2_CH1_ETR', PA1: 'S_TIM2_CH2',
  PA6: 'S_TIM3_CH1', PA7: 'S_TIM3_CH2', PB6: 'S_TIM4_CH1', PB7: 'S_TIM4_CH2',
  PB0: 'GPIO_Output', PB1: 'GPIO_Output', PB12: 'GPIO_Output', PB13: 'GPIO_Output',
  PA2: 'USART2_TX', PA3: 'USART2_RX', PB10: 'USART3_TX', PB11: 'USART3_RX',
  PA13: 'SYS_JTMS-SWDIO', PA14: 'SYS_JTCK-SWCLK',
})) {
  assert(pins.includes(pin), pin);
  assert.equal(ioc[pin + '.Signal'], signal);
}
for (const timer of [2, 4]) {
  assert.equal(ioc[`TIM${timer}.Period`], '65535');
  assert.equal(ioc[`TIM${timer}.EncoderMode`], 'TIM_ENCODERMODE_TI12');
  for (const ch of [1, 2]) assert.equal(ioc[`TIM${timer}.IC${ch}Filter`], '4');
  assert(main.includes(`htim${timer}.Init.Period = UINT16_MAX;`));
  assert(main.includes(`HAL_TIM_Encoder_Start(&htim${timer}, TIM_CHANNEL_ALL)`));
}
assert.equal(+ioc['RCC.APB1TimFreq_Value'] /
  ((+ioc['TIM3.Prescaler'] + 1) * (+ioc['TIM3.Period'] + 1)), 1000);
assert(main.includes('htim3.Init.Prescaler = 63;'));
assert(main.includes('htim3.Init.Period = 999;'));
assert(main.includes('RCC_PLLSOURCE_HSI_DIV2'));
assert(main.includes('huart3.Init.BaudRate = 115200;'));
assert.equal(ioc['USART3.BaudRate'], '115200');
assert.match(msp, /TIM2_CLK_ENABLE[\s\S]*?GPIO_PIN_0 \| GPIO_PIN_1;\s*HAL_GPIO_Init\(GPIOA/);
assert.match(msp, /TIM4_CLK_ENABLE[\s\S]*?GPIO_PIN_6 \| GPIO_PIN_7;\s*HAL_GPIO_Init\(GPIOB/);
assert.match(msp, /HAL_TIM_MspPostInit[\s\S]*?GPIO_PIN_6 \| GPIO_PIN_7;[\s\S]*?HAL_GPIO_Init\(GPIOA/);
assert.match(msp, /USART3_CLK_ENABLE[\s\S]*?GPIO_PIN_10;[\s\S]*?GPIO_PIN_11;[\s\S]*?HAL_NVIC_EnableIRQ\(USART3_IRQn\)/);
assert.match(msp, /__HAL_AFIO_REMAP_SWJ_NOJTAG\(\)/);
assert.match(irq, /USART3_IRQHandler\(void\)\s*\{\s*HAL_UART_IRQHandler\(&huart3\);/);
console.log('PASS F103RB pin assignments, PWM frequency, encoder setup, SWD and UART IRQ');
