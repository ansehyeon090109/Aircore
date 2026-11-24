const fs = require('fs');
const assert = require('assert/strict');
const entries = fs.readFileSync('uart_test.ioc','utf8').split(/\r?\n/).filter(l=>l && !l.startsWith('#')).map(l=>{const at=l.indexOf('='); return [l.slice(0,at).replace(/\\ /g,' '), l.slice(at+1)];});
const config = Object.fromEntries(entries);
assert.equal(entries.length,Object.keys(config).length,'duplicate IOC keys');
const pins = Object.entries(config).filter(([k])=>/^Mcu.Pin\d+$/.test(k)).map(([,v])=>v);
assert.equal(pins.length,Number(config['Mcu.PinsNb']));
assert.equal(new Set(pins).size,pins.length);
const required = {'PB4':'S_TIM3_CH1','PB5':'S_TIM3_CH2','PB0':'GPIO_Output','PB1':'GPIO_Output','PE7':'GPIO_Output','PE8':'GPIO_Output','PA0-WKUP':'S_TIM5_CH1','PA1':'S_TIM5_CH2','PA15':'S_TIM2_CH1_ETR','PB3':'S_TIM2_CH2','PD8':'USART3_TX','PD9':'USART3_RX','PA13':'SYS_JTMS-SWDIO','PA14':'SYS_JTCK-SWCLK'};
for (const [pin,signal] of Object.entries(required)) { assert(pins.includes(pin)); assert.equal(config[pin+'.Signal'],signal); }
assert.equal(config['PA13.Mode'],'Serial_Wire'); assert.equal(config['PA14.Mode'],'Serial_Wire');
assert(!Object.values(config).includes('I2C1')); assert(!pins.includes('PB8')); assert(!pins.includes('PB9'));
for(const [k,v] of Object.entries({'TIM3.Prescaler':'83','TIM3.Period':'999','TIM3.Pulse-PWM Generation1 CH1':'0','TIM3.Pulse-PWM Generation2 CH2':'0','TIM3.OCPolarity_1':'TIM_OCPOLARITY_HIGH','TIM3.OCPolarity_2':'TIM_OCPOLARITY_HIGH','TIM2.EncoderMode':'TIM_ENCODERMODE_TI12','TIM5.EncoderMode':'TIM_ENCODERMODE_TI12','TIM2.Period':'4294967295','TIM5.Period':'4294967295','VP_TIM3_VS_ClockSourceINT.Mode':'Internal','USART3.BaudRate':'115200'})) assert.equal(config[k],v,k);
const main=fs.readFileSync('Core/Src/main.c','utf8');
const msp=fs.readFileSync('Core/Src/stm32f4xx_hal_msp.c','utf8');
const it=fs.readFileSync('Core/Src/stm32f4xx_it.c','utf8');
for(const call of ['HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1)','HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2)','HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL)','HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL)','HAL_UART_Receive_IT(&huart3, &rxByte, 1U)']) assert(main.includes(call),call);
assert(main.includes('htim3.Init.Prescaler = 83;')); assert(main.includes('htim3.Init.Period = 999;'));
assert(!main.includes('MX_I2C1_Init')); assert(!msp.includes('__HAL_RCC_I2C1_CLK_ENABLE'));
assert(msp.includes('GPIO_AF2_TIM3') && msp.includes('GPIO_PIN_4 | GPIO_PIN_5'));
assert(msp.includes('GPIO_AF1_TIM2') && msp.includes('GPIO_AF2_TIM5') && msp.includes('GPIO_AF7_USART3'));
assert(it.includes('HAL_UART_IRQHandler(&huart3)'));
const backup=fs.readdirSync('backups').sort()[0];
for (const file of ['Core/Src/main.c','Core/Src/stm32f4xx_hal_msp.c']) {
  const original=fs.readFileSync('backups/'+backup+'/'+file,'utf8');
  const modified=fs.readFileSync(file,'utf8');
  for(const match of original.matchAll(/\/\* USER CODE (?:BEGIN|END) .*?\*\//g)) assert(modified.includes(match[0]),match[0]);
}
console.log('PASS IOC pin uniqueness, peripheral registration, PWM parameters, encoder modes, SWD, UART IRQ, I2C removal and USER CODE markers');
