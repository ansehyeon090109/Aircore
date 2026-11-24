const fs = require('fs');
const text = fs.readFileSync('uart_test.ioc', 'utf8');
const rows = text.split(/\r?\n/);
const values = new Map(rows.filter(r => r && !r.startsWith('#')).map(r => {
  const split = r.indexOf('=');
  return [r.slice(0, split), r.slice(split + 1)];
}));
const ips = [...values].filter(([k]) => /^Mcu.IP\d+$/.test(k)).sort((a,b)=>Number(a[0].slice(6))-Number(b[0].slice(6))).map(([,v])=>v).filter(v=>v!=='I2C1');
const pins = [...values].filter(([k]) => /^Mcu.Pin\d+$/.test(k)).sort((a,b)=>Number(a[0].slice(7))-Number(b[0].slice(7))).map(([,v])=>v).filter(v=>v!=='PB8' && v!=='PB9');
for(const key of [...values.keys()]) if(/^Mcu.IP\d+$|^Mcu.Pin\d+$|^PB[89]\.|^I2C1\./.test(key)) values.delete(key);
ips.forEach((v,i)=>values.set('Mcu.IP'+i,v)); values.set('Mcu.IPNb',String(ips.length));
pins.forEach((v,i)=>values.set('Mcu.Pin'+i,v)); values.set('Mcu.PinsNb',String(pins.length));
const params = {
  'TIM3.CounterMode':'TIM_COUNTERMODE_UP', 'TIM3.ClockDivision':'TIM_CLOCKDIVISION_DIV1',
  'TIM3.Prescaler':'83', 'TIM3.Period':'999',
  'TIM3.Pulse-PWM\\ Generation1\\ CH1':'0', 'TIM3.Pulse-PWM\\ Generation2\\ CH2':'0',
  'TIM3.OCMode_PWM-PWM\\ Generation1\\ CH1':'TIM_OCMODE_PWM1',
  'TIM3.OCMode_PWM-PWM\\ Generation2\\ CH2':'TIM_OCMODE_PWM1',
  'TIM3.OCPolarity_1':'TIM_OCPOLARITY_HIGH', 'TIM3.OCPolarity_2':'TIM_OCPOLARITY_HIGH',
  'USART3.BaudRate':'115200', 'USART3.WordLength':'WORDLENGTH_8B',
  'USART3.StopBits':'STOPBITS_1', 'USART3.Parity':'PARITY_NONE'
};
for(const [k,v] of Object.entries(params)) values.set(k,v);
values.set('TIM3.IPParameters','Channel-PWM Generation1 CH1,Channel-PWM Generation2 CH2,Prescaler,Period,CounterMode,ClockDivision,Pulse-PWM Generation1 CH1,Pulse-PWM Generation2 CH2,OCMode_PWM-PWM Generation1 CH1,OCMode_PWM-PWM Generation2 CH2,OCPolarity_1,OCPolarity_2');
values.set('USART3.IPParameters','VirtualMode,BaudRate,WordLength,StopBits,Parity');
const functions = values.get('ProjectManager.functionlistsort').split(',').filter(v=>!v.includes('MX_I2C1_Init') && !v.includes('MX_TIM3_Init'));
functions.splice(functions.findIndex(v=>v.includes('MX_TIM2_Init')),0,'0-MX_TIM3_Init-TIM3-false-HAL-true');
values.set('ProjectManager.functionlistsort',functions.map((v,i)=>v.replace(/^\d+-/,(i+1)+'-')).join(','));
const output = ['#MicroXplorer Configuration settings - do not modify', ...[...values].sort(([a],[b])=>a<b?-1:a>b?1:0).map(([k,v])=>k+'='+v),''].join('\r\n');
fs.writeFileSync('validation/repaired.ioc', output);
