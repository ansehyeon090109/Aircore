const fs = require('fs');
const main = fs.readFileSync('Core/Src/main.c', 'utf8');
const block = name => main.split(`/* USER CODE BEGIN ${name} */`)[1].split(`/* USER CODE END ${name} */`)[0];
const loop = block('3').trim().replace(/\}\s*$/, '');
const source = `
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef unsigned __int64 size_t;
#define UINT32_MAX 4294967295U
#define PRIu32 "u"
int printf(const char *, ...);
int puts(const char *);
int _snprintf(char *, size_t, const char *, ...);
void *memcpy(void *, const void *, size_t);
int strcmp(const char *, const char *);
__declspec(noreturn) void exit(int);
#define snprintf _snprintf
#define HAL_OK 0
#define HAL_ERROR 1
#define HAL_UART_STATE_READY 0
#define USART3 3
#define TIM_CHANNEL_1 0
#define TIM_CHANNEL_2 1
#define GPIO_PIN_0 1
#define GPIO_PIN_1 2
#define GPIO_PIN_3 8
#define GPIO_PIN_7 128
#define GPIO_PIN_8 256
#define GPIO_PIN_15 32768
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
typedef struct { unsigned pins, IDR; } GPIO_TypeDef;
typedef struct { uint32_t counter, period, compare[2]; } TIM_HandleTypeDef;
typedef struct { int Instance, gState; uint8_t *pTxBuffPtr; uint16_t TxXferSize; } UART_HandleTypeDef;
static GPIO_TypeDef gpioA, gpioB, gpioE;
#define GPIOA (&gpioA)
#define GPIOB (&gpioB)
#define GPIOE (&gpioE)
static TIM_HandleTypeDef htim2, htim3, htim5;
static UART_HandleTypeDef huart3 = {USART3, 0, 0, 0};
static uint32_t tick, lastCommand, lastEncoder;
static unsigned delayCount, rxCalls, abortCalls, txCalls;
static int rxStatus;
static char transmitted[64];
static void check(int condition, const char *message) {
  if (!condition) { printf("FAIL: %s\\n", message); exit(1); }
}
#define CHECK(c) check((c), #c)
static uint32_t HAL_GetTick(void) { return tick; }
static void HAL_Delay(uint32_t ms) {
  CHECK(htim3.compare[0] == 0U || htim3.compare[1] == 0U);
  CHECK(ms == 2U); tick += ms; ++delayCount;
}
static uint32_t __get_PRIMASK(void) { return 0; }
static void __disable_irq(void) { }
static void __set_PRIMASK(uint32_t state) { (void)state; }
static void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pins, int state) {
  if (state) port->pins |= pins; else port->pins &= ~(unsigned)pins;
}
#define __HAL_TIM_SET_COMPARE(t,c,v) ((t)->compare[(c)] = (v))
#define __HAL_TIM_GET_AUTORELOAD(t) ((t)->period)
#define __HAL_TIM_GET_COUNTER(t) ((t)->counter)
static int HAL_UART_Transmit_IT(UART_HandleTypeDef *u, uint8_t *p, uint16_t n) {
  CHECK(n < sizeof(transmitted)); memcpy(transmitted,p,n); transmitted[n]=0;
  u->pTxBuffPtr=p+n; u->TxXferSize=n;
  u->gState=1; ++txCalls; return HAL_OK;
}
static int HAL_UART_Receive_IT(UART_HandleTypeDef *u, uint8_t *p, uint16_t n) {
  (void)u; (void)p; CHECK(n==1); ++rxCalls; return rxStatus;
}
static int HAL_UART_AbortReceive(UART_HandleTypeDef *u) { (void)u; ++abortCalls; return HAL_OK; }
static void Error_Handler(void) { CHECK(0); }
${block('PD')}
${block('PV')}
${block('0')}
${block('4')}
static void step(void) { ${loop} }
static void receive(const char *text) {
  while (*text) { rxByte=(uint8_t)*text++; HAL_UART_RxCpltCallback(&huart3); step(); }
}
int main(void) {
  int left=0, right=0;
  htim3.period=999;
  CHECK(parseCommand("M,-100,100", &left,&right) && left==-100 && right==100);
  const char *invalid[] = {"", "M", "M,", "M,101,0", "M,-101,0", "M,+1,2", "M,1,2x", "M,1", "X,1,2", "M,99999999999999,0", "M,1, 2"};
  for (unsigned i=0;i<sizeof(invalid)/sizeof(invalid[0]);++i) CHECK(!parseCommand(invalid[i],&left,&right));
  puts("PASS strict parser and signed limits");
  receive("M,50,25\\r\\n");
  CHECK(htim3.compare[0]==500 && htim3.compare[1]==250);
  CHECK(gpioB.pins==GPIO_PIN_0 && gpioE.pins==GPIO_PIN_7);
  receive("M,-50,25\\n");
  CHECK(htim3.compare[0]==500 && htim3.compare[1]==250 && delayCount==1);
  CHECK(gpioB.pins==GPIO_PIN_1 && gpioE.pins==GPIO_PIN_7);
  puts("PASS independent PWM, direction and reversal delay");
  receive("M,0,100\\r"); CHECK(htim3.compare[0]==0 && htim3.compare[1]==1000);
  tick=lastCommand+299; step(); CHECK(htim3.compare[1]==1000);
  tick=lastCommand+300; step(); CHECK(htim3.compare[0]==0 && htim3.compare[1]==0);
  tick=UINT32_MAX-100U; receive("M,10,10\\n");
  tick=lastCommand+300U; step(); CHECK(htim3.compare[0]==0 && htim3.compare[1]==0);
  puts("PASS 300ms timeout including tick wrap");
  receive("M,10,10\\n"); receive("M,101,0\\n"); CHECK(htim3.compare[0]==0 && htim3.compare[1]==0);
  receive("M,10,10\\n"); receive("12345678901234567890123456789"); CHECK(htim3.compare[0]==0 && htim3.compare[1]==0);
  receive("\\nM,10,10\\n"); CHECK(htim3.compare[0]==100);
  rxByte=0; HAL_UART_RxCpltCallback(&huart3); step(); CHECK(htim3.compare[0]==0);
  receive("\\nM,10,10\\n"); HAL_UART_ErrorCallback(&huart3); step(); CHECK(htim3.compare[0]==0 && abortCalls==1);
  receive("\\nM,10,10\\n"); rxStatus=HAL_ERROR; rxByte='M'; HAL_UART_RxCpltCallback(&huart3); step(); CHECK(htim3.compare[0]==0);
  rxStatus=HAL_OK; receive("\\n");
  puts("PASS invalid, oversized, NUL and UART error stop/recovery");
  const char *burst="M,50,50\\nM,60,60\\n";
  while (*burst) { rxByte=(uint8_t)*burst++; HAL_UART_RxCpltCallback(&huart3); }
  step(); CHECK(htim3.compare[0]==0 && htim3.compare[1]==0);
  puts("PASS pending command overflow stops both motors");
  huart3.gState=HAL_UART_STATE_READY; htim5.counter=UINT32_MAX; htim2.counter=123;
  sendEncoder(); CHECK(strcmp(transmitted,"ENC,4294967295,123\\r\\n")==0);
  unsigned before=txCalls; htim5.counter=0; sendEncoder(); CHECK(txCalls==before);
  CHECK(strcmp(encoderMessage,"ENC,4294967295,123\\r\\n")==0);
  huart3.gState=HAL_UART_STATE_READY; sendEncoder(); CHECK(strcmp(transmitted,"ENC,0,123\\r\\n")==0);
  huart3.gState=HAL_UART_STATE_READY; lastEncoder=tick; tick+=49; before=txCalls; step(); CHECK(txCalls==before);
  ++tick; step(); CHECK(txCalls==before+1);
#if DEBUG_PIN_LEVEL
  gpioA.IDR=GPIO_PIN_15;
  huart3.gState=HAL_UART_STATE_READY; HAL_UART_TxCpltCallback(&huart3);
  CHECK(strcmp(transmitted,"PIN,1,0\\r\\n")==0 && txCalls==before+2);
  huart3.gState=HAL_UART_STATE_READY; HAL_UART_TxCpltCallback(&huart3);
  CHECK(txCalls==before+2);
#endif
  CHECK(rxCalls>0);
  puts("PASS independent raw encoder counts, wrap, 50ms schedule and async TX buffer lifetime");
  return 0;
}
`;
fs.writeFileSync('validation/firmware_test.c', source);
