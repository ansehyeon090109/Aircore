
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef int int32_t;
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
#define GPIO_PIN_12 4096
#define GPIO_PIN_13 8192
#define GPIO_PIN_6 64
#define INT16_MAX 32767
#define GPIO_PIN_15 32768
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
typedef struct { unsigned pins, IDR; } GPIO_TypeDef;
typedef struct { uint32_t counter, period, compare[2]; } TIM_HandleTypeDef;
typedef struct { int Instance, gState; uint8_t *pTxBuffPtr; uint16_t TxXferSize; } UART_HandleTypeDef;
static GPIO_TypeDef gpioB;
#define GPIOB (&gpioB)
static TIM_HandleTypeDef htim2, htim3, htim4;
static UART_HandleTypeDef huart3 = {USART3, 0, 0, 0};
static uint32_t tick, lastCommand, lastEncoder;
static unsigned delayCount, rxCalls, abortCalls, txCalls;
static int rxStatus;
static char transmitted[64];
static void check(int condition, const char *message) {
  if (!condition) { printf("FAIL: %s\n", message); exit(1); }
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

#define DEBUG_PIN_LEVEL 0
#define ENCODER_TX_INTERVAL_MS 50U
#define MOTOR_TIMEOUT_MS 300U
#define COMMAND_BUFFER_SIZE 24U


static uint8_t rxByte;
static char rxLine[COMMAND_BUFFER_SIZE];
static char pendingCommand[COMMAND_BUFFER_SIZE];
static uint8_t rxLength;
static uint8_t discardLine;
static volatile uint8_t commandReady;
static volatile uint8_t stopRequested;
static uint8_t previousCR;
static uint16_t previousLeftEncoder;
static uint16_t previousRightEncoder;
static uint32_t leftEncoder;
static uint32_t rightEncoder;
static char encoderMessage[sizeof("ENC,4294967295,4294967295\r\n")];


static int readDuty(const char **text, int *value)
{
  const char *p = *text;
  int sign = 1;
  int number = 0;

  if (*p == '-')
  {
    sign = -1;
    ++p;
  }
  if (*p < '0' || *p > '9')
  {
    return 0;
  }

  while (*p >= '0' && *p <= '9')
  {
    number = number * 10 + (*p - '0');
    if (number > 100)
    {
      return 0;
    }
    ++p;
  }

  *text = p;
  *value = sign * number;
  return 1;
}

static int parseCommand(const char *text, int *left, int *right)
{
  if (text[0] != 'M' || text[1] != ',')
  {
    return 0;
  }

  text += 2;
  if (!readDuty(&text, left) || *text != ',')
  {
    return 0;
  }
  ++text;

  return readDuty(&text, right) && *text == '\0';
}

static void setMotor(uint32_t channel, GPIO_TypeDef *port,
                     uint16_t in1, uint16_t in2, int duty, int *previousDuty)
{
  if (duty == *previousDuty)
  {
    return;
  }

  if ((duty > 0) != (*previousDuty > 0) ||
      (duty < 0) != (*previousDuty < 0))
  {
    __HAL_TIM_SET_COMPARE(&htim3, channel, 0U);
    if (duty != 0 && *previousDuty != 0)
    {
      HAL_Delay(2U);
    }
    HAL_GPIO_WritePin(port, in1 | in2, GPIO_PIN_RESET);
    if (duty != 0)
    {
      HAL_GPIO_WritePin(port, duty > 0 ? in1 : in2, GPIO_PIN_SET);
    }
  }

  uint32_t percent = (uint32_t)(duty < 0 ? -duty : duty);
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim3) + 1U;
  __HAL_TIM_SET_COMPARE(&htim3, channel, period * percent / 100U);
  *previousDuty = duty;
}

static void setMotors(int left, int right)
{
  static int previousLeft;
  static int previousRight;
  setMotor(TIM_CHANNEL_1, GPIOB, GPIO_PIN_0, GPIO_PIN_1, left, &previousLeft);
  setMotor(TIM_CHANNEL_2, GPIOB, GPIO_PIN_12, GPIO_PIN_13, right, &previousRight);
}

static void accumulateEncoder(TIM_HandleTypeDef *timer, uint16_t *previous, uint32_t *total)
{
  /* Samples must be less than half of a 16-bit counter cycle apart. */
  uint16_t current = (uint16_t)__HAL_TIM_GET_COUNTER(timer);
  uint16_t wrappedDelta = (uint16_t)(current - *previous);
  int32_t delta = wrappedDelta <= INT16_MAX ? wrappedDelta : (int32_t)wrappedDelta - 65536;

  *previous = current;
  *total += (uint32_t)delta;
}

static void sendEncoder(void)
{
  uint32_t left = leftEncoder;
  uint32_t right = rightEncoder;
  if (huart3.gState != HAL_UART_STATE_READY)
  {
    return;
  }
  int length = snprintf(encoderMessage, sizeof(encoderMessage), "ENC,%" PRIu32 ",%" PRIu32 "\r\n", left, right);

  if (length > 0 && (size_t)length < sizeof(encoderMessage))
  {
    HAL_UART_Transmit_IT(&huart3, (uint8_t *)encoderMessage, (uint16_t)length);
  }
}


#if DEBUG_PIN_LEVEL
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  static uint8_t pinMessage[] = "PIN,0,0\r\n";

  /* HAL advances the TX pointer to the end of the completed buffer. */
  if (huart != &huart3 ||
      huart->TxXferSize >= sizeof(encoderMessage) ||
      huart->pTxBuffPtr != (uint8_t *)encoderMessage + huart->TxXferSize)
  {
    return;
  }

  pinMessage[4] = (GPIOB->IDR & GPIO_PIN_6) ? '1' : '0';
  pinMessage[6] = (GPIOB->IDR & GPIO_PIN_7) ? '1' : '0';
  HAL_UART_Transmit_IT(huart, pinMessage, sizeof(pinMessage) - 1U);
}
#endif

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART3)
  {
    return;
  }

  if (rxByte == '\r' || rxByte == '\n')
  {
    if (!(rxByte == '\n' && previousCR))
    {
      if (discardLine || rxLength == 0U || commandReady || stopRequested)
      {
        stopRequested = 1U;
      }
      else
      {
        rxLine[rxLength] = '\0';
        memcpy(pendingCommand, rxLine, rxLength + 1U);
        commandReady = 1U;
      }
    }
    previousCR = (rxByte == '\r');
    rxLength = 0U;
    discardLine = 0U;
  }
  else
  {
    previousCR = 0U;
    if (!discardLine)
    {
      if (rxByte == '\0' || rxLength >= sizeof(rxLine) - 1U)
      {
        discardLine = 1U;
        stopRequested = 1U;
      }
      else
      {
        rxLine[rxLength++] = (char)rxByte;
      }
    }
  }

  if (HAL_UART_Receive_IT(huart, &rxByte, 1U) != HAL_OK)
  {
    stopRequested = 1U;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    stopRequested = 1U;
    rxLength = 0U;
    discardLine = 1U;
    previousCR = 0U;
    HAL_UART_AbortReceive(huart);
    if (HAL_UART_Receive_IT(huart, &rxByte, 1U) != HAL_OK)
    {
      Error_Handler();
    }
  }
}

static void step(void) { uint32_t now = HAL_GetTick();
    accumulateEncoder(&htim2, &previousLeftEncoder, &leftEncoder);
    accumulateEncoder(&htim4, &previousRightEncoder, &rightEncoder);

    if (stopRequested)
    {
      uint32_t irqState = __get_PRIMASK();
      __disable_irq();
      commandReady = 0U;
      stopRequested = 0U;
      __set_PRIMASK(irqState);
      setMotors(0, 0);
    }

    if (commandReady)
    {
      char command[COMMAND_BUFFER_SIZE];
      uint32_t irqState = __get_PRIMASK();
      __disable_irq();
      memcpy(command, pendingCommand, sizeof(command));
      commandReady = 0;
      __set_PRIMASK(irqState);

      int left;
      int right;
      if (parseCommand(command, &left, &right))
      {
        setMotors(left, right);
        lastCommand = now;
      }
      else
      {
        setMotors(0, 0);
      }
    }

    if (now - lastCommand >= MOTOR_TIMEOUT_MS)
    {
      setMotors(0, 0);
    }

    if (now - lastEncoder >= ENCODER_TX_INTERVAL_MS)
    {
      lastEncoder = now;
      sendEncoder();
    }
   }
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
  receive("M,50,25\r\n");
  CHECK(htim3.compare[0]==500 && htim3.compare[1]==250);
  CHECK(gpioB.pins==(GPIO_PIN_0 | GPIO_PIN_12));
  receive("M,-50,25\n");
  CHECK(htim3.compare[0]==500 && htim3.compare[1]==250 && delayCount==1);
  CHECK(gpioB.pins==(GPIO_PIN_1 | GPIO_PIN_12));
  puts("PASS independent PWM, direction and reversal delay");
  receive("M,0,100\r"); CHECK(htim3.compare[0]==0 && htim3.compare[1]==1000);
  tick=lastCommand+299; step(); CHECK(htim3.compare[1]==1000);
  tick=lastCommand+300; step(); CHECK(htim3.compare[0]==0 && htim3.compare[1]==0);
  tick=UINT32_MAX-100U; receive("M,10,10\n");
  tick=lastCommand+300U; step(); CHECK(htim3.compare[0]==0 && htim3.compare[1]==0);
  puts("PASS 300ms timeout including tick wrap");
  receive("M,10,10\n"); receive("M,101,0\n"); CHECK(htim3.compare[0]==0 && htim3.compare[1]==0);
  receive("M,10,10\n"); receive("12345678901234567890123456789"); CHECK(htim3.compare[0]==0 && htim3.compare[1]==0);
  receive("\nM,10,10\n"); CHECK(htim3.compare[0]==100);
  rxByte=0; HAL_UART_RxCpltCallback(&huart3); step(); CHECK(htim3.compare[0]==0);
  receive("\nM,10,10\n"); HAL_UART_ErrorCallback(&huart3); step(); CHECK(htim3.compare[0]==0 && abortCalls==1);
  receive("\nM,10,10\n"); rxStatus=HAL_ERROR; rxByte='M'; HAL_UART_RxCpltCallback(&huart3); step(); CHECK(htim3.compare[0]==0);
  rxStatus=HAL_OK; receive("\n");
  puts("PASS invalid, oversized, NUL and UART error stop/recovery");
  const char *burst="M,50,50\nM,60,60\n";
  while (*burst) { rxByte=(uint8_t)*burst++; HAL_UART_RxCpltCallback(&huart3); }
  step(); CHECK(htim3.compare[0]==0 && htim3.compare[1]==0);
  puts("PASS pending command overflow stops both motors");
  huart3.gState=HAL_UART_STATE_READY; htim2.counter=65535; htim4.counter=123;
  accumulateEncoder(&htim2, &previousLeftEncoder, &leftEncoder);
  accumulateEncoder(&htim4, &previousRightEncoder, &rightEncoder);
  sendEncoder(); CHECK(strcmp(transmitted,"ENC,4294967295,123\r\n")==0);
  unsigned before=txCalls; htim2.counter=0; accumulateEncoder(&htim2, &previousLeftEncoder, &leftEncoder); sendEncoder(); CHECK(txCalls==before);
  CHECK(strcmp(encoderMessage,"ENC,4294967295,123\r\n")==0);
  huart3.gState=HAL_UART_STATE_READY; sendEncoder(); CHECK(strcmp(transmitted,"ENC,0,123\r\n")==0);
  huart3.gState=HAL_UART_STATE_READY; lastEncoder=tick; tick+=49; before=txCalls; step(); CHECK(txCalls==before);
  ++tick; step(); CHECK(txCalls==before+1);
#if DEBUG_PIN_LEVEL
  gpioB.IDR=GPIO_PIN_6;
  huart3.gState=HAL_UART_STATE_READY; HAL_UART_TxCpltCallback(&huart3);
  CHECK(strcmp(transmitted,"PIN,1,0\r\n")==0 && txCalls==before+2);
  huart3.gState=HAL_UART_STATE_READY; HAL_UART_TxCpltCallback(&huart3);
  CHECK(txCalls==before+2);
#endif
  CHECK(leftEncoder==0 && rightEncoder==123);
  htim4.counter=65530; step(); CHECK(rightEncoder==UINT32_MAX-5U);
  htim4.counter=15; step(); CHECK(rightEncoder==15U);
  CHECK(rxCalls>0);
  puts("PASS independent raw encoder counts, wrap, 50ms schedule and async TX buffer lifetime");
  return 0;
}
