#include "stm32f4xx.h"

#define ENCODER_INPUT_FILTER 4U

volatile int32_t leftEncoderCount;
volatile int32_t rightEncoderCount;

static void initEncoderPin(GPIO_TypeDef *port, uint32_t pin, uint32_t alternate)
{
  const uint32_t modeShift = 2U * pin;
  const uint32_t afShift = 4U * (pin % 8U);

  MODIFY_REG(port->AFR[pin / 8U], 0xFU << afShift, alternate << afShift);
  CLEAR_BIT(port->PUPDR, 3U << modeShift);
  MODIFY_REG(port->MODER, 3U << modeShift, 2U << modeShift);
}

static void initEncoder(TIM_TypeDef *timer)
{
  /* Direct TI1/TI2 inputs, non-inverted polarity, count both channels. */
  timer->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 |
                 (ENCODER_INPUT_FILTER << TIM_CCMR1_IC1F_Pos) |
                 (ENCODER_INPUT_FILTER << TIM_CCMR1_IC2F_Pos);
  timer->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
  timer->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
  timer->ARR = UINT32_MAX;
  timer->EGR = TIM_EGR_UG;
  timer->CNT = 0U;
  timer->SR = 0U;
  timer->CR1 = TIM_CR1_CEN;
}

int main(void)
{
  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN);
  (void)RCC->AHB1ENR;
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM5EN);
  (void)RCC->APB1ENR;

  initEncoderPin(GPIOA, 0U, 2U);  /* Left A: TIM5_CH1 */
  initEncoderPin(GPIOA, 1U, 2U);  /* Left B: TIM5_CH2 */
  initEncoderPin(GPIOA, 15U, 1U); /* Right A: TIM2_CH1 */
  initEncoderPin(GPIOB, 3U, 1U);  /* Right B: TIM2_CH2 */

  initEncoder(TIM5);
  initEncoder(TIM2);

  for (;;)
  {
    leftEncoderCount = (int32_t)TIM5->CNT;
    rightEncoderCount = (int32_t)TIM2->CNT;
  }
}
