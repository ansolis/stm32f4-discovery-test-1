#include "stm32f4xx.h"
#include <stdio.h>

/*
 * Processor used:  STM32F407VG.  Board:  STM32F4Discovery.
 *
 * The external clock frequency is specified as a preprocessor definition
 * passed to the compiler via a command line option (see the 'C/C++ General' ->
 * 'Paths and Symbols' -> the 'Symbols' tab, if you want to change it).
 * The value selected during project creation was HSE_VALUE=8000000.
 *
 * Note1: The default clock settings assume that the HSE_VALUE is a multiple
 * of 1MHz, and try to reach the maximum speed available for the
 * board. It does NOT guarantee that the required USB clock of 48MHz is
 * available. If you need this, please update the settings of PLL_M, PLL_N,
 * PLL_P, PLL_Q in libs/CMSIS/src/system_stm32f4xx.c to match your needs.
 *
 * Note2: The external memory controllers are not enabled. If needed, you
 * have to define DATA_IN_ExtSRAM or DATA_IN_ExtSDRAM and to configure
 * the memory banks in libs/CMSIS/src/system_stm32f4xx.c to match your needs.
 *
 * The build does not use startup files, and on Release it does not even use
 * any standard library function (on Debug the printf() brings lots of
 * functions; removing it should also use no other standard lib functions).
 *
 */


/* ----- Preprocessor definitions ------------------------------------------ */

#define SYSTICK_FREQUENCY_HZ   1000
#define BLINK_PORT             GPIOD    /* Port used for LEDs. */
#define LED_GREEN              12       /* Port pin used for green LED. */
#define LED_BLUE               15       /* Port pin used for blue LED.  */
#define BLINK_RCC_BIT          RCC_AHB1Periph_GPIOD    /* Port D used for LED output. */
#define BLINK_TICKS            SYSTICK_FREQUENCY_HZ/2  /* LED on/off duration in ticks. */
#define MAX_STRLEN             12       /* Length of string holding received chracters. */

/* ----- Forward declarations ---------------------------------------------- */

/* Inserts a delay time in ticks. */
static void Delay(__IO uint32_t nTime);

/* SysTick event handler. */
static void TimingDelay_Decrement(void);

/* SysTick event handler. */
void SysTick_Handler(void);

/* Configures user button on PA0 pin to generate EXTI Line0 interrupt. */
static void EXTILine0_Config(void);

/* Configure blue and green LED pins for output. */
static void LED_Config(void);

/* Configures the USART2 parameters and initializes the peripheral. */
static void USART_Config(void);

/* Initializes the USART2 Peripheral. */
static void STM_EVAL_COMInit(USART_InitTypeDef* USART_InitStruct);

/* Writes a character to a USART port. */
void USART_puts(USART_TypeDef* USARTx, volatile char *s);

/* ----- Global variables -------------------------------------------------- */

static __IO uint32_t uwTimingDelay;  /* Counter used for delays. */

char received_string[MAX_STRLEN+1];  /* this will hold the received string. */

// ----------------------------------------------------------------------------

/* main function. */
int main(void) {

    int i;
    int seconds = 0;
    RCC_ClocksTypeDef RCC_Clocks;  /* Structure containing CPU clock settings. */

#if defined(DEBUG)
    /*
     * Send a greeting to the standard output (the semihosting debug channel
     * on Debug, ignored on Release).
     */
    printf("Hello ARM World!\n");
#endif

    /*
     * At this stage the microcontroller clock setting is already configured,
     * this is done through SystemInit() function which is called from startup
     * files (startup_cm.c) before the branch to the
     * application main. To reconfigure the default setting of SystemInit()
     * function, refer to system_stm32f4xx.c file.
     */

    /* Use SysTick as reference for the timer */
    RCC_GetClocksFreq(&RCC_Clocks);
    SysTick_Config(RCC_Clocks.HCLK_Frequency / SYSTICK_FREQUENCY_HZ);

    /* GPIO Peripheral clock enable */
    RCC_AHB1PeriphClockCmd(BLINK_RCC_BIT, ENABLE);

    /* Enable GPIOA clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /* Configure GPIO pins for LED and USART2 peripheral. */
    LED_Config();
    USART_Config();
    
    /* Output "Hello World!" on USART2 port. */
    USART_puts(USART2, "Hello World!\r\n");

    /* Send data out the USART port */
    for(i = 'a'; i<='z'; i++)
    {
        /* Send out a characters 'a' through 'z'. */
        Delay(10);
        USART_SendData(USART2, i);
  
        /* Loop until the end of transmission */
        while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    }

    /* Configure EXTI Line0 (connected to PA0 pin and user button) in interrupt mode */
    EXTILine0_Config();


    /* Infinite loop */
    while (1) {

        /* Receive a character on USART2 port. */
        if((USART2->SR & USART_SR_RXNE))
            printf("received %c\n", USART2->DR & 0x1FF);

        /* Assume the LED is active low */

        /* Turn on led by setting the pin low */
        GPIO_ResetBits(BLINK_PORT, (1 << LED_GREEN));

        Delay(BLINK_TICKS);

        /* Turn off led by setting the pin high */
        GPIO_SetBits(BLINK_PORT, (1 << LED_GREEN));

        Delay(BLINK_TICKS);

        ++seconds;

#if defined(DEBUG)
        /* Count seconds on the debug channel. */
        printf("Second %d\n", seconds);
#endif
    }
}

// ----------------------------------------------------------------------------

/**
 * @brief  Inserts a delay time.
 * @param  nTime: specifies the delay time length, in SysTick ticks.
 * @retval None
 */
void Delay(__IO uint32_t nTime) {
    uwTimingDelay = nTime;

    while (uwTimingDelay != 0);
}

// ----------------------------------------------------------------------------

/**
 * @brief  Decrements the TimingDelay variable.
 * @param  None
 * @retval None
 */
void TimingDelay_Decrement(void) {
    if (uwTimingDelay != 0x00) {
        uwTimingDelay--;
    }
}

// ----------------------------------------------------------------------------

/**
 * @brief  This function is the SysTick Handler.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void) {
    TimingDelay_Decrement();
}

// ----------------------------------------------------------------------------

/**
 * @brief  Configures EXTI Line0 (connected to PA0 pin and user button) in interrupt mode
 * @param  None
 * @retval None
 */
static void EXTILine0_Config(void) {
    EXTI_InitTypeDef EXTI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Configure PA0 pin as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Connect EXTI Line0 to PA0 pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

    /* Configure EXTI Line0 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set EXTI Line0 Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

// ----------------------------------------------------------------------------

/**
 * @brief  This function configures GPIO port that controls LED on the boards.
 * @param  None
 * @retval None
 */
static void LED_Config(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Configure pin in output push/pull mode */
    GPIO_InitStructure.GPIO_Pin = (1 << LED_GREEN) | (1 << LED_BLUE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(BLINK_PORT, &GPIO_InitStructure);
}

// ----------------------------------------------------------------------------

/**
 * @brief  This function handles External line 0 interrupt request associated with the user button press
 * @param  None
 * @retval None
 */
void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        /* Toggle the blue LED */
        GPIO_ToggleBits(BLINK_PORT, (1 << LED_BLUE));

        /* Clear the EXTI line 0 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

// ----------------------------------------------------------------------------

/**
 * @brief  Configures the USART Peripheral.
 * @param  None
 * @retval None
 */
static void USART_Config(void)
{
  USART_InitTypeDef USART_InitStructure;

  /* USARTx configured as follows:
   - BaudRate = 115200 baud
   - Word Length = 8 Bits
   - One Stop Bit
   - No parity
   - Hardware flow control disabled (RTS and CTS signals)
   - Receive and transmit enabled. */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  STM_EVAL_COMInit(&USART_InitStructure);
}

// ----------------------------------------------------------------------------

/**
 * @brief  Configures COM port.
 * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
 *   contains the configuration information for the specified USART peripheral.
 * @retval None
 */
static void STM_EVAL_COMInit(USART_InitTypeDef* USART_InitStruct) {
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;  // this is used to configure the NVIC (nested vector interrupt controller)

  /* Enable GPIO clock for port A */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOA, ENABLE); // may have already been enabled
  /* Enable UART clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  /* Connect PA2 to USARTx_Tx*/
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);

  /* Connect PA3 to USARTx_Rx*/
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

  /* Configure USART Tx as alternate function  */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART Rx as alternate function  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(USART2, USART_InitStruct);
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);           // enable the USART2 receive interrupt

  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;        // we want to configure the USART2 interrupts
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART2 interrupts
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       // this sets the subpriority inside the group
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;          // the USART2 interrupts are globally enabled
  NVIC_Init(&NVIC_InitStructure);                          // the properties are passed to the NVIC_Init function which takes care of the low level stuff

  /* Enable USART */
  USART_Cmd(USART2, ENABLE);
}

// ----------------------------------------------------------------------------

/**
 * @brief  Outputs a character string to a serial port.
 * @param  USARTx: pointer to a USART_InitTypeDef structure that
 *   contains the configuration information for the specified USART peripheral.
 * @param  s: pointer to the c-string array.
 * @retval None
 */
void USART_puts(USART_TypeDef* USARTx, volatile char *s)
{

  while(*s)
  {
      /* Sending characters until string is empty. */
      while( !(USARTx->SR & 0x00000040) );
      Delay(10);
      USART_SendData(USARTx, *s);
      s++;
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief  This is the interrupt request handler (IRQ) for ALL USART2 interrupts.
 * @param  None
 * @retval None
 */
void USART2_IRQHandler(void)
{
  /* check if the USART2 receive interrupt flag was set. */
  if( USART_GetITStatus(USART2, USART_IT_RXNE) )
  {
      static uint8_t cnt = 0; /* this counter is used to determine the string length. */
      char t = USART2->DR;    /* the character from the USART2 data register is saved. */

      /* Check if the received character is not the LF character (used to determine end of string)
       * or the if the maximum string length has been been reached. */
      if( (t != '\n') && (cnt < MAX_STRLEN) )
      {
          received_string[cnt] = t;
          cnt++;
          printf("Received:  %c\n", t);
      }
      else
      {   /* Otherwise, reset the character counter and print the received string. */
          cnt = 0;
          received_string[cnt] = 0;
          printf("String: %s\n", received_string);
      }
  }
}
