/**
*****************************************************************************
**  File        : stm32f4xx_it.c
**  Abstract    : Main Interrupt Service Routines.
**                provides template for all exceptions handler and
**                peripherals interrupt service routines
**  Environment : Atollic TrueSTUDIO(R)
**                STMicroelectronics STM32F4xx Standard Peripherals Library
**  Distribution: The file is distributed as is, without any warranty
**                of any kind. (c)Copyright Atollic AB.
*****************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx_conf.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define UART_ERR_INT_MASK        0x0000015F     /* UART error interrupt flags */

/* Private macro -------------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
extern volatile uint32_t    TimingDelay;
extern volatile uint32_t    toDelay;
extern volatile uint32_t    opTimeOut;
extern volatile uint32_t    sysMode;

extern volatile uint8_t     devStatus;
extern volatile uint32_t    runChain;

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/


/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/* handle NMI exceptions
 */
void NMI_Handler(void)
{
}

/* handle Hard Fault exceptions
 */
static volatile uint32_t  __hvi;

void  HardFault_Handler (void)
{
    while (1)  /* infinite loop */
    {
        __hvi++;
    }
}


/* handle Memory Manage exceptions
 */
void MemManage_Handler(void)
{
    while (1);  /* infinite loop */
}

/* handle Bus Fault exceptions
 */
void BusFault_Handler(void)
{
    while (1);  /* infinite loop */
}

/* handle Usage Fault exceptions
 */
void UsageFault_Handler(void)
{
    while (1);  /* infinite loop */
}

/* handle SVCall exceptions
 */
void SVC_Handler(void)
{
}

/* handle Debug Monitor exceptions
 */
void DebugMon_Handler(void)
{
}

/* handle PendSVC exceptions
 */
void PendSV_Handler(void)
{
}

/* SysTick Handler.
 */
void  SysTick_Handler (void)
{
    /* decrement the delay counter */
    if (TimingDelay)
        TimingDelay--;

    if (devStatus == DEV_STATUS_RUN)
        runChain = 1;

    // delay timer
    if (toDelay)
        toDelay--;

}


/***********************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers            */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP),   */
/*  for the available peripheral interrupt handler's name please refer */
/*  to the startup  file (startup_stm32f4xx.s).                        */
/***********************************************************************/


#define  TIM3_INT_CLR_MASK     0xE1A0    // mask to clar all pending TIM3 interrupts
void  TIM3_IRQHandler (void)
{
}



/* handle DMA interrupts
 */
void  DMA2_Stream1_IRQHandler (void)
{
}



void  ADC_IRQHandler (void)
{
}


/* handler for the USART3 interrupt (GPS receiver)
 * a new sentence is only started (and characters stored), if the start-character is correct (NMEA_SENTENCE_ID);
 * if the terminator charactor is received, it is checked if the sentence is of the expected type (GPGGA);
 * if the type is correct AND the 'eval finished' flag 'eval_status' is set, the rcv_status is set accordingly;
 * if the 'eval finished' input flag is not set, the current buffer content is invalidated and reused immediately;
 */
void  USART3_IRQHandler(void)
{
#if 0
    volatile uint32_t  databyte;

    if (USART_GetITStatus (USART3, USART_IT_RXNE) != RESET)
        databyte = USART3->DR;       /* Empfangsbyte lesen; loescht das Interrupt flag */

  #ifdef FAST_ERR_INT_HANDLING
    if (USART3->SR & UART_ERR_INT_MASK)
    {
        databyte    = USART3->SR;
        USART3->SR &= ~UART_ERR_INT_MASK;
        databyte    = USART3->DR;
    }
  #else
    /* Empfangszaehler inkrementieren, Fehlerflag setzen, Fehler-Interruptflag loeschen */
    if ((USART_GetITStatus (USART3, USART_IT_ORE) != RESET) || (USART_GetITStatus (USART3, USART_IT_NE) != RESET))
    {
        databyte = USART3->SR;
        databyte = USART3->DR;
    }

    /* Empfangszaehler inkrementieren, Fehlerflag setzen, Fehler-Interruptflag loeschen */
    if ((USART_GetITStatus (USART3, USART_IT_FE) != RESET) || (USART_GetITStatus (USART3, USART_IT_PE) != RESET))
    {
        databyte = USART3->SR;
        databyte = USART3->DR;
    }
  #endif
#endif
}



/* handler for the USART6 interrupt (console)
 */
void  USART6_IRQHandler(void)
{
#if 0
    volatile uint32_t  databyte;

    /* Datenbyte aus dem Empfangsregister lesen;
     * hoeherwertige Bits liefern 0 (Referenzmanual); loescht das RNXE Interrupt-Flag */
    if (USART_GetITStatus (USART6, USART_IT_RXNE) != RESET)
    {
        databyte = USART6->DR;
        szConBuffer[ConBufCount++] = (uint8_t) databyte;
        if (ConBufCount >= CON_BUFSIZE)
            ConBufCount = 0;  /* wieder von vorn beginnen ... */
    }

    /* Empfangszaehler inkrementieren, Fehlerflag setzen, Fehler-Interruptflag loeschen */
    else if ((USART_GetITStatus (USART6, USART_IT_ORE) != RESET) || (USART_GetITStatus (USART6, USART_IT_NE) != RESET))
    {
        databyte = USART6->SR;
        databyte = USART6->DR;
    }

    /* Empfangszaehler inkrementieren, Fehlerflag setzen, Fehler-Interruptflag loeschen */
    else if ((USART_GetITStatus (USART6, USART_IT_FE) != RESET) || (USART_GetITStatus (USART6, USART_IT_PE) != RESET))
    {
        databyte = USART6->SR;
        databyte = USART6->DR;
    }
#endif
}
