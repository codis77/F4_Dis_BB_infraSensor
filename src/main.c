/*********************************************************************
*
* File :         main.c
*
*        infraSensor
*
* An application to collect and display infrasound data via the
* BMP280 sensor, store them, and display them on a connected LCD.
* The application is based upon the STM32F4 Discovery board with
* the baseboard and the LCD extension, and the KY051 sensor module, 
* which contains a Bosch BMP280 pressure & temperature sensor.
* 
* The sensor is interfaced via SPI, and configured for a 16-bit
* high-speed pressure readout, without any additional compensation
* or calibration. The sensor sampling rate is set to 150Hz.
*
* Data are stored on an inserted SD card (if inserted), and also
* displayed on the attached LCD display.
*
**********************************************************************
*
* pin usage to connect the BMP280 module:
* ---------------------------------------
#  SCK        PB.13
#  MOSI       PB.15
#  MISO       PB.14
#  SS         PB.12
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx.h"
#include "main.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_lcd.h"
#include "bmp280.h"
#include "hal_spi.h"

/* external variables ---------------------------*/

/* variables ------------------------------------*/
RCC_ClocksTypeDef     RCC_Clocks;
volatile uint32_t     toDelay             = 0;   /* Timeout Delay, in ms */
volatile uint32_t     runChain            = 0;
volatile uint32_t     txTimer             = 0;
uint16_t              calValue            = 0;   /* calibration value */
uint8_t               sysMode             = 0;   /* system state      */
uint8_t               btnState            = 0;
uint16_t              buffer0[DB_SIZE]    = {0};
uint16_t              buffer1[DB_SIZE]    = {0};

static uint8_t        msgBuffer[MSG_SIZE] = {0};
volatile uint8_t      devStatus           = DEV_STATUS_NONE;

volatile uint32_t     TimingDelay         = 0;
static uint16_t       btCount             = 0;  // received button press counter
static uint16_t       btLastState         = 0;  // button press flag

static const int8_t   DbgMsg[]            = "Infrasound sensing Application V1.0";
static const int8_t   AtMsg[]             = "< @f.m.  04 / 2024 >";
static const int8_t   RxMsg[]             = "system in receive mode !";
static const int8_t   CalMsg[]            = "calibrate system ...";

/* function prototypes --------------------------
 */
static void      initF4LEDsButtons   (void);
static void      sysdelay            (uint32_t delaytime);

static void      eLoop               (void);
void             tdelay              (uint16_t ticks);
void             putItem             (uint16_t);
void             writeItem           (void);
void             writeBuffer         (uint8_t *str, uint8_t size);
static uint16_t  getCalibrationValue (uint16_t *pBuffer, uint16_t items);



/* -------- main() --------
 */
int  main (void)
{
    int       i;
    uint16_t  data;
    uint8_t   len, ret;
    char     *pm;

    i = 0;
    RCC_GetClocksFreq (&RCC_Clocks);
    SysTick_Config (RCC_Clocks.HCLK_Frequency / 100);  /* 100 => <10ms-tick> */

    /* init Discovery LEDs and user button */
    initF4LEDsButtons ();
    STM_EVAL_LEDOn (LED4);    /// green LED, main init done

    if (STM_EVAL_PBGetState (BUTTON_USER))
        sysMode = DEV_STATUS_CALIBRATE;

    delay (50);
    STM32f4_Discovery_LCD_Init();
    LCD_Clear (LCD_COLOR_BLACK);
    LCD_SetBackColor (LCD_COLOR_BLACK);
    LCD_SetTextColor (LCD_COLOR_WHITE);

    LCD_DisplayStringLine (LINE(HEADER_LINE), (uint8_t *) DbgMsg);
    LCD_DisplayStringLine (LINE(CUR_POS_LINE), (uint8_t *) AtMsg);

    // setup SPI for the sensor
    setup_spi ();

    // init the sensor
    ret = initSensor (BMP280_CONFIG_MODE_0);
    if (ret != BMP280_ID)
    {
        sprintf ((char *) msgBuffer, "sensor init failure (ID read) !\n");
        LCD_DisplayStringLine (LINE(ERR_MSG_LINE), msgBuffer);
        devStatus = DEV_STATUS_ERROR;
        eLoop ();
    }

#ifdef _HW_TEST_
    ret = getReg (REG_STATUS);
    printf ("STAT = 0x%02x\n", ret);
    ret = getReg (REG_CTRL);
    printf ("CTRL = 0x%02x\n", ret);
    ret = getReg (REG_CONFIG);
    printf ("CFG  = 0x%02x\n", ret);
#endif

    devStatus = DEV_STATUS_RUN;

    if (sysMode == DEV_STATUS_CALIBRATE)
    {
        pm = (char *) CalMsg;
        LCD_DisplayStringLine (LINE(SYSMOD_LINE), (uint8_t *) pm);
    }

    ///> main loop; read pressure value regularly
    do
    {
        if (runChain)
        {
            runChain = 0;
            STM_EVAL_LEDOn (LED6);    /// blue LED, Tx ongoing
            data = readPSensor ();
            putItem (data);
            STM_EVAL_LEDOff (LED6);   // blue LED off
        }

    }
    while (1);
}



/* init the measurement config inputs, and the Discovery LEDs
 */
static void  initF4LEDsButtons (void)
{
#ifndef _DARK_MODE_
    STM_EVAL_LEDInit (LED3);    ///> orange
    STM_EVAL_LEDInit (LED4);    ///> green
    STM_EVAL_LEDInit (LED5);    ///> red
    STM_EVAL_LEDInit (LED6);    ///> blue
#else
  #warning "dark mode active, no LEDs are driven !"
#endif
    STM_EVAL_PBInit  (BUTTON_USER, BUTTON_MODE_GPIO);
}



static void  sysdelay (uint32_t delaytime)
{
    TimingDelay = delaytime / 10;  // 10ms granularity
    while (TimingDelay);            // decremented in Systick interrupt
}



// display debug status information
static void  putLcdDbgLine (void)
{
    char  dBuf[24] = { 0 };

    sprintf (dBuf, "some data...");
    LCD_DisplayStringLine (LINE(CUR_POS_LINE), (uint8_t *) dBuf);
}



/* a simpler delay function version, based on ticks instead of milliseconds;
 */
void  tdelay (uint16_t ticks)
{
    toDelay = ticks;
    while (toDelay > 0);
}


/* prepare a data item for storage, and initiate the write;
 * 
 */
void  writeItem (void)
{
#if 0
    int       len;
    uint16_t  data;

    data = *bufPtr++;
    currentBufIndex++;

    /* double buffer management */
    if (currentBufIndex >= DB_SIZE)
    {
        currentBufIndex = 0;
        if (txBuffer == 0)
        {
            bufPtr   = buffer1;
            txBuffer = 1;
        }
        else
        {
            bufPtr   = buffer0;
            txBuffer = 0;
        }
    }

    /* create tx string, and init transmission */
    len = sprintf ((char *)sBuffer, "%hX\n", data);
    writeBuffer ((uint8_t *) sBuffer, len);
#endif
}



static uint8_t  initLatency = 1;

/* put sample item into the double-buffer;
 * consequently, trigger the next states of the sample event chain:
 * either calculate the calibration data and transmit them,
 * or send the next data item once one buffer is full;
 */
void  putItem (uint16_t data)
{
#if 0
    uint8_t  bufFull = 0;

    *smpPtr++ = data;
    currentSmpIndex++;

    /* double buffer management */
    if (currentSmpIndex > DB_SIZE)
    {
        currentSmpIndex = 0;
        if (SmplBuffer == 0)
        {
            smpPtr      = buffer1;
            SmplBuffer  = 1;
            bufFull     = 1;
            initLatency = 0;  // reset the delay marker
        }
        else
        {
            smpPtr     = buffer0;
            SmplBuffer = 0;
        }
    }
    if (sysMode == DEV_STATUS_CALIBRATE)
    {
        if (bufFull)
        {
            int       len;

            sysMode  = DEV_STATUS_RUN;
            calValue = getCalibrationValue (buffer0, DB_SIZE);
            /* create tx string, and init transmission */
            len = sprintf ((char *)sBuffer, "#XC=%hd\n", data);
            writeBuffer ((uint8_t *) sBuffer, len);
        }
    }
    else  // run mode, interleave data sampling with transmission
        writeItem();
#endif
}


/* initiate the write of a string to the SD card file
 */
void  writeBuffer (uint8_t *str, uint8_t size)
{
#if 0
//  LL_USART_EnableIT_TXE (USART1);
    USART1->CR1 |= USART_CR1_TXEIE_TXFNFIE;  // enable TXE interrupt
    sBufIndex    = 1;                        // set index for first interrupt
    sBufChars    = size - 1;                 // and string length
    USART1->TDR  = str[0];                   // push first character
#endif
}


/* endless error loop;
 * cannot init sensor; blink LED
 */
static void  eLoop (void)
{
    while (1)
    {
        STM_EVAL_LEDOn (LED5);    /// red LED
        sysdelay (150);
        STM_EVAL_LEDOff (LED5);
        sysdelay (850);
    }
}


/*************************** End of file ****************************/
