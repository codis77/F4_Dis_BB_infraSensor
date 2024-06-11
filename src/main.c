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
*  SPI      F407 GPIO    KY052
* ---------------------------------------
#  SCK        PB.13      (SCL)
#  MOSI       PB.15      (SDA)
#  MISO       PB.14      (SDO)
#  SS         PB.12      (CSB)
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
#include "ff.h"

#define _HW_TEST_

/* external variables ---------------------------*/

/* variables ------------------------------------*/
RCC_ClocksTypeDef     RCC_Clocks;
volatile uint32_t     toDelay             = 0;   /* Timeout Delay, in ms    */
volatile uint16_t     currentAPvalue      = 0;   /* current pressure sample */
volatile uint32_t     runChain            = 0;
volatile uint32_t     txTimer             = 0;
uint16_t              calValue            = 0;   /* calibration value      */
uint16_t              serialActive        = 0;   /* activate serial output */
uint8_t               sysMode             = 0;   /* system state           */
uint8_t               btnState            = 0;
#if 0  // double-buffer method not used ...
  uint16_t            buffer0[DB_SIZE]    = {0};
  uint16_t            buffer1[DB_SIZE]    = {0};
  volatile uint16_t  *bufPtr              = buffer0;  /* transmit buffer pointer */
  volatile uint16_t  *smpPtr              = buffer0;  /* sample buffer pointer   */
  uint8_t             currentBufIndex     = 0;
  uint8_t             currentSmpIndex     = 0;
#endif
char                  sBuffer[32]         = {0};  // string buffer for some UART operations
uint8_t               txIndex             = 0;    // associated index
uint8_t               SmplBuffer          = 0;

static uint8_t        msgBuffer[MSG_SIZE] = {0};
volatile uint8_t      devStatus           = DEV_STATUS_NONE;


volatile uint32_t     TimingDelay         = 0;
static uint16_t       btCount             = 0;  // received button press counter
static uint16_t       btLastState         = 0;  // button press flag

static const int8_t   DbgMsg[]            = "Infrasound sensing Application V1.0";
static const int8_t   AtMsg[]             = "< @f.m.  04 / 2024 >";
static const int8_t   RxMsg[]             = "system in receive mode !";
static const int8_t   CalMsg[]            = "calibrate system ...";

/// --- SD card related ---
static FATFS          fatfs;
static int32_t        fileState = 0;
static uint32_t       FileID    = 0;
static FIL            file;

/// --- graphics related ---
#define GFX_AVGBUF_SIZE             8   // buffer for Gfx averaging
#define GFX_AVG                     3   // number of data items per gfx point
static uint32_t       dataCount   = 0;
static uint16_t       gfxCalValue = 0;
static uint16_t       fgColor     = GFX_COLOR_TEXT;
static uint16_t       bgColor     = GFX_COLOR_BACKGOUND;
static uint16_t       curGX       = 0;
static uint16_t       avBuffer[GFX_AVGBUF_SIZE];
static uint16_t       avIndex     = 0;


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

static uint32_t  openDataFile        (void);
static uint32_t  getNextFileID       (void);
static uint32_t  putHeader           (FIL *pFile);
static uint32_t  openOutputFile      (uint32_t curID, FIL *pFile);
static uint32_t  putDataItem         (uint16_t data, FIL *pFile);

static void      initGfx             (void);
static void      gfxUpdate           (uint16_t data);

static void      initUART6           (void);
static void      sendDataItem        (uint16_t data);
static void      sendHeader          (void);


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
    SysTick_Config (RCC_Clocks.HCLK_Frequency / 150);  /* 100 => <10ms-tick> */

    /* init Discovery LEDs and user button */
    initF4LEDsButtons ();
    STM_EVAL_LEDOn (LED4);    /// green LED, main init done

    // initialize serial output; possibly used
    initUART6 ();

    // check user button press at startup
    if (STM_EVAL_PBGetState (BUTTON_USER))
#ifdef _AUTO_CALIBRATION_
        serialActive = 1;
#else
        sysMode = DEV_STATUS_CALIBRATE;
#endif

    // init the LCD display; wait 15 ticks (100ms) to settle before
    tdelay (15);
    STM32f4_Discovery_LCD_Init();
    LCD_Clear (LCD_COLOR_BLACK);
    LCD_SetBackColor (bgColor);
    LCD_SetTextColor (fgColor);

    LCD_DisplayStringLine (LINE(HEADER_LINE), (uint8_t *) DbgMsg);
    LCD_DisplayStringLine (LINE(CUR_POS_LINE), (uint8_t *) AtMsg);

    // setup SPI for the sensor
    setup_spi ();

    // init the sensor; failure is application-critical
    ret = initSensor (BMP280_CONFIG_MODE_0);
    if (ret != BMP280_ID)
    {
        sprintf ((char *) msgBuffer, "sensor init failure (ID read) !");
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

    // open SD card file; don't choke on errors
    if (openDataFile () != 0)
    {
        sprintf ((char *) msgBuffer, "SD card file failure; no storage !");
        LCD_DisplayStringLine (LINE(ERR_MSG_LINE), msgBuffer);
        serialActive = 1;
    }

#ifdef _AUTO_CALIBRATION_
    sysMode = DEV_STATUS_CALIBRATE;
#endif

    devStatus = DEV_STATUS_RUN;

    if (sysMode == DEV_STATUS_CALIBRATE)
    {
        pm = (char *) CalMsg;
        LCD_DisplayStringLine (LINE(SYSMOD_LINE), (uint8_t *) pm);
    }

    // init data display graphics
    initGfx ();

    if (serialActive == 1)
        sendHeader ();

    ///> main loop; read pressure value regularly
    do
    {
        if (runChain)
        {
            STM_EVAL_LEDOn (LED6);    // blue LED on
            data = currentAPvalue;    // get value from interrupt handler
            runChain = 0;
            putItem (data);
            gfxUpdate (data);
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


/* a delay function the Disco LCD code uses
 */
void  delay (uint32_t  count)
{
    tdelay ((uint16_t) count);
}


/* process the sample item;
 * consequently, save it to file in run mode;
 * in calibration mode, just evaluate the calibration value
 */
void  putItem (uint16_t data)
{
    static uint32_t  avg     = 0;
    static uint32_t  avcount = 0;

    if (sysMode == DEV_STATUS_CALIBRATE)
    {
        avg += data;
        avcount++;

        if (avcount >= CAL_ITEMS)
        {
            sysMode  = DEV_STATUS_RUN;
            calValue = avg / CAL_ITEMS;
        }
    }
    else
    {
        (void) putDataItem (data, &file);
        if (serialActive)
            sendDataItem (data);
    }
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



/* **************************************************************
 * ******************** SD card related code ********************
 */

static char   tBuffer[80] = {0};  // string buffer for some file operations

/* open the SD card file for writing the sample data;
 * use a fixed file name base with a running 2-digit number;
 * if no free name is found, force "1" as running number;
 * return 0 if everything went well;
 * return 1 upon error
 */
static uint32_t   openDataFile (void)
{
    uint32_t  i;

    if (fileState == 0)
    {
        if (f_mount (0, &fatfs) != FR_OK)
            fileState = -1;
        fileState = 1;
        FileID = getNextFileID ();
        i      = openOutputFile (FileID, &file);
        putHeader (&file);
    }

    if (fileState == -1)
        return (1);
    else
        return 0;
}



/* find the next name that does not yet exist as file on the SD card;
 * if all file names are used, '1' is returned (i.e. older files are overwritten)
 */
static uint32_t  getNextFileID (void)
{
    uint32_t  curID, found, r;
    FIL       F1;

    curID = 1;
    found = 0;
    while (!found)
    {
        sprintf (tBuffer, "%s%02d%s", DATA_FILENAME_BASE, (int) curID, DATA_FILENAME_EXT);
        r = f_open (&F1, tBuffer, FA_READ);
        /* if already exists, close and try next */
        if (r == FR_OK)
        {
            curID++;
            f_close (&F1);
        }
        else
            found = 1;
        /* if there are too much files, force use of ID '1' */
        if ((!found) && (curID > MAX_FILE_ID_NUM))
        {
            curID = 1;
            found = 1;
        }
    }

    return (curID);
}



/* open the output file, including an ID number in the file name;
 * the ID number is given as argument;
 * return value is that of the called f_open() function
 */
static uint32_t  openOutputFile (uint32_t curID, FIL *pFile)
{
    sprintf (tBuffer, "%s%02d%s", DATA_FILENAME_BASE, (int) curID, DATA_FILENAME_EXT);
    return (f_open (pFile, (const char *) tBuffer, FA_WRITE));
}



/* write header information to the output (SD card file);
 * parameter is the current sensitivity value;
 * return value is a success/error message from the file system
 */
static uint32_t  putHeader (FIL *pFile)
{
    uint32_t  bCnt, ret = 0;

    sprintf (tBuffer, "#! -Air Pressure / Infrasound Logger V%d.%d (c)fm ---\n# @150\n", SW_VERSION_MAJOR, SW_VERSION_MINOR);
    ret  = f_write (pFile, tBuffer, strlen(tBuffer), (UINT *) &bCnt);
    if (ret == 0)
        f_sync (pFile);

    return (ret);
}



/* write a data item to the output (SD card file);
 * parameter is the current data value and the file pointer;
 * return value is a success/error message from the file system
 * (0 = o.k; 1..n = error)
 */

static uint32_t  putDataItem (uint16_t data, FIL *pFile)
{
    uint32_t         ret, bCnt = 0;
    char             lbuf[16];
    static uint16_t  wcount = 0;

    sprintf (lbuf,"%hX\n", data);
    ret = f_write (pFile, lbuf, strlen(lbuf), (UINT *) &bCnt);
    if (ret != 0)
        return (ret);

    // do a file sync once in a while ...
    wcount++;
    if (wcount >= FSYNC_SIZE)
    {
        f_sync (pFile);
        wcount = 0;
    }
    return 0;
}



/* initialize the transmission of a data item via serial line;
 * function is executed in "real time", and supposedly finishes
 * long before the 6.6ms sample cycle (ca. 10Byte/ms throughput);
 * otherwise Tx data could overlap / be corrupted;
 */
static void  sendDataItem (uint16_t data)
{
    sprintf (sBuffer, "%d\n", data);

    // initialize UART TXE interrupt, send first character
    USART_ITConfig (USART6, USART_IT_TXE, ENABLE);
    txIndex    = 1;
    USART6->DR = sBuffer[0];
}



/* initialize internal graphics-related variables,
 * and draw the diagram frame
 */
static void  initGfx (void)
{
    uint32_t  i;

    dataCount   = 0;
    gfxCalValue = 0;
    avIndex     = 0;
    for (i=0; i<GFX_AVGBUF_SIZE; i++)
        avBuffer[i] = 0;

    // draw the diagram frame
    fgColor = AXIS_COLOR;
    LCD_SetColors (fgColor, bgColor);
    LCD_DrawLine (X_AXIS_START, Y_AXIS_MID, X_AXIS_END - X_AXIS_START, LCD_DIR_HORIZONTAL);
    LCD_DrawLine (X_AXIS_START, Y_AXIS_HIGH, Y_AXIS_LOW - Y_AXIS_HIGH, LCD_DIR_VERTICAL);
}



/* update the graphics display
 */
static void  gfxUpdate (uint16_t data)
{
    uint32_t  dg;
    int16_t   y;

    // set calibration value upon first data item
    if (dataCount == 0)
    {
        if (calValue == 0)  // no calibration, just use first value ...
            gfxCalValue = data;
        else
            gfxCalValue = calValue;
    }
    dataCount++;

    // average over <n> data
    avBuffer[avIndex++] = data;
    if (avIndex < GFX_AVG)
        return;
    avIndex = 0;
    for (dg=0, y=0; y<GFX_AVG; y++)
        dg += avBuffer[y];
    dg /= GFX_AVG;

    // draw the data
    // reset x index on last item of cycle
    if (((dataCount+1) % GFX_CYCLE) == 0)
        curGX = 0;

    // set x drawing index
    curGX++;

    // overpaint previous cursor/data line
    LCD_SetColors (GFX_COLOR_BACKGOUND, bgColor);
    LCD_DrawLine (X_AXIS_START + curGX, Y_AXIS_HIGH, Y_AXIS_LOW - Y_AXIS_HIGH, LCD_DIR_VERTICAL);

    // draw data
    y = Y_AXIS_MID - dg;  // perhaps add some adaptive scaling here later ...
    if (y < Y_AXIS_HIGH)
        y = Y_AXIS_HIGH;
    else if (y > Y_AXIS_LOW)
        y = Y_AXIS_LOW;
    fgColor = DATA_COLOR;
    LCD_SetColors (fgColor, bgColor);
    LCD_DrawLine (X_AXIS_START + curGX, Y_AXIS_MID, y, LCD_DIR_VERTICAL);

    // new cursor line (unless for the very last)
    if (curGX >= GFX_CYCLE)
        return;
    fgColor = CURSOR_COLOR;
    LCD_SetColors (fgColor, bgColor);
    LCD_DrawLine (X_AXIS_START + curGX + 1, Y_AXIS_MID - GFX_CURSOR_SIZE, 2 * GFX_CURSOR_SIZE, LCD_DIR_VERTICAL);
}



/* Initialize the USART6 (115200,8,n,1,none) for the console;
 * as used on the STM32F4DIS_BB board:
 *   PC6  =>  usart6.TX
 *   PC7  =>  usart6.RX
 */
static void  initUART6 (void)
{
    GPIO_InitTypeDef   GPIO_InitStructure;
    USART_InitTypeDef  USART_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOC, ENABLE);      // configure clock for GPIO
    RCC_APB2PeriphClockCmd (RCC_APB2Periph_USART6, ENABLE);     // configure clock for USART

    GPIO_PinAFConfig (GPIOC, GPIO_PinSource6, GPIO_AF_USART6);  // configure AF
    GPIO_PinAFConfig (GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

    GPIO_StructInit (&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init (GPIOC, &GPIO_InitStructure);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init (GPIOC, &GPIO_InitStructure);

    // configure UART
    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init (USART6, &USART_InitStructure);
    USART_Cmd (USART6, ENABLE);

    /* parametrize USART6-Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel            = USART6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd         = ENABLE;
    NVIC_Init (&NVIC_InitStructure);

    /* Interrupt at reception and receive errors */
#if 0
    USART_ITConfig (USART6, USART_IT_RXNE, ENABLE);
    USART_ITConfig (USART6, USART_IT_ERR, ENABLE);
#endif
    /* transmit interrupt enabled on demand */
}



static void  sendHeader (void)
{
    int  sl, index;

    sl = index = 0;

    sl = sprintf (sBuffer, "#infra_%d @150\n", PROTOCOL_VERSION);
    if (sl <= 0)  // an unlikely sprintf() error
        return;

    // send string in a busy loop
    while (index < sl)
    {
        USART6->DR = sBuffer[index++];
        while ((USART6->SR & USART_FLAG_TXE)==0);
    }
}


/*************************** End of file ****************************/
