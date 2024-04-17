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
#include "ff.h"

/* external variables ---------------------------*/

/* variables ------------------------------------*/
RCC_ClocksTypeDef     RCC_Clocks;
volatile uint32_t     toDelay             = 0;   /* Timeout Delay, in ms    */
volatile uint16_t     currentAPvalue      = 0;   /* current pressure sample */
volatile uint32_t     runChain            = 0;
volatile uint32_t     txTimer             = 0;
uint16_t              calValue            = 0;   /* calibration value */
uint8_t               sysMode             = 0;   /* system state      */
uint8_t               btnState            = 0;
uint16_t              buffer0[DB_SIZE]    = {0};
uint16_t              buffer1[DB_SIZE]    = {0};
volatile uint16_t    *bufPtr              = buffer0;  /* transmit buffer pointer */
volatile uint16_t    *smpPtr              = buffer0;  /* sample buffer pointer   */
uint8_t               currentBufIndex     = 0;
uint8_t               currentSmpIndex     = 0;
uint8_t               txBuffer            = 0;
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
    }

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
            STM_EVAL_LEDOn (LED6);    // blue LED on
            data = currentAPvalue;    // get value from interrupt handler
            runChain = 0;
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
        (void) putDataItem (data, &file);
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



#warning "implement file-open and periodic write functionality"


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

    sprintf (tBuffer, "#! -Air Pressure / Infrasound Logger V%d.%d (c)fm ---\n", SW_VERSION_MAJOR, SW_VERSION_MINOR);
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



/*************************** End of file ****************************/
