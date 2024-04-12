/* 
 * low-level implementation of SPI access to the BMP280 sensor
 * for the F4 Discovery board (STM32F407VG);
 * the pins PB.12, PB.13, PB.14 and PB.15 are used
 * for the SPI interface (SPI2)
 */
#include <string.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "hal_spi.h"
#include "bmp280.h"

/**
 * SPI pins
 * STM32F4-Disco pin mappings
 */

#define BMP_SPI_SCK_PIN        GPIO_Pin_13
#define BMP_SPI_MOSI_PIN       GPIO_Pin_15
#define BMP_SPI_MISO_PIN       GPIO_Pin_14
#define BMP_SPI_SS_PIN         GPIO_Pin_12
#define BMP_SPI_GPIO_PORT      GPIOB

#define BMP_SPI2_AF            GPIO_AF_SPI2
#define BMP_SPI_SCK_SOURCE     GPIO_PinSource13
#define BMP_SPI_MISO_SOURCE    GPIO_PinSource14
#define BMP_SPI_MOSI_SOURCE    GPIO_PinSource15


///> setup
void                    delay     (uint16_t time);

///> pin functions
static void inline      setSS     (uint8_t state);
static void inline      setSCK    (uint8_t state);
static void inline      setMOSI   (uint8_t state);
static uint16_t inline  getMISO   (void);

///> SPI functions
static void             spi_init  (void);
uint8_t                 spi_send  (uint8_t tx);

///> API functions
void                    setup_spi (void);
uint8_t                 getReg    (uint8_t regAddr);
void                    writeReg  (uint8_t regAddr, uint8_t value);
uint32_t                readData  (uint8_t regAddr, uint8_t bytes);




/* --- pin setting / pin checking routines ---
 */
static void inline setSCK (uint8_t state)
{
    if (state)
        BMP_SPI_GPIO_PORT->BSRRL = BMP_SPI_SCK_PIN;
    else
        BMP_SPI_GPIO_PORT->BSRRH = BMP_SPI_SCK_PIN;
}


static void inline setMOSI (uint8_t state)
{
    if (state)
        BMP_SPI_GPIO_PORT->BSRRL = BMP_SPI_MOSI_PIN;
    else
        BMP_SPI_GPIO_PORT->BSRRH = BMP_SPI_MOSI_PIN;
}


static uint16_t  inline getMISO (void)
{
    return (BMP_SPI_GPIO_PORT->IDR & BMP_SPI_MISO_PIN);
}


/* select CC1101 by LOW SS signal
 */
void  setSS (uint8_t state)
{
    if (state == 0)
        BMP_SPI_GPIO_PORT->BSRRH = BMP_SPI_SS_PIN;
    else
        BMP_SPI_GPIO_PORT->BSRRL = BMP_SPI_SS_PIN;  // H, deselect
};




// ***********************************
// ******* SPI SPECIFIC ROUTINES
// ***********************************

void   setup_spi (void)
{
    spi_init();
}


/* SPI initialization
 */
static void  spi_init (void)
{
    GPIO_InitTypeDef  gpio_init;
    SPI_InitTypeDef   spi_init;

    /* enable SCK, MOSI and MISO GPIO clocks */
    RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd (RCC_APB1Periph_SPI2, ENABLE);

    /* SPI pin configuration */
    GPIO_PinAFConfig (BMP_SPI_GPIO_PORT, BMP_SPI_SCK_SOURCE,  BMP_SPI2_AF);
    GPIO_PinAFConfig (BMP_SPI_GPIO_PORT, BMP_SPI_MISO_SOURCE, BMP_SPI2_AF);
    GPIO_PinAFConfig (BMP_SPI_GPIO_PORT, BMP_SPI_MOSI_SOURCE, BMP_SPI2_AF);
    gpio_init.GPIO_Mode  = GPIO_Mode_AF;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd  = GPIO_PuPd_DOWN;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init.GPIO_Pin   = BMP_SPI_SCK_PIN | BMP_SPI_MOSI_PIN | BMP_SPI_MISO_PIN;
    GPIO_Init (BMP_SPI_GPIO_PORT, &gpio_init);

    /* Configure GPIO pins for chip select */
    gpio_init.GPIO_Pin   = BMP_SPI_SS_PIN;
    gpio_init.GPIO_Mode  = GPIO_Mode_OUT;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init (BMP_SPI_GPIO_PORT, &gpio_init);
    GPIO_SetBits (BMP_SPI_GPIO_PORT, BMP_SPI_SS_PIN);

    setSS (1);    /*!< deselect the device: Chip Select high */

    /* SPI peripheral configuration */
    SPI_I2S_DeInit (SPI2);
    spi_init.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    spi_init.SPI_Mode              = SPI_Mode_Master;
    spi_init.SPI_DataSize          = SPI_DataSize_8b;
    spi_init.SPI_CPOL              = SPI_CPOL_Low;
    spi_init.SPI_CPHA              = SPI_CPHA_1Edge;
    spi_init.SPI_NSS               = SPI_NSS_Soft;
    spi_init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
    spi_init.SPI_FirstBit          = SPI_FirstBit_MSB;
    spi_init.SPI_CRCPolynomial     = 0;
    SPI_Init (SPI2, &spi_init);

    SPI_Cmd (SPI2, ENABLE);    /* enable SPI2 */
}


/* send uint8_t via SPI
 * 'tx'  Value to be sent
 * Return:
 *  Response received from SPI slave
 */
uint8_t  spi_send (uint8_t tx)
{
    uint8_t  rx = 0;
    // check if we need to use a byte-pointer to DR !

    // write, and wait for the transfer to finish before read-back
    SPI2->DR = tx;
    while (!(SPI2->SR & SPI_I2S_FLAG_RXNE));
    rx = SPI2->DR;
    return (rx);
}



/* wait until SPI operation is terminated;
 * bit-banging implementation does not need a wait
 */
void  wait_Spi (void)
{
    return;
};


// ********************************************************************
// ********** BMP280 CHIP LOW LEVEL COMMUNICATION ROUTINES ************
// ********************************************************************


/* Wait until SPI MISO line goes low
 */
void  wait_Miso (void)
{
    while (getMISO ());
};



/* cc1101_writeReg
 * Write single register into the CC1101 IC via SPI
 * 'regAddr'    Register address
 * 'value'  Value to be writen
 */
void  writeReg (uint8_t regAddr, uint8_t value)
{
    setSS (0);
    regAddr &= REG_WRITE_MASK;  /* reset MSB of address  */ 
    spi_send (regAddr);
    spi_send (value);
    setSS (1);
}



/* readReg 
 * Read BMP280 register via SPI
 *  'regAddr'   Register address
 *  'regType'   Type of register: CC1101_CONFIG_REGISTER or CC1101_STATUS_REGISTER
 * Return:
 *  Data uint8_t returned by the CC1101 IC
 */
uint8_t  getReg (uint8_t regAddr)
{
    uint8_t  val;

    regAddr |= REG_READ_MASK;  /* set address MSB       */
    setSS (0);
    spi_send (regAddr);        /* send register address */
    val = spi_send (0x00);     /* read result           */
    setSS (1);

    return val;
}



uint32_t  readData  (uint8_t regAddr, uint8_t bytes)
{
    uint32_t  readval;
    uint8_t   bread;

    readval = 0;

    if ((bytes > 4) || (!bytes))
        return 0;

    setSS (0);

    spi_send (regAddr);    /* send first register address */

    while (bytes)
    {
        bread = spi_send (0);
        readval  = (readval << 8);  // make room ...
        readval += bread;           // ... and insert into 32-bit word
        bytes--;
    }

    setSS (1);
    return (readval);
}



/* ad hoc implementation of a short delay;
 * TODO: might need to check the timing !
 */
void  delay (uint16_t  delayTime)
{
    volatile int  i;

    do
    {
        for (i=0; i<16; )
            i++;
    }
    while (--delayTime);
}

