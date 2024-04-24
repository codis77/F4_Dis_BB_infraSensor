
/* version number;
 */
#define SW_VERSION_MAJOR        0
#define SW_VERSION_MINOR        3

// enable serial data output (UART 1 / PC4 + PC5, 115200 baud)
#define _SERIAL_OUTPUT_

/* USART Communication boards Interface */
#define ABS(x)                  (x < 0) ? (-x) : x

#define USARTx                  USART1
#define USARTx_CLK              RCC_APB2Periph_USART1

#define USARTx_APBPERIPHCLOCK   RCC_APB2PeriphClockCmd
#define USARTx_IRQn             USART1_IRQn
#define USARTx_IRQHandler       USART1_IRQHandler

#define USARTx_TX_PIN           GPIO_Pin_4
#define USARTx_TX_GPIO_PORT     GPIOC
#define USARTx_TX_GPIO_CLK      RCC_AHBPeriph_GPIOC
#define USARTx_TX_SOURCE        GPIO_PinSource4
#define USARTx_TX_AF            GPIO_AF_7

#define USARTx_RX_PIN           GPIO_Pin_5
#define USARTx_RX_GPIO_PORT     GPIOC
#define USARTx_RX_GPIO_CLK      RCC_AHBPeriph_GPIOC
#define USARTx_RX_SOURCE        GPIO_PinSource5
#define USARTx_RX_AF            GPIO_AF_7

#define TX_BUF_SIZE             80
#define RX_BUF_SIZE             32
#define TX_IDX_BTSTATE          4    //> data index into Tx/Rx buffer
#define RX_TIMEOUT              11   //> >100ms receive timeout

/* --------  (system mode)  -------- */
#define DEV_STATUS_NONE         0x00
#define DEV_STATUS_INIT         0x01
#define DEV_STATUS_CALIBRATE    0x02
#define DEV_STATUS_RUN          0x03
#define DEV_STATUS_ERROR        0x80
#define BUFFER_0                0       // transmit definitions ...
#define BUFFER_1                1
#define DB_SIZE                 32
#define CAL_ITEMS               32
#define MSG_SIZE                48      // display message buffer size
#define WR_LSIZE                8       // size of a data file line
#define FSYNC_SIZE              64      // fwrite operations before sync

#define DATA_FILENAME_BASE      "APsmpl"
#define DATA_FILENAME_EXT       ".dat"
#define MAX_FILE_ID_NUM         100

/* ------------------ graphics/display settings  ------------------ */
#define X_RESOLUTION            320
#define Y_RESOLUTION            240
#define X_HALF_RES              (X_RESOLUTION/2)
#define Y_HALF_RES              (Y_RESOLUTION/2)
#define HEADER_POS_X            0               // header (logo) position on display -> x
#define HEADER_POS_Y            0               // header (logo) position on display -> y
#define HEADER_LINE             0
#define SYSMOD_LINE             3               // system mode display line
#define ERR_MSG_LINE            12              // error message display line
#define CUR_POS_X               0               // current position on display -> x
#define CUR_POS_Y               295             // current position on display -> y
#define CUR_POS_LINE            29
#define CUR_HEIGHT_X            0               // current height on display -> x
#define CUR_HEIGHT_Y            312             // current height on display -> y

/* a function used by the LCD code
 */
void  delay (uint32_t  count);

// #define _HW_TEST_
