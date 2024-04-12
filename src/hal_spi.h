
#ifndef HAL_SPI_H
  #define HAL_SPI_H


/* ---------------- definitions ----------------
 */
#define BUFFER_SIZE            64       // UART ring buffer

#define false                  0
#define true                   (!false)

#define RET_SPI_SUCCESS        0x00
#define RET_SPI_ERR            0xFF

typedef unsigned char  bool;


/* ------------ function prototypes ------------
 */

///> setup
void      delay     (uint16_t time);

///> SPI functions
uint8_t   spi_send  (uint8_t tx);

///> API functions
void      setup_spi (void);
uint8_t   getReg    (uint8_t regAddr);
void      writeReg  (uint8_t regAddr, uint8_t value);
uint32_t  readData  (uint8_t regAddr, uint8_t bytes);

#endif  //  HAL_SPI_H
