
/* ---------------- Constants ----------------
 */

/* ---- BMP280 register addresses
 */
#define REG_RESET              0xE0
#define REG_ID                 0xD0
#define REG_STATUS             0xF3
#define REG_CTRL               0xF4
#define REG_CONFIG             0xF5
#define REG_DATA_P             0xF7
#define REG_DATA_T             0xFA

#define REG_WRITE_MASK         0x7F
#define REG_READ_MASK          0x80

/* ---- BMP280 read/write values
 */
#define RESET_VAL              0xB6  // write to reset reg to facilitate a reset
#define BMP280_ID              0x58  // expected chip ID
#define MODE_0_CTRL            0x07  // skip t, sample p@1x, normal mode
#define MODE_0_CONFIG          0x00  // minimal standy time, no filter, 4-wire SPI

/* ---- BMP280 config modes
 */
#define BMP280_CONFIG_MODE_0   0x00


/* -------------- API functions --------------
 */
uint8_t    initSensor  (uint8_t mode);  // initialize sensor
uint16_t   readPSensor (void);          // read current values
