
/* SD Card related code / definitions
 */

#include "ff.h"

/* ---- interface functions ----
 */
uint32_t  openDataFile        (void);
uint32_t  getNextFileID       (void);
uint32_t  putHeader           (FIL *pFile);
uint32_t  openOutputFile      (uint32_t curID, FIL *pFile);
uint32_t  putDataItem         (uint16_t data, FIL *pFile);
