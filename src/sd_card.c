#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx.h"
#include "main.h"
#include "sd_card.h"
#include "stm32f4_discovery.h"


/// ------- data -------
extern FATFS         fatfs;
extern int32_t       fileState;
extern uint32_t      FileID;
extern FIL           file;

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
uint32_t   openDataFile (void)
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
uint32_t  getNextFileID (void)
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
uint32_t  openOutputFile (uint32_t curID, FIL *pFile)
{
    sprintf (tBuffer, "%s%02d%s", DATA_FILENAME_BASE, (int) curID, DATA_FILENAME_EXT);
    return (f_open (pFile, (const char *) tBuffer, FA_WRITE));
}



/* write header information to the output (SD card file);
 * parameter is the current sensitivity value;
 * return value is a success/error message from the file system
 */
uint32_t  putHeader (FIL *pFile)
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

uint32_t  putDataItem (uint16_t data, FIL *pFile)
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

