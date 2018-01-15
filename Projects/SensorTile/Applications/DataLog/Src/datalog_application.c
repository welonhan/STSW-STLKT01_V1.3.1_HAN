/**
  ******************************************************************************
  * @file    DataLog/Src/datalog_application.c
  * @author  Central Labs
  * @version V2.0.0
  * @date    27-April-2017
  * @brief   This file provides a set of functions to handle the datalog
  *          application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "datalog_application.h"
#include "main.h"
#include "usbd_cdc_interface.h"
#include "string.h"
#include "SensorTile.h"
#include <math.h>
    
/* FatFs includes component */
#include "ff_gen_drv.h"
#include "sd_diskio.h"

FRESULT res;                                          /* FatFs function common result code */
uint32_t byteswritten, bytesread;                     /* File write/read counts */
FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */
char SDPath[4]; /* SD card logical drive path */
    
volatile uint8_t SD_Log_Enabled = 0;

char newLine[] = "\r\n";

extern void *LSM6DSM_X_0_handle;
extern void *LSM6DSM_G_0_handle;
extern void *LSM303AGR_M_0_handle;
extern void *LSM303AGR_X_0_handle;
extern void *LPS22HB_P_0_handle;
extern void *LPS22HB_T_0_handle;
extern void *HTS221_H_0_handle; 
extern void *HTS221_T_0_handle;

extern volatile uint8_t no_H_HTS221;
extern volatile uint8_t no_T_HTS221;

/**
  * @brief  Start SD-Card demo
  * @param  None
  * @retval None
  */
void DATALOG_SD_Init(void)
{
  if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
  {
    /* Register the file system object to the FatFs module */
    if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)
    {
      /* FatFs Initialization Error */
      while(1)
      {
        BSP_LED_On(LED1);
        HAL_Delay(500);
        BSP_LED_Off(LED1);
        HAL_Delay(100);
      }
    }
  }
}
  
/**
  * @brief  Start SD-Card demo
  * @param  None
  * @retval None
  */
uint8_t DATALOG_SD_Log_Enable(void)
{
  static uint16_t sdcard_file_counter = 0;
  char header[] = "T [ms],AccX [mg],AccY [mg],AccZ [mg],GyroX [mdps],GyroY [mdps],GyroZ [mdps],MagX [mgauss],MagY [mgauss],MagZ [mgauss],P [mB],T [°C],H [%]\r\n";
  uint32_t byteswritten; /* written byte count */
  char file_name[30] = {0};
  
  /* SD SPI CS Config */
  SD_IO_CS_Init();
  
  sprintf(file_name, "%s%.3d%s", "SensorTile_Log_N", sdcard_file_counter, ".csv");
  sdcard_file_counter++;

  HAL_Delay(100);

  if(f_open(&MyFile, (char const*)file_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
  {
    sdcard_file_counter--;
    return 0;
  }
  
  if(f_write(&MyFile, (const void*)&header, sizeof(header)-1, (void *)&byteswritten) != FR_OK)
  {
    return 0;
  }
  return 1;
}

uint8_t DATALOG_SD_writeBuf(char *s, uint32_t size)
{
  uint32_t byteswritten;
  
  if(f_write(&MyFile, s, size, (void *)&byteswritten) != FR_OK)
  {
    return 0;
  }
  return 1;
}


/**
  * @brief  Disable SDCard Log
  * @param  None
  * @retval None
  */
void DATALOG_SD_Log_Disable(void)
{
  f_close(&MyFile);
  
  /* SD SPI Config */
  SD_IO_CS_DeInit();
}

void DATALOG_SD_DeInit(void)
{
  FATFS_UnLinkDriver(SDPath);
}

/**
  * @brief  Write New Line to file
  * @param  None
  * @retval None
  */
void DATALOG_SD_NewLine(void)
{
  uint32_t byteswritten; /* written byte count */
  f_write(&MyFile, (const void*)&newLine, 2, (void *)&byteswritten);
}


DrvStatusTypeDef getSensorsData( T_SensorsData *mptr)
{
  DrvStatusTypeDef ret = COMPONENT_OK;
 
  mptr->ms_counter = HAL_GetTick();
  
  if ( BSP_ACCELERO_Get_Axes( LSM303AGR_X_0_handle, &mptr->acc ) == COMPONENT_ERROR )
  {
    mptr->acc.AXIS_X = 0;
    mptr->acc.AXIS_Y = 0;
    mptr->acc.AXIS_Z = 0;
    ret = COMPONENT_ERROR;
  }
  
  if ( BSP_GYRO_Get_Axes( LSM6DSM_G_0_handle, &mptr->gyro ) == COMPONENT_ERROR )
  {
    mptr->gyro.AXIS_X = 0;
    mptr->gyro.AXIS_Y = 0;
    mptr->gyro.AXIS_Z = 0;
    ret = COMPONENT_ERROR;
  }
  
  if ( BSP_MAGNETO_Get_Axes( LSM303AGR_M_0_handle, &mptr->mag ) == COMPONENT_ERROR )
  {
    mptr->mag.AXIS_X = 0;
    mptr->mag.AXIS_Y = 0;
    mptr->mag.AXIS_Z = 0;
    ret = COMPONENT_ERROR;
  }
  
  if ( BSP_PRESSURE_Get_Press( LPS22HB_P_0_handle, &mptr->pressure ) == COMPONENT_ERROR )
  {
    mptr->pressure = 0.0f;
    ret = COMPONENT_ERROR;
  }

  if(!no_T_HTS221)
  {
    if ( BSP_TEMPERATURE_Get_Temp( HTS221_T_0_handle, &mptr->temperature ) == COMPONENT_ERROR )
    {
      mptr->temperature = 0.0f;
      ret = COMPONENT_ERROR;
    }
  }
  
  if(!no_H_HTS221)
  {
    if ( BSP_HUMIDITY_Get_Hum( HTS221_H_0_handle, &mptr->humidity ) == COMPONENT_ERROR )
    {
      mptr->humidity = 0.0f;
      ret = COMPONENT_ERROR;
    }
  }
  return ret;
}

/**
* @brief  Splits a float into two integer values.
* @param  in the float value as input
* @param  out_int the pointer to the integer part as output
* @param  out_dec the pointer to the decimal part as output
* @param  dec_prec the decimal precision to be used
* @retval None
*/
void floatToInt( float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec )
{
  *out_int = (int32_t)in;
  if(in >= 0.0f)
  {
    in = in - (float)(*out_int);
  }
  else
  {
    in = (float)(*out_int) - in;
  }
  *out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
