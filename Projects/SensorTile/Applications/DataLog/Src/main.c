/**
  ******************************************************************************
  * @file    main.c
  * @author  Central Labs
  * @version V2.0.0
  * @date    27-April-2017
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "datalog_application.h"
    
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
    
#define DATAQUEUE_SIZE     ((uint32_t)100)
                                                              
typedef enum
{
  THREAD_1 = 0,
  THREAD_2
} Thread_TypeDef;
  
/* Private variables ---------------------------------------------------------*/

osThreadId GetDataThreadId, WriteDataThreadId;

osMessageQId dataQueue_id;
osMessageQDef(dataqueue, DATAQUEUE_SIZE, int);

osPoolId sensorPool_id;
osPoolDef(sensorPool, DATAQUEUE_SIZE, T_SensorsData);

osSemaphoreId readDataSem_id;
osSemaphoreDef(readDataSem);

osSemaphoreId doubleTapSem_id;
osSemaphoreDef(doubleTapSem);

/* LoggingInterface = USB_Datalog  --> Save sensors data on SDCard (enable with double click) */
/* LoggingInterface = SDCARD_Datalog  --> Send sensors data via USB */
LogInterface_TypeDef LoggingInterface = USB_Datalog;

USBD_HandleTypeDef  USBD_Device;
static volatile uint8_t MEMSInterrupt = 0;
volatile uint8_t no_H_HTS221 = 0;
volatile uint8_t no_T_HTS221 = 0;

void *LSM6DSM_X_0_handle = NULL;
void *LSM6DSM_G_0_handle = NULL;
void *LSM303AGR_X_0_handle = NULL;
void *LSM303AGR_M_0_handle = NULL;
void *LPS22HB_P_0_handle = NULL;
void *LPS22HB_T_0_handle = NULL; 
void *HTS221_H_0_handle = NULL; 
void *HTS221_T_0_handle = NULL;

/* Private function prototypes -----------------------------------------------*/
static void GetData_Thread(void const *argument);
static void WriteData_Thread(void const *argument);

static void Error_Handler( void );
static void initializeAllSensors( void );
void enableAllSensors( void );
void disableAllSensors( void );
void setOdrAllSensors( void );

void dataTimer_Callback(void const *arg);
void dataTimerStart(void);
void dataTimerStop(void);

osTimerId sensorTimId;
osTimerDef(SensorTimer, dataTimer_Callback);

uint32_t  exec;
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  HAL_Init();

  /* Configure the System clock to 80 MHz */
  SystemClock_Config();

  if(LoggingInterface == USB_Datalog)
  {
    /* Initialize LED */
    BSP_LED_Init(LED1);
    BSP_LED_Off(LED1);
  }
 
  /* enable USB power on Pwrctrl CR2 register */
  HAL_PWREx_EnableVddUSB();
  
  if(LoggingInterface == USB_Datalog) /* Configure the USB */
  {
    /*** USB CDC Configuration ***/
    /* Init Device Library */
    USBD_Init(&USBD_Device, &VCP_Desc, 0);
    /* Add Supported Class */
    USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
    /* Add Interface callbacks for AUDIO and CDC Class */
    USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
    /* Start Device Process */
    USBD_Start(&USBD_Device);
  }
  else /* Configure the SDCard */
  {
    DATALOG_SD_Init();
  }

  /* Thread 1 definition */
  osThreadDef(THREAD_1, GetData_Thread, osPriorityAboveNormal, 0, configMINIMAL_STACK_SIZE*4);
  
  /* Thread 2 definition */
  osThreadDef(THREAD_2, WriteData_Thread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE*4);
  
  /* Start thread 1 */
  GetDataThreadId = osThreadCreate(osThread(THREAD_1), NULL);

  /* Start thread 2 */
  WriteDataThreadId = osThreadCreate(osThread(THREAD_2), NULL);  
  
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  for (;;);

}

/**
  * @brief  Get data raw from sensors to queue
  * @param  thread not used
  * @retval None
  */
static void GetData_Thread(void const *argument)
{
  (void) argument;
  uint8_t doubleTap = 0;
  T_SensorsData *mptr;
  
  sensorPool_id = osPoolCreate(osPool(sensorPool));     
  dataQueue_id = osMessageCreate(osMessageQ(dataqueue), NULL);
  
  readDataSem_id = osSemaphoreCreate(osSemaphore(readDataSem), 1);
  osSemaphoreWait(readDataSem_id, osWaitForever);
  
  doubleTapSem_id = osSemaphoreCreate(osSemaphore(doubleTapSem), 1);
  osSemaphoreWait(doubleTapSem_id, osWaitForever);
  
  /* Configure and disable all the Chip Select pins */
  Sensor_IO_SPI_CS_Init_All();
  
  /* Initialize and Enable the available sensors */
  initializeAllSensors();
  enableAllSensors();
  
  if(LoggingInterface == USB_Datalog)
  {
    dataTimerStart();
  }
  
  for (;;)
  {
    osSemaphoreWait(readDataSem_id, osWaitForever);
    if(MEMSInterrupt && LoggingInterface == SDCARD_Datalog)
    {
      MEMSInterrupt = 0;
      BSP_ACCELERO_Get_Double_Tap_Detection_Status_Ext(LSM6DSM_X_0_handle, &doubleTap);
      if(doubleTap)
      {
        if(SD_Log_Enabled) 
        {
          dataTimerStop();
          osMessagePut(dataQueue_id, 0x00000007, osWaitForever);
        }
        else
        {
          osMessagePut(dataQueue_id, 0x00000007, osWaitForever);
        }
      }
    }
    else
    {
      /* Try to allocate a memory block and check if is not NULL */
      mptr = osPoolAlloc(sensorPool_id);
      if(mptr != NULL)
      {
        /* Get Data from Sensors */
        if(getSensorsData(mptr) == COMPONENT_OK)
        {
          /* Push the new memory Block in the Data Queue */
          if(osMessagePut(dataQueue_id, (uint32_t)mptr, osWaitForever) != osOK)
          {
             Error_Handler();
          }
        }
        else
        {
          Error_Handler();
        }
      }
      else
      {
        Error_Handler();
      }
    }
  }
}


/**
  * @brief  Write data in the queue on file or streaming via USB
  * @param  argument not used
  * @retval None
  */
static void WriteData_Thread(void const *argument)
{
  (void) argument;
  osEvent evt;
  T_SensorsData *rptr;
  int size;
  char data_s[256];
  
  for (;;)
  {
    evt = osMessageGet(dataQueue_id, osWaitForever);  // wait for message
    if (evt.status == osEventMessage)
    {
      if(evt.value.v == 0x00000007)
      {
        if (SD_Log_Enabled) 
        {
          DATALOG_SD_Log_Disable();
          SD_Log_Enabled=0;
        }
        else
        {
          while(SD_Log_Enabled != 1)
          {
            if(DATALOG_SD_Log_Enable())
            {
              SD_Log_Enabled=1;
              osDelay(100);
              dataTimerStart();
            }
            else
            {
              //DATALOG_SD_Log_Disable();
              DATALOG_SD_DeInit();
              DATALOG_SD_Init();
              osDelay(100);
            }
          }
        }
      }
      else
      {
        rptr = evt.value.p;

        if(LoggingInterface == USB_Datalog)
        {
          size = sprintf(data_s, "TimeStamp: %d\r\n Acc_X: %d, Acc_Y: %d, Acc_Z :%d\r\n Gyro_X:%d, Gyro_Y:%d, Gyro_Z:%d\r\n Magn_X:%d, Magn_Y:%d, Magn_Z:%d\r\n Press:%5.2f, Temp:%5.2f, Hum:%4.1f\r\n",
                       rptr->ms_counter,
                       (int)rptr->acc.AXIS_X, (int)rptr->acc.AXIS_Y, (int)rptr->acc.AXIS_Z,
                       (int)rptr->gyro.AXIS_X, (int)rptr->gyro.AXIS_Y, (int)rptr->gyro.AXIS_Z,
                       (int)rptr->mag.AXIS_X, (int)rptr->mag.AXIS_Y, (int)rptr->mag.AXIS_Z,
                       rptr->pressure, rptr->temperature, rptr->humidity);
          osPoolFree(sensorPool_id, rptr);      // free memory allocated for message
          BSP_LED_Toggle(LED1);
          CDC_Fill_Buffer(( uint8_t * )data_s, size);
        }
        else
        {
          size = sprintf(data_s, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %5.2f, %5.2f, %4.1f\r\n",
                       rptr->ms_counter,
                       (int)rptr->acc.AXIS_X, (int)rptr->acc.AXIS_Y, (int)rptr->acc.AXIS_Z,
                       (int)rptr->gyro.AXIS_X, (int)rptr->gyro.AXIS_Y, (int)rptr->gyro.AXIS_Z,
                       (int)rptr->mag.AXIS_X, (int)rptr->mag.AXIS_Y, (int)rptr->mag.AXIS_Z,
                       rptr->pressure, rptr->temperature, rptr->humidity);
          osPoolFree(sensorPool_id, rptr);      // free memory allocated for message
          DATALOG_SD_writeBuf(data_s, size);
        }
      }
    }
  }
}


void dataTimer_Callback(void const *arg)
{ 
  osSemaphoreRelease(readDataSem_id);
} 


void dataTimerStart(void)
{
  osStatus  status;
 
  // Create periodic timer
  exec = 1;
  sensorTimId = osTimerCreate(osTimer(SensorTimer), osTimerPeriodic, &exec);
  if (sensorTimId)  {
    status = osTimerStart (sensorTimId, DATA_PERIOD_MS);                // start timer
    if (status != osOK)  {
      // Timer could not be started
    } 
  }
}

void dataTimerStop(void)
{
  osTimerStop(sensorTimId);
}




/**
* @brief  Initialize all sensors
* @param  None
* @retval None
*/
static void initializeAllSensors( void )
{
  if (BSP_ACCELERO_Init( LSM6DSM_X_0, &LSM6DSM_X_0_handle ) != COMPONENT_OK)
  {
    while(1);
  }
  
  if (BSP_GYRO_Init( LSM6DSM_G_0, &LSM6DSM_G_0_handle ) != COMPONENT_OK)
  {
    while(1);
  }
  
  if (BSP_ACCELERO_Init( LSM303AGR_X_0, &LSM303AGR_X_0_handle ) != COMPONENT_OK)
  {
    while(1);
  }
  
  if (BSP_MAGNETO_Init( LSM303AGR_M_0, &LSM303AGR_M_0_handle ) != COMPONENT_OK)
  {
    while(1);
  }
  
  if (BSP_PRESSURE_Init( LPS22HB_P_0, &LPS22HB_P_0_handle ) != COMPONENT_OK)
  {
    while(1);
  }
  
  if (BSP_TEMPERATURE_Init( LPS22HB_T_0, &LPS22HB_T_0_handle ) != COMPONENT_OK)
  {
    while(1);
  }
  
  if(BSP_TEMPERATURE_Init( HTS221_T_0, &HTS221_T_0_handle ) == COMPONENT_ERROR)
  {
    no_T_HTS221 = 1;
  }
  
  if(BSP_HUMIDITY_Init( HTS221_H_0, &HTS221_H_0_handle ) == COMPONENT_ERROR)
  {
    no_H_HTS221 = 1;
  }
  
  if(LoggingInterface == SDCARD_Datalog)
  {
    /* Enable HW Double Tap detection */
    BSP_ACCELERO_Enable_Double_Tap_Detection_Ext(LSM6DSM_X_0_handle, INT2_PIN);
    BSP_ACCELERO_Set_Tap_Threshold_Ext(LSM6DSM_X_0_handle, LSM6DSM_TAP_THRESHOLD_MID);
  }
  
}

/**
* @brief  Enable all sensors
* @param  None
* @retval None
*/
void enableAllSensors( void )
{
  BSP_ACCELERO_Sensor_Enable( LSM6DSM_X_0_handle );
  BSP_GYRO_Sensor_Enable( LSM6DSM_G_0_handle );
  BSP_ACCELERO_Sensor_Enable( LSM303AGR_X_0_handle );
  BSP_MAGNETO_Sensor_Enable( LSM303AGR_M_0_handle );
  BSP_PRESSURE_Sensor_Enable( LPS22HB_P_0_handle );
  BSP_TEMPERATURE_Sensor_Enable( LPS22HB_T_0_handle );
  if(!no_T_HTS221)
  {
    BSP_TEMPERATURE_Sensor_Enable( HTS221_T_0_handle );
  }
  if(!no_H_HTS221)
  {
    BSP_HUMIDITY_Sensor_Enable( HTS221_H_0_handle );
  }
}
/**
* @brief  Set ODR all sensors
* @param  None
* @retval None
*/
void setOdrAllSensors( void )
{
  BSP_ACCELERO_Set_ODR_Value( LSM303AGR_X_0_handle, ACCELERO_ODR);
  BSP_MAGNETO_Set_ODR_Value( LSM303AGR_M_0_handle, MAGNETO_ODR);
  BSP_GYRO_Set_ODR_Value(LSM6DSM_G_0_handle, GYRO_ODR);
  BSP_PRESSURE_Set_ODR_Value( LPS22HB_P_0_handle, PRESSURE_ODR);
  BSP_TEMPERATURE_Set_ODR_Value( HTS221_T_0_handle, TEMPERATURE_ODR);
  BSP_HUMIDITY_Set_ODR_Value( HTS221_H_0_handle, TEMPERATURE_ODR );    
}


/**
* @brief  Disable all sensors
* @param  None
* @retval None
*/
void disableAllSensors( void )
{
  BSP_ACCELERO_Sensor_Disable( LSM6DSM_X_0_handle );
  BSP_ACCELERO_Sensor_Disable( LSM303AGR_X_0_handle );
  BSP_GYRO_Sensor_Disable( LSM6DSM_G_0_handle );
  BSP_MAGNETO_Sensor_Disable( LSM303AGR_M_0_handle );
  BSP_HUMIDITY_Sensor_Disable( HTS221_H_0_handle );
  BSP_TEMPERATURE_Sensor_Disable( HTS221_T_0_handle );
  BSP_TEMPERATURE_Sensor_Disable( LPS22HB_T_0_handle );
  BSP_PRESSURE_Sensor_Disable( LPS22HB_P_0_handle );
}

/**
* @brief  EXTI line detection callbacks
* @param  GPIO_Pin: Specifies the pins connected EXTI line
* @retval None
*/
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
  MEMSInterrupt=1;
  osSemaphoreRelease(readDataSem_id);
}

/**
* @brief  This function is executed in case of error occurrence
* @param  None
* @retval None
*/
static void Error_Handler( void )
{
  while (1)
  {}
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
