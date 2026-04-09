/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    chassis_task.c
  * @brief   Task
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "chassis_task.h"
#include "usart.h"
#include <stdbool.h>
#include "usbd_cdc_if.h"
#include "bsp_usb.h"
#include "cmsis_os2.h"
#include "usb_device.h"
#include "DJI_Motor.h"
#include "MyMath.h"
#include "chassis.h"
#include "dr16.h"

/*  Task层全局变量 ------------------------------------------------------------*/
bool Global_Init_Finished = false;

/*  Task层数据    ------------------------------------------------------------*/


Class_DR16 DR16;


typedef struct
{
  float X;
  float Y;
  float Z;
}Chassis_Remote_Data_t;

Chassis_Remote_Data_t Chassis_Remote_Data;


float X,Y,Z;
/*  Task层初始化函数    ------------------------------------------------------*/


/*  Task层自定义回调函数类型 --------------------------------------------------*/

void USB_CallBack(uint8_t *Buffer, uint16_t Length)
{
  if(Length == 0)
  {
    return;
  }
  
  //数据处理
  

}

/*  Task层FreeRTOS函数 任务函数 -----------------------------------------------*/

extern "C" void StartInitTask(void *argument)
{
 /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartInitTask */
  Chassis_Task_Init();
  Global_Init_Finished = true;
  osThreadTerminate(osThreadGetId());
  /* Infinite loop */
  for(;;)
  {
    ;
  }
  /* USER CODE END StartInitTask */
}

extern "C" void Data_ptintf_task(void *argument)
{
  /* USER CODE BEGIN Data_ptintf_task */
  /* Infinite loop */
  for(;;)
  {
    //待加数据输出

    //顺便100ms检测
    DR16.task_100ms_alive_detection();
    osDelay(100);
  }
  /* USER CODE END Data_ptintf_task */
}

extern "C" void main_Task_1ms(void *argument)
{
  /* USER CODE BEGIN main_Task_1ms */
  /* Infinite loop */
  for(;;)
  {
    DR16.task_1ms_data_calculate();
    Chassis_DR16_Get_Data();
    Chassis_Remote_Calculate();
    Chassis_loop(X,Y,Z);
    osDelay(1);
  }
  /* USER CODE END main_Task_1ms */
}

/*  Task层函数 ----------------------------------------------------------------*/

/**
 * @brief Chassis层初始化函数
 * 
 */
void Chassis_Task_Init(void)
{
  Chassis_Init();
  DR16.Init(&huart5);
}

/**
 * @brief 底盘遥控数据计算
 * 
 */
void Chassis_Remote_Calculate(void)
{
  //暂时先这样加法 后面加个不同程度的XYZ？
  X = Chassis_Remote_Data.X;
  Y = Chassis_Remote_Data.Y;
  Z = Chassis_Remote_Data.Z;
}

/**
 * @brief 底盘DR16获取数据
 * 
 */
void Chassis_DR16_Get_Data(void)
{
  Chassis_Remote_Data.X = DR16.Get_Left_X();
  Chassis_Remote_Data.Y = DR16.Get_Left_Y();
  Chassis_Remote_Data.Z = DR16.Get_Right_X();
}