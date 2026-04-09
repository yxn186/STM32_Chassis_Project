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
#include <cstdint>
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

typedef enum
{
  Normal_Running_Mode = 0,
  No_Power_Mode,
  Error_Mode,

} Chassis_State_e;

Chassis_State_e Chassis_State = No_Power_Mode;

typedef struct
{
  float X;
  float Y;
  float Z;

  // 遥控器摇杆归一化值(-1~1)到底盘目标速度的缩放系数
  // 单位: X/Y 为 m/s, Z 为 rad/s
  float CHASSIS_VX_MAX = 1.0f;    // 前后最大线速度 (m/s)
  float CHASSIS_VY_MAX = 1.0f;    // 左右最大线速度 (m/s)
  float CHASSIS_VZ_MAX = 3.0f;    // 最大旋转角速度 (rad/s)

} Chassis_Remote_Data_t;

Chassis_Remote_Data_t Chassis_Remote_Data;

/**
 * @brief 底盘目标速度
 * 
 */
float Vx, Vy, Vz;

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
    DR16.Task_100ms_Alive_Detection();
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
    DR16.Task_1ms_Data_Calculate();

    Chassis_DR16_Get_Data();
    Chassis_DR16_Switch_Detect();

    Chassis_Remote_Calculate();

    Chassis_Control();

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
  DR16.Init(&huart3);
}

/**
 * @brief 底盘控制函数
 * 模式切换时会检测是否从无力/错误状态恢复到运行状态，并重置PID积分项
 */
void Chassis_Control(void)
{
  static Chassis_State_e Last_State = No_Power_Mode;

  switch (Chassis_State)
  {
    case Normal_Running_Mode:
    {
      // 从无力/错误模式切换到运行模式时，重置所有PID积分项，防止积分残留导致输出突变
      if (Last_State != Normal_Running_Mode)
      {
        Chassis_PID_Reset();
      }
      Chassis_loop(Vx, Vy, Vz);
      break;
    }
    case No_Power_Mode:
    {
      Chassis_Motor_No_Power();
      break;
    }
    case Error_Mode:
    {
      Chassis_Motor_No_Power();
      break;
    }
    default:
      break;
  }

  Last_State = Chassis_State;
}

/**
 * @brief 底盘遥控数据计算
 * 将摇杆归一化值(-1~1)乘以最大速度系数，转换为底盘实际目标速度
 */
void Chassis_Remote_Calculate(void)
{
  Vx = Chassis_Remote_Data.X * Chassis_Remote_Data.CHASSIS_VX_MAX;
  Vy = Chassis_Remote_Data.Y * Chassis_Remote_Data.CHASSIS_VY_MAX;
  Vz = Chassis_Remote_Data.Z * Chassis_Remote_Data.CHASSIS_VZ_MAX;
}

/**
 * @brief 底盘DR16获取数据
 * 遥控器在线时读取摇杆数据，离线时清零防止底盘失控
 */
void Chassis_DR16_Get_Data(void)
{
  if (DR16.Get_Status() != DR16_Status_ENABLE)
  {
    Chassis_Remote_Data.X = 0.0f;
    Chassis_Remote_Data.Y = 0.0f;
    Chassis_Remote_Data.Z = 0.0f;
    return;
  }

  Chassis_Remote_Data.X = DR16.Get_Left_X();
  Chassis_Remote_Data.Y = DR16.Get_Left_Y();
  Chassis_Remote_Data.Z = DR16.Get_Right_X();
}

/**
 * @brief 底盘DR16拨码开关状态检测
 * 左拨码开关(Left_Switch) / 右拨码开关(Right_Switch)
 * 每1ms调用一次，TRIG状态仅在拨动瞬间触发一次
 */
void Chassis_DR16_Switch_Detect(void)
{
  //上3中2下1
  uint8_t Left_Mode = 0;
  uint8_t Right_Mode = 0;
  /* ---------- 左拨码开关 ---------- */
  switch (DR16.Get_Left_Switch())
  {
    case DR16_Switch_Status_UP:
    {
      // 左拨码 持续拨到上档
      Left_Mode = 3;
      break;
    }
    case DR16_Switch_Status_TRIG_UP_MIDDLE:
    {
      // 左拨码 从上档拨到中档 瞬间触发
      Left_Mode = 2;
      break;
    }
    case DR16_Switch_Status_MIDDLE:
    {
      // 左拨码 持续拨到中档
      Left_Mode = 2;
      break;
    }
    case DR16_Switch_Status_TRIG_MIDDLE_UP:
    {
      // 左拨码 从中/下档拨到上档 瞬间触发
      Left_Mode = 3;
      break;
    }
    case DR16_Switch_Status_TRIG_MIDDLE_DOWN:
    {
      // 左拨码 从中档拨到下档 瞬间触发
      Left_Mode = 1;
      break;
    }
    case DR16_Switch_Status_TRIG_DOWN_MIDDLE:
    {
      // 左拨码 从下档拨到中档 瞬间触发
      Left_Mode = 2;
      break;
    }
    case DR16_Switch_Status_DOWN:
    {
      // 左拨码 持续拨到下档
      Left_Mode = 1;
      break;
    }
    default:
      break;
  }

  /* ---------- 右拨码开关 ---------- */
  switch (DR16.Get_Right_Switch())
  {
    case DR16_Switch_Status_UP:
    {
      // 右拨码 持续拨到上档
      Right_Mode = 3;
      break;
    }
    case DR16_Switch_Status_TRIG_UP_MIDDLE:
    {
      // 右拨码 从上档拨到中档 瞬间触发
      Right_Mode = 2;
      break;
    }
    case DR16_Switch_Status_MIDDLE:
    {
      // 右拨码 持续拨到中档
      Right_Mode = 2;
      break;
    }
    case DR16_Switch_Status_TRIG_MIDDLE_UP:
    {
      // 右拨码 从中/下档拨到上档 瞬间触发
      Right_Mode = 3;
      break;
    }
    case DR16_Switch_Status_TRIG_MIDDLE_DOWN:
    {
      // 右拨码 从中档拨到下档 瞬间触发
      Right_Mode = 1;
      break;
    }
    case DR16_Switch_Status_TRIG_DOWN_MIDDLE:
    {
      // 右拨码 从下档拨到中档 瞬间触发
      Right_Mode = 2;
      break;
    }
    case DR16_Switch_Status_DOWN:
    {
      // 右拨码 持续拨到下档
      Right_Mode = 1;
      break;
    }
    default:
      break;
  }

  //模式设置
  if(Left_Mode == 2 && Right_Mode == 1)
  {
    Chassis_State = Normal_Running_Mode;
  }
  else if(Left_Mode == 1 && Right_Mode == 1)
  {
    Chassis_State = No_Power_Mode;
  }
  else
  {
    Chassis_State = Error_Mode;
  }
}