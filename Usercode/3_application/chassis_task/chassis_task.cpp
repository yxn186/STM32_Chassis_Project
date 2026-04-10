/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    chassis_task.c
  * @brief   Task
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "Chassis_Task.h"
#include "usart.h"
#include <cstdint>
#include <stdbool.h>
#include "usbd_cdc_if.h"
#include "bsp_usb.h"
#include "cmsis_os2.h"
#include "usb_device.h"
#include "DJI_Motor.h"
#include "MyMath.h"
#include "Chassis.h"
#include "dr16.h"
#include "app_bmi088.h"
#include "bmi088_math.h"
#include "math.h"

/*  Task层全局变量 ------------------------------------------------------------*/
bool Global_Init_Finished = false;

/*  Task层数据    ------------------------------------------------------------*/

Class_DR16 DR16;

typedef enum
{
  No_Power_Mode = 0,
  Normal_Running_Mode_Low,
  Normal_Running_Mode_High,
  Little_Top_Mode_Low,
  Little_Top_Mode_High,
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
  
  float CHASSIS_VX_MAX_HIGH = 2.0f;    // 前后最大线速度 (m/s)
  float CHASSIS_VY_MAX_HIGH = 2.0f;    // 左右最大线速度 (m/s)
  float CHASSIS_VZ_MAX_HIGH = 6.0f;    // 最大旋转角速度 (rad/s)
} Chassis_Remote_Data_t;

Chassis_Remote_Data_t Chassis_Remote_Data;

/**
 * @brief 底盘目标速度
 * 
 */
float Vx, Vy, Vz;

/**
 * @brief 底盘姿态角
 * 
 */
float Yaw, Picth, Roll;

/**
 * @brief 小陀螺行进角度前馈系数
 * 单位：s
 * 先从 0 开始调通，再慢慢加
 */
static float Little_Top_Angle_FeedForward_K = 0.005f;

/**
 * @brief 获取云台yaw相对底盘的夹角（单位：rad）
 * @note 你必须把这里替换成你自己的真实反馈接口
 */
static float Get_Gimbal_Yaw_Relative_Chassis_Rad(void)
{
  // ---------------------------
  return Yaw * 0.01745329251994f;
}

/**
 * @brief 将云台坐标系下的速度矢量，变换到底盘坐标系
 * 
 * @param vx_g 云台系X速度
 * @param vy_g 云台系Y速度
 * @param theta_rad 云台相对底盘yaw角(rad)
 * @param vx_c 输出：底盘系X速度
 * @param vy_c 输出：底盘系Y速度
 */
static void Chassis_Vector_Gimbal_To_Chassis(float vx_g, float vy_g, float theta_rad,
                                             float *vx_c, float *vy_c)
{
  float cos_theta = cosf(theta_rad);
  float sin_theta = sinf(theta_rad);

  // 世界系（遥控/云台系）→ 底盘体系：乘以 R^T(theta)
  *vx_c =  vx_g * cos_theta + vy_g * sin_theta;
  *vy_c = -vx_g * sin_theta + vy_g * cos_theta;
}

/**
 * @brief 小陀螺模式下，用底盘实际角速度积分得到的相对转角
 * 单位：rad
 */
static float Little_Top_Follow_Angle_Rad = 0.0f;

/**
 * @brief 弧度包角到 [-pi, pi]
 */
static float Wrap_Rad(float angle_rad)
{
  const float PI_F = 3.14159265358979f;
  const float TWO_PI_F = 6.28318530717959f;

  while(angle_rad > PI_F)   angle_rad -= TWO_PI_F;
  while(angle_rad < -PI_F)  angle_rad += TWO_PI_F;

  return angle_rad;
}

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
  app_bmi088_init();
  
  /* Infinite loop */
  for(;;)
  {
    if(app_bmi088_init_process_loop())
    {
      Chassis_Task_Init();
      Global_Init_Finished = true;
      osThreadTerminate(osThreadGetId());
    }
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
    app_bmi088_1ms_task_get_now_pitch_yaw_roll(&Yaw, &Picth, &Roll);
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

  float vx_chassis = 0.0f;
  float vy_chassis = 0.0f;
  float wz_cmd = 0.0f;
  float theta_rad = 0.0f;

  switch (Chassis_State)
  {
    case Normal_Running_Mode_Low:
    {
      // 从无力/错误模式切换到运行模式时，重置所有PID积分项，防止积分残留导致输出突变
      if (Last_State != Normal_Running_Mode_Low)
      {
        Chassis_PID_Reset();
      }
      Chassis_loop(Vx, Vy, Vz);
      break;
    }
    case Normal_Running_Mode_High:
    {
      if (Last_State != Normal_Running_Mode_High)
      {
        Chassis_PID_Reset();
      }
      Chassis_loop(Vx, Vy, Vz);
      break;
    }
    case No_Power_Mode:
    {
      if (Last_State != No_Power_Mode)
      {
        Chassis_PID_Reset();
      }
      Chassis_Motor_No_Power();
      break;
    }
    case Error_Mode:
    {
      if (Last_State != Error_Mode)
      {
        Chassis_PID_Reset();
      }
      Chassis_loop(0, 0,0);
      //Chassis_Motor_No_Power();
      break;
    }
    case Little_Top_Mode_Low:
    {
      if (Last_State != Little_Top_Mode_Low)
      {
        Chassis_PID_Reset();
        Little_Top_Follow_Angle_Rad = 0.0f;   // 进入小陀螺时清零
      }

      // 小陀螺角速度命令（正值=逆时针，负值=顺时针，按需取负）
      wz_cmd = -Chassis_Remote_Data.CHASSIS_VZ_MAX;

      // ① 两路角速度测量
      //    IMU body-Z：噪声低、无轮滑，但存在随温升变化的 gyro bias
      //    轮式里程计：(w0+w1+w2+w3)/4，无温漂，但有轮滑/量化误差
      float imu_wz   = BMI088_GetYawAngleSpeed();            // rad/s，已减去上电静态 bias
      float wheel_wz = Chassis_Get_Current_AngleSpeed_w();   // rad/s，四轮正运动学求和

      // ② 在线温漂估计：低通（τ ≈ 30 s）追踪 IMU 的热偏移
      //    轮式里程计作为无温漂“漂移参考”，缓慢修正 IMU bias
      Little_Top_IMU_Bias_Correction += (wheel_wz - imu_wz) * (0.001f / 30.0f);

      // ③ 用修正后的 IMU 积分相对转角（高频精度 + 长期稳定）
      float corrected_wz = imu_wz + Little_Top_IMU_Bias_Correction;
      Little_Top_Follow_Angle_Rad += corrected_wz * 0.001f;
      Little_Top_Follow_Angle_Rad = Wrap_Rad(Little_Top_Follow_Angle_Rad);

      theta_rad = Little_Top_Follow_Angle_Rad
                + Little_Top_Angle_FeedForward_K * corrected_wz;

      // 把遥控器（云台系）速度，转换到底盘系
      Chassis_Vector_Gimbal_To_Chassis(Vx, Vy, theta_rad, &vx_chassis, &vy_chassis);

      Chassis_loop(vx_chassis, vy_chassis, wz_cmd);
      break;
    }
    case Little_Top_Mode_High:
    {
      if (Last_State != Little_Top_Mode_High)
      {
        Chassis_PID_Reset();
        Little_Top_Follow_Angle_Rad = 0.0f;   // 高速进入时同样清零，重新从当前朝向积分
      }

      // 小陀螺角速度命令（正值=逆时针，负值=顺时针，按需取负）
      wz_cmd = -Chassis_Remote_Data.CHASSIS_VZ_MAX_HIGH;

      // 与低速模式相同的融合策略，保证切换速度档时漂移特性一致
      float imu_wz   = BMI088_GetYawAngleSpeed();
      float wheel_wz = Chassis_Get_Current_AngleSpeed_w();

      Little_Top_IMU_Bias_Correction += (wheel_wz - imu_wz) * (0.001f / 30.0f);

      float corrected_wz = imu_wz + Little_Top_IMU_Bias_Correction;
      Little_Top_Follow_Angle_Rad += corrected_wz * 0.001f;
      Little_Top_Follow_Angle_Rad = Wrap_Rad(Little_Top_Follow_Angle_Rad);

      theta_rad = Little_Top_Follow_Angle_Rad
                + Little_Top_Angle_FeedForward_K * corrected_wz;

      // 把遥控器（云台系）速度，转换到底盘系
      Chassis_Vector_Gimbal_To_Chassis(Vx, Vy, theta_rad, &vx_chassis, &vy_chassis);

      Chassis_loop(vx_chassis, vy_chassis, wz_cmd);
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
  if(Chassis_State == Normal_Running_Mode_High || Chassis_State == Little_Top_Mode_High)
  {
    Vx = Chassis_Remote_Data.X * Chassis_Remote_Data.CHASSIS_VX_MAX_HIGH;
    Vy = Chassis_Remote_Data.Y * Chassis_Remote_Data.CHASSIS_VY_MAX_HIGH;
    Vz = Chassis_Remote_Data.Z * Chassis_Remote_Data.CHASSIS_VZ_MAX_HIGH;
  }
  else if(Chassis_State == Normal_Running_Mode_Low || Chassis_State == Little_Top_Mode_Low)
  {
    Vx = Chassis_Remote_Data.X * Chassis_Remote_Data.CHASSIS_VX_MAX;
    Vy = Chassis_Remote_Data.Y * Chassis_Remote_Data.CHASSIS_VY_MAX;
    Vz = Chassis_Remote_Data.Z * Chassis_Remote_Data.CHASSIS_VZ_MAX;
  } 
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
  Chassis_Remote_Data.Z = -DR16.Get_Right_X();
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
  if(Left_Mode == 2 && Right_Mode == 2)
  {
    Chassis_State = Normal_Running_Mode_Low;
  }
  else if(Left_Mode == 2 && Right_Mode == 3)
  {
    Chassis_State = Normal_Running_Mode_High;
  }
  else if(Left_Mode == 1 && Right_Mode == 1)
  {
    Chassis_State = No_Power_Mode;
  }
  else if(Left_Mode == 3 && Right_Mode == 2)
  {
    Chassis_State = Little_Top_Mode_Low;
  }
  else if(Left_Mode == 3 && Right_Mode == 3)
  {
    Chassis_State = Little_Top_Mode_High;
  }
  else
  {
    Chassis_State = Error_Mode;
  }
}