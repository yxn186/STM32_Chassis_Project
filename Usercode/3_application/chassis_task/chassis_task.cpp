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
#include "SlopePlaning.h"
#include "DWT.h"

/*  Task层全局变量 ------------------------------------------------------------*/
bool Global_Init_Finished = false;

//查看任务执行时间和周期的变量
volatile uint32_t main_task_tick = 0;
volatile uint32_t main_task_count = 0;
volatile uint32_t main_task_period_us = 0;   // 实际任务周期 (us)
volatile uint32_t main_task_exec_us = 0;     // 任务单次执行耗时 (us)

/*  Task层数据    ------------------------------------------------------------*/

//斜坡规划相关
///斜坡规划器，分别对应底盘的 vx, vy, wz 三个维度
Class_SlopePlaning SlopePlaning_X;
Class_SlopePlaning SlopePlaning_Y;
Class_SlopePlaning SlopePlaning_Z;
Class_DR16 DR16;

static constexpr float CHASSIS_SLOPE_DT = 0.001f;
static constexpr float CHASSIS_SLOPE_ACCEL_X = 10.0f;
static constexpr float CHASSIS_SLOPE_ACCEL_Y = 10.0f;
static constexpr float CHASSIS_SLOPE_ACCEL_Z = 30.0f;

//底盘模式枚举
typedef enum
{
  No_Power_Mode = 0,
  Normal_Running_Mode_Low,
  Normal_Running_Mode_High,
  Little_Top_Mode_Low,
  Little_Top_Mode_High,
  Little_Top_Sine_Mode,
  Error_Mode,
  Stop_Mode
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
  float CHASSIS_VZ_MAX_HIGH = 7.0f;    // 最大旋转角速度 (rad/s)

  // 正弦小陀螺模式参数
  float LITTLE_TOP_SINE_MIN_SPEED = 5.0f;   // 正弦角速度最低值 (rad/s)
  float LITTLE_TOP_SINE_MAX_SPEED = 9.0f;   // 正弦角速度最高值 (rad/s)
  float LITTLE_TOP_SINE_FREQ_HZ   = 0.8f;   // 正弦频率 (Hz)
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
 * @brief IMU 陀螺仪在线温漂修正量（rad/s）
 * 通过轮式里程计做参考，配合 innovation-gated 滤波估计
 */
static float Little_Top_IMU_Bias_Correction = 0.0f;

/**
 * @brief Innovation-gated 自适应偏差更新
 * 
 * @param imu_wz    IMU 角速度 (rad/s)
 * @param wheel_wz  轮式里程计角速度 (rad/s)
 * @param tau       基础时间常数 (s)，越小收敛越快
 * @param sigma     正常噪声阈值 (rad/s)，超过此值视为轮滑异常
 * 
 * 原理：innovation = (wheel - imu) - bias_estimate
 *       gate = σ² / (σ² + innovation²)
 *       innovation 小 → 正常更新；innovation 大（轮滑）→ 几乎冻结
 */
static void IMU_Bias_Gated_Update(float imu_wz, float wheel_wz, float tau, float sigma)
{
  float innovation = (wheel_wz - imu_wz) - Little_Top_IMU_Bias_Correction;
  float sigma_sq = sigma * sigma;
  float gate = sigma_sq / (sigma_sq + innovation * innovation);

  Little_Top_IMU_Bias_Correction += innovation * gate * (0.001f / tau);

  // 钳位到物理合理范围（BMI088 温漂通常 < 0.05 rad/s ≈ 3°/s）
  const float BIAS_MAX = 0.05f;
  if (Little_Top_IMU_Bias_Correction > BIAS_MAX)  Little_Top_IMU_Bias_Correction = BIAS_MAX;
  if (Little_Top_IMU_Bias_Correction < -BIAS_MAX) Little_Top_IMU_Bias_Correction = -BIAS_MAX;
}

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

static void Chassis_SlopePlaning_Reset(float Reset_Value)
{
  SlopePlaning_X.Reset(Reset_Value);
  SlopePlaning_Y.Reset(Reset_Value);
  SlopePlaning_Z.Reset(Reset_Value);

  Vx = Reset_Value;
  Vy = Reset_Value;
  Vz = Reset_Value;
}

static void Chassis_SlopePlaning_Calculate(float Target_Vx, float Target_Vy, float Target_Vz)
{
  SlopePlaning_X.Set_Target(Target_Vx);
  SlopePlaning_Y.Set_Target(Target_Vy);
  SlopePlaning_Z.Set_Target(Target_Vz);

  SlopePlaning_X.Calculate_Loop();
  SlopePlaning_Y.Calculate_Loop();
  SlopePlaning_Z.Calculate_Loop();

  Vx = SlopePlaning_X.Get_After_Target();
  Vy = SlopePlaning_Y.Get_After_Target();
  Vz = SlopePlaning_Z.Get_After_Target();
}

/*  Task层初始化函数    ------------------------------------------------------*/
/**
 * @brief Chassis层初始化函数
 * 
 */
void Chassis_Task_Init(void)
{
  Chassis_Init();
  DR16.Init(&huart3);

  SlopePlaning_X.Init(CHASSIS_SLOPE_ACCEL_X, CHASSIS_SLOPE_ACCEL_X, CHASSIS_SLOPE_DT, SlopePlaning_TARGET);
  SlopePlaning_Y.Init(CHASSIS_SLOPE_ACCEL_Y, CHASSIS_SLOPE_ACCEL_Y, CHASSIS_SLOPE_DT, SlopePlaning_TARGET);
  SlopePlaning_Z.Init(CHASSIS_SLOPE_ACCEL_Z, CHASSIS_SLOPE_ACCEL_Z, CHASSIS_SLOPE_DT, SlopePlaning_TARGET);
  Chassis_SlopePlaning_Reset(0.0f);
}


/*  Task层自定义回调函数类型 --------------------------------------------------*/


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
    //无数据输出

    if(Global_Init_Finished)
    {
      //顺便100ms检测
      DR16.Task_100ms_Alive_Detection();
    }
    osDelay(100);
  }
  /* USER CODE END Data_ptintf_task */
}

extern "C" void main_Task_1ms(void *argument)
{
  /* USER CODE BEGIN main_Task_1ms */
  const uint32_t period_ticks = 1U;              // 1 tick，前提是你的RTOS tick就是1ms
  uint32_t next_tick = osKernelGetTickCount();   // 记录下一次唤醒基准
  
  /* Infinite loop */
  for(;;)
  {
    if(Global_Init_Finished)
    {
      static uint64_t last_enter_us = 0;
      uint64_t enter_us = DWT_GetUs();
      if (last_enter_us != 0U)
      {
        main_task_period_us = (uint32_t)(enter_us - last_enter_us);
      }
      last_enter_us = enter_us;

      main_task_tick = osKernelGetTickCount();
      main_task_count++;

      app_bmi088_1ms_task_get_now_pitch_yaw_roll(&Yaw, &Picth, &Roll);

      DR16.Task_1ms_Data_Calculate();

      Chassis_DR16_Get_Data();
      Chassis_DR16_Switch_Detect();

      Chassis_Remote_Calculate();

      Chassis_Control();

      main_task_exec_us = (uint32_t)(DWT_GetUs() - enter_us);
    }

    next_tick += period_ticks;
    osDelayUntil(next_tick);
    //osDelay(1);
  }
  /* USER CODE END main_Task_1ms */
}

/*  Task层函数 ----------------------------------------------------------------*/


/**
 * @brief 底盘控制函数
 * 模式切换时会检测是否从无力/错误状态恢复到运行状态，并重置PID积分项
 */
void Chassis_Control(void)
{
  static Chassis_State_e Last_State = No_Power_Mode;

  static uint32_t Stop_Mode_Brake_Counter = 0;// 停止模式刹停计数器，单位ms，初始值 1000ms

  float vx_chassis = 0.0f;
  float vy_chassis = 0.0f;
  float wz_cmd = 0.0f;
  float theta_rad = 0.0f;

  //模式切换检测
  if (Last_State != Chassis_State)
  {
    // 状态切换时统一清空控制残留，避免上一模式的积分项和规划值带入当前模式
    Chassis_PID_Reset();
    Chassis_SlopePlaning_Reset(0.0f);

    Chassis_Slope_Gravity_FeedForward_Reset();

    // 切换到停止模式时，先刹停 1s 再断电
    if (Chassis_State == Stop_Mode)
    {
      Stop_Mode_Brake_Counter = 1000;
    }
  }

  // 检测上一次状态是否是小陀螺模式，用于决定是否需要重置小陀螺的跟随角度
  bool last_state_is_little_top = (Last_State == Little_Top_Mode_Low || Last_State == Little_Top_Mode_High || Last_State == Little_Top_Sine_Mode);

  //模式控制
  switch (Chassis_State)
  {
    //正常行驶模式，低速档
    case Normal_Running_Mode_Low:
    {
      // 背景 bias 学习：正常行驶时轮式里程计可靠，用较快收敛预热 bias 估计
      // 这样切换到小陀螺时已有准确的初始 bias，大幅减少进入旋转后的漂移
      IMU_Bias_Gated_Update(BMI088_GetYawAngleSpeed(),
                            Chassis_Get_Current_AngleSpeed_w(),
                            5.0f,     // τ = 5s，非旋转时加速收敛
                            0.05f);   // σ = 0.05 rad/s，正常行驶允许稍大误差

      Chassis_loop(Vx, Vy, Vz);
      //Chassis_Loop_With_Slope_Gravity_FeedForward(Vx, Vy, Vz, Picth, Roll);
      break;
    }
    //正常行驶模式，高速档
    case Normal_Running_Mode_High:
    {
      // 背景 bias 学习（同上）
      IMU_Bias_Gated_Update(BMI088_GetYawAngleSpeed(),
                            Chassis_Get_Current_AngleSpeed_w(),
                            5.0f, 0.05f);
      
      Chassis_loop(Vx, Vy, Vz);
      //Chassis_Loop_With_Slope_Gravity_FeedForward(Vx, Vy, Vz, Picth, Roll);
      break;
    }
    //停止模式：先刹停再无力
    case Stop_Mode:
    {
      if (Stop_Mode_Brake_Counter > 0u)
      {
        if(MyMath_Abs(Picth) > 10.0f || MyMath_Abs(Roll) > 10.0f)
        {
          Chassis_Loop_With_Slope_Gravity_FeedForward(0.0f, 0.0f, 0.0f, Picth, Roll);
        }
        else
        {
          // 刹停阶段，保持制动力输出
          Chassis_loop(0.0f, 0.0f, 0.0f);
          Stop_Mode_Brake_Counter--;
        }
      }
      else
      {
        // 刹停完成，断电
        Chassis_State = No_Power_Mode;
      }
      break;
    }
    //无力模式
    case No_Power_Mode:
    {
      Chassis_Motor_No_Power();
      break;    
    }
    //错误模式
    case Error_Mode:
    {
      Chassis_loop(0, 0,0);
      //Chassis_Motor_No_Power();
      break;
    }
    //小陀螺模式：低速档
    case Little_Top_Mode_Low:
    {
      if (!last_state_is_little_top)
      {
        Little_Top_Follow_Angle_Rad = 0.0f;   // 进入小陀螺时清零
      }

      // 小陀螺角速度命令（正值=逆时针，负值=顺时针，按需取负）
      wz_cmd = -Chassis_Remote_Data.CHASSIS_VZ_MAX;

      // ① 两路角速度测量 + innovation-gated 自适应偏差修正
      float imu_wz   = BMI088_GetYawAngleSpeed();            // rad/s，已减去上电静态 bias
      float wheel_wz = Chassis_Get_Current_AngleSpeed_w();   // rad/s，四轮正运动学求和

      // ② 自适应温漂估计（门控滤波：轮滑大时自动冻结，避免轮滑污染 bias）
      //    τ=20s, σ=0.03 rad/s（旋转时用保守参数，防止轮滑误差带偏 bias）
      IMU_Bias_Gated_Update(imu_wz, wheel_wz, 20.0f, 0.03f);

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
    //小陀螺模式：高速档
    case Little_Top_Mode_High:
    {
      if (!last_state_is_little_top)
      {
        Little_Top_Follow_Angle_Rad = 0.0f;   // 高速进入时同样清零，重新从当前朝向积分
      }

      // 小陀螺角速度命令（正值=逆时针，负值=顺时针，按需取负）
      wz_cmd = -Chassis_Remote_Data.CHASSIS_VZ_MAX_HIGH;

      // 与低速模式相同的融合策略，保证切换速度档时漂移特性一致
      float imu_wz   = BMI088_GetYawAngleSpeed();
      float wheel_wz = Chassis_Get_Current_AngleSpeed_w();

      // 高速旋转用更保守的 σ（轮滑更严重），τ 同样 20s
      IMU_Bias_Gated_Update(imu_wz, wheel_wz, 20.0f, 0.02f);

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

    //正弦小陀螺模式：wz 按正弦函数周期变化，行进方向仍跟随云台坐标系
    case Little_Top_Sine_Mode:
    {
      static float sine_time = 0.0f;

      if (!last_state_is_little_top)
      {
        Little_Top_Follow_Angle_Rad = 0.0f;
        sine_time = 0.0f;
      }

      // 正弦角速度命令，速度在 [MIN_SPEED, MAX_SPEED] 之间平滑变化，始终单向旋转
      // wz = MIN + (MAX - MIN) * (1 - cos(ωt)) / 2
      const float omega = 2.0f * 3.14159265358979f * Chassis_Remote_Data.LITTLE_TOP_SINE_FREQ_HZ;
      float sine_speed = Chassis_Remote_Data.LITTLE_TOP_SINE_MIN_SPEED
                       + (Chassis_Remote_Data.LITTLE_TOP_SINE_MAX_SPEED - Chassis_Remote_Data.LITTLE_TOP_SINE_MIN_SPEED)
                       * (1.0f - cosf(omega * sine_time)) * 0.5f;
      wz_cmd = -sine_speed;
      sine_time += 0.001f;

      float imu_wz   = BMI088_GetYawAngleSpeed();
      float wheel_wz = Chassis_Get_Current_AngleSpeed_w();

      IMU_Bias_Gated_Update(imu_wz, wheel_wz, 20.0f, 0.02f);

      float corrected_wz = imu_wz + Little_Top_IMU_Bias_Correction;
      Little_Top_Follow_Angle_Rad += corrected_wz * 0.001f;
      Little_Top_Follow_Angle_Rad = Wrap_Rad(Little_Top_Follow_Angle_Rad);

      theta_rad = Little_Top_Follow_Angle_Rad
                + Little_Top_Angle_FeedForward_K * corrected_wz;

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
  float Target_Vx = 0.0f;
  float Target_Vy = 0.0f;
  float Target_Vz = 0.0f;

  if(Chassis_State == Normal_Running_Mode_High || Chassis_State == Little_Top_Mode_High || Chassis_State == Little_Top_Sine_Mode)
  {
    Target_Vx = Chassis_Remote_Data.X * Chassis_Remote_Data.CHASSIS_VX_MAX_HIGH;
    Target_Vy = Chassis_Remote_Data.Y * Chassis_Remote_Data.CHASSIS_VY_MAX_HIGH;
    Target_Vz = Chassis_Remote_Data.Z * Chassis_Remote_Data.CHASSIS_VZ_MAX_HIGH;
  }
  else if(Chassis_State == Normal_Running_Mode_Low || Chassis_State == Little_Top_Mode_Low)
  {
    Target_Vx = Chassis_Remote_Data.X * Chassis_Remote_Data.CHASSIS_VX_MAX;
    Target_Vy = Chassis_Remote_Data.Y * Chassis_Remote_Data.CHASSIS_VY_MAX;
    Target_Vz = Chassis_Remote_Data.Z * Chassis_Remote_Data.CHASSIS_VZ_MAX;
  }

  Chassis_SlopePlaning_Calculate(Target_Vx, Target_Vy, Target_Vz);
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
  // 遥控器离线时，强制停止模式，防止底盘失控
  if (DR16.Get_Status() != DR16_Status_ENABLE)
  {
    Chassis_State = No_Power_Mode;
    return;
  }

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
  else if(Left_Mode == 1 && Right_Mode == 2)
  {
    Chassis_State = Stop_Mode;
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
  else if(Left_Mode == 3 && Right_Mode == 1)
  {
    Chassis_State = Little_Top_Sine_Mode;
  }
  else
  {
    Chassis_State = Error_Mode;
  }
}