/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    chassis_task.h
  * @brief   This file contains all the function prototypes for
  *          the chassis_task.c file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "bsp_usb.h"
#include "Serial.h"

/*YOUR CODE*/
extern bool init_finished;

/* Printf配置------------------------------------------------- */
#ifndef STM32_PRINTF_USE_USB
#define STM32_PRINTF_USE_USB 0
#endif

#if STM32_PRINTF_USE_USB
#define STM32_Printf(...) USB_Printf(__VA_ARGS__)
#else
#define STM32_Printf(...) Serial_Printf(__VA_ARGS__)
#endif

/* 函数---------------------------------------------------------*/
/**
 * @brief Chassis层初始化函数
 * 
 */
void Chassis_Task_Init(void);

/**
 * @brief 底盘遥控数据计算
 * 
 */
void Chassis_Remote_Calculate(void);

/**
 * @brief 底盘DR16获取数据
 * 
 */
void Chassis_DR16_Get_Data(void);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_TASK_H__ */
