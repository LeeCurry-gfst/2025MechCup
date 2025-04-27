/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Chassis_AngleLoop */
osThreadId_t Chassis_AngleLoopHandle;
const osThreadAttr_t Chassis_AngleLoop_attributes = {
  .name = "Chassis_AngleLoop",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime2,
};
/* Definitions for My_Init */
osThreadId_t My_InitHandle;
const osThreadAttr_t My_Init_attributes = {
  .name = "My_Init",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for UI_Cmp */
osThreadId_t UI_CmpHandle;
const osThreadAttr_t UI_Cmp_attributes = {
  .name = "UI_Cmp",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for UI_BLErocker */
osThreadId_t UI_BLErockerHandle;
const osThreadAttr_t UI_BLErocker_attributes = {
  .name = "UI_BLErocker",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for UI_BLEarm */
osThreadId_t UI_BLEarmHandle;
const osThreadAttr_t UI_BLEarm_attributes = {
  .name = "UI_BLEarm",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for UI_OLED */
osThreadId_t UI_OLEDHandle;
const osThreadAttr_t UI_OLED_attributes = {
  .name = "UI_OLED",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow6,
};
/* Definitions for Status_Monitor */
osThreadId_t Status_MonitorHandle;
const osThreadAttr_t Status_Monitor_attributes = {
  .name = "Status_Monitor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal1,
};
/* Definitions for Chassis_SpeedLoop */
osThreadId_t Chassis_SpeedLoopHandle;
const osThreadAttr_t Chassis_SpeedLoop_attributes = {
  .name = "Chassis_SpeedLoop",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime3,
};
/* Definitions for UI_CmpTra */
osThreadId_t UI_CmpTraHandle;
const osThreadAttr_t UI_CmpTra_attributes = {
  .name = "UI_CmpTra",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
/* Definitions for Buzzer */
osThreadId_t BuzzerHandle;
const osThreadAttr_t Buzzer_attributes = {
  .name = "Buzzer",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for SteppingMotor */
osThreadId_t SteppingMotorHandle;
const osThreadAttr_t SteppingMotor_attributes = {
  .name = "SteppingMotor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal5,
};
/* Definitions for MechArm */
osThreadId_t MechArmHandle;
const osThreadAttr_t MechArm_attributes = {
  .name = "MechArm",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal7,
};
/* Definitions for Launch */
osThreadId_t LaunchHandle;
const osThreadAttr_t Launch_attributes = {
  .name = "Launch",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal6,
};
/* Definitions for BLErockerRec */
osMessageQueueId_t BLErockerRecHandle;
const osMessageQueueAttr_t BLErockerRec_attributes = {
  .name = "BLErockerRec"
};
/* Definitions for BLEarmRec */
osMessageQueueId_t BLEarmRecHandle;
const osMessageQueueAttr_t BLEarmRec_attributes = {
  .name = "BLEarmRec"
};
/* Definitions for CmpRec */
osMessageQueueId_t CmpRecHandle;
const osMessageQueueAttr_t CmpRec_attributes = {
  .name = "CmpRec"
};
/* Definitions for TestRec */
osMessageQueueId_t TestRecHandle;
const osMessageQueueAttr_t TestRec_attributes = {
  .name = "TestRec"
};
/* Definitions for TmpArmRec */
osMessageQueueId_t TmpArmRecHandle;
const osMessageQueueAttr_t TmpArmRec_attributes = {
  .name = "TmpArmRec"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartChassis_AngleLoop(void *argument);
void StartMy_Init(void *argument);
void StartUI_Cmp(void *argument);
void StartUI_BLErocker(void *argument);
void StartUI_BLEarm(void *argument);
void StartUI_OLED(void *argument);
void StartStatus_Monitor(void *argument);
void StartChassis_SpeedLoop(void *argument);
void StartUI_CmpTra(void *argument);
void StartBuzzer(void *argument);
void StartSteppingMotor(void *argument);
void StartMechArm(void *argument);
void StartLaunch(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of BLErockerRec */
  BLErockerRecHandle = osMessageQueueNew (16, sizeof(uint8_t), &BLErockerRec_attributes);

  /* creation of BLEarmRec */
  BLEarmRecHandle = osMessageQueueNew (16, sizeof(uint8_t), &BLEarmRec_attributes);

  /* creation of CmpRec */
  CmpRecHandle = osMessageQueueNew (32, sizeof(uint8_t), &CmpRec_attributes);

  /* creation of TestRec */
  TestRecHandle = osMessageQueueNew (16, sizeof(uint8_t), &TestRec_attributes);

  /* creation of TmpArmRec */
  TmpArmRecHandle = osMessageQueueNew (16, sizeof(uint8_t), &TmpArmRec_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Chassis_AngleLoop */
  Chassis_AngleLoopHandle = osThreadNew(StartChassis_AngleLoop, NULL, &Chassis_AngleLoop_attributes);

  /* creation of My_Init */
  My_InitHandle = osThreadNew(StartMy_Init, NULL, &My_Init_attributes);

  /* creation of UI_Cmp */
  UI_CmpHandle = osThreadNew(StartUI_Cmp, NULL, &UI_Cmp_attributes);

  /* creation of UI_BLErocker */
  UI_BLErockerHandle = osThreadNew(StartUI_BLErocker, NULL, &UI_BLErocker_attributes);

  /* creation of UI_BLEarm */
  UI_BLEarmHandle = osThreadNew(StartUI_BLEarm, NULL, &UI_BLEarm_attributes);

  /* creation of UI_OLED */
  UI_OLEDHandle = osThreadNew(StartUI_OLED, NULL, &UI_OLED_attributes);

  /* creation of Status_Monitor */
  Status_MonitorHandle = osThreadNew(StartStatus_Monitor, NULL, &Status_Monitor_attributes);

  /* creation of Chassis_SpeedLoop */
  Chassis_SpeedLoopHandle = osThreadNew(StartChassis_SpeedLoop, NULL, &Chassis_SpeedLoop_attributes);

  /* creation of UI_CmpTra */
  UI_CmpTraHandle = osThreadNew(StartUI_CmpTra, NULL, &UI_CmpTra_attributes);

  /* creation of Buzzer */
  BuzzerHandle = osThreadNew(StartBuzzer, NULL, &Buzzer_attributes);

  /* creation of SteppingMotor */
  SteppingMotorHandle = osThreadNew(StartSteppingMotor, NULL, &SteppingMotor_attributes);

  /* creation of MechArm */
  MechArmHandle = osThreadNew(StartMechArm, NULL, &MechArm_attributes);

  /* creation of Launch */
  LaunchHandle = osThreadNew(StartLaunch, NULL, &Launch_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartChassis_AngleLoop */
/**
* @brief Function implementing the Chassis_AngleLoop thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassis_AngleLoop */
__weak void StartChassis_AngleLoop(void *argument)
{
  /* USER CODE BEGIN StartChassis_AngleLoop */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartChassis_AngleLoop */
}

/* USER CODE BEGIN Header_StartMy_Init */
/**
* @brief Function implementing the My_Init thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMy_Init */
__weak void StartMy_Init(void *argument)
{
  /* USER CODE BEGIN StartMy_Init */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMy_Init */
}

/* USER CODE BEGIN Header_StartUI_Cmp */
/**
* @brief Function implementing the UI_Cmp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUI_Cmp */
__weak void StartUI_Cmp(void *argument)
{
  /* USER CODE BEGIN StartUI_Cmp */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUI_Cmp */
}

/* USER CODE BEGIN Header_StartUI_BLErocker */
/**
* @brief Function implementing the UI_BLErocker thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUI_BLErocker */
__weak void StartUI_BLErocker(void *argument)
{
  /* USER CODE BEGIN StartUI_BLErocker */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUI_BLErocker */
}

/* USER CODE BEGIN Header_StartUI_BLEarm */
/**
* @brief Function implementing the UI_BLEarm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUI_BLEarm */
__weak void StartUI_BLEarm(void *argument)
{
  /* USER CODE BEGIN StartUI_BLEarm */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUI_BLEarm */
}

/* USER CODE BEGIN Header_StartUI_OLED */
/**
* @brief Function implementing the UI_OLED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUI_OLED */
__weak void StartUI_OLED(void *argument)
{
  /* USER CODE BEGIN StartUI_OLED */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUI_OLED */
}

/* USER CODE BEGIN Header_StartStatus_Monitor */
/**
* @brief Function implementing the Status_Monitor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStatus_Monitor */
__weak void StartStatus_Monitor(void *argument)
{
  /* USER CODE BEGIN StartStatus_Monitor */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartStatus_Monitor */
}

/* USER CODE BEGIN Header_StartChassis_SpeedLoop */
/**
* @brief Function implementing the Chassis_SpeedLoop thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassis_SpeedLoop */
__weak void StartChassis_SpeedLoop(void *argument)
{
  /* USER CODE BEGIN StartChassis_SpeedLoop */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartChassis_SpeedLoop */
}

/* USER CODE BEGIN Header_StartUI_CmpTra */
/**
* @brief Function implementing the UI_CmpTra thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUI_CmpTra */
__weak void StartUI_CmpTra(void *argument)
{
  /* USER CODE BEGIN StartUI_CmpTra */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUI_CmpTra */
}

/* USER CODE BEGIN Header_StartBuzzer */
/**
* @brief Function implementing the Buzzer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBuzzer */
__weak void StartBuzzer(void *argument)
{
  /* USER CODE BEGIN StartBuzzer */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartBuzzer */
}

/* USER CODE BEGIN Header_StartSteppingMotor */
/**
* @brief Function implementing the SteppingMotor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSteppingMotor */
__weak void StartSteppingMotor(void *argument)
{
  /* USER CODE BEGIN StartSteppingMotor */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSteppingMotor */
}

/* USER CODE BEGIN Header_StartMechArm */
/**
* @brief Function implementing the MechArm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMechArm */
__weak void StartMechArm(void *argument)
{
  /* USER CODE BEGIN StartMechArm */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMechArm */
}

/* USER CODE BEGIN Header_StartLaunch */
/**
* @brief Function implementing the Launch thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLaunch */
__weak void StartLaunch(void *argument)
{
  /* USER CODE BEGIN StartLaunch */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartLaunch */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

