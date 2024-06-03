/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "crc.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "CAN_receive.h"
#include "motor_pid.h"
#include "mytype.h"

#include "usbd_cdc_if.h"
#include "Debug.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
float getAngle(void);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float angle = 0;
float radian = 0;
float last_radian = 0;
float varepsilon = 0;
int encoder = 0;

// ���rpm��gkf����m/s�ٶȵ�ת����ϵ
// vel_rpm = vel_gkf * k1
// vel_gkf = vel_rpm / k1
float k_vel = 6.75 * 19 * 60 / (0.046 * 2 * 3.1415926);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */
  pid_t pid_position[motor_num]; // ���λ�û�pid
  pid_t pid_speed[motor_num];    // ����ٶ�PID��
  uint8_t i;
  // float set_current[3]; 	// ����
  int16_t delta;                  // �趨�ٶ���ʵ���ٶȵĲ�ֵ
  int16_t max_speed_change = 500; // ����������仯�ٶȣ��Ӽ�����
  float set_speed_temp;           // �Ӽ���ʱ����ʱ�趨�ٶ�

  int j = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM1_Init();
  MX_CRC_Init();
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  can_filter_init();
  // PID��ʼ��
  for (i = 0; i < 2; i++)
  {
    // PID_struct_init(&pid_position[i], POSITION_PID, 8000, 2000, 1.5f, 0.0f, 0.0f);
    PID_struct_init(&pid_speed[i], POSITION_PID, 16384, 16384, 1.8f, 0.1f, 0.0f); // 4 motos angular rate close loop.
  }

  if (pos_flag) // ʹ�ýǶ�λ�û���pid
  {
    PID_struct_init(&pid_position[1], POSITION_PID, 8000, 500, 4.6f, 0.0f, 0.0f);
  }
  else // ʹ�ý��ٶȵ�pid
  {
    PID_struct_init(&pid_position[2], POSITION_PID, 8000, 2500, 8000.6f, 20.0f, 0.0f);
  }

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); // ������ʱ��1

  // ���ó�ʼ��λ�ã��ٶȣ�������Ϊ0

  // ע��debug����
  // state variables
  // Debug_RegisterVar(&angle, "angle", DVar_Float);
  Debug_RegisterVar(&varepsilon, "varepsilon", DVar_Float);
  Debug_RegisterVar(&radian, "radian", DVar_Float);

  //  Debug_RegisterVar(&motor_chassis[1].total_angle, "total_angle", DVar_Int32);
  Debug_RegisterVar(&motor_chassis[0].speed_rpm, "motor1_speed_rpm", DVar_Int16);
  Debug_RegisterVar(&motor_chassis[1].speed_rpm, "motor2_speed_rpm", DVar_Int16);

  // Communication set variables
  Debug_RegisterVar(&set_encoder, "set_encoder", DVar_Float);
  Debug_RegisterVar(&set_vel, "set_vel", DVar_Int16);

  // Communication set variables
  Debug_RegisterVar(&set_varepsilon, "set_varepsilon", DVar_Float);
  Debug_RegisterVar(&set_a, "set_a", DVar_Float);

  // �ٶȻ�pid
  Debug_RegisterVar(&pid_speed[0].p, "speed1_kp", DVar_Float);
  Debug_RegisterVar(&pid_speed[0].i, "speed1_ki", DVar_Float);
  Debug_RegisterVar(&pid_speed[0].d, "speed1_kd", DVar_Float);
  Debug_RegisterVar(&pid_speed[1].p, "speed2_kp", DVar_Float);
  Debug_RegisterVar(&pid_speed[1].i, "speed2_ki", DVar_Float);
  Debug_RegisterVar(&pid_speed[1].d, "speed2_kd", DVar_Float);
  // λ�û�pid
  Debug_RegisterVar(&pid_position[1].p, "position1_kp", DVar_Float);
  Debug_RegisterVar(&pid_position[1].i, "position1_ki", DVar_Float);
  Debug_RegisterVar(&pid_position[1].d, "position1_kd", DVar_Float);

  // ���ٶȻ�pid
  Debug_RegisterVar(&pid_position[2].p, "varepsilon_kp", DVar_Float);
  Debug_RegisterVar(&pid_position[2].i, "varepsilon_ki", DVar_Float);
  Debug_RegisterVar(&pid_position[2].d, "varepsilon_kd", DVar_Float);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_TIM_Base_Start_IT(&htim4);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    j++; // j��Ϊ��־λ��ÿ�ν�������1��
    if (j == 2)
    {
      // ��2�ν�����������ֵ����Ӧ��������250HZ
      //      encoder = getTimEncoder();
      //      angle = getAngle();
      if (send_flag)
      {
        //			usb_printf("A:%.2f\nS:%d\nM:%d\n",
        //               angle, motor_chassis[0].speed_rpm, motor_chassis[1].speed_rpm);
        usb_printf("A:%.2f\nE:%d\nS:%d\nM:%d\n",
                   angle, encoder, motor_chassis[0].speed_rpm, motor_chassis[1].speed_rpm);
      }
      j = 0; // ��0
    }

    // PID�������
    // PID����

    // ���1���������ֻ�����ٶȻ�pid
    // pid_calc(&pid_speed[0], (float)motor_chassis[0].speed_rpm, set_speed[0]);
    pid_calc(&pid_speed[0], (float)motor_chassis[0].speed_rpm, set_vel);

    // ���2������λ�û�+�ٶȻ�pid�ļ���
    // ��ʵ��ƽ̨�Ƕ���Ϊ����ֵ
    // λ�û�pid
    if (pos_flag)
    {
      pid_calc(&pid_position[1], (float)encoder, set_encoder);
      // �Ӽ���
      //    delta = (int16_t)pid_position[1].pos_out - motor_chassis[1].speed_rpm;
      //    if (delta > max_speed_change)
      //      set_speed_temp = (float)(motor_chassis[1].speed_rpm + max_speed_change);
      //    else if (delta < -max_speed_change)
      //      set_speed_temp = (float)(motor_chassis[1].speed_rpm - max_speed_change);
      //    else
      set_speed_temp = pid_position[1].pos_out;
    }
    else
    {
      pid_calc(&pid_position[2], (float)varepsilon, set_varepsilon);
      set_speed_temp = pid_position[2].pos_out;
    }

    // �ٶȻ�pid
    pid_calc(&pid_speed[1], (float)motor_chassis[1].speed_rpm, set_speed_temp);

    // ������
    // ʹ�õ��������ٶ���Ϊ����ֵ
    // pid_calc(&pid_position[1], (float)motor_chassis[1].total_angle, set_encoder);
    // pid_calc(&pid_speed[1], (float)motor_chassis[1].speed_rpm, pid_position[1].pos_out);
    // ���Ƶ��2���ٶ�
    // pid_calc(&pid_speed[1], (float)motor_chassis[1].speed_rpm, set_vel);

    // PID ���
    CAN_cmd_chassis((s16)(pid_speed[0].pos_out), (s16)(pid_speed[1].pos_out), 0, 0);

    // 500Hz
    HAL_Delay(2);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
float getAngle(void)
{
  int ENCODER_PPR = 2048;
  float angle;
  int iTimerEncoder = 0;
  iTimerEncoder = (short)(__HAL_TIM_GET_COUNTER(&htim1)); // ǿ��ת��Ϊshort���ֱ�����ת
  // ����Ƕ�
  angle = 360 / (float)ENCODER_PPR / 4 * (float)iTimerEncoder;
  //__HAL_TIM_SET_COUNTER(&htim1, 0);                        //����ʱ����0���������
  return angle;
}

float getRadian(void)
{
  return getAngle() * 3.1415926 / 180;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static int n = 0;
  static float varepsilon_data[4] = {0, 0, 0, 0};
  if (htim == &htim4)
  {
    // 1khz trigger
    // ��Ӧ��������250hz
    float vel_r = motor_chassis[0].speed_rpm / k_vel;

    encoder = getTimEncoder();
    angle = getAngle();
    radian = getRadian();
    if (last_radian != 0)
    {
      radian = radian * 0.8f + last_radian * 0.2f;
      varepsilon = (radian - last_radian) / 0.004f;
    }
    last_radian = radian;
    n++;

    // ���ݼ��ٶȼ����ٶ�
    if (!vel_flag)
    {
      vel_r = vel_r + set_a * 0.004f;
      set_vel = vel_r * k_vel;
    }
  }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
