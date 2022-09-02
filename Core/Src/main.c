/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Delay.h"
#include "Print.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
typedef struct __PID_Increment_Struct
{
    float Kp, Ki, Kd;  //系数
    float Error_Last1; //上次误差
    float Error_Last2; //上次误差
    float Out_Last;    //上次输出
} PID_Increment_Struct;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define Motor_TIM_Handle htim1
#define Motor_TIM_Channel1 TIM_CHANNEL_1
#define Motor_TIM_Channel2 TIM_CHANNEL_2
#define Motor_MAX_Duty 5000
#define Encoder_TIM_Handle htim2
PID_Increment_Struct PID_Speed = {80, 5};
PID_Increment_Struct PID_Angle = {3, 0, 0};
float Get_Speed()
{
    int16_t zj;
    float Speed = 0;
    zj = __HAL_TIM_GetCounter(&Encoder_TIM_Handle);
    __HAL_TIM_SetCounter(&Encoder_TIM_Handle, 0);
    Speed = (float)zj / (4 * 15 * 34) * 100 * 60;

    return Speed;
}
float Get_Angle()
{
    int16_t zj;
    float angle = 0;
    zj = __HAL_TIM_GetCounter(&Encoder_TIM_Handle);
    angle = (float)zj / (4 * 15 * 34) * 360;
    return angle;
}
float PID_Increment(PID_Increment_Struct *PID, float Current, float Target)
{
    float err,                                                                                                       //误差
        out,                                                                                                         //输出
        proportion,                                                                                                  //比例
        differential;                                                                                                //微分
    err = (float)Target - (float)Current;                                                                            //计算误差
    proportion = (float)err - (float)PID->Error_Last1;                                                               //计算比例项
    differential = (float)err - 2 * (float)PID->Error_Last1 + (float)PID->Error_Last2;                               //计算微分项
    out = (float)PID->Out_Last + (float)PID->Kp * proportion + (float)PID->Ki * err + (float)PID->Kd * differential; //计算PID
    PID->Error_Last2 = PID->Error_Last1;                                                                             //更新上上次误差
    PID->Error_Last1 = err;                                                                                          //更新误差
    PID->Out_Last = out;                                                                                             //更新上此输出
    return out;
}
void motor(int16_t Speed)
{
    if (Speed == 0)
    {
        __HAL_TIM_SET_COMPARE(&Motor_TIM_Handle, Motor_TIM_Channel1, Motor_MAX_Duty + 1);
        __HAL_TIM_SET_COMPARE(&Motor_TIM_Handle, Motor_TIM_Channel2, Motor_MAX_Duty + 1);
    }
    else if (Speed > 0)
    {
        __HAL_TIM_SET_COMPARE(&Motor_TIM_Handle, Motor_TIM_Channel1, Speed);
        __HAL_TIM_SET_COMPARE(&Motor_TIM_Handle, Motor_TIM_Channel2, 0);
    }
    else if (Speed < 0)
    {
        Speed *= -1;
        __HAL_TIM_SET_COMPARE(&Motor_TIM_Handle, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&Motor_TIM_Handle, TIM_CHANNEL_2, Speed);
    }
}

float angle;
float mb_speed_last;
int aa = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    float Speed = 0;
    int16_t set_speed = 0;
    float mb_speed;
    if (htim == &htim2)
    {
    }
    else if (htim == &htim3)
    {
        angle += Get_Angle();
        Speed = Get_Speed();
        mb_speed = (int16_t)PID_Increment(&PID_Angle, angle, aa);

        if (PID_Angle.Error_Last1 > 360)
            mb_speed = 300;
        else if (PID_Angle.Error_Last1 < -360)
            mb_speed = -300;
        set_speed = PID_Increment(&PID_Speed, Speed, mb_speed);

        if (set_speed > 5000)
            set_speed = 5000;
        else if (set_speed < -5000)
            set_speed = -5000;

        motor(set_speed);
        // printf("%f,%f\r\n", Speed, mb_speed);
        Speed = aa;
        printf("%f,%f\r\n", angle, Speed);
    }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */
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
    MX_USART1_UART_Init();
    MX_TIM9_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        aa = 90;
        HAL_Delay(2000);
        aa = -90;
        HAL_Delay(2000);
        // aa = -1000;
        // HAL_Delay(2000);
        // motor(-5000);
        // HAL_Delay(2000);
        // motor(3000);
        // HAL_Delay(2000);
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
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
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
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
