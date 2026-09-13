/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Complete fixed version)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>
#include <stdint.h>

#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

#include "VL53L1X_api.h"
#include "VL53L1X_calibration.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MLX90640_ADDRESS    0x33
#define VL53L1X_ADDRESS     0x29

#define FIRE_TEMPERATURE   150.0f

#define SERVO2_MAX_ANGLE    25
#define SERVO2_MIN_ANGLE    0

#define SERVO_SCAN_STEP     10

#define PUMP_PORT   GPIOA
#define PUMP_PIN    GPIO_PIN_1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint16_t eeMLX90640[832];
static paramsMLX90640 mlx90640Params;
static uint16_t mlx90640Frame[834];
static float mlx90640To[768];

static uint16_t currentDistance = 0;

volatile float temperature_vdd = 0.0f;
volatile float temperature_ta = 0.0f;
volatile float temperature_tr = 0.0f;
volatile float temperature_max = 0.0f;
volatile int temperature_hottest_x = 0;
volatile int temperature_hottest_y = 0;

volatile uint8_t vl53Ready = 0;
volatile uint16_t distanceMM = 0;

volatile int mlxDumpStatus = -1;
volatile int mlxExtractStatus = -1;
volatile int mlxRefreshStatus = -1;
volatile int mlxChessStatus = -1;

volatile int vl53InitStatus = -1;
volatile int vl53StartStatus = -1;
volatile uint8_t vl53XSHUT_State;
volatile uint8_t fire_found = 0;
volatile int servo2_angle = SERVO2_MAX_ANGLE;
volatile uint8_t diagnostic_mode = 0;
volatile uint32_t last_button_press = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void MLX90640_UpdateTemperature(void);

static void VL53L1X_UpdateDistance(void);

static void Servo_SetAngle(uint32_t angle);
static void Servo2_SetAngle(uint32_t angle);
static void Servo2_NextStep(void);
static void Target_And_Extinguish(int *p_angle);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Servo_SetAngle(uint32_t angle)
{
    if (angle > 180)
    {
        angle = 180;
    }

    uint32_t pulse = 500 + (angle * 2000) / 180;

    __HAL_TIM_SET_COMPARE(
        &htim1,
        TIM_CHANNEL_1,
        pulse
    );
}

static void Servo2_SetAngle(uint32_t angle)
{
    if (angle > SERVO2_MAX_ANGLE)
    {
        angle = SERVO2_MAX_ANGLE;
    }

    uint32_t pulse = 500 + (angle * 2000) / 180;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse);
}

static void Servo2_NextStep(void)
{
    if (servo2_angle == 25)
    {
        servo2_angle = 20;
    }
    else if (servo2_angle == 20)
    {
        servo2_angle = 10;
    }
    else if (servo2_angle == 10)
    {
        servo2_angle = 0;
    }
    else
    {
        servo2_angle = 25;
    }

    Servo2_SetAngle((uint32_t)servo2_angle);
    HAL_Delay(500);
}
static void MLX90640_UpdateTemperature(void)
{
    int frameStatus;

    float max_temp = -1000.0f;
    float sum_temp = 0.0f;

    int hottest_pixel = 0;
    int valid_pixels = 0;

    for (int subpage = 0; subpage < 2; subpage++)
    {
        frameStatus = MLX90640_GetFrameData(
            MLX90640_ADDRESS,
            mlx90640Frame
        );

        if (frameStatus < 0)
        {
            return;
        }

        temperature_vdd =
            MLX90640_GetVdd(
                mlx90640Frame,
                &mlx90640Params
            );

        temperature_ta =
            MLX90640_GetTa(
                mlx90640Frame,
                &mlx90640Params
            );

        temperature_tr =
            temperature_ta - 8.0f;

        MLX90640_CalculateTo(
            mlx90640Frame,
            &mlx90640Params,
            0.95f,
            temperature_tr,
            mlx90640To
        );
    }
    for (int i = 0; i < 768; i++)
    {
        float t = mlx90640To[i];

        if (!isfinite(t))
        {
            continue;
        }

        if (t > max_temp)
        {
            max_temp = t;
            hottest_pixel = i;
        }

        sum_temp += t;
        valid_pixels++;
    }

    if (valid_pixels == 0)
    {
        return;
    }

    temperature_max = max_temp;

    temperature_hottest_x =
        hottest_pixel % 32;

    temperature_hottest_y =
        hottest_pixel / 32;

    if (temperature_max > FIRE_TEMPERATURE)
    {
        fire_found = 1;
    }
}
static void VL53L1X_UpdateDistance(void)
{
    uint8_t dataReady = 0;

    uint16_t distance = 0;
    if (vl53InitStatus != 0)
    {
        vl53Ready = 0;

        return;
    }

    if (VL53L1X_CheckForDataReady(
            VL53L1X_ADDRESS,
            &dataReady
        ) != 0)
    {
        vl53Ready = 0;

        return;
    }

    if (dataReady)
    {
        if (VL53L1X_GetDistance(
                VL53L1X_ADDRESS,
                &distance
            ) == 0)
        {

            currentDistance = distance;

            distanceMM = distance;

            vl53Ready = 1;
            VL53L1X_ClearInterrupt(
                VL53L1X_ADDRESS
            );
        }
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
  MX_I2C1_Init();
  MX_TIM1_Init();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  /* USER CODE BEGIN 2 */

  vl53Ready = 0;
    for (int attempt = 0; attempt < 5; attempt++) {
        vl53InitStatus = VL53L1X_SensorInit(VL53L1X_ADDRESS);
        if (vl53InitStatus == 0) {
            break;
        }
        HAL_Delay(20);
    }

    if (vl53InitStatus == 0)
    {
        vl53StartStatus = VL53L1X_StartRanging(VL53L1X_ADDRESS);
        if (vl53StartStatus == 0)
        {
            vl53Ready = 1;
        }
    }
      MLX90640_I2CInit();
      MLX90640_I2CGeneralReset();

      HAL_Delay(80);

      mlxRefreshStatus = MLX90640_SetRefreshRate(
          MLX90640_ADDRESS,
          0x03
      );

      if (mlxRefreshStatus != 0)
      {
          Error_Handler();
      }

      mlxChessStatus = MLX90640_SetChessMode(
          MLX90640_ADDRESS
      );

      if (mlxChessStatus != 0)
      {
          Error_Handler();
      }

      mlxDumpStatus = MLX90640_DumpEE(
          MLX90640_ADDRESS,
          eeMLX90640
      );

      if (mlxDumpStatus != 0)
      {
          Error_Handler();
      }

      mlxExtractStatus = MLX90640_ExtractParameters(
          eeMLX90640,
          &mlx90640Params
      );

      if (mlxExtractStatus != 0)
      {
          Error_Handler();
      }

      HAL_Delay(100);

      vl53Ready = 0;

      for (int attempt = 0; attempt < 5; attempt++) {
    	  vl53InitStatus = VL53L1X_SensorInit(VL53L1X_ADDRESS);
          if (vl53InitStatus == 0) {
        	  break;
          }
          HAL_Delay(20);
      }

      if (vl53InitStatus == 0)
      {
          vl53StartStatus = VL53L1X_StartRanging(
        		  VL53L1X_ADDRESS
          );

          if (vl53StartStatus == 0)
          {
        	  vl53Ready = 1;
          }
      }
  /* USER CODE END 2 */

  /* Infinite loop */
      /* USER CODE BEGIN WHILE */
      servo2_angle = 25;
      Servo2_SetAngle(25);
      while (1)
	  {
			  fire_found = 0;
			  for (int angle = 0; angle <= 80; angle += SERVO_SCAN_STEP)
			  {
				  Servo_SetAngle((uint32_t)angle);
				  HAL_Delay(500);

				  MLX90640_UpdateTemperature();
				  VL53L1X_UpdateDistance();

				  if (fire_found)
				  {
					  Target_And_Extinguish(&angle);
				  }
			  }

			  Servo2_NextStep();

			  for (int angle = 80; angle >= 0; angle -= SERVO_SCAN_STEP)
			  {
				  Servo_SetAngle((uint32_t)angle);
				  HAL_Delay(500);

				  MLX90640_UpdateTemperature();
				  VL53L1X_UpdateDistance();

				  if (fire_found)
				  {
					  Target_And_Extinguish(&angle);
				  }
			  }

              Servo2_NextStep();
	  }
     /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13)
    {
        uint32_t current_time = HAL_GetTick();

        if (current_time - last_button_press > 300)
        {
            diagnostic_mode = !diagnostic_mode;
            last_button_press = current_time;
        }
    }
}
/* USER CODE BEGIN 4 */

static void Target_And_Extinguish(int *p_angle)
{
    int angle = *p_angle;
    uint8_t extinguish_attempts = 0;

    Servo_SetAngle((uint32_t)angle);
    Servo2_SetAngle((uint32_t)servo2_angle);

    while (fire_found)
    {
        int sum_x = 0;
        int sum_y = 0;

        for (int i = 0; i < 2; i++)
        {
            MLX90640_UpdateTemperature();
            sum_x += temperature_hottest_x;
            sum_y += temperature_hottest_y;
            HAL_Delay(20);
        }

        int avg_x = sum_x / 2;
        int avg_y = sum_y / 2;

        if (temperature_max < FIRE_TEMPERATURE)
        {
            fire_found = 0;
            break;
        }

        uint8_t centered_x = 0;
        uint8_t centered_y = 0;

        if (avg_x < 8)
        {
            angle += 1;
            if (angle < 0) angle = 0;
        }
        else if (avg_x > 23)
        {
            angle -= 1;
            if (angle > 180) angle = 180;
        }
        else
        {
            centered_x = 1;
        }

        if (avg_y < 7)
        {
            servo2_angle -= 1;
            if (servo2_angle < 0) servo2_angle = 0;
        }
        else if (avg_y > 16)
        {
            servo2_angle += 1;
            if (servo2_angle > SERVO2_MAX_ANGLE) servo2_angle = SERVO2_MAX_ANGLE;
        }
        else
        {
            centered_y = 1;
        }

        if (centered_x == 0 || centered_y == 0)
        {

            Servo_SetAngle((uint32_t)angle);
            Servo2_SetAngle((uint32_t)servo2_angle);
            HAL_Delay(120);
        }
        else
        {
            VL53L1X_UpdateDistance();

            int offset_x = -1;
            int offset_y = 0;

            if (distanceMM <= 100)
            {
                offset_y = 0;
            }
            else if (distanceMM <= 150)
            {
                offset_y = -4;
            }
            else
            {
                offset_y = 0;
            }

            if (extinguish_attempts >= 1)
            {
                offset_y -= 2;
            }

            int camera_angle_x = angle;
            int camera_angle_y = servo2_angle;

            int target_angle = angle + offset_x;
            int target_servo2 = servo2_angle + offset_y;

            if (target_angle > 180) target_angle = 180;
            if (target_angle < 0) target_angle = 0;
            if (target_servo2 < 0) target_servo2 = 0;
            if (target_servo2 > SERVO2_MAX_ANGLE) target_servo2 = SERVO2_MAX_ANGLE;

            Servo_SetAngle((uint32_t)target_angle);
            Servo2_SetAngle((uint32_t)target_servo2);
            HAL_Delay(200);

            if (diagnostic_mode == 0)
            {
                HAL_GPIO_WritePin(PUMP_PORT, PUMP_PIN, GPIO_PIN_SET);
                for (int sweep = 0; sweep < 3; sweep++)
                {
                    Servo_SetAngle((uint32_t)(target_angle - 2));
                    HAL_Delay(400);
                    Servo_SetAngle((uint32_t)(target_angle + 2));
                    HAL_Delay(400);
                }

                HAL_GPIO_WritePin(PUMP_PORT, PUMP_PIN, GPIO_PIN_RESET);
                HAL_Delay(800);
            }
            else
            {
                HAL_Delay(2500);
            }

            extinguish_attempts++;

            angle = camera_angle_x;
            servo2_angle = camera_angle_y;
            Servo_SetAngle((uint32_t)angle);
            Servo2_SetAngle((uint32_t)servo2_angle);
            HAL_Delay(200);
        }
    }

    *p_angle = angle;
}

/* USER CODE END 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

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
