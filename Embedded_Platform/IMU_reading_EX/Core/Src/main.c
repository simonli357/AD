/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>

#include "bno055.h"
#include "bno_config.h"

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
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
bno055_t bno;
error_bno err;
s8 temperature = 0;
// f32 acc_x = 0.0f, acc_y = 0.0f, acc_z = 0.0f;
bno055_vec3_t acc = {0, 0, 0};
bno055_vec3_t lia = {0, 0, 0};
bno055_vec3_t gyr = {0, 0, 0};
bno055_vec3_t mag = {0, 0, 0};
bno055_vec3_t grv = {0, 0, 0};
bno055_euler_t eul = {0, 0, 0};
bno055_vec4_t qua = {0, 0, 0};
float roll_init = 0;
float pitch_init = 0;
float yaw_init = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
//#ifdef __GNUC__
//#define PUTCHAR_PROTOTYPE int __io_putchat(int ch)
//#else
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif
//
//PUTCHAR_PROTOTYPE{
//	HAL_UART_Transmit(&huart2,(uint8_t *)&ch,1, HAL_MAX_DELAY);
//	return ch;
//}
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
  /* USER CODE BEGIN 2 */
  printf("Hello\n");
  bno = (bno055_t) {
	.i2c = &hi2c1, .addr = BNO_ADDR, .mode = BNO_MODE_NDOF, ._temp_unit = 0,
	        // .ptr = &bno,
	    };
  HAL_Delay(1000);
  if ((err = bno055_init(&bno)) == BNO_OK) {
          printf("[+] BNO055 init success\r\n");
          HAL_Delay(100);
      } else {
          printf("[!] BNO055 init failed\r\n");
          printf("%s\n", bno055_err_str(err));
          Error_Handler();
      }
  HAL_Delay(100);
  err = bno055_set_unit(&bno, BNO_TEMP_UNIT_C, BNO_GYR_UNIT_DPS,
						BNO_ACC_UNITSEL_M_S2, BNO_EUL_UNIT_DEG);
  if (err != BNO_OK) {
	  printf("[BNO] Failed to set units. Err: %d\r\n", err);
  } else {
	  printf("[BNO] Unit selection success\r\n");
  }
  HAL_Delay(1000);


//  Calibrate_BNO055();

//
//      bno.euler(&bno, &eul);
//      roll_init = eul.roll;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//	  printf("hello \n");
//	  bno.temperature(&bno, &temperature);
		  bno.acc(&bno, &acc);
		  bno.linear_acc(&bno, &lia);
//		  bno.gyro(&bno, &gyr);
//		  bno.mag(&bno, &mag);
//		  bno.gravity(&bno, &grv);
		  bno.euler(&bno, &eul);
//		  bno.quaternion(&bno, &qua);
//	  	          printf("[+] Temperature: %2d°C\r\n", temperature);
//	  	          printf("[+] ACC - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", acc.x, acc.y,
//	  	                 acc.z);
//	  	          printf("[+] LIA - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", lia.x, lia.y,
//	  	                 lia.z);
//	  	          printf("[+] GYR - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", gyr.x, gyr.y,
//	  	                 gyr.z);
//	  	          printf("[+] MAG - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", mag.x, mag.y,
//	  	                 mag.z);
//	  	          printf("[+] GRV - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", grv.x, grv.y,
//	  	                 grv.z);
//	  	          printf("[+] Roll: %+2.2f | Pitch: %+2.2f | Yaw: %+2.2f\r\n", eul.roll,
//	  	                 eul.pitch, eul.yaw);
//	  	          printf("[+] QUA - w: %+2.2f | x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n",
//	  	                 qua.w, qua.x, qua.y, qua.z);


		  printf("--------------------------");
//	  	          printf("Temperature,%2d\n", temperature);
		  printf("ACC,%+2.2f,%+2.2f,%+2.2f\n", acc.x, acc.y,
				 acc.z);
		  printf("LIA,%+2.2f,%+2.2f,%+2.2f\n", lia.x, lia.y,
				 lia.z);
//				  printf("GYR,%+2.2f,%+2.2f,%+2.2f\n", gyr.x, gyr.y,
//						 gyr.z);
//				  printf("MAG,%+2.2f,%+2.2f,%+2.2f\n", mag.x, mag.y,
//						 mag.z);
//				  printf("GRV,%+2.2f,%+2.2f,%+2.2f\n", grv.x, grv.y,
//						 grv.z);
		  printf("Roll,%+2.2f,%+2.2f,%+2.2f\n", eul.roll - roll_init,
				 eul.pitch - pitch_init, eul.yaw - yaw_init);
//				  printf("QUA,%+2.2f,%+2.2f,%+2.2f,%+2.2f\n",
//						 qua.w, qua.x, qua.y, qua.z);


//	  	            // Write data to file in the specified format
//	  	            fprintf(file, "[+] Temperature: %2d°C\r\n", temperature);
//	  	            fprintf(file, "[+] ACC - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", acc.x, acc.y, acc.z);
//	  	            fprintf(file, "[+] LIA - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", lia.x, lia.y, lia.z);
//	  	            fprintf(file, "[+] GYR - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", gyr.x, gyr.y, gyr.z);
//	  	            fprintf(file, "[+] MAG - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", mag.x, mag.y, mag.z);
//	  	            fprintf(file, "[+] GRV - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", grv.x, grv.y, grv.z);
//	  	            fprintf(file, "[+] Roll: %+2.2f | Pitch: %+2.2f | Yaw: %+2.2f\r\n", eul.roll, eul.pitch, eul.yaw);
//	  	            fprintf(file, "[+] QUA - w: %+2.2f | x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", qua.w, qua.x, qua.y, qua.z);

			// Close the file

//	  	          HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED);
		  HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		fclose(file);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00100D14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : mybutton_Pin */
  GPIO_InitStruct.Pin = mybutton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(mybutton_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Calibrate_BNO055(void) {

		Calib_status_t calib={0};
        printf("Calibrating BNO055 sensor...\n");

//         Set operation mode to FUSION_MODE or appropriate mode for calibration
//        &bno->mode = 0x0C; //BNO055_OPR_MODE_NDOF 0x0C
//        err = bno055_set_opmode(&bno, &bno->mode);
//        if (err!= BNO_OK){
//        	printf("set normal node failed\n");
//        }
    	HAL_Delay(100);
        // Gyroscope calibration
        printf("Calibrating gyroscope...\n");
        printf("Place the device in a single stable position\n");
        HAL_Delay(1000);  // Simulated gyroscope calibration time

        do {
            getCalibration(&calib);
        	HAL_Delay(500);
		} while (calib.Gyro !=3);
        printf("Gyroscope calibration complete.\n");

        // Accelerometer calibration
        printf("Calibrating accelerometer...\n");
        printf("Place the device in 6 different stable positions\n");
        for (int i = 0; i < 6; i++) {
            printf("Position %d\n", i + 1);
            HAL_Delay(10000);  // Simulated accelerometer calibration time
        }

        do {
            getCalibration(&calib);
        	HAL_Delay(500);
		} while (calib.Acc !=3);
        printf("Accelerometer calibration complete.\n");

        // Magnetometer calibration
        printf("Calibrating magnetometer...\n");
        printf("Make some random movements\n");
        HAL_Delay(1000);  // Simulated gyroscope calibration time

        do {
            getCalibration(&calib);
        	HAL_Delay(500);
		} while (calib.MAG !=3);
        printf("Magnetometer calibration complete.\n");

        // System calibration
        printf("Calibrating system...\n");
        printf("Keep the device stationary until system calibration reaches level 3\n");
        do {
            getCalibration(&calib);
        	HAL_Delay(500);
		} while (calib.System !=3);
        HAL_Delay(500);

        // Check calibration status

        printf("Sensor is fully calibrated.\n");

        printf("System: %d      Gyro: %d       Accel: %d       MAG: %d\n",calib.System,calib.Gyro , calib.Acc, calib.MAG);

}

void getCalibration(Calib_status_t *calib) {
    uint8_t calData;

    // Read calibration status register using I2C
//    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, BNO_ADDR, BNO_CALIB_STAT, 1, &calData, 1, HAL_MAX_DELAY); //BNO055_CALIB_STAT_REG 0x35
    err = bno055_read_regs(bno,BNO_CALIB_STAT,&calData,1);
    // Check if read was successful
    if (err == BNO_OK) {

        // Extract calibration status values

        	calib->System= (calData >> 6) & 0x03;


        	calib->Gyro = (calData >> 4) & 0x03;


        	calib->Acc = (calData >> 2) & 0x03;


        	calib->MAG = calData & 0x03;

    } else {
        printf("Failed to read calibration status register.\n");
    }
}

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin){
	if(GPIO_Pin == mybutton_Pin){
		roll_init = eul.roll;
		pitch_init = eul.pitch;
		yaw_init = eul.yaw;
		printf("euler reset\n");
	}
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
