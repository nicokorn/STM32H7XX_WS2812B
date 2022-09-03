// ****************************************************************************
/// \file      main.c
///
/// \brief     Main C Source File
///
/// \details   This is an example code of controlling an ws2812b led stripe, 
///            with 10 leds thus the used library is configured as 1 row with 10 
///            cols. You can change row and col in the ws2812b header file.
///            You can connect up to 16 led stripes. Data is written in
///            parallel to the stripes from a GPIO Bank (GPIO A in this example)
///            This is why up to 16 stripes can be controlled in parallel.
///            A Timer is used in which 3 DMA transfer are triggered used to 
///            write data to the gpio's on which the stripes are connected to.
///            This 3 DMA transfer are triggered as following:
///            First trigger is on each period. It set all gpios to high.
///            Second trigger is on the first capture compare event on the 8th
///            tick/pulse. The GPIOS are set accordingly if the bit for the
///            ws2812b shall be a 1 or a 0. 
///            The third trigger is the second capture compare event an sets
///            all gpio's always to 0 through a dma transfer. It doesn't matter
///            if the pins are already set to 0 by the first capture compare
///            event.
///            Please read the ws2812b datasheet to understand the communication
///            protocol with the ws2812b led chips.
///            This example is programmed in the IAR Embedded Workbench IDE for
///            a Nucleo STM32H743 Board. 
///            But you can use this library for any other IDE or stm32 
///            microcontroller. Just be sure to set the correct DMA
///            streams/channels, otherwise it won't work.
///
/// \author    Nico Korn
///
/// \version   1.0.0.0
///
/// \date      03092022
/// 
/// \copyright Copyright (c) 2022 Nico Korn
/// 
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
/// 
/// The above copyright notice and this permission notice shall be included in all
/// copies or substantial portions of the Software.
/// 
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
/// SOFTWARE.
///
/// \pre       
///
/// \bug       
///
/// \warning   
///
/// \todo      
///
// ****************************************************************************

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "ws2812b.h"
#include <stdio.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
     
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void colorWheelPlus( uint8_t *red, uint8_t *green, uint8_t *blue );

/* Private user code ---------------------------------------------------------*/
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
   uint16_t cnt=0;
   uint8_t rgb_r=0xff;
   uint8_t rgb_g=0x00;
   uint8_t rgb_b=0x00;

   /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
   HAL_Init();
   
   /* Configure the system clock */
   SystemClock_Config();

   /* init peripherals for using the ws2812b leds */
   WS2812B_init();
   
   while (1)
   {
      WS2812B_clearBuffer();
      colorWheelPlus(&rgb_r, &rgb_g, &rgb_b);
      WS2812B_setPixel( 0, ++cnt%COL, rgb_r, rgb_g, rgb_b );
      WS2812B_sendBuffer();
      HAL_Delay(100);
   }
}

// ----------------------------------------------------------------------------
/// \brief     Led color wheel function
///
/// \param     [in]  uint8_t *red
/// \param     [in]  uint8_t *green
/// \param     [in]  uint8_t *blue
///
/// \return    uint16_t ticks
static void colorWheelPlus( uint8_t *red, uint8_t *green, uint8_t *blue )
{
	//set next colors
	if(*green == 0x00 && *red < 0xff && *blue >= 0x00){
		*red += 0x01;
		*green = 0x00;
		*blue -= 0x01;
	}else if(*green < 0xff && *red >= 0x00 && *blue == 0x00){
		*red -= 0x01;
		*green += 0x01;
		*blue = 0x00;
	}else if(*green >= 0x00 && *red == 0x00 && *blue < 0xff){
		*red = 0x00;
		*green -= 0x01;
		*blue += 0x01;
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 12;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
