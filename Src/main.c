/*
* This file is part of the stmbl project.
*
* Copyright (C) 2013-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2013-2018 Nico Stute <crinq@crinq.de>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "hd44780.h"

void SystemClock_Config(void);
TIM_HandleTypeDef htim3;

LCD_PCF8574_HandleTypeDef lcd;
extern I2C_HandleTypeDef hi2c2;
extern uint32_t rc_delay;
extern uint32_t temp_delay;
extern uint8_t ppm_count;

int cmd1;
int cmd2;
int cmd3;

int steer;
int speed;

extern TIM_HandleTypeDef htim_right;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern volatile adc_buf_t adc_buffer;


extern volatile int pwmr;
extern volatile int speedr;

volatile int pwmrl = 0;

extern uint8_t enable;

float voltage = 0;

uint16_t set_frequency = 0;
uint16_t is_frequency = 0;
uint8_t started = 0;

int main(void) {
  HAL_Init();
  __HAL_RCC_AFIO_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  SystemClock_Config();

  HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);

  __HAL_RCC_DMA1_CLK_DISABLE();
  MX_GPIO_Init();
  MX_TIM_Init();
  ASYNC_Init();
  MX_ADC1_Init();
  //MX_ADC2_Init();
  UART_Init();
  I2C_Init();
  PPM_Init();


  lcd.pcf8574.PCF_I2C_ADDRESS = 0x27;
	lcd.pcf8574.PCF_I2C_TIMEOUT = 5;
	lcd.pcf8574.i2c = hi2c2;
	lcd.NUMBER_OF_LINES = NUMBER_OF_LINES_2;
	lcd.type = TYPE0;

	if(LCD_Init(&lcd)!=LCD_OK){
		// error occured
		//TODO while(1);
	}

	LCD_ClearDisplay(&lcd);
  LCD_SetLocation(&lcd, 0, 0);
	LCD_WriteString(&lcd, "TurboOtter V2.0");
  LCD_SetLocation(&lcd, 0, 1);
  LCD_WriteString(&lcd, "Initializing...");


  HAL_ADC_Start(&hadc1);
  //HAL_ADC_Start(&hadc2);

  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);

  enable = 1;
  pwmr = 200;


  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM8_CLK_ENABLE();

  HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 0);

  int processcounter = 0;
  set_frequency = 300;
  LCD_ClearDisplay(&lcd);
  while(1) {
    if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == 0) {
      while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == 0) {}
      started = !started;
      HAL_Delay(500);
    }

    processcounter++;
    if (started == 0) {
      enable = 0;
    }
    if (processcounter > 5 && started == 1) { // do the frequency and voltage measurement
      processcounter = 0;
      enable = 0;
      HAL_Delay(3);
      ppm_count = 0;
      temp_delay = 0;
      HAL_NVIC_EnableIRQ(EXTI3_IRQn); // enable capture interrupt
      HAL_Delay(130); // wait for measurement
      is_frequency = 1000000 / rc_delay;
      HAL_NVIC_DisableIRQ(EXTI3_IRQn); // disable capture interrupt
      enable = 1;
    }

    // ############ STATE MASCHINE FOR START #################


    if (is_frequency < 180) { // we start with a freerunning set frequncy of 300 Hz
      //set_frequency += 5;
      set_frequency = 300;
    }

    if (is_frequency < 400) { // at this point of ramping up speed we start with a lower voltage of ~130V to minimize losses in the TMP
      //set_frequency += 5;
      pwmr = 260;
    }

    if (is_frequency > 400) { // at higher speeds, we need more voltage to keep up. We increse voltage with speed to about 200V max.
      //set_frequency += 5;
      pwmr = MIN(((is_frequency - 400) * 0.8f) + 260, 325);
    }

    if (is_frequency > 180 && is_frequency < 1200) {  // once the TMP has some momentum, we increse the set frequency as a fixed percentage of the is frequency. Slippage is 35%
      //set_frequency += 5;
      set_frequency = is_frequency * 1.35f;
    }

    if (is_frequency > 1200) {
      //set_frequency += 5;
      set_frequency = is_frequency * 1.15f;
    }

    if (is_frequency > 1250) { // once the TMP has full speed, we reduce the torque. Slippage is 10%
      //set_frequency += 5;
      set_frequency = is_frequency * 1.1f;
    }

    TIM3->ARR = 166666 / set_frequency;
    HAL_Delay(70);

    HAL_ADC_Start(&hadc1);

    HAL_ADC_PollForConversion(&hadc1, 100);

    int adcResult1 = HAL_ADC_GetValue(&hadc1); //PA0
    int adcResult2 = HAL_ADC_GetValue(&hadc1); //PA1

    voltage = adcResult1 / 15.65;

    HAL_ADC_Stop(&hadc1);

    refreshDisplay();

    //setScopeChannel(2, (int)adcResult1);
    //setScopeChannel(3, (int)adcResult2);


    // ####### LOG TO CONSOLE #######
    //consoleScope();
  }
}


int displaycounter = 0;
void refreshDisplay() {
  displaycounter++;

  if (displaycounter % 20 > 10) {
    //LCD_ClearDisplay(&lcd);
    LCD_SetLocation(&lcd, 0, 0);
  	LCD_WriteString(&lcd, "Voltage: ");
    LCD_SetLocation(&lcd, 10, 0);
    LCD_WriteString(&lcd, "      ");
    LCD_SetLocation(&lcd, 10, 0);
    LCD_WriteFloat(&lcd,voltage,0);
    LCD_SetLocation(&lcd, 14, 0);
    LCD_WriteString(&lcd, "V");
  } else {
    LCD_SetLocation(&lcd, 0, 0);
  	LCD_WriteString(&lcd, "Set Freq: ");
    LCD_SetLocation(&lcd, 10, 0);
    LCD_WriteString(&lcd, "    ");
    LCD_SetLocation(&lcd, 10, 0);
    LCD_WriteFloat(&lcd,set_frequency,0);
    LCD_SetLocation(&lcd, 14, 0);
    LCD_WriteString(&lcd, "Hz");
  }

  if (started == 1) {
    LCD_SetLocation(&lcd, 0, 1);
    LCD_WriteString(&lcd, "Is Freq: ");
    LCD_SetLocation(&lcd, 10, 1);
    LCD_WriteString(&lcd, "    ");
    LCD_SetLocation(&lcd, 10, 1);
    LCD_WriteFloat(&lcd,is_frequency,0);
    LCD_SetLocation(&lcd, 14, 1);
    LCD_WriteString(&lcd, "Hz");
  } else {
    LCD_SetLocation(&lcd, 0, 1);
    LCD_WriteString(&lcd, "                ");
    LCD_SetLocation(&lcd, 0, 1);
    LCD_WriteString(&lcd, "Stopped!");
  }
}

void ASYNC_Init() { // timer that generates the rotating field
  __HAL_RCC_TIM3_CLK_ENABLE();
  htim3.Instance = TIM3;
  htim3.Init.Period = 500;
  htim3.Init.Prescaler = (SystemCoreClock/DELAY_TIM_FREQUENCY_US)-1;;
  htim3.Init.ClockDivision = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  HAL_TIM_Base_Init(&htim3);
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  HAL_TIM_Base_Start_IT(&htim3);
}

uint16_t secondCounter = 0;

void TIM3_IRQHandler(void)
{
  if(enable == 0) {
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
    HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);
  } else {
    RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
    HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 0);
  }
  setPWM();
  HAL_TIM_IRQHandler(&htim3);
}


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);


    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
