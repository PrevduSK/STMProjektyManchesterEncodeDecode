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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

TIM_HandleTypeDef * htim1_p, * htim2_p;
GPIO_PinState last_state_pin;//

const _Bool manchester_IEEE= true, MSB_Frst_E= true;
//_Bool * p_rising_Flag, * p_falling_Flag;
volatile _Bool  rising_Flag;
volatile _Bool  falling_Flag;
volatile _Bool  tim_count_reset_Flag;
 //manchester_IEEE= true//, log. 0 --> 10, 1 --> 01
						// manchester_IEEE= false, log. 0 --> 01, 1 --> 10
char uart_mass_buf[40];
volatile uint8_t  massage, count_bit= 0, count_edge= 0, comp_bit= 0; //"hello!"  massage = 'h',
volatile uint8_t mask_8;
volatile uint8_t clk = 0;
volatile uint16_t  manchester_mass;

volatile uint32_t tick_count_prim, tick_count_zero, tick_count_one;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_UCPD1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_MEMORYMAP_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

void manchester_send_masage_half_tick(uint8_t massage_bite);

void send_manchester_data(uint8_t  massage, uint8_t * clk);
void receive_manchester_data(uint8_t * massage_recived, uint8_t * clk);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//
void manchester_send_bite_half_tick(uint8_t massage_bite)
{
	clk = !clk; // log. 1
	// Manchester: as per IEEE 802.3
	// LSB firsth
	comp_bit =  (massage_bite & mask_8 )? 1: 0 ;
	if (manchester_IEEE){
	   if( (comp_bit ^ (clk & 0x01)) ) {
	    	manchester_mass |= 1;
	   }
	   else{
	    	manchester_mass |= 0;
	   }
	}
	else {
	   if( !(comp_bit ^ (clk & 0x01)) ) {
	    	manchester_mass |= 1;
	   }
	   else{
	    	manchester_mass |= 0;
	   }
	}


	if ( clk == 0){
	   if(MSB_Frst_E){ mask_8 >>= 1; }
	   else { mask_8 <<= 1; }
	}
	if (count_bit != 15) {
	    count_bit++;

	    manchester_mass <<= 1; // ked je vstup 11 vypise 00
	}
	return;
}



void send_manchester_data(uint8_t  massage, uint8_t * clk)
{
	_Bool manchester_IEEE= true, MSB_Frst_E= true;

	uint8_t count= 0, comp_bit= 0; //"hello!"  massage = 'h',
	uint8_t mask_8;
	if (MSB_Frst_E){ mask_8 = 0x80;}
	else { mask_8 = 0x01;}
	//uint8_t clk = 0;
	uint16_t  manchester_mass;

	while(1)
	{
		*clk = !*clk; // log. 1
		// Menchester: as per IEEE 802.3
		// LSB first
		comp_bit =  (massage & mask_8 )? 1: 0 ;
		if (manchester_IEEE){
			if( (comp_bit ^ (*clk & 0x01)) ) {
				manchester_mass |= 1;
			}
			else{
				manchester_mass |= 0;
			}
		}
		else {
			if( !(comp_bit ^ (*clk & 0x01)) ) {
				manchester_mass |= 1;
			}
			else{
				manchester_mass |= 0;
			}
		}


		if ( *clk == 0){
			if(MSB_Frst_E){ mask_8 >>= 1; }
			else { mask_8 <<= 1; }
		}
		if (count == 15) {
			break;
		}
		count++;

		manchester_mass <<= 1; // ked je vstup 11 vypise 00
		// posun
		/*  if ((manchester_mass & mask) == 1){
		  	  	  //HAL_GPIO_WritePin(Menches_Out_GPIO_Port, Menches_Out_Pin, GPIO_PIN_SET);
		  	  	  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
	  	  	  }
	  	  	  else {
		  	  	  //HAL_GPIO_WritePin(Menches_Out_GPIO_Port, Menches_Out_Pin, GPIO_PIN_RESET);
		  	  	  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
	  	  	  } */

		// namiesto delay pouzit potom timer na posielanie
		HAL_Delay(10); //20ms 50Hz; 1ms 1000Hz
		// cas medzi nabeznou a dobeznou hranou
		// ked su data 0 treba maerat cas medzi nabeznou a dobeznou hranou
	}
}

void receive_manchester_data(uint8_t * massage_recived, uint8_t * clk)
{
	_Bool manchester_IEEE= true, MSB_Frst_E= true;

		uint8_t count= 0, comp_bit= 0; //"hello!"  massage = 'h',
		uint8_t mask_8;
		if (MSB_Frst_E){ mask_8 = 0x80;}
		else { mask_8 = 0x01;}
		//uint8_t clk = 0;
		uint16_t  manchester_mass;


		//HAL_TIM_OC_Start_IT(&htim1, manchester_IEEE);

		while(1)
		{
		*clk = !*clk; // log. 1
		// Menchester: as per IEEE 802.3
		// LSB firsth
		comp_bit =  (*massage_recived & mask_8 )? 1: 0 ;
		if (manchester_IEEE){
			if( (comp_bit ^ (*clk & 0x01)) ) {
				manchester_mass |= 1;
			}
			else{
				manchester_mass |= 0;
			}
		}
		else {
			if( !(comp_bit ^ (*clk & 0x01)) ) {
				manchester_mass |= 1;
			}
			else{
				manchester_mass |= 0;
			}
		}

		if ( *clk == 0){
			if(MSB_Frst_E){ mask_8 >>= 1; }
			else { mask_8 <<= 1; }
		}
		if (count == 15) {
			break;
			}
		count++;

		manchester_mass <<= 1; // ked je vstup 11 vypise 00

		// posun
		/*  if ((manchester_mass & mask) == 1){
			  	//HAL_GPIO_WritePin(Menches_Out_GPIO_Port, Menches_Out_Pin, GPIO_PIN_SET);
			  	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
		  	}
		  	else {
			  	 //HAL_GPIO_WritePin(Menches_Out_GPIO_Port, Menches_Out_Pin, GPIO_PIN_RESET);
			  	 HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
		  	} */

			// namiesto delay pouzit potom timer na posielanie
			HAL_Delay(10); //20ms 50Hz; 1ms 1000Hz
			// cas medzi nabeznou a dobeznou hranou
			// ked su data 0 treba maerat cas medzi nabeznou a dobeznou hranou

			//nastavim timer na max periodu, pre citanie
		}
}


void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim)
{
	// are executed by set HAL_TIM_Base_Start_IT(&htim2);
	if (htim->Instance == TIM2)
	{
		tim_count_reset_Flag = true;
		//snprintf(uart_mass_buf, sizeof(uart_mass_buf), "Enterd to Period.\r\n");
		//HAL_UART_Transmit(&huart1, (uint8_t *) uart_mass_buf, sizeof(uart_mass_buf), 1);
	}
}

void HAL_TIM_OC_DelayElapsedCallback (TIM_HandleTypeDef * htim)
{
	// are executed by set HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
    if (htim->Instance == TIM1)
    {
    	//HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
    	clk = !clk; // log. 1
  		// Manchester: as per IEEE 802.3
    	// LSB firsth
    	comp_bit =  (massage & mask_8 )? 1: 0 ;
    	if (manchester_IEEE){
    		if( (comp_bit ^ (clk & 0x01)) ) {
    			manchester_mass |= 1;
    		}
    		else{
    			manchester_mass |= 0;
    		}
    	}
    	else {
    		if( !(comp_bit ^ (clk & 0x01)) ) {
    			manchester_mass |= 1;
    		}
    		else{
    			manchester_mass |= 0;
    		}
    	}


    	if ( clk == 0){
    		if(MSB_Frst_E){ mask_8 >>= 1; }
    		else { mask_8 <<= 1; }
    	}
    	if (count_bit != 15) {
    		count_bit++;

    		manchester_mass <<= 1; // ked je vstup 11 vypise 00
    	}

    }

    if (htim->Instance == TIM2)
    {
    	tim_count_reset_Flag = true;
    	//snprintf(uart_mass_buf, sizeof(uart_mass_buf), "Enterd to OC Delay.\r\n");
    	//HAL_UART_Transmit(&huart1, (uint8_t *) uart_mass_buf, sizeof(uart_mass_buf), 1);
    }
    // return;

}
/*
void HAL_EXTI_IRQHandler (EXTI_HandleTypeDef * hexti)
{
	if (hexti-> == TIM2_IRQn)
	    {

	    }
} */


void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == Manchaster_In_Pin)
	{
		//HAL_SuspendTick();
		if (count_edge >= 2)		// >= 2
		{	//tick_count_one = HAL_GetTick();  HAL_ResumeTick();
			rising_Flag = false;
			falling_Flag = true;
		}
		else 		// == 0, 1
		{	if (manchester_IEEE){
				tick_count_prim = __HAL_TIM_GET_COUNTER(&htim2); //HAL_GetTick();  HAL_ResumeTick();
			}
			else {
				HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1); //HAL_ResumeTick();
			}
			rising_Flag = false;
			falling_Flag = true;
		}
		count_edge++;
		//snprintf(uart_mass_buf, sizeof(uart_mass_buf), "%ld\r\n", tick_count_prim);
		//HAL_UART_Transmit(&huart1, (uint8_t *) uart_mass_buf, sizeof(uart_mass_buf), 1);
		//count++;
//-----------------TREBA prepísať snímanie prvého last state aby sa zachytil v preruseny
		//if ( count > 0 ) {}
		/*while ( HAL_GetTick() <= (uint32_t) (tick_count_prim/2) ) {}


		if ( tick_count_prim < tick_count_one  ) // && (tick_count_prim*2) >= tick_count_one
		{
			if(last_state_pin == GPIO_PIN_SET && count > 0)
			{ // read start val, Higl
				manchester_mass |= 11;
				manchester_mass <<= 2;
				count+=2;
				last_state_pin = HAL_GPIO_ReadPin(Manchaster_In_GPIO_Port, Manchaster_In_Pin);
				HAL_UART_Transmit(&huart1, (uint8_t *) "11", sizeof("11"), 1);
			//	if (count ==9){
			//		HAL_UART_Transmit(&huart1, (uint8_t *) "\t", sizeof("\t"), 1);
			//	}
			}
			tick_count_one = 0;
		}
		else // tick_count_prim >= tick_count_one  && tick_count_one >= 0
		{
			if(last_state_pin == GPIO_PIN_SET && count > 0)
			{ // read start val, Higl
				manchester_mass |= 1;
				manchester_mass <<= 1;
				count++;
				last_state_pin = HAL_GPIO_ReadPin(Manchaster_In_GPIO_Port, Manchaster_In_Pin);
				HAL_UART_Transmit(&huart1, (uint8_t *) "1", sizeof("1"), 1);
			}
			else if(HAL_GPIO_ReadPin(Manchaster_In_GPIO_Port, Manchaster_In_Pin) == GPIO_PIN_RESET && count == 0 )
			{ // read start val, Low
				manchester_mass |= 0;
				manchester_mass <<= 1;
				count++;
				last_state_pin = HAL_GPIO_ReadPin(Manchaster_In_GPIO_Port, Manchaster_In_Pin);
				HAL_UART_Transmit(&huart1, (uint8_t *) "0", sizeof("0"), 1);

			}
			tick_count_one = 0;
		} */
	}
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin){

	if (GPIO_Pin == Manchaster_In_Pin)
	{
		//HAL_SuspendTick();
		if (count_edge >= 2) 		// >= 2
		{	//tick_count_zero = HAL_GetTick();  HAL_ResumeTick();
			rising_Flag = true;
			falling_Flag = false;
		}
		else
		{												// == 0,1
			if (manchester_IEEE)
			{	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1); //HAL_ResumeTick();
			}
			else
			{	tick_count_prim = __HAL_TIM_GET_COUNTER(&htim2); //HAL_GetTick();  HAL_ResumeTick();
			}
			rising_Flag = true;
			falling_Flag = false;
		}
		count_edge++;
		//snprintf(uart_mass_buf, sizeof(uart_mass_buf), "%ld\r\n", tick_count_prim);
		//HAL_UART_Transmit(&huart1, (uint8_t *) uart_mass_buf, sizeof(uart_mass_buf), 1);
		//count++;

		/*while ( HAL_GetTick() <= (uint32_t) (tick_count_prim/2) ) {}


		if ( tick_count_prim < tick_count_zero ) //  && (tick_count_prim*2) >= tick_count_zero
		{
			if(last_state_pin == GPIO_PIN_RESET && count > 0 )
			{ // read start val, Low
				  manchester_mass |= 00;
				  manchester_mass <<= 2;
				  count+=2;
				  last_state_pin = HAL_GPIO_ReadPin(Manchaster_In_GPIO_Port, Manchaster_In_Pin);
				  HAL_UART_Transmit(&huart1, (uint8_t *) "00", sizeof("00"), 1);
				//  if (count ==9){
				//	  HAL_UART_Transmit(&huart1, (uint8_t *) "\t", sizeof("\t"), 1);
				 // }
			}
			tick_count_zero = 0;
		}
		else  //tick_count_prim >= tick_count_zero && tick_count_zero >= 0
		{
			if(last_state_pin == GPIO_PIN_RESET && count > 0 )
			{ // read start val, Higl
				manchester_mass |= 0;
				manchester_mass <<= 1;
				count++;
				last_state_pin = HAL_GPIO_ReadPin(Manchaster_In_GPIO_Port, Manchaster_In_Pin);
				HAL_UART_Transmit(&huart1, (uint8_t *) "0", sizeof("0"), 1);
			}
			else if ( HAL_GPIO_ReadPin(Manchaster_In_GPIO_Port, Manchaster_In_Pin) == GPIO_PIN_SET && count == 0 )
			{
				  manchester_mass |= 1;
				  manchester_mass <<= 1;
				  count++;
				  last_state_pin = HAL_GPIO_ReadPin(Manchaster_In_GPIO_Port, Manchaster_In_Pin);
				  HAL_UART_Transmit(&huart1, (uint8_t *) "1", sizeof("1"), 1);
			}
			tick_count_zero = 0;
		} */
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

  /* Configure the System Power */
  SystemPower_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ICACHE_Init();
  MX_UCPD1_Init();
  MX_USART1_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_MEMORYMAP_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  //_Bool manchester_IEEE= true, MSB_Frst_E= true;

  //*htim1_p = htim1;

  massage = 0;//, count= 0, comp_bit= 0; //"hello!"
 // uint8_t mask_8;
  uint16_t mask_16;
  if (MSB_Frst_E){ mask_8 = 0x80;
  	  mask_16= 0xC0;}
  else { mask_8 = 0x01;
  	  mask_16= 0x003;}
 // uint8_t clk = 0;
  //_Bool clk = 0;
  //GPIO_PinState last_state_pin;//
 // uint16_t  manchester_mass; // * mass_out;
  //manchester_mass;
  rising_Flag = false;
  falling_Flag = false;
  tim_count_reset_Flag = false;
 // p_rising_Flag = rising_Flag;
  //p_falling_Flag = falling_Flag;
  uint32_t tim2_chan1_comp=0, tim2_count=0;
  uint16_t recived_mass[10]={0};
  uint8_t recived_decod_mass[10]={0};
  //register _Bool massg_ovf= false;
  register uint8_t massg_count= 0;
  register volatile _Bool bit_read_enab= false;
  register uint32_t tick_count_prim_half =0;
  //manchester_mass = 0x8000;

 // *manchester_mass = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  snprintf(uart_mass_buf, sizeof(uart_mass_buf), "%d Zaciatok behu.\r\n", count_bit);
  HAL_UART_Transmit(&huart1, (uint8_t *) uart_mass_buf, sizeof(uart_mass_buf), 1);

  snprintf(uart_mass_buf, sizeof(uart_mass_buf), "Cas medzi v ms:\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *) uart_mass_buf, sizeof(uart_mass_buf), 1);
  *uart_mass_buf = 0;
  //snprintf(uart_mass_buf, sizeof(uart_mass_buf), "");
 // HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
  //last_state_pin = HAL_GPIO_ReadPin(Manchaster_In_GPIO_Port, Manchaster_In_Pin);
  tick_count_prim = 0, tick_count_one = 0, tick_count_zero = 0;
  /*if( last_state_pin ) { // read start val
	  manchester_mass |= 1;
	  manchester_mass <<= 1;
	  count++;
  }
  else{
	  manchester_mass |= 0;
	  manchester_mass <<= 1;
	  count++;
  } */
#define MAX_mass_recived 1
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
 // HAL_IncTick();
  //HAL_TIM_Base_Start_IT(&htim2);
  //HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);

 // HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
  //while ( __HAL_TIM_GET_COUNTER(&htim2)  )
  //__HAL_TIM_DISABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_1);

  //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 3 ); // for chanel of timer

  while (1)
  {
	  //tim2_count = __HAL_TIM_GET_COUNTER(&htim2); // from timer
	  //__HAL_TIM_SET_COUNTER(&htim2, 19);
	 // snprintf(uart_mass_buf, sizeof(uart_mass_buf), "Cas: %ld\r\n", tim2_count);
	 // HAL_UART_Transmit(&huart1, (uint8_t *) uart_mass_buf, sizeof(uart_mass_buf), 1);
	  if ( count_edge > 1 )
	  {
		  HAL_NVIC_DisableIRQ(EXTI0_IRQn);
		  __HAL_TIM_SET_COUNTER(&htim2, tick_count_prim);
		  tick_count_prim_half =   (uint32_t) (tick_count_prim/2 );
	  }
	  if (count_bit >= 16) {
		 // HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
		 // count++;
		  recived_mass[massg_count] = manchester_mass;
		  if (massg_count < MAX_mass_recived){massg_count++;}
		 // HAL_UART_Transmit(&huart1, (uint8_t *) "\r\n", sizeof("\r\n"), 1);
		  count_bit = 0; //manchester_mass = 0;
	  }

	  if ( massg_count == MAX_mass_recived ) { //count == 31 ||
		  //HAL_SuspendTick();
		  break;
	  }

	 // send_manchester_data( 0, &clk);
	 // send_manchester_data( massage, &clk);


	  if ( (rising_Flag || falling_Flag) || tim_count_reset_Flag )  //  == true OR  == true
	  {
	  	  if ( tick_count_prim != 0 )
	  	  {
	  		  if( __HAL_TIM_GET_COUNTER(&htim2) <= tick_count_prim_half )
	  		  {
	  			bit_read_enab= true;
	  		  }
	  	  }
	  	  else
	  	  {
	  		  if( __HAL_TIM_GET_COUNTER(&htim2) <= 20U )
	  		  {
	  			bit_read_enab= true;
	  		  }
	  	  }
	  	  if ( bit_read_enab ) // if true read
	  	  {
			  if ( HAL_GPIO_ReadPin(Manchaster_In_GPIO_Port, Manchaster_In_Pin) == GPIO_PIN_RESET)  //&& count == 0
			  {
				  manchester_mass |= 0;
				  manchester_mass <<= 1;
				  count_bit++;
				  last_state_pin = HAL_GPIO_ReadPin(Manchaster_In_GPIO_Port, Manchaster_In_Pin);
				  HAL_UART_Transmit(&huart1, (uint8_t *) "0", sizeof("0"), 1);
				  // falling_Flag = false;//!falling_Flag;
			  }

		      if(HAL_GPIO_ReadPin(Manchaster_In_GPIO_Port, Manchaster_In_Pin) == GPIO_PIN_SET) // && count == 0
		      { // read start val, Low
		    	 manchester_mass |= 1;
		    	 manchester_mass <<= 1;
		    	 count_bit++;
		    	 last_state_pin = HAL_GPIO_ReadPin(Manchaster_In_GPIO_Port, Manchaster_In_Pin);
		    	 HAL_UART_Transmit(&huart1, (uint8_t *) "1", sizeof("1"), 1);
		    	  //rising_Flag = false;//!rising_Flag;
		      }
		      tim_count_reset_Flag = false;
		      bit_read_enab = false;
		      rising_Flag = false;
			  falling_Flag = false;

	  	  }


  	  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  // Time stpo
  HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);
  //HAL_TIM_Base_Stop_IT(&htim2);

  // podal timeru kazdu sekundu posle log. 1 / 0
  /*if(tick_count_prim != 0){
	  snprintf(uart_mass_buf, sizeof(uart_mass_buf), "\r\nprim: %ld\r\n", tick_count_prim);
	  HAL_UART_Transmit(&huart1, (uint8_t *) uart_mass_buf, sizeof(uart_mass_buf), 1);
	  snprintf(uart_mass_buf, sizeof(uart_mass_buf), "zero: %ld\r\n", tick_count_zero);
	  HAL_UART_Transmit(&huart1, (uint8_t *) uart_mass_buf, sizeof(uart_mass_buf), 1);
	  snprintf(uart_mass_buf, sizeof(uart_mass_buf), "one %ld\r\n", tick_count_one);
	  HAL_UART_Transmit(&huart1, (uint8_t *) uart_mass_buf, sizeof(uart_mass_buf), 1);
  } */

  if (count_bit == 17) {
	  HAL_Delay(10);
	  snprintf(uart_mass_buf, sizeof(uart_mass_buf), "%d Toci sa v cykle.\r\n", count_bit);
	  HAL_UART_Transmit(&huart1, (uint8_t *) uart_mass_buf, sizeof(uart_mass_buf), 1);
	  count_bit++;
  }
  //HAL_Delay(100);
/*
  while ( massg_count !=0){ // --------------Decoding of manchaster
	  manchester_mass = recived_mass[(massg_count-1)];

	  for (int bite_cnt = 0; bite_cnt <= 15; ++bite_cnt) {

		if (manchester_IEEE){
			//comp_bit =  ((manchester_mass & mask_16) == 0b01 )? 1: 0 ;
			switch((manchester_mass & mask_16) ){
			  	case 0b01: massage |= 1; break;  // 0b01 = 1 dec
			  	case 0b10: massage |= 0; break;
			  	default: break; 	// 0b10 = 2 dec
			}
		}
		else {
			switch((manchester_mass & mask_16) ){
				case 0b10: massage |= 1; break;  // comp_bit = 1   0b01 = 1 dec
				case 0b01: massage |= 0; break; //  comp_bit = 0
				default: break; 	// 0b10 = 2 dec
			}
		}

  		if(MSB_Frst_E){ mask_16 >>= 2; }
  		else { mask_16 <<= 2; }
  			massage <<= 1; // ked je vstup 11 vypise 00
	  }

	  recived_decod_mass[(massg_count-1)] = massage;
	  massg_count--;
  } */

  	//recived_decod_mas

  snprintf(uart_mass_buf, sizeof(uart_mass_buf), " : %c | %x \r\n %c :%x \r\n", recived_mass[0], recived_mass[0], manchester_mass, manchester_mass);
  HAL_UART_Transmit(&huart1, (uint8_t *) uart_mass_buf, sizeof(uart_mass_buf), 1);

  snprintf(uart_mass_buf, sizeof(uart_mass_buf), "Koniec.\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *) uart_mass_buf, sizeof(uart_mass_buf), 1);
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_0;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{
  HAL_PWREx_EnableVddIO2();

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_14B;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  ICACHE_RegionConfigTypeDef pRegionConfig = {0};

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Configure and enable a region for memory remapping.
  */
  if (HAL_ICACHE_Disable() != HAL_OK)
  {
    Error_Handler();
  }
  pRegionConfig.BaseAddress = 0x10000000;
  pRegionConfig.RemapAddress = 0x60000000;
  pRegionConfig.Size = ICACHE_REGIONSIZE_2MB;
  pRegionConfig.TrafficRoute = ICACHE_MASTER1_PORT;
  pRegionConfig.OutputBurstType = ICACHE_OUTPUT_BURST_WRAP;
  if (HAL_ICACHE_EnableRemapRegion(_NULL, &pRegionConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief MEMORYMAP Initialization Function
  * @param None
  * @retval None
  */
static void MX_MEMORYMAP_Init(void)
{

  /* USER CODE BEGIN MEMORYMAP_Init 0 */

  /* USER CODE END MEMORYMAP_Init 0 */

  /* USER CODE BEGIN MEMORYMAP_Init 1 */

  /* USER CODE END MEMORYMAP_Init 1 */
  /* USER CODE BEGIN MEMORYMAP_Init 2 */

  /* USER CODE END MEMORYMAP_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  	  // (160 000 000 -1)/ 10 000 = 15999,999 =~ 16000 divider
  	  // 1/10 000 = 0,0001 s *(999 +1) = 0,1 s
  	  	  // Prescaler: 160 000 000 / 16000 = 10 000 ---> 1/10 000 = 0,0001 secunds = 0,1 ms
    	  // Period: 0,0001 * (4999 +1) = 0,5 s = 500 ms end of period
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  	  // Prescaler: 160 000 000-1 / 16000-1 = 10 000 ---> 1/10 000 = 0,0001 secunds = 0,1 ms
  	  // Period: 0,0001 * (9 +1) = 0,001 s = 1 ms end of period
  	  // 599 999 +1 ~= 60 s
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 299999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief UCPD1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UCPD1_Init(void)
{

  /* USER CODE BEGIN UCPD1_Init 0 */

  /* USER CODE END UCPD1_Init 0 */

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_UCPD1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**UCPD1 GPIO Configuration
  PB15   ------> UCPD1_CC2
  PA15 (JTDI)   ------> UCPD1_CC1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN UCPD1_Init 1 */

  /* USER CODE END UCPD1_Init 1 */
  /* USER CODE BEGIN UCPD1_Init 2 */

  /* USER CODE END UCPD1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Manchaster_Out_GPIO_Port, Manchaster_Out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Menches_Out_GPIO_Port, Menches_Out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, UCPD_DBn_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Manchaster_In_Pin */
  GPIO_InitStruct.Pin = Manchaster_In_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Manchaster_In_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Manchaster_Out_Pin */
  GPIO_InitStruct.Pin = Manchaster_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Manchaster_Out_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UCPD_FLT_Pin */
  GPIO_InitStruct.Pin = UCPD_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UCPD_FLT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Menches_Out_Pin */
  GPIO_InitStruct.Pin = Menches_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Menches_Out_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UCPD_DBn_Pin */
  GPIO_InitStruct.Pin = UCPD_DBn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UCPD_DBn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_BLUE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_DisableIRQ(EXTI0_IRQn);
/* USER CODE END MX_GPIO_Init_2 */
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
