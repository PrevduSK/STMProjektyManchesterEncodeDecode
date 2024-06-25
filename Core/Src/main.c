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

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

//TIM_HandleTypeDef * htim1_p, * htim2_p;
//GPIO_PinState last_state_pin;//

_Bool manchester_IEEE= true, MSB_Frst_E= true;

//_Bool * p_rising_Flag, * p_falling_Flag;
//volatile _Bool  rising_Flag;
//volatile _Bool  falling_Flag;
// flags for interups
volatile _Bool 	edge_rise_fall_Flag = false, tim_count_reset_Flag = false;

volatile uint32_t count_from_if_cond =0u;

//enum {}
 //manchester_IEEE= true//, log. 0 --> 10, 1 --> 01
						// manchester_IEEE= false, log. 0 --> 01, 1 --> 10
#define MAX_mess_received 30U
#define Time_delay_Count_not_set 40u  // 80u 120U
#define Time_delay_Transmit 200

// time count of edge and timer compare
volatile uint32_t tick_count_current_edge =0u, tick_current_timer =0u;
// tick_count_last;

//volatile uint32_t interval_btw_tick =0u, time_delay =0u, timer_interval =0u, actual_tick = 0u, count_from_if_cond =0u, last_time_tick = 0u;


#define UART_MESS_LEN 500//50
char uart_mess_buf[UART_MESS_LEN];
volatile uint8_t  message, count_edge= 0, comp_bit= 0; //"hello!"  message = 'h',
//volatile uint8_t mask_8;
volatile uint8_t clk = 0;
volatile uint16_t  manchester_mess;//, mask_16;;

//volatile _Bool bit_read_enab= false;// interval_long = false;


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
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

// ------------------------------ send and encode ----------------------------
void manchester_encode_and_send_char(uint8_t  message); //, uint8_t * clk
void manchester_send_encoded_data(uint16_t  message_encod);

void manchester_encode_and_send_char_array(unsigned char message[], uint8_t length); //, uint8_t * clk

uint16_t manchester_encode_char(uint8_t message);

// ----------------------------- receive and decode ---------------------------
void manchester_receive_data_array( volatile uint16_t received_mess[], uint8_t* messg_count );
void manchester_receive_data_array_via_timer_count( volatile uint16_t received_mess[], uint8_t* messg_count );

void manchester_decode_data_array( volatile uint16_t received_mess[], volatile uint8_t received_decod_mess[], uint8_t messg_count );

// ---------------------------------         -----------------------------------------
uint8_t clear_char_array_len(char * array, uint16_t a_len);

void manchester_IEEE_Set(_Bool n_val);
_Bool manchester_IEEE_Get();
void message_MSB_Frst_E_Set(_Bool n_val);
_Bool message_MSB_Frst_E_Get();

void manchester_send_and_receive( _Bool send_data_mode_enable, _Bool timer_counter_enable, volatile uint16_t received_mess[], volatile uint8_t received_decod_mess[], uint8_t * received_messg_count, unsigned char send_mess[], uint8_t send_messg_count );

HAL_StatusTypeDef my_UART_receive_char_array( unsigned char mess_buff[], uint8_t mess_max_len, uint8_t * readed_char );


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ------------------------------ send and encode ----------------------------

/**
  * @brief Send manchester data
  * 	This function encode and send char by setting bit value of a parameter mess_age as state on a pin Manchester_Out_Pin .
  * 	Between toggle state on pin is waiting by use function HAL_Delay(), waiting time is made by directive value Time_delay_Transmit.
  * @param uint8_t  message
  * @retval None
  */
void manchester_encode_and_send_char(uint8_t  message) // , uint8_t * clk
{
	//_Bool manchester_IEEE= true, MSB_Frst_E= true;

	uint8_t volatile count= 0, comp_bit= 0; //"hello!"  message = 'h',
	register volatile uint8_t mask_8;
	if (MSB_Frst_E){ mask_8 = 0x80;}
	else { mask_8 = 0x01;}
	 clk = 0; //uint8_t
//	register volatile uint16_t manchester_mess;

	while(1)
	{
		clk = !clk; // log. 1 // *clk = !*clk;
		// Menchester: as per IEEE 802.3
		// LSB first
		comp_bit =  (message & mask_8 )? 1: 0 ;
		if (manchester_IEEE){
			if( (comp_bit ^ (clk & 0x01)) ) {
				//manchester_mess |= 1;
				HAL_GPIO_WritePin (Manchester_Out_GPIO_Port, Manchester_Out_Pin, GPIO_PIN_SET);
			}
			else{
				//manchester_mess |= 0;
				HAL_GPIO_WritePin (Manchester_Out_GPIO_Port, Manchester_Out_Pin, GPIO_PIN_RESET);
			}
		}
		else {
			if( !(comp_bit ^ (clk & 0x01)) ) {
				//manchester_mess |= 1;
				HAL_GPIO_WritePin (Manchester_Out_GPIO_Port, Manchester_Out_Pin, GPIO_PIN_SET);
			}
			else{
				//manchester_mess |= 0;
				HAL_GPIO_WritePin (Manchester_Out_GPIO_Port, Manchester_Out_Pin, GPIO_PIN_RESET);
			}
		}


		if ( clk == 0){
			if(MSB_Frst_E){ mask_8 >>= 1; }
			else { mask_8 <<= 1; }
		}
		if (count == 15) {
			HAL_Delay(Time_delay_Transmit);
			break;
		}
		count++;

		//manchester_mess <<= 1; // ked je vstup 11 vypise 00

		// namiesto delay pouzit potom timer na posielanie
		HAL_Delay(Time_delay_Transmit); //20ms 50Hz; 1ms 1000Hz
		// cas medzi nabeznou a dobeznou hranou
		// ked su data 0 treba maerat cas medzi nabeznou a dobeznou hranou
	}
}

/**
  * @brief Send manchester data encoded
  * 	This function send encoded char by setting bit value of message_encod as state on a pin Manchester_Out_Pin .
  * 	Between toggle state on pin is waiting by use function HAL_Delay(), waiting time is made by directive value Time_delay_Transmit.
  * @param uint16_t  message_encod
  * @retval None
  */
void manchester_send_encoded_data(uint16_t  message_encod) // , uint8_t * clk
{
	//_Bool manchester_IEEE= true, MSB_Frst_E= true;

	register uint8_t volatile count= 0, comp_bit= 0; //"hello!"  message = 'h',
	register volatile uint16_t mask_16;
	if (MSB_Frst_E){
	    mask_16 = 0x8000;
	  } else {
	    mask_16 = 0x0001;
	  }
	//uint8_t clk = 0;
//	register volatile uint16_t manchester_mess;

	while(1)
	{
		//clk = !clk; // log. 1 // *clk = !*clk;
		// Menchester: as per IEEE 802.3
		// LSB first
		comp_bit =  (message_encod & mask_16 )? 1: 0 ;
		if( comp_bit  ) {
				//manchester_mess |= 1;
			HAL_GPIO_WritePin (Manchester_Out_GPIO_Port, Manchester_Out_Pin, GPIO_PIN_SET);
		}
		else{
			//manchester_mess |= 0;
			HAL_GPIO_WritePin (Manchester_Out_GPIO_Port, Manchester_Out_Pin, GPIO_PIN_RESET);
		}

		if (MSB_Frst_E) {
			mask_16 >>= 1;  // shift mask to the right
		} else {
			mask_16 <<= 1;  // shift mask to the left
		}

		if (count == 15) {
			HAL_Delay(Time_delay_Transmit);
			break;
		}
		count++;

		//manchester_mess <<= 1; // ked je vstup 11 vypise 00

		// namiesto delay pouzit potom timer na posielanie
		HAL_Delay(Time_delay_Transmit); //20ms 50Hz; 1ms 1000Hz
		// cas medzi nabeznou a dobeznou hranou
		// ked su data 0 treba maerat cas medzi nabeznou a dobeznou hranou
	}
}

/**
  * @brief Send manchester data array
  * 	This function send char array by calling function manchester_encode_char() to encode chars
  * 	and by calling function manchester_send_encoded_data() to set values on pin to send data.
  * @param unsigned char message[], uint8_t length
  * @retval None
  */
void manchester_encode_and_send_char_array(unsigned char message[], uint8_t length) // , uint8_t * clk
{
	volatile uint16_t mess_buf[length];
	for (uint8_t i = 0; i < length; i++) {
	    mess_buf[i] = manchester_encode_char( (uint8_t) message[i] );
	 }
	for (uint8_t i = 0; i < length; i++) {
		manchester_send_encoded_data( mess_buf[i] );
	}
	manchester_encode_and_send_char(0);

}

/**
  * @brief Encode manchester char
  * 	This function encode char to manchester encoding format ready to send.
  * 	Char is encode by bit and it is return in encoded format.
  * @param uint8_t message
  * @retval uint16_t manchester_mess
  */
uint16_t manchester_encode_char(uint8_t message) {
  _Bool print_hex= true, print_dec_ofMess= false;


  uint8_t count = 0, comp_bit = 0; // uint8_t as byte
  uint16_t manchester_mess = 0;

  uint8_t mask_8;
  if (MSB_Frst_E) {
    mask_8 = 0x80;
  } else {
    mask_8 = 0x01;
  }

   clk = 0;

  while (true) {
    clk = !clk;  // toggle clock

    // Manchester encoding as per IEEE 802.3
    // LSB first
    comp_bit = (message & mask_8) ? 1 : 0;
    if (manchester_IEEE) {
      if ( comp_bit ^ (clk & 0x01) ) {
        manchester_mess |= 1;
       // HAL_GPIO_WritePin(Menches_Out_GPIO_Port, Menches_Out_Pin, GPIO_PIN_SET);

      } else {
        manchester_mess |= 0;
       // HAL_GPIO_WritePin(Menches_Out_GPIO_Port, Menches_Out_Pin, GPIO_PIN_RESET);
      }
    } else {
      if (!(comp_bit ^ (clk & 0x01))) {
        manchester_mess |= 1;
       // HAL_GPIO_WritePin(Menches_Out_GPIO_Port, Menches_Out_Pin, GPIO_PIN_SET);
      } else {
        manchester_mess |= 0;
      //  HAL_GPIO_WritePin(Menches_Out_GPIO_Port, Menches_Out_Pin, GPIO_PIN_RESET);

      }
    }

    if (!clk) {
      if (MSB_Frst_E) {
        mask_8 >>= 1;  // shift mask to the right
      } else {
        mask_8 <<= 1;  // shift mask to the left
      }
    }

    if (count == 15) {
      if(print_hex) {
    	  clear_char_array_len(uart_mess_buf, UART_MESS_LEN);
    	  snprintf(uart_mess_buf, sizeof(uart_mess_buf), "%x \r\n", manchester_mess);
    	  HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), 1);
      }
      if(print_dec_ofMess){
    	  clear_char_array_len(uart_mess_buf, UART_MESS_LEN);
    	  snprintf(uart_mess_buf, sizeof(uart_mess_buf), "%d \r\n", manchester_mess);
    	  HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), 1);
      }

      return manchester_mess;
    }

    count++;
    manchester_mess <<= 1;  // shift the Manchester code
  }
}

// ------------------------------ send and encode END ----------------------------

// ----------------------------- receive and decode ---------------------------

/**
  * @brief Receive Manchester coded data
  * 	This function react on  flags edge_rise_fall_Flag,
  * 	and it use current timer value via function HAL_GetTick() to get interval a length between states toggle.
  * 	Overread data are send back via array received_mess[], and via * messg_count are saved length of array.
  *
  * 	Function have 4 phases that are in endless loop till end of receiving,
  * 	- 1st Getting current timer value when is not any flag and when condition are true it set a flag tim_count_reset_Flag,
  * 	- 2nd Delay about length of 1/2 of length toggling, and get length of toggling,
  * 	- 3rd Pin state read from a Manchester_In_Pin,
  * 	- 4th End of receive, when a last read 2 bytes have value: 0xAAAA, 0x0, 0x5555, 0xFFFF.
  * 	In this phase is disable interrupt from GPIO, and function end breaking from endless loop
  * @param volatile uint16_t received_mess[], uint8_t* messg_count
  * @retval None
  */
void manchester_receive_data_array( volatile uint16_t received_mess[], uint8_t* messg_count )
{


	register volatile uint32_t interval_btw_tick =0u, time_delay =0u, timer_interval =0u, actual_tick = 0u, last_time_tick = 0u, current_time_tick =0u, tick_last_timer = 0u, tick_count_last_edge =0u;

	register volatile uint32_t count_from_if_cond =0u;
	uint32_t tick_count_prim_and_half =0u, tick_count_prim =0u, tick_count_prim_half = 0u;

	register volatile _Bool bit_read_enab = false ;
	_Bool end_word = false;
	register volatile uint8_t count_bit= 0;

	message = 0;

	while (1)
	{
		// --Non blocking wait to time interval for read from GPIO
		if ( ( !edge_rise_fall_Flag  && !tim_count_reset_Flag ) && count_edge >= 2 ) // F and F  and 2,3 ...
		{
			actual_tick = HAL_GetTick(); // __HAL_TIM_GET_COUNTER(&htim2); HAL_GetTick();
			if ( tick_last_timer != 0 ) {
				timer_interval = actual_tick - tick_last_timer;
				// --When is time it set flag to do read from GPIO without waiting to interrupt
				if (   ( timer_interval > tick_count_prim ) && ( timer_interval <=  tick_count_prim_and_half  )   ) //
				{
					tick_current_timer = HAL_GetTick(); // __HAL_TIM_GET_COUNTER(&htim2); HAL_GetTick();
					tim_count_reset_Flag = true;
				}
			}

			count_from_if_cond++;
			if ( count_from_if_cond == 0xFA0000 ){
				clear_char_array_len(uart_mess_buf, UART_MESS_LEN);
				snprintf(uart_mess_buf, sizeof(uart_mess_buf), "edge: %d, bit: %d\r\n", count_edge, count_bit );
				HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), 2);
				clear_char_array_len(uart_mess_buf, 40);
				snprintf(uart_mess_buf, sizeof(uart_mess_buf), "Nedokaze pocitat cas.\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), 2);
	  				  //end_word = false;
	  		break;
			}
		}

		if ( edge_rise_fall_Flag  || tim_count_reset_Flag )  //  == true OR  == true  //|| tim_count_reset_Flag
		{

			//__HAL_TIM_GET_COUNTER(&htim2);
			time_delay = (tick_count_prim != 0) ?  tick_count_prim_half : Time_delay_Count_not_set; //(Time_delay_Count_not_set*10)

			//current_time_tick = edge_rise_fall_Flag ? tick_count_current_edge : tick_current_timer;
			last_time_tick = edge_rise_fall_Flag ? tick_count_last_edge :  tick_last_timer;

			if ( ( HAL_GetTick() - last_time_tick) > time_delay )  //HAL_GetTick()  __HAL_TIM_GET_COUNTER(&htim2)
			{
				//snprintf(uart_mess_buf, sizeof(uart_mess_buf), "tim %ld, del %ld !=\r\n", (HAL_GetTick() - tick_count_current_edge), time_delay);
				//HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), 1);

				if ( edge_rise_fall_Flag )
				{
					interval_btw_tick = tick_count_current_edge - tick_count_last_edge;
					tick_count_last_edge = tick_count_current_edge;
					tick_last_timer = tick_count_current_edge;
				}
				else
				{
					tick_count_last_edge = tick_current_timer;
					tick_last_timer = tick_current_timer;
				}

				bit_read_enab = true;
				edge_rise_fall_Flag = false;
				tim_count_reset_Flag = false;
			}

			if ( count_edge == 2  )
			{
				tick_count_prim = interval_btw_tick;
				// __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, tick_count_prim);
				tick_count_prim_half =   (uint32_t) (tick_count_prim/2 );
				tick_count_prim_and_half = (uint32_t) tick_count_prim*1.5f; //	  tick_count_prim_and_sixth = (uint32_t) ((tick_count_prim *7) /6);

				//  snprintf(uart_mess_buf, sizeof(uart_mess_buf), "ph %ld, p %ld !=\r\n", tick_count_prim_half, tick_count_prim);
				//  HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), 1);
			}

			if ( bit_read_enab ) // if true read
			{
				bit_read_enab = false;

				if ( HAL_GPIO_ReadPin(Manchester_In_GPIO_Port, Manchester_In_Pin) == GPIO_PIN_RESET)  //&& count == 0
				{
					// manchester_mess |= 0; // manchester_mess <<= 1;
				}
				else
				{ // read start val, Low
					manchester_mess |= 1;
				}

				count_bit++;

				if (count_bit >= 16) {
					//HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
					//HAL_NVIC_DisableIRQ(EXTI0_IRQn);
					// count++;
					received_mess[*messg_count] = manchester_mess;
					if (*messg_count < MAX_mess_received){(*messg_count)++;}
					else {end_word = true;}
					if ( (manchester_mess == 0xaaaa || manchester_mess == 0x0) || (manchester_mess == 0x5555 || manchester_mess == 0xffff) ) {end_word = true;}

					//clear_char_array_len(uart_mess_buf,UART_MESS_LEN);
					//  HAL_UART_Transmit(&huart1, (uint8_t *) "\r\n", sizeof("\r\n"), 1);
					//snprintf(uart_mess_buf, sizeof(uart_mess_buf), "edge: %d, bit: %d\r\n", count_edge, count_bit );
					//HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), 2);
					count_bit = 0;
					manchester_mess = 0;
				}
				else {  manchester_mess <<= 1; }
				count_from_if_cond = 0;
			}

		}

	  		 // snprintf(uart_mess_buf, sizeof(uart_mess_buf), "Cas: %ld\r\n", tim2_count);
	  		 // HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), 1);
	  		  //return; //end_word
		if ( end_word ) { //messg_count == MAX_mess_received  //count == 31 ||
			HAL_NVIC_DisableIRQ(EXTI0_IRQn);

			clear_char_array_len(uart_mess_buf, UART_MESS_LEN);
			snprintf(uart_mess_buf, sizeof(uart_mess_buf), ": %c |%x \r\n\r\n", received_mess[0], received_mess[0]); // "%c :%x" , manchester_mess, manchester_mess
			HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), 2);
			break;
		}
	}
	return;
}

/**
  * @brief Decode Manchester coded data array
  * 	This function decode received data from array received_mess[]. Decoded data are send back via array received_decod_mess[] as chars.
  *
  * 	Function going through array received_mess[] using index *messg_count, from char to char.
  * @param volatile uint16_t received_mess[], volatile uint8_t received_decod_mess[], uint8_t* messg_count
  * @retval None
  */
void manchester_decode_data_array( volatile uint16_t received_mess[], volatile uint8_t received_decod_mess[], uint8_t messg_count )
{
	register volatile uint16_t mask_16;
	if (MSB_Frst_E){ mask_16= 0x8000; }
	else { mask_16= 0x001; }
	register volatile uint8_t count_messg =  messg_count;

	while ( count_messg >0)
	{ // --------------Decoding of manchaster
		(count_messg)--;
	    manchester_mess = received_mess[count_messg]; //(messg_count-1)

	    if(MSB_Frst_E){ mask_16 = 0x8000; }
	    else { mask_16 = 0x01; }
	    message= 0;

	    for (int bite_cnt = 0; bite_cnt < 8; bite_cnt++) {
	    	if (manchester_IEEE)
	    	{  // (manchester_mess & mask_16 ) ==0 &&  ((manchester_mess & (mask_16>>1) ) > 0)
	    		if ( !(manchester_mess & mask_16 ) && (manchester_mess & (mask_16>>1) ) ) // 0b01 = 1 dec -> 1
	    		{
	    			message |= 1;

	    		} else if ( !(manchester_mess & (mask_16>>1) ) ) // 0b10 -> 0
	    		{
	    			message |= 0;
	    		}
	    	}
	    	else
	    	{
	    		if ( (manchester_mess & mask_16) ) // 0b10 = 1 dec -> 1
	    		{  message |= 1; }
	    		else // 0b01 -> 0
		        {  message |= 0; }
	    	}
	    	if(MSB_Frst_E){ mask_16 >>= 2; }
	    	else { mask_16 <<= 2; }

	    	if ( bite_cnt < 7) {
	    		message <<= 1; // ked je vstup 11 vypise 00
	    	}
	    }

	    received_decod_mess[count_messg] = message; //(messg_count-1)
	    //messg_count--

	    clear_char_array_len(uart_mess_buf, UART_MESS_LEN);
	    snprintf(uart_mess_buf, sizeof(uart_mess_buf), "%c %d \t %x\r\n", received_decod_mess[count_messg], received_decod_mess[count_messg], manchester_mess);
	    HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), 2);
	    //clear_char_array_len(uart_mess_buf, UART_MESS_LEN);
	    //snprintf(uart_mess_buf, sizeof(uart_mess_buf), "%x\r\n", manchester_mess);
	    //HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), 2);
	}
	return;
}

// ----------------------------- receiv and decode END ---------------------------

/**
  * @brief Receive Manchester coded data via Timer Counter
  * 	This function react on  flags edge_rise_fall_Flag and tim_count_reset_Flag,
  * 	and it use current timer value to get interval a length between states toggle.
  * 	Overread data are send back via array received_mess[], and via * messg_count are saved length of array.
  *
  * 	Function have 3 phases that are in endless loop till end of receiving,
  * 	- 1st Delay about length of 1/2 of length toggling, and get length of toggling
  * 	- 2nd Pin state read from a Manchester_In_Pin
  * 	- 3rd End of receive, when a last read 2 bytes have value: 0xAAAA, 0x0, 0x5555, 0xFFFF.
  * 	In this phase is disable interrupt from GPIO and TIM_OC, and function end breaking from endless loop
  * @param volatile uint16_t received_mess[], uint8_t* messg_count
  * @retval None
  */
void manchester_receive_data_array_via_timer_count( volatile uint16_t received_mess[], uint8_t* messg_count )
{

	register volatile uint32_t interval_btw_tick =0u, time_delay =0u, timer_interval =0u, actual_tick = 0u;
	register volatile uint32_t last_time_tick = 0u, tick_last_timer = 0u, current_time_tick = 0u , tick_count_last_edge =0u;

	//register volatile uint32_t count_from_if_cond =0u;
	uint32_t  tick_count_prim =0u, tick_count_prim_half = 0u; //tick_count_prim_and_half =0u;

	register volatile _Bool bit_read_enab = false ;
	_Bool end_word = false;
	register volatile uint8_t count_bit= 0;

	message = 0;
	HAL_NVIC_SetPriority(TIM2_IRQn, 1, 1); //

	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);

	while (1)
	{
		// --Non blocking wait to time interval for read from GPIO
		if ( edge_rise_fall_Flag || tim_count_reset_Flag  )  //  == true OR  == true  //|| tim_count_reset_Flag
		{

			//__HAL_TIM_GET_COUNTER(&htim2);
			time_delay = (tick_count_prim != 0) ?  tick_count_prim_half : Time_delay_Count_not_set; //(Time_delay_Count_not_set*10000)

			last_time_tick = edge_rise_fall_Flag ? tick_count_last_edge :  tick_last_timer;

			//HAL_Delay( time_delay);

			if ( ( HAL_GetTick() - last_time_tick) > time_delay )  //HAL_GetTick()  __HAL_TIM_GET_COUNTER(&htim2)
			{
				//snprintf(uart_mess_buf, sizeof(uart_mess_buf), "tim %ld, del %ld !=\r\n", (HAL_GetTick() - tick_count_current_edge), time_delay);
				//HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), 1);

				if ( edge_rise_fall_Flag )
				{
					interval_btw_tick = tick_count_current_edge - tick_count_last_edge;
					tick_count_last_edge = tick_count_current_edge;
					tick_last_timer = tick_count_current_edge;
				}
				else
				{
					tick_count_last_edge = tick_current_timer;
					tick_last_timer = tick_current_timer;
				}

				bit_read_enab = true;
				edge_rise_fall_Flag = false;
				tim_count_reset_Flag = false;
				//__HAL_TIM_ENABLE_IT(&htim1, TIM1_CC_IRQn);
			}

			if ( count_edge == 2  )
			{
				tick_count_prim = interval_btw_tick;
				__HAL_TIM_SET_AUTORELOAD(&htim2, tick_count_prim );
				//__HAL_TIM_SET_COUNTER(&htim1, tick_count_prim );
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, tick_count_prim);
				tick_count_prim_half =   (uint32_t) (tick_count_prim/2 );
				//HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
				//tick_count_prim_and_half = (uint32_t) tick_count_prim*1.5f;

				//  snprintf(uart_mess_buf, sizeof(uart_mess_buf), "ph %ld, p %ld !=\r\n", tick_count_prim_half, tick_count_prim);
				//  HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), 1);
			}


			if ( bit_read_enab ) // if true read
			{
				bit_read_enab = false;

				if ( HAL_GPIO_ReadPin(Manchester_In_GPIO_Port, Manchester_In_Pin) == GPIO_PIN_RESET)  //&& count == 0
				{
					// manchester_mess |= 0; // manchester_mess <<= 1;
				}
				else
				{ // read start val, Low
					manchester_mess |= 1;
				}

				count_bit++;

				if (count_bit >= 16) {

					received_mess[*messg_count] = manchester_mess;
					if (*messg_count < MAX_mess_received){(*messg_count)++;}
					else {end_word = true;}
					if ( (manchester_mess == 0xaaaa || manchester_mess == 0x0) || (manchester_mess == 0x5555 || manchester_mess == 0xffff) ) {end_word = true;}

					count_bit = 0;
					manchester_mess = 0;
				}
				else {  manchester_mess <<= 1; }
				//count_from_if_cond = 0;

			}



		}

	  		  //return; //end_word
		if ( end_word ) { //messg_count == MAX_mess_received  //count == 31 ||
			HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);
			HAL_NVIC_DisableIRQ(EXTI0_IRQn);
			//HAL_TIM_Base_Stop_IT(&htim1);

			clear_char_array_len(uart_mess_buf, UART_MESS_LEN);
			snprintf(uart_mess_buf, sizeof(uart_mess_buf), ": %c |%x \r\n\r\n", received_mess[0], received_mess[0]); // "%c :%x" , manchester_mess, manchester_mess
			HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), 2);
			break;
		}
	}
	HAL_TIM_Base_Stop(&htim2);
	return;
}

/**
  * @brief TIM Output Compare external interrupt handler
  * 	in this case it check state of TIM1_OC
  * 	This function set flag tim_count_reset_Flag, and get current timer value
  * @param TIM_HandleTypeDef * htim
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback (TIM_HandleTypeDef * htim)
{

	// are executed by set HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
  /*  if (htim->Instance == TIM1)
    {
    	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);

    	//HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
  		clk = !clk; // log. 1
  		// Manchester: as per IEEE 802.3
    	// LSB firsth
    	comp_bit =  (message & mask_8 )? 1: 0 ;
    	if (manchester_IEEE){
    		if( (comp_bit ^ (clk & 0x01)) ) {
    			manchester_mess |= 1;
    		}
    		else{
    			manchester_mess |= 0;
    		}
    	}
    	else {
    		if( !(comp_bit ^ (clk & 0x01)) ) {
    			manchester_mess |= 1;
    		}
    		else{
    			manchester_mess |= 0;
    		}
    	}


    	if ( clk == 0){
    		if(MSB_Frst_E){ mask_8 >>= 1; }
    		else { mask_8 <<= 1; }
    	}
    	if (count_bit != 15) {
    		count_bit++;

    		manchester_mess <<= 1; // ked je vstup 11 vypise 00
    	} //

    } */

  /*  if (htim->Instance == TIM1)
    {
    	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);

    	 if (!edge_rise_fall_Flag)
    	{
        	tick_current_timer = HAL_GetTick();//HAL_GetTick(); //__HAL_TIM_GET_COUNTER(&htim2)
        	__HAL_TIM_SET_COUNTER(&htim2, 0);
        	//snprintf(uart_mess_buf, sizeof(uart_mess_buf), "F OC, %ld\r\n", __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1) );
        	//HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), 1);
        	tim_count_reset_Flag = true;

    	}

    } */
    if (htim->Instance == TIM2)
        {
        	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);

        	 if (!edge_rise_fall_Flag)
        	{
            	tick_current_timer = HAL_GetTick();//HAL_GetTick(); //__HAL_TIM_GET_COUNTER(&htim2)
            	__HAL_TIM_SET_COUNTER(&htim2, 0);
            	//snprintf(uart_mess_buf, sizeof(uart_mess_buf), "F OC, %ld\r\n", __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1) );
            	//HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), 1);
            	tim_count_reset_Flag = true;

        	}

        }
    // return;

}

/**
  * @brief GPIO external interrupt handler for falling edge
  * 	in this case it check state on manchester_In_pin.
  * 	This function set flag edge_rise_fall_Flag, and get current timer value.
  * @param uint16_t GPIO_Pin
  * @retval None
  */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == Manchester_In_Pin)
	{
		//__HAL_TIM_DISABLE_IT(&htim1, TIM1_CC_IRQn); // disable intrupt TIM2
		__HAL_GPIO_EXTI_CLEAR_FLAG(Manchester_In_Pin);


		//tick_current_timer = __HAL_TIM_GET_COUNTER(&htim2);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		//tick_count_current_edge = HAL_GetTick();
		tick_count_current_edge = HAL_GetTick();// timer_counter_enable ? __HAL_TIM_GET_COUNTER(&htim2) : HAL_GetTick();
		/*if ( timer_counter_enable ) {
		 *

		}
		else {

		} */

		 edge_rise_fall_Flag = true;
		 count_edge++;
		// __HAL_TIM_ENABLE_IT(&htim2, TIM2_IRQn);
	}
}

/**
  * @brief GPIO external interrupt handler for rising edge
  * 	in this case it check state on manchester_In_pin.
  * 	This function set flag edge_rise_fall_Flag, and get current timer value.
  * @param uint16_t GPIO_Pin
  * @retval None
  */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == Manchester_In_Pin)
	{
		//__HAL_TIM_DISABLE_IT(&htim1, TIM1_CC_IRQn); // disable intrupt TIM2
		 __HAL_GPIO_EXTI_CLEAR_FLAG(Manchester_In_Pin);

		 __HAL_TIM_SET_COUNTER(&htim2, 0);
		 tick_count_current_edge = HAL_GetTick(); /// timer_counter_enable ? __HAL_TIM_GET_COUNTER(&htim2) : HAL_GetTick();

		 edge_rise_fall_Flag = true;
		 count_edge++;
	}
}

/**
  * @brief Clear char array of length
  * 	This function set all char in array of length a_len to value 0.
  * @param char * array, uint16_t a_len
  * @retval uint8_t  retrun 0  like index == 0
  */
uint8_t  clear_char_array_len(char * array, uint16_t a_len)
{
	for (uint16_t i =0; i < a_len; i++) { *(array + i) = 0; }
	return 0;
}


void manchester_IEEE_Set(_Bool n_val) 	{	manchester_IEEE = n_val;	}
_Bool manchester_IEEE_Get() 	{	return manchester_IEEE;	}

void message_MSB_Frst_E_Set(_Bool n_val) 	{	MSB_Frst_E = n_val;	}
_Bool message_MSB_Frst_E_Get() 		{	return MSB_Frst_E;		}

/**
  * @brief Manchester send and receive
  * 	This function can send or receive by setting 1st and 2nd parameter value.
  * @param 	_Bool 	send_data_mode_enable: 		if is TRUE then function send data that are in buffer send_mess[],
  * 		_Bool 	timer_counter_enable:		if is TRUE then function receive data using timer compare,
  *
  * 	volatile uint16_t 	received_mess[]:			parameter use for return received message as manchester code array,
  * 	volatile uint8_t 	received_decod_mess[]:		parameter use for return received decoded message as char array,
  * 	uint8_t * 			received_messg_count:		parameter return count of char in received_mess and received_decod_mess array,
  *
  * 	unsigned char 	send_mess[]:		parameter use as buffer for send char array,
  * 	uint8_t 		send_messg_count:	parameter save count of char in send_mess array.
  * @retval None
  */
void manchester_send_and_receive( _Bool send_data_mode_enable, _Bool timer_counter_enable, volatile uint16_t received_mess[], volatile uint8_t received_decod_mess[], uint8_t * received_messg_count, unsigned char send_mess[], uint8_t send_messg_count )
{
	//unsigned char sprava[] = "Hello!"; // STM

	if (send_data_mode_enable) // ---------------------- send
	{
		// manchester_encode_and_send_char( 0);
		//manchester_encode_and_send_char( message );
		HAL_Delay(4000);
		HAL_UART_Transmit(&huart1, (uint8_t *) "Send m!\r\n", sizeof("Send m!\r\n"), 1);

		HAL_Delay(400);
		manchester_encode_and_send_char_array(send_mess, send_messg_count); //( sizeof(send_mess)/sizeof(send_mess[0]) )

	}
	else // ------------------------ rcieve
	{
		HAL_NVIC_EnableIRQ(EXTI0_IRQn);
		edge_rise_fall_Flag = false;
		count_edge = 0;

		HAL_InitTick(SystemCoreClock);

		snprintf(uart_mess_buf, sizeof(uart_mess_buf), "Receiv m!\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), 1);



		if ( timer_counter_enable ) {

			// for compare with value in channe
			manchester_receive_data_array_via_timer_count(received_mess, received_messg_count);
			// Time stpo
			//HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);
			// HAL_TIM_Base_Stop(&htim2);

		}
		else {
			//  HAL_InitTick(SystemCoreClock);

			manchester_receive_data_array(received_mess, received_messg_count);

		}


		clear_char_array_len(uart_mess_buf, UART_MESS_LEN);
		snprintf(uart_mess_buf, sizeof(uart_mess_buf), "mess_count: %d \r\n", received_messg_count);
		HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), 2);

		manchester_decode_data_array(received_mess, received_decod_mess, *received_messg_count);

	}

    clear_char_array_len(uart_mess_buf, UART_MESS_LEN);
    snprintf(uart_mess_buf, sizeof(uart_mess_buf), "\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), 1);

    for ( uint8_t index = 0; index < received_messg_count; ++index) {

  	  snprintf(uart_mess_buf, sizeof(uart_mess_buf), "%c", received_decod_mess[index]);
  	  HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), 1);
    }

}

/**
  * @brief my UART receive char array
  * 	This function receive catched char array  via UART1 char by char,
  * 		function stopped receive char when find 0x0A (enter press) or end of array.
  * @param 	unsigned char 	mess_buff[]:  	parameter return a char array,
  * 		uint8_t 	mess_max_len:		parameter set a max count of array length,
  * 		uint8_t * 	readed_char:		parameter return current array length.
  * @retval HAL_StatusTypeDef:	in the end of receive return HAL_OK state.
  */
HAL_StatusTypeDef my_UART_receive_char_array( unsigned char mess_buff[], uint8_t mess_max_len, uint8_t * readed_char ) {
	uint8_t rec_char = 0, index=0;
	_Bool end_catch =false;
    while (1) {
        if (HAL_UART_Receive(&huart1, &rec_char, 1, 5000) == HAL_OK) { //HAL_MAX_DELAY
        	if ( (rec_char == '\r' || rec_char == '\e'  ) || index >= mess_max_len) { //'\n' // enter asci code \r
        		end_catch = true;
        		mess_buff[index] = '\0';
        	}
        	else mess_buff[index++] = rec_char;

        }
        if (end_catch) break;
    }
    *readed_char = index;
    return HAL_OK;
}


void terminal_send_recive(){

	volatile uint16_t received_mess_local[MAX_mess_received]={0};
	volatile uint8_t received_decod_mess_local[MAX_mess_received]={0};

	uint8_t messg_count_local= 0u, sprava_count_local =0;

	unsigned char sprava_local[MAX_mess_received] = {0};

	volatile uint8_t termin_mess =6, user_choice = 0;
	volatile _Bool send_term_mess = true, end_term= false, read_term= false, read_info_not_send= true;
	read_info_not_send = false;
	HAL_StatusTypeDef read_state;


	 // if (0){
	  if (send_term_mess)
	  {
		  switch( termin_mess )
		  {
		  	  case 6: snprintf(uart_mess_buf, sizeof(uart_mess_buf), "\r\n=================================" // hlavna ponuka
		  			  "\r\nNatavenie komunikacie: \r\nPosielanie\t\t1 \r\nPrijimanie\t\t2"
		  			  "\r\nNatavenie kodovania\t3 \r\nNastavenie MSB/LSB\t4"
		  			  "\r\nKoniec komunikacie\t5\r\n================================="
		  			  "\r\npredvolene nastavenie: \tNorma IEEE, a MSB");
		  	  	  send_term_mess = false; read_term= true; 	read_info_not_send = true;
		  	  	  break; // Nastavenie komunikacie

		  	  case 1: snprintf(uart_mess_buf, sizeof(uart_mess_buf), "\r\nPosielanie: \r\n  Napis spravu o dlzke max %d znakov :"
		  			  " a stlac enter \r\n  Dalsia sprava\t \r\n  Navrt do nastaveni komunikacie\t 5", MAX_mess_received);
		  	  	  send_term_mess = false; read_term= true; 	read_info_not_send = false;
		  	  	  break;// Posielanie

		  	  case 2: snprintf(uart_mess_buf, sizeof(uart_mess_buf), "\r\nPrijimanie: \r\n  Caka sa na prijatie spravy:");
		  	  	  send_term_mess = false; read_term= true; read_info_not_send = false;
		  		  break;	// Prijimanie
		  	  case 3:
		  		  snprintf(uart_mess_buf, sizeof(uart_mess_buf), "\r\n=================================" // hlavna ponuka
										  "\r\nNatavenie Manchester normi kodovania: \r\nIEEE 802.3\t\t1 \r\nG. E. Thomas\t\t2"
										  "\r\nNavret do nastavnie komunikacie\t6 \r\n=================================");
		  		  send_term_mess = false; read_term= true; 	read_info_not_send = true;
		  		  break;	// Manchester kodovanie nastavenie
		  	  case 4:
		  		  snprintf(uart_mess_buf, sizeof(uart_mess_buf), "\r\n=================================" // hlavna ponuka
										  "\r\nNatavenie posielanie MSB/LSB ako prve: \r\nMSB prve\t\t1 \r\nLSB prve\t\t2"
										  "\r\nNavret do nastavnie komunikacie\t6 \r\n=================================");
		  		  send_term_mess = false; read_term= true; 	read_info_not_send = true;
		  		  break;	// MSB/LSB nastavnie
		  	  //case 3: snprintf(uart_mess_buf, sizeof(uart_mess_buf), "\r\nPosielanie\t\t1"); termin_mess =4; break;
		  	  //case 4: snprintf(uart_mess_buf, sizeof(uart_mess_buf), "\r\nPrijimanie\t\t2\r\n"); termin_mess =5; break;


		  	  //default: snprintf(uart_mess_buf, sizeof(uart_mess_buf), "\r\n"); termin_mess =1; send_term_mess = false; read_term= true; break;

		  }
		  HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), HAL_MAX_DELAY);
		  clear_char_array_len(uart_mess_buf, UART_MESS_LEN);

	  }
	  else if (read_term) {

		  if(read_info_not_send )
		  {
			  HAL_UART_Transmit(&huart1, (uint8_t *) "\r\nnapiste volbu:\r\n", sizeof("\r\nnapiste volbu:\r\n"), HAL_MAX_DELAY);
			  read_info_not_send = false;
		  }
		  if ( !read_info_not_send  )
		  {

			  if (my_UART_receive_char_array(sprava_local, MAX_mess_received, &sprava_count_local) == HAL_OK ) // enter in ascii 0x0A //HAL_UART_Receive(&huart1, &user_choice, 1,  8000)
			  {
				  if ( sprava_count_local == 1  && (  ( termin_mess == 3 || termin_mess == 4 ) || termin_mess == 6 )   ) // je v halvnom menu
				  {
					  if ( sprava_local[0] >= '0' &&  sprava_local[0] <= '9' )
					  {
						  snprintf(uart_mess_buf, sizeof(uart_mess_buf),"\r\nECHO:%c",sprava_local[0]);
						  HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), HAL_MAX_DELAY);
						  clear_char_array_len(uart_mess_buf, (uint16_t) (UART_MESS_LEN/10));

						  read_term = false;
						  send_term_mess = true;
						  read_info_not_send = true;

						  user_choice = ( sprava_local[0] - '0');

						  //sprava_local[0] = 0;
						 // sprava_count_local = 0;
					  }
					  else
					  {
						  snprintf(uart_mess_buf, sizeof(uart_mess_buf),"\r\n%c\r\nVolba musi byt cislo <0,9>!\r\n",sprava_local[0]);
						  HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), HAL_MAX_DELAY);
						  clear_char_array_len(uart_mess_buf, (uint16_t) (UART_MESS_LEN/5));
						  read_info_not_send = true;
					  }

					  if ( termin_mess == 6 ) { // Nastavenie komunikacie
				  	  	   switch(user_choice){
				  	  	   	   case 1: termin_mess =1; break;	// Posielanie
				  	  	   	   case 2: termin_mess =2; break;	// Prijimanie
				  	  	   	   case 3: termin_mess =3;break;	// Manchester kodovanie nastavenie
				  	  	   	   case 4: termin_mess =4;break;	// MSB/LSB nastavnie
				  	  	   	   case 5: end_term = true; break; // Nastavenie komunikacie
				  	  	   	   default: HAL_UART_Transmit(&huart1, (uint8_t *) "\r\nNieje v ponuke!\r\n", sizeof("\r\nNieje v ponuke!\r\n"), HAL_MAX_DELAY);
								  break; // ostan
				  	  	   }
					  }
					  else if ( termin_mess == 4 ) { // MSB/LSB nastavnie
						  switch(user_choice){
						  	  case 1: message_MSB_Frst_E_Set(true); break;	// Nastavi MSB first
						  	  case 2: message_MSB_Frst_E_Set(false); break;	// Nastavit to druhe
						  	  case 6: termin_mess =6; break; // Nastavenie komunikacie
						  	  default: HAL_UART_Transmit(&huart1, (uint8_t *) "\r\nNieje v ponuke!\r\n", sizeof("\r\nNieje v ponuke!\r\n"), HAL_MAX_DELAY);
						  	  	  break; // ostan
						  }
					  }
					  else if ( termin_mess == 3 ) { // Manchester kodovanie nastavenie
				  	  	  switch(user_choice){
				  	  	   	   case 1: manchester_IEEE_Set(true); break;	// Nastavi IEEE normu
				  	  	   	   case 2: manchester_IEEE_Set(false); break;	// Nastavit to druhe
				  	  	   	   case 6: termin_mess =6; break; // Nastavenie komunikacie
				  	  	   	   default: HAL_UART_Transmit(&huart1, (uint8_t *) "\r\nNieje v ponuke!\r\n", sizeof("\r\nNieje v ponuke!\r\n"), HAL_MAX_DELAY);
				  	  	   	   	   break; // ostan
				  	  	  }
					  }

				  }
				  else if ( sprava_count_local != 0 && termin_mess == 1 ) // je v menu posielanie
				  { // my_UART_receive_char_array(sprava_local, MAX_mess_received, &sprava_count_local)
					  snprintf(uart_mess_buf, sizeof(uart_mess_buf), "\r\nECHO: %s", sprava_local);
					  HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), HAL_MAX_DELAY);
					  clear_char_array_len(uart_mess_buf, (uint16_t) (UART_MESS_LEN/5));
					  if ( (sprava_local[0] >= '0' &&  sprava_local[0] <= '9') ) {
						  // nastavenie a monosti

					  }
					  else
					  {
						  // volaj funkciu na posielanie
					  }
				  }


				  sprava_count_local = clear_char_array_len( (char *)sprava_local, MAX_mess_received );

			  } // koniec citania s terminalu

			  if ( termin_mess == 2 ) // je v menu prijimania
			  {
				  //snprintf(uart_mess_buf, sizeof(uart_mess_buf), "\r\nECHO: %s", sprava_local);
				 // HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), HAL_MAX_DELAY);
				  if ( (sprava_local[0] >= '0' &&  sprava_local[0] <= '9') ) {
					  // nastavenie a monosti

				  }
				  else
				  {
					  // volaj funkciu na prijimanie
				  }
			  }
		  }

	  }

	  if (end_term) break;
	    //else {HAL_Delay(2000); }
	  	//}

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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */


  message = 0;

  //_Bool send_data_mode_enable_local = false, timer_counter_enable_local = false;

  volatile uint16_t received_mess_local[MAX_mess_received]={0};
  volatile uint8_t received_decod_mess_local[MAX_mess_received]={0};

  uint8_t messg_count_local= 0u, sprava_count_local =0;

  unsigned char sprava_local[MAX_mess_received] = {0}; // STM //unsigned char

  HAL_StatusTypeDef read_state;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  snprintf(uart_mess_buf, sizeof(uart_mess_buf), " Zaciatok behu.\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), 1);

  clear_char_array_len(uart_mess_buf, UART_MESS_LEN);


  //manchester_send_and_receive( false, false, received_mess_local, received_decod_mess_local, messg_count_local, sprava_local, sprava_count_local);
  volatile uint8_t termin_mess =6, user_choice = 0;
  volatile _Bool send_term_mess = true, end_term= false, read_term= false, read_info_not_send= true;



  read_info_not_send = false;





  while (1)
  {



 break;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }




  	//received_decod_mas
  clear_char_array_len(uart_mess_buf, UART_MESS_LEN);
  snprintf(uart_mess_buf, sizeof(uart_mess_buf), "\r\nKoniec.\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *) uart_mess_buf, sizeof(uart_mess_buf), 1);




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
  htim2.Init.Prescaler = 159999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 65535;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4.294967295E9;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  HAL_GPIO_WritePin(Manchester_Out_GPIO_Port, Manchester_Out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, UCPD_DBn_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Manchester_In_Pin */
  GPIO_InitStruct.Pin = Manchester_In_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Manchester_In_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pin : Manchester_Out_Pin */
  GPIO_InitStruct.Pin = Manchester_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Manchester_Out_GPIO_Port, &GPIO_InitStruct);

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
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_DisableIRQ(EXTI0_IRQn);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//


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
