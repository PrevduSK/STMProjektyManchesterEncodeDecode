//  A. IDE ver 2.3.2
const int pin_manch_out = 2;
const int pin_manch_in = 3;
char from_user;

const boolean manchester_IEEE= true, MSB_Frst_E= true;
volatile boolean edge_rise_fall_Flag = false, tim_count_reset_Flag = false;

volatile uint16_t tick_count_current_edge =0u, tick_current_timer =0u;

volatile uint8_t  massage,  clk = 0, count_edge = 0u, comp_bit= 0;

volatile uint16_t  manchester_mass;

#define MAX_mass_recived 30U
#define Time_delay_Count_not_set 40u

#define UART_MASS_LEN 50
char uart_mass_buf[UART_MASS_LEN];



#define TIME_DELAY_TRANSMIT 100 // 125 pre 8 bit za 1s
// ------------------------------ send and encode ----------------------------
void manchester_encode_and_send_char(uint8_t message, boolean print_hex= false, boolean print_dec_ofMess= false);
void manchester_send_encoded_data(uint16_t message_encod, boolean print_hex= false, boolean print_dec_ofMess= false);

void manchester_encode_and_send_char_array(unsigned char message[], uint8_t length);

uint16_t manchester_encode_char(uint8_t message, boolean print_hex= false, boolean print_dec_ofMess= false);

// ----------------------------- receive and decode ---------------------------
void manchester_receive_data_array( volatile uint16_t recived_mess[], uint8_t* messg_count ); 
void manchester_receive_data_array_via_timer_count( volatile uint16_t recived_mess[], uint8_t* messg_count );

void manchester_decode_data_array( volatile uint16_t recived_mess[], volatile uint8_t recived_decod_mass[], uint8_t* messg_count );

void clear_char_array_len(char * array, uint16_t a_len);

// ------------------------------------------------------------------
void setup() {
  pinMode(pin_manch_out, OUTPUT);
  digitalWrite(pin_manch_out, LOW);

  pinMode(pin_manch_in, INPUT_PULLUP);
  

  boolean const send_data_mode_enable = true;

  volatile uint16_t recived_mess[MAX_mass_recived]={0};
  volatile uint8_t recived_decod_mass[MAX_mass_recived]={0};
  uint8_t messg_count= 0u;

  Serial.begin(9600);
 // Serial.println("Kontrola ci je interupt pin"); // Serial.println(digitalPinToInterrupt(pin_manch_in), DEC);
  Serial.println("Zaciatok!");

  

  if (send_data_mode_enable) 
  {
    delay(1200);
    Serial.println("Send m!"); //Posielanie!
    delay(400);
    unsigned char odkaz[] ="Prevdu"; //  Prevdu
    manchester_encode_and_send_char_array( odkaz, sizeof(odkaz));
    //manchester_encode_and_send_char( (uint8_t) 'd', true );
    //manchester_encode_and_send_char(0xff, true);// 0xff
  }
  else
  {   
   
    Serial.println("Receiv m!"); //Posielanie!
    attachInterrupt(digitalPinToInterrupt(pin_manch_in), gpio_irq_handler, CHANGE);
    edge_rise_fall_Flag = false;
	  count_edge = 0;
    // timer 1 16bit set to compare match mode
     cli();
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1B |= 0b00001101;  // presceler set 1024 B00000101 ~ clk/1024// B00000001 clk/1 // bit 3. WGM12 max val is teke from OCR1A
   // TCNT1 = 0;  // Time1 value
   TIMSK1 |= B00000010;        //Set OCIE1A to 1 so we enable compare match A 
    OCR1A = 62500; //Finally we set compare register A to this valu
    // 4 s ak prsceler je 1024
    sei();
    
    manchester_receive_data_array_via_timer_count( recived_mess, &messg_count);

    clear_char_array_len(uart_mass_buf, UART_MASS_LEN);
	  snprintf(uart_mass_buf, sizeof(uart_mass_buf), "mass_count: %d", messg_count);
	  Serial.println(uart_mass_buf);
    manchester_decode_data_array( recived_mess, recived_decod_mass, &messg_count);
  }


  
  //char buffer[40];
  //sprintf(buffer, "size of array %d\n", sizeof(odkaz)  );
  //Serial.println(buffer);

 
  //manchester_encode_and_send_char((uint8_t) 'c', true);  // 0x96A9 ,0b1001 0110 1010 1001
  Serial.println("Konice!");
  //manchester_encode_and_send_char(0);
  //manchester_encode_and_send_char(0xff); //(0b11111111);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// ---------------------------------------------------------------------
void gpio_irq_handler()
{
  tick_count_current_edge = millis();
  tick_current_timer = TCNT1;
  TCNT1 = 0;
	edge_rise_fall_Flag = true;
	count_edge++;
}


void manchester_receive_data_array( volatile uint16_t recived_mess[], uint8_t* messg_count )
{

	register volatile uint32_t interval_btw_tick =0u, time_delay =0u, timer_interval =0u, actual_tick = 0u, last_time_tick = 0u, tick_last_timer = 0u, tick_count_last_edge =0u;

	register volatile uint32_t count_from_if_cond =0u;
	uint32_t tick_count_prim_and_half =0u, tick_count_prim =0u, tick_count_prim_half = 0u;

	register volatile boolean bit_read_enab = false ;
	boolean end_word = false;
  volatile uint8_t count_bit= 0;

	massage = 0;

	while (1)
	{
		// --Non blocking wait to time interval for read from GPIO
		if ( ( !edge_rise_fall_Flag  && !tim_count_reset_Flag ) && count_edge >= 2 ) // F and F  and 2,3 ...
		{
			actual_tick = millis();
			if ( tick_last_timer != 0 ) {
				timer_interval = actual_tick - tick_last_timer;
				// --When is time it set flag to do read from GPIO without waiting to interrupt
				if (   ( timer_interval > tick_count_prim ) && ( timer_interval <=  tick_count_prim_and_half  )   ) //
				{
					tick_current_timer = millis();
					tim_count_reset_Flag = true;
				}
			}

			count_from_if_cond++;
			if ( count_from_if_cond == 0xFA0000 ){
				clear_char_array_len(uart_mass_buf, UART_MASS_LEN);
				snprintf(uart_mass_buf, sizeof(uart_mass_buf), "edge: %d, bit: %d", count_edge, count_bit );
				Serial.println(uart_mass_buf);
        clear_char_array_len(uart_mass_buf, UART_MASS_LEN);
				snprintf(uart_mass_buf, sizeof(uart_mass_buf), "Nedokaze pocitat cas.");
				Serial.println(uart_mass_buf);
        		  //end_word = false;
	  		break;
			}
		}

		if ( edge_rise_fall_Flag  || tim_count_reset_Flag )  //  == true OR  == true  //|| tim_count_reset_Flag
		{

			//__HAL_TIM_GET_COUNTER(&htim2);
			time_delay = (tick_count_prim != 0) ?  tick_count_prim_half : Time_delay_Count_not_set;

			last_time_tick = edge_rise_fall_Flag ? tick_count_last_edge :  tick_last_timer;

			if ( ( millis() - last_time_tick) > time_delay )  
			{

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
				tick_count_prim_and_half = (uint32_t) tick_count_prim*1.5f; 

			}

			if ( bit_read_enab ) // if true read
			{
				bit_read_enab = false;

				if ( digitalRead(pin_manch_in) == LOW )  //&& count == 0
				{
          
					// manchester_mass |= 0; // manchester_mass <<= 1;
				}
				else
				{ // read start val, Low
					manchester_mass |= 1;
				}

				count_bit++;

				if (count_bit >= 16) {
					//HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
					//HAL_NVIC_DisableIRQ(EXTI0_IRQn);
					// count++;
					recived_mess[*messg_count] = manchester_mass;
					if (*messg_count < MAX_mass_recived){(*messg_count)++;}
					else {end_word = true;}
					if ( (manchester_mass == 0xaaaa || manchester_mass == 0x0) || (manchester_mass == 0x5555 || manchester_mass == 0xffff) ) {end_word = true;}


					//clear_char_array_len(uart_mass_buf,UART_MASS_LEN);
					//  HAL_UART_Transmit(&huart1, (uint8_t *) "\r\n", sizeof("\r\n"), 1);
					//snprintf(uart_mass_buf, sizeof(uart_mass_buf), "edge: %d, bit: %d\r\n", count_edge, count_bit );
					//HAL_UART_Transmit(&huart1, (uint8_t *) uart_mass_buf, sizeof(uart_mass_buf), 2);
					count_bit = 0;
					manchester_mass = 0;
				}
				else {  manchester_mass <<= 1; }
				count_from_if_cond = 0;
			}

		}
	  		  //tim2_count = __HAL_TIM_GET_COUNTER(&htim2); // from timer
	  		  //__HAL_TIM_SET_COUNTER(&htim2, 19);
	  		 // snprintf(uart_mass_buf, sizeof(uart_mass_buf), "Cas: %ld\r\n", tim2_count);
	  		 // HAL_UART_Transmit(&huart1, (uint8_t *) uart_mass_buf, sizeof(uart_mass_buf), 1);
	  		  //return; //end_word
		if ( end_word ) { //messg_count == MAX_mass_recived  //count == 31 ||
      detachInterrupt(digitalPinToInterrupt(pin_manch_in));
	
			clear_char_array_len(uart_mass_buf, UART_MASS_LEN);
			snprintf(uart_mass_buf, sizeof(uart_mass_buf), ": %c |%x ", recived_mess[0], recived_mess[0]); // "%c :%x" , manchester_mass, manchester_mass
			Serial.println(uart_mass_buf);
			break;
		}
	}
	return;
}


void manchester_receive_data_array_via_timer_count( volatile uint16_t recived_mess[], uint8_t* messg_count )
{

	register volatile uint16_t interval_btw_tick =0u, time_delay =0u, timer_interval =0u, actual_tick = 0u, last_time_tick = 0u, tick_last_timer = 0u, tick_count_last_edge =0u;

	register volatile uint32_t count_from_if_cond =0u;
	uint32_t tick_count_prim_and_half =0u, tick_count_prim =0u, tick_count_prim_half = 0u;

	register volatile boolean bit_read_enab = false ;
	boolean end_word = false;
  volatile uint8_t count_bit= 0;

	massage = 0;

	while (1)
	{
		// --Non blocking wait to time interval for read from GPIO
    /*
		if ( ( !edge_rise_fall_Flag  && !tim_count_reset_Flag ) && count_edge >= 2 ) // F and F  and 2,3 ...
		{
			actual_tick = millis();
			if ( tick_last_timer != 0 ) {
				timer_interval = actual_tick - tick_last_timer;
				// --When is time it set flag to do read from GPIO without waiting to interrupt
				if (   ( timer_interval > tick_count_prim ) && ( timer_interval <=  tick_count_prim_and_half  )   ) //
				{
					tick_current_timer = millis();
					tim_count_reset_Flag = true;
				}
			} 

			count_from_if_cond++;
			if ( count_from_if_cond == 0xFA0000 ){
				clear_char_array_len(uart_mass_buf, UART_MASS_LEN);
				snprintf(uart_mass_buf, sizeof(uart_mass_buf), "edge: %d, bit: %d", count_edge, count_bit );
				Serial.println(uart_mass_buf);
        clear_char_array_len(uart_mass_buf, UART_MASS_LEN);
				snprintf(uart_mass_buf, sizeof(uart_mass_buf), "Nedokaze pocitat cas.");
				Serial.println(uart_mass_buf);
        		  //end_word = false;
	  		break;
			}
		}*/

		if ( edge_rise_fall_Flag  || tim_count_reset_Flag )  //  == true OR  == true  //|| tim_count_reset_Flag
		{

			//__HAL_TIM_GET_COUNTER(&htim2);
			time_delay = (tick_count_prim != 0) ?  tick_count_prim_half : Time_delay_Count_not_set;

			//last_time_tick = edge_rise_fall_Flag ? tick_count_last_edge :  tick_last_timer;
      

			if ( ( TCNT1 - tick_current_timer) > time_delay )  //timer_value TCNT0 // millis()
			{

				if ( edge_rise_fall_Flag )
				{
					interval_btw_tick = tick_current_timer;// - tick_last_timer //tick_count_current_edge - tick_count_last_edge;
					//tick_count_last_edge = tick_count_current_edge;
					//tick_last_timer = tick_current_timer;//tick_count_current_edge;
				}
			/*	else
				{
					tick_count_last_edge = tick_current_timer;
					tick_last_timer = tick_current_timer;
				} */

				bit_read_enab = true;
				edge_rise_fall_Flag = false;
				tim_count_reset_Flag = false;
			}

			if ( count_edge == 2  )
			{
				tick_count_prim = interval_btw_tick;
				OCR1A = tick_count_prim;                                    // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, tick_count_prim);
				tick_count_prim_half =   (uint32_t) (tick_count_prim/2 );
			//	tick_count_prim_and_half = (uint32_t) tick_count_prim*1.5f; 

			}

			if ( bit_read_enab ) // if true read
			{
				bit_read_enab = false;

				if ( digitalRead(pin_manch_in) == LOW )  //&& count == 0
				{
          
					// manchester_mass |= 0; // manchester_mass <<= 1;
				}
				else
				{ // read start val, Low
					manchester_mass |= 1;
				}

				count_bit++;

				if (count_bit >= 16) {
					//HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
					//HAL_NVIC_DisableIRQ(EXTI0_IRQn);
					// count++;
					recived_mess[*messg_count] = manchester_mass;
					if (*messg_count < MAX_mass_recived){(*messg_count)++;}
					else {end_word = true;}
					if ( (manchester_mass == 0xaaaa || manchester_mass == 0x0) || (manchester_mass == 0x5555 || manchester_mass == 0xffff) ) {end_word = true;}


					//clear_char_array_len(uart_mass_buf,UART_MASS_LEN);
					//  HAL_UART_Transmit(&huart1, (uint8_t *) "\r\n", sizeof("\r\n"), 1);
					//snprintf(uart_mass_buf, sizeof(uart_mass_buf), "edge: %d, bit: %d\r\n", count_edge, count_bit );
					//HAL_UART_Transmit(&huart1, (uint8_t *) uart_mass_buf, sizeof(uart_mass_buf), 2);
					count_bit = 0;
					manchester_mass = 0;
				}
				else {  manchester_mass <<= 1; }
				count_from_if_cond = 0;
			}

		}
	  		  //tim2_count = __HAL_TIM_GET_COUNTER(&htim2); // from timer
	  		  //__HAL_TIM_SET_COUNTER(&htim2, 19);
	  		 // snprintf(uart_mass_buf, sizeof(uart_mass_buf), "Cas: %ld\r\n", tim2_count);
	  		 // HAL_UART_Transmit(&huart1, (uint8_t *) uart_mass_buf, sizeof(uart_mass_buf), 1);
	  		  //return; //end_word
		if ( end_word ) { //messg_count == MAX_mass_recived  //count == 31 ||
      detachInterrupt(digitalPinToInterrupt(pin_manch_in));
	
			clear_char_array_len(uart_mass_buf, UART_MASS_LEN);
			snprintf(uart_mass_buf, sizeof(uart_mass_buf), ": %c |%x ", recived_mess[0], recived_mess[0]); // "%c :%x" , manchester_mass, manchester_mass
			Serial.println(uart_mass_buf);
			break;
		}
	}
	return;
}


void manchester_decode_data_array( volatile uint16_t recived_mess[], volatile uint8_t recived_decod_mass[], uint8_t* messg_count )
{
	register volatile uint16_t mask_16;
	if (MSB_Frst_E){ mask_16= 0x8000; }
	else { mask_16= 0x001; }

	while ( (*messg_count) >0)
	{ // --------------Decoding of manchaster
		(*messg_count)--;
	    manchester_mass = recived_mess[(*messg_count)]; //(messg_count-1)

	    if(MSB_Frst_E){ mask_16 = 0x8000; }
	    else { mask_16 = 0x01; }
	    massage= 0;

	    for (int bite_cnt = 0; bite_cnt < 8; bite_cnt++) {
	    	if (manchester_IEEE)
	    	{  // (manchester_mass & mask_16 ) ==0 &&  ((manchester_mass & (mask_16>>1) ) > 0)
	    		if ( !(manchester_mass & mask_16 ) && (manchester_mass & (mask_16>>1) ) ) // 0b01 = 1 dec -> 1
	    		{
	    			massage |= 1;

	    		} else if ( !(manchester_mass & (mask_16>>1) ) ) // 0b10 -> 0
	    		{
	    			massage |= 0;
	    		}
	    	}
	    	else
	    	{
	    		if ( (manchester_mass & mask_16) ) // 0b10 = 1 dec -> 1
	    		{  massage |= 1; }
	    		else // 0b01 -> 0
		        {  massage |= 0; }
	    	}
	    	if(MSB_Frst_E){ mask_16 >>= 2; }
	    	else { mask_16 <<= 2; }

	    	if ( bite_cnt < 7) {
	    		massage <<= 1; // ked je vstup 11 vypise 00
	    	}
	    }

	    recived_decod_mass[(*messg_count)] = massage; //(messg_count-1)
	    //messg_count--

	    clear_char_array_len(uart_mass_buf, UART_MASS_LEN);
	    snprintf(uart_mass_buf, sizeof(uart_mass_buf), "%c %d \t %x", recived_decod_mass[(*messg_count)], recived_decod_mass[*messg_count], manchester_mass);
	    Serial.println(uart_mass_buf);
	}
	return;
}



void clear_char_array_len(char * array, uint16_t a_len)
{
	for (uint16_t i =0; i < a_len; i++) { *(array + i) = 0; }
}



void manchester_encode_and_send_char_array(unsigned char message[], uint8_t length) 
{ 
  register uint16_t mess_buf[length];
  for (uint8_t i = 0; i < length; i++) {
    mess_buf[i] = manchester_encode_char(message[i], true);
  }
  //manchester_encode_and_send_char(0xAA);
  for (uint8_t i = 0; i < length; i++) {
    manchester_send_encoded_data( mess_buf[i] );
  }
  manchester_encode_and_send_char(0);
}


uint16_t manchester_encode_char(uint8_t message, bool print_hex= false, bool print_dec_ofMess= false) {
 // bool manchester_IEEE = true;
 // bool MSB_Frst_E = true;

  byte count = 0, comp_bit = 0; // uint8_t as byte
  uint16_t manchester_mass = 0;

  byte mask_8;
  if (MSB_Frst_E) {
    mask_8 = 0x80;
  } else {
    mask_8 = 0x01;
  }

  bool clk = 0;

  while (true) {
    clk = !clk;  // toggle clock

    // Manchester encoding as per IEEE 802.3
    // LSB first
    comp_bit = (message & mask_8) ? 1 : 0;
    if (manchester_IEEE) {
      if ( comp_bit ^ (clk & 0x01) ) {
        manchester_mass |= 1;
        
      } else {
        manchester_mass |= 0;
        
      }
    } else {
      if (!(comp_bit ^ (clk & 0x01))) {
        manchester_mass |= 1;
        
      } else {
        manchester_mass |= 0;
        
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
        Serial.println(manchester_mass, HEX);
      }
      if(print_dec_ofMess){
        Serial.println(message, DEC);
      }

      return manchester_mass;
    }

    count++;
    manchester_mass <<= 1;  // shift the Manchester code
  }
}


void manchester_send_encoded_data(uint16_t message_encod, bool print_hex= false, bool print_dec_ofMess= false) {
 // bool MSB_Frst_E = true;

  byte count = 0, comp_bit = 0; // uint8_t as byte

  uint16_t mask_16;
  if (MSB_Frst_E) {
    mask_16 = 0x8000;
  } else {
    mask_16 = 0x0001;
  }

  while (true) {
    // Manchester encoding as per IEEE 802.3
    // LSB first
    comp_bit = (message_encod & mask_16) ? 1 : 0;
    if (comp_bit) {
      digitalWrite(pin_manch_out, HIGH);
    } else {
      digitalWrite(pin_manch_out, LOW);
    } 

    if (MSB_Frst_E) {
      mask_16 >>= 1;  // shift mask to the right
    } else {
      mask_16 <<= 1;  // shift mask to the left
    }

    if (count == 15) {
      if(print_hex) {
        Serial.println(message_encod, HEX);
      }
      if(print_dec_ofMess){
        Serial.println(message_encod, DEC);
      }

      delay(TIME_DELAY_TRANSMIT);
      break;
    }

    count++;
    delay(TIME_DELAY_TRANSMIT);  // 100ms delay
  }

}


void manchester_encode_and_send_char(uint8_t message, bool print_hex= false, bool print_dec_ofMess= false) {
 // bool manchester_IEEE = true;
 // bool MSB_Frst_E = true;

  byte volatile count = 0, comp_bit = 0; // uint8_t as byte
  uint16_t volatile manchester_mass = 0;

  byte volatile mask_8;
  if (MSB_Frst_E) {
    mask_8 = 0x80;
  } else {
    mask_8 = 0x01;
  }

  bool clk = 0;

  while (true) {
    clk = !clk;  // toggle clock

    // Manchester encoding as per IEEE 802.3
    // LSB first
    comp_bit = (message & mask_8) ? 1 : 0;
    if (manchester_IEEE) {
      if ( (comp_bit ^ (clk & 0x01)) ) {
        manchester_mass |= 1;
        digitalWrite(pin_manch_out, HIGH);
      } else {
        manchester_mass |= 0;
        digitalWrite(pin_manch_out, LOW);
      }
    } else {
      if ( !(comp_bit ^ (clk & 0x01)) ) {
        manchester_mass |= 1;
        digitalWrite(pin_manch_out, HIGH);
      } else {
        manchester_mass |= 0;
        digitalWrite(pin_manch_out, LOW);
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
        Serial.println(manchester_mass, HEX);
      }
      if(print_dec_ofMess){
        Serial.println(message, DEC);
      }

      delay(TIME_DELAY_TRANSMIT);
      break;
    }

    count++;
    manchester_mass <<= 1;  // shift the Manchester code

    delay(TIME_DELAY_TRANSMIT);  // 100ms delay
  }
  //digitalWrite(pin_manch_out, LOW);
  
}

ISR(TIMER1_CAPT_vect) {
  tick_current_timer = TCNT1;
  TCNT1 = 0; //First, set the timer back to 0 so it resets
  tim_count_reset_Flag = true;

}