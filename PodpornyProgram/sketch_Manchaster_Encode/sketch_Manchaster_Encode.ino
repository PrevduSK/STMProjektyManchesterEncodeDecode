//  A. IDE ver 2.3.2
const int pin_manch_out = 2;
char from_user;


#define TIME_DELAY_TRANSMIT 140 // 125 pre 8 bit za 1s

void manchester_transmit_char(uint8_t message, bool print_hex= false, bool print_dec_ofMess= false) {
  bool manchester_IEEE = true;
  bool MSB_Frst_E = true;

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
      if ((comp_bit ^ (clk & 0x01))) {
        manchester_mass |= 1;
        digitalWrite(pin_manch_out, HIGH);
      } else {
        manchester_mass |= 0;
        digitalWrite(pin_manch_out, LOW);
      }
    } else {
      if (!(comp_bit ^ (clk & 0x01))) {
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

void manchester_transmit_string(unsigned char message[], uint8_t length) 
{
  for (uint8_t i = 0; i < length; i++) {
    manchester_transmit_char( message[i], true );
  }
  manchester_transmit_char(0);
}

void setup() {
  pinMode(pin_manch_out, OUTPUT);
  digitalWrite(pin_manch_out, LOW);
  Serial.begin(9600);
  Serial.println("Zaciatok!");

  //for(uint i=0; i <)
  delay(1200);
  Serial.println("Posielanie!");
  delay(400);
  //manchester_transmit_char(0);
  //manchester_transmit_char(0xff); //manchester_transmit_massage(0b11111111);
  
  from_user = 0;
  unsigned char odkaz[] ="Prevdu"; //  
  manchester_transmit_string( odkaz, 7);
  //manchester_transmit_char((uint8_t) 'd', true, true); // 0x96A9 ,0b1001 0110 1010 1001
  Serial.println("Konice!");
  //manchester_transmit_char(0xAA);// 0x6666
  //manchester_transmit_char(0x55);// 0x9999
  
 /* while(1)
  {
    //if (Serial.available() > 0) {
      Serial.println("zadaj znak");
      from_user = Serial.read();
      if (from_user == 'e' || from_user == 'E') {break;}
      manchester_transmit_massage((uint8_t) from_user);
      Serial.println(from_user);
      delay(100);
    //}
  } */
  
  
}

void loop() {
  // put your main code here, to run repeatedly:
   
}
