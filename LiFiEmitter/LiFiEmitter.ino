/*
LiFi Emitter and Receiver

The purpose of this demos is to demonstrate data communication using a pair of blue LED (one led as emitter one led as receiver). 
Communication can go at up to 600bs (can depend on led quality) 


Hardware is the following :


I/O 13 ------------- led -------------- GND

Using a blue led should not require resistor, one may be needed for red or green


A byte is sent as follow :

Start(0) 8bit data Stop(1)
 
Each bit is coded in manchester with 
0 -> 10
1 -> 01


A data frame is formatted as follow :

0x55 or 0xAA : sent a number of time to help the received compute a signal average for the thresholding of analog values
0xD5 : synchronization byte to indicate start of a frame
0x02 : STX start of frame
N times Effective data excluding command symbols
0x03 : ETX end of frame
*/

//Start of what should be an include ...
#define WORD_LENGTH 10
#define LED_PIN 13
//Fast manipulation of LED IO. 
#define SET_LED() PORTB |= (1 << 5)
#define CLR_LED() PORTB &= ~(1 << 5)


char * msg = "\xAA\xD5\x02Visible Light Communication est une liaison de donnees utilisant l'eclairage ambiant\x03" ;

int msg_length = strlen(msg);
int msg_index = 0 ;

//state variables of the manchester encoder
unsigned char bit_counter = 0 ;
unsigned short data_word = 0 ;  //8bit data + start + stop
unsigned char half_bit = 0 ;

//emitter interrupt
ISR(TIMER2_COMPA_vect){
  if(bit_counter > 0){ // if there is bits to send
    unsigned char data_bit = (data_word >> (WORD_LENGTH - bit_counter) ) & 0x01 ; //LSB first  
    #ifdef INVERT_BIT
      data_bit = (~data_bit) & 0x01 ;
    #endif
    if(!half_bit){ // first half of the bit (manchester encoding)
      if(data_bit){
          
          CLR_LED();
      }else{
          SET_LED();
      }
      half_bit = 1 ;
    }else{ // second half of the bit (manchester encoding)
      if(data_bit){
          SET_LED();
      }else{
          CLR_LED();
      }
      half_bit = 0 ;
      bit_counter -- ;
    }
  }else{ // keep sending ones if there is nothing to send
      if(!half_bit){ // first half of the bit (manchester encoding)
      CLR_LED();
      half_bit = 1 ;
    }else{// second half of the bit (manchester encoding)
      SET_LED();
      half_bit = 0 ;
    } 
  }
}

void setupTimer2(unsigned char prescaler, unsigned int period){
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = period;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  
  switch(prescaler){
    case 0 :
      // Set CS20 bit for no prescaler
      TCCR2B |= (1 << CS20);
    case 1 :
      // Set CS21 bit for 8 prescaler
      TCCR2B |= (1 << CS21);
    case 2 :
      // Set CS21 bit for 64 prescaler
      TCCR2B |= (1 << CS21) | (1 << CS20);
      break ;   
    case 3 :
      // Set CS21 bit for 128 prescaler
      TCCR2B |= (1 << CS22) ;
      break ; 
    case 4 :
      // Set CS21 bit for 256 prescaler
      TCCR2B |= (1 << CS22) | (1 << CS21);
      break ; 
     case 5 :
      // Set CS21 bit for 1024 prescaler
      TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
      break ;  
    default :
      TCCR2B |= (1 << CS20);
      break ;
      
  }
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A); 
  
}


int send_symbol(unsigned char data){
   if(bit_counter > 0) return -1 ;
   //           DATAT | START - STOP            
   data_word = (data << 1) | 0 | (1 << (WORD_LENGTH-1));
   //Serial.println(data_word, BIN);
   while(half_bit); // wait for sender to be in sync, sending of the first half of the symbol
   bit_counter = WORD_LENGTH ;
   return 0 ;
}


// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  pinMode(13, OUTPUT);  
  pinMode(12, OUTPUT); 
  cli();//stop interrupts
  setupTimer2(4, 48); // transmitter rate = 16_000_000/256/52 
  //setupTimer2(5, 255);
  sei();//allow interrupts

}


// the loop routine runs over and over again forever:
void loop() {
  if(send_symbol(msg[msg_index]) >= 0){
     msg_index ++ ;
     if(msg_index >= msg_length) msg_index = 0 ;
  }
}
