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

0xAA : sent a number of time to help the received compute a signal average for the thresholding of analog values
0xD5 : synchronization byte to break preamble
0x02 : STX start of frame
N times Effective data excluding command symbols, with N < 32
0x03 : ETX end of frame
*/

//Start of what should be an include ...
#define WORD_LENGTH 10
#define SYNC_SYMBOL 0xD5
#define ETX 0x03
#define STX 0x02

//Fast manipulation of LED IO. 
//These defines are for a LED connected on D13
/*#define OUT_LED() DDRB |= (1 << 5);
#define SET_LED() PORTB |= (1 << 5)
#define CLR_LED() PORTB &= ~(1 << 5)
*/

//These defines are for a RGB led connected to D2, D3, D4
#define OUT_LED() DDRD |= ((1 << 2) | (1 << 3) | (1 << 4))
#define SET_LED() PORTD |= ((1 << 2) | (1 << 3) | (1 << 4))
#define CLR_LED() PORTD &= ~((1 << 2) | (1 << 3) | (1 << 4))


char frame_buffer [38] ; //buffer for frame
int frame_index = -1; // index in frame
int frame_size = -1  ; // size of the frame to be sent

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
      if(bit_counter == 0){
        frame_index ++ ;   
        //is there still bytes to send in the frame ?
        if(frame_index < frame_size){
          data_word = (frame_buffer[frame_index] << 1) | 0 | (1 << (WORD_LENGTH-1));
          bit_counter = WORD_LENGTH ;
        }else{
          //not more bytes setting indexes to -1
          frame_index = -1 ;
          frame_size = -1 ;
        }
      }
    }
  }else{ // keep sending ones if there is nothing to send
      if(!half_bit){ // first half of the bit (manchester encoding)
      CLR_LED();
      half_bit = 1 ;
    }else{// second half of the bit (manchester encoding)
      SET_LED();
      half_bit = 0 ;
      //Transmitter was IDLE and a new frame is ready to send
      if(frame_index < frame_size){      
          data_word = (frame_buffer[frame_index] << 1) | 0 | (1 << (WORD_LENGTH-1));
          bit_counter = WORD_LENGTH ;
      }
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
      // Set CS21 bit for 128 prescaler
      TCCR2B |= (1 << CS22) | (1 << CS20);
      break ; 
     case 5 :
      // Set CS21 bit for 256 prescaler
      TCCR2B |= (1 << CS22) | (1 << CS21) ;
      break ;
    case 6 :
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


void init_frame(char * frame){
  memset(frame, 0xAA, 3);
  frame[3] = SYNC_SYMBOL ;
  frame[4] = STX;
  frame_index = -1 ;
  frame_size = -1 ;
}

int create_frame(char * data, int data_size, char * frame){
  memcpy(&(frame[5]), data, data_size);
  frame[5+data_size] = ETX;
  return 1 ;
}

int write(char * data, int data_size){
  if(frame_index > 0) return -1 ;
  create_frame(data, data_size,frame_buffer);
  frame_index = 0 ;
  frame_size = data_size + 6 ;
}


// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  OUT_LED();
  init_frame(frame_buffer);
  cli();//stop interrupts
  setupTimer2(5, 48); // transmitter rate = 16_000_000/256/52 
  //setupTimer2(5, 255);
  sei();//allow interrupts

}


// the loop routine runs over and over again forever:
void loop() {
  write("Hello World", 11);
  delay(2);
}
