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

#include <TimerOne.h>
#include <util/atomic.h>
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
/*#define OUT_LED() DDRD |= ((1 << 2) | (1 << 3) | (1 << 4))
#define SET_LED() PORTD |= ((1 << 2) | (1 << 3) | (1 << 4))
#define CLR_LED() PORTD &= ~((1 << 2) | (1 << 3) | (1 << 4))
*/
#define OUT_LED() DDRD |= ((1 << 2))
#define SET_LED() PORTD |= ((1 << 2))
#define CLR_LED() PORTD &= ~((1 << 2))


unsigned char frame_buffer [38] ; //buffer for frame
char frame_index = -1; // index in frame
char frame_size = -1  ; // size of the frame to be sent

//state variables of the manchester encoder
unsigned char bit_counter = 0 ;
unsigned short data_word = 0 ;  //8bit data + start + stop
unsigned char half_bit = 0 ;
unsigned long int manchester_data ;

void to_manchester(unsigned char data, unsigned long int * data_manchester){
  unsigned int i ;
 (*data_manchester) = 0x02 ; // STOP symbol
 (*data_manchester) = (*data_manchester) << 2 ;
  for(i = 0 ; i < 8; i ++){
    if(data & 0x80) (*data_manchester) |=  0x02  ; // data LSB first
    else (*data_manchester) |= 0x01 ;
    (*data_manchester) = (*data_manchester) << 2 ;
    data = data << 1 ; // to next bit
  }
  (*data_manchester) |= 0x01 ; //START symbol
}

//emitter interrupt
void emit_half_bit(){
     if(manchester_data & 0x01){
       SET_LED();
     }else{
       CLR_LED();
     }
     bit_counter -- ;
     manchester_data = (manchester_data >> 1);
     if(bit_counter == 0){   
        //is there still bytes to send in the frame ?
        manchester_data = 0xAAAAAAAA ; // keep sending ones if nothing to send
        if(frame_index >= 0 ){
          if(frame_index < frame_size){
            /*Serial.println(frame_index, DEC);
            Serial.println(frame_buffer[frame_index], HEX);*/
            to_manchester(frame_buffer[frame_index], &manchester_data);
            frame_index ++ ;
          }else{
            frame_index = -1 ;
            frame_size = -1 ;
          }
        }
        bit_counter = WORD_LENGTH * 2 ;
        //Serial.println(manchester_data, BIN);
      }
}

void init_frame(unsigned char * frame){
  memset(frame, 0xAA, 3);
  frame[3] = SYNC_SYMBOL ;
  frame[4] = STX;
  frame_index = -1 ;
  frame_size = -1 ;
}

int create_frame(char * data, int data_size, unsigned char * frame){
  memcpy(&(frame[5]), data, data_size);
  frame[5+data_size] = ETX;
  return 1 ;
}

int write(char * data, int data_size){
  if(frame_index >=  0) return -1 ;
  if(data_size > 32) return -1 ;
  create_frame(data, data_size,frame_buffer);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    frame_index = 0 ;
    frame_size = data_size + 6 ;
  }
  return 0 ;
}

void init_emitter(){
  manchester_data = 0xFFFFFFFF ;
  bit_counter = WORD_LENGTH * 2 ;
}

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  OUT_LED();
  init_frame(frame_buffer);
  init_emitter();
  Timer1.initialize(800); //1200 bauds
  Timer1.attachInterrupt(emit_half_bit); 
}


// the loop routine runs over and over again forever:
char * msg = "Hello World" ;
void loop() {
  static int i = 0 ;
  char com_buffer [32] ;
  memcpy(com_buffer, msg, 11);
  com_buffer[11] = i + '0' ;
  if(write(com_buffer, 12) < 0){
    delay(10);
  }else{
    i ++ ; 
    if(i > 9) i = 0 ;
  }
}
