#include <TimerOne.h>

/*
LiFi Emitter and Receiver

The purpose of this demos is to demonstrate data communication using a pair of blue LED (one led as emitter one led as receiver). 
Communication can go at up to 600bs (can depend on led quality) 


Receiver hardware :

         |----1Mohm-----|
A0 ------|--- +led- ----|-------GND 

A byte is sent as follow :

Start(0) 8bit data Stop(1), LSB first : 0 b0 b1 b2 b3 b4 b5 b6 b7 1
 
Each bit is coded in manchester with 
time is from left to right 
0 -> 10
1 -> 01


A data frame is formatted as follow :

0xAA : sent a number of time to help the receiver compute a signal average for the thresholding of analog values
0xD5 : synchronization byte to indicate start of a frame, breaks the regularity of the 0x55 pattern to be easily 
0x02 : STX start of frame
N times Effective data excluding command symbols, max length 32 bytes
0x03 : ETX end of frame
*/


//#define DEBUG

enum receiver_state {
  IDLE, //waiting for sync
  SYNC, //synced, waiting for STX
  START, //STX received
  DATA //receiving DATA
};
enum receiver_state frame_state = IDLE ;

//This defines receiver properties
#define SENSOR_PIN 3
#define SYMBOL_PERIOD 500
#define SAMPLE_PER_SYMBOL 4
#define WORD_LENGTH 10 // a byte is encoded as a 10-bit value with start and stop bits
#define SYNC_SYMBOL 0xD5 // this symbol breaks the premanble of the frame
#define ETX 0x03 // End of frame symbol
#define STX 0x02 //Start or frame symbol 


// global variables for frame decoding
char frame_buffer[38] ;
int frame_index  = -1 ;
int frame_size = -1 ;

//state variables of the thresholder
unsigned int signal_mean = 0 ;
unsigned long acc_sum = 0 ; //used to compute the signal mean value
unsigned int acc_counter = 0 ;

//manechester decoder state variable
long shift_reg = 0;


//Start of ADC managements functions
void ADC_setup(){
  ADCSRA =  bit (ADEN);                      // turn ADC on
  ADCSRA |= bit (ADPS0) |  bit (ADPS1) | bit (ADPS2);  // Prescaler of 128
  ADMUX  =  bit (REFS0) | bit (REFS1);    // internal 1.1v reference
  //ADMUX  =  bit (REFS0) ;   // external 5v reference
}

void ADC_start_conversion(int adc_pin){
  ADMUX &= ~(0x07) ; //clearing enabled channels
  ADMUX  |= (adc_pin & 0x07) ;    // AVcc and select input port
  bitSet (ADCSRA, ADSC) ;
}

int ADC_read_conversion(){
 while(bit_is_set(ADCSRA, ADSC));
 return ADC ;
}
//End of ADC management functions

#define START_SYMBOL 0x02
#define STOP_SYMBOL 0x01
#define START_STOP_MASK  ((STOP_SYMBOL << 20) | (START_SYMBOL << 18) | STOP_SYMBOL) //STOP/START/16bits/STOP
#define SYNC_SYMBOL_MANCHESTER  (0x6665)
inline int is_a_word(long  * manchester_word, int time_from_last_sync, unsigned int * detected_word){
        if(time_from_last_sync >= 20  || frame_state == IDLE){ // we received enough bits to test the sync      
            if(((*manchester_word) & START_STOP_MASK) == (START_STOP_MASK)){ // testing first position 
                  (*detected_word) = ((*manchester_word) >> 2) & 0xFFFF;
                  if(frame_state == IDLE){
                     if((*detected_word) == SYNC_SYMBOL_MANCHESTER) return 2 ;
                  }
                  return 1 ;
                  // byte with correct framing
            }else if(frame_state != IDLE && time_from_last_sync == 20){
               (*detected_word)= ((*manchester_word) >> 2) & 0xFFFF;
               return 1 ;
            }
          }
          return 0 ;
}

inline int insert_edge( long  * manchester_word, char edge, int edge_period, int * time_from_last_sync, unsigned int * detected_word){
   int new_word = 0 ;
   int is_a_word_value = 0 ;
   int sync_word_detect = 0 ;
   if( ((*manchester_word) & 0x01) != edge ){ //mak sure we don't have same edge ...
             if(edge_period > (SAMPLE_PER_SYMBOL+1)){
                unsigned char last_bit = (*manchester_word) & 0x01 ;
                (*manchester_word) = ((*manchester_word) << 1) | last_bit ; // signal was steady for longer than a single symbol, 
                (*time_from_last_sync) += 1 ;
                is_a_word_value = is_a_word(manchester_word, (*time_from_last_sync), detected_word);
                if(is_a_word_value > 0){ //found start stop framing
                   new_word = 1 ;
                  (*time_from_last_sync) =  0 ;
                  if(is_a_word_value > 1) sync_word_detect = 1 ; //we detected framing and sync word in manchester format
                }
             }
             //storing edge value in word
             if(edge < 0){
              (*manchester_word) = ( (*manchester_word) << 1) | 0x00 ; // signal goes down
             }else{
              (*manchester_word) = ( (*manchester_word) << 1) | 0x01 ; // signal goes up
             }
             (*time_from_last_sync) += 1 ;
             is_a_word_value = is_a_word(manchester_word, (*time_from_last_sync), detected_word);
             if(sync_word_detect == 0 && is_a_word_value > 0){ //if sync word was detected at previous position, don't take word detection into account
               new_word = 1 ;
               (*time_from_last_sync) =  0 ;
             }
          }else{
            new_word = -1 ;
          }
          return new_word ;
}


#define EDGE_THRESHOLD 8
int oldValue = 0 ;
int steady_count = 0 ;
int dist_last_sync = 0 ;
unsigned int detected_word = 0;
int new_word = 0;
char old_edge_val = 0 ;
void sample_signal_edge(){
  char edge_val ;
  //int sensorValue = analogRead(SENSOR_PIN); // this is too slow and should be replaced with interrupt-driven ADC
  int sensorValue  = ADC_read_conversion(); // read result of previously triggered conversion
  ADC_start_conversion(SENSOR_PIN); // start a conversion for next loop
  //Serial.println(sensorValue, DEC);
  if((sensorValue - oldValue) > EDGE_THRESHOLD) edge_val = 1 ;
  else if((oldValue - sensorValue) > EDGE_THRESHOLD) edge_val = -1;
  else edge_val = 0 ;
  oldValue = sensorValue ;
  if(edge_val == 0 || edge_val == old_edge_val || (edge_val != old_edge_val && steady_count < 2)){
    if( steady_count < (4 * SAMPLE_PER_SYMBOL)){
      steady_count ++ ;
    }
  }else{  
          new_word = insert_edge(&shift_reg, edge_val, steady_count, &(dist_last_sync), &detected_word); 
          if(dist_last_sync > (8*SAMPLE_PER_SYMBOL)){ // limit dist_last_sync to avoid overflow problems
            dist_last_sync = 32 ;
          }
          //if(new_word >= 0){
            steady_count = 0 ;
          //}
        }
        old_edge_val = edge_val ;
}

int add_byte_to_frame(char * frame_buffer, int * frame_index, int * frame_size, enum receiver_state * frame_state ,unsigned char data){
  if(data == SYNC_SYMBOL/* && (*frame_index) < 0*/){
    (*frame_index) = 0 ;
    (*frame_size) = 0 ;
    (*frame_state) = SYNC ;
    //Serial.println("SYNC");
    return 0 ;
  }
  if((*frame_state) != IDLE){ // we are synced
  frame_buffer[*frame_index] = data ;
  (*frame_index) ++ ;
    if(data == STX){
      //Serial.println("START");
      (*frame_state) = START ;
       return 0 ;
    }else if(data == ETX){
      //Serial.println("END");
      (*frame_size) = (*frame_index) ;
      (*frame_index) = -1 ;
      (*frame_state) = IDLE ;
      //Serial.println("END");
       return 1 ;
    }else if((*frame_index) >= 38){ //frame is larger than max size of frame ...
      (*frame_index) = -1 ;
      (*frame_size) = -1 ;
      (*frame_state) = IDLE ;
      return -1 ;
    }else{
      (*frame_state) = DATA ;
    }
    return 0 ;
  }
  return -1 ;
}

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 115200 bits per second:
  int i; 
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  ADC_setup();
  ADC_start_conversion(SENSOR_PIN);
  //analogReference(INTERNAL); // internal reference is 1.1v, should give better accuracy for the mv range of the led output.
  Timer1.initialize(SYMBOL_PERIOD/SAMPLE_PER_SYMBOL); //1200 bauds oversampled by factor 4
  Timer1.attachInterrupt(sample_signal_edge);

}


// the loop routine runs over and over again forever:
void loop() {
  int i; 
  unsigned char received_data;
  char received_data_print ;
  int nb_shift ;
  int byte_added = 0 ;
  if(new_word == 1){
    received_data = 0 ;
    for(i = 0 ; i < 16 ; i = i + 2){ //decoding Manchester
             received_data = received_data << 1 ;
             if(((detected_word >> i) & 0x03) == 0x01){
                 received_data |= 0x01 ;
             }else{
                 received_data &= ~0x01 ;
             }
    }
    received_data = received_data & 0xFF ;
    #ifdef DEBUG
      Serial.print(received_data & 0xFF, HEX);
      Serial.print(", ");
      Serial.println((char) received_data);
    #endif
    new_word = 0 ;
    if((byte_added = add_byte_to_frame(frame_buffer, &frame_index, &frame_size, &frame_state,received_data)) > 0){
      frame_buffer[frame_size] = '\0';
      Serial.println(&(frame_buffer[1]));
    }
    //if(frame_state != IDLE) Serial.println(received_data, HEX);
  }
}
