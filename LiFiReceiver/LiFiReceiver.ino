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

0x55 or 0xAA : sent a number of time to help the receiver compute a signal average for the thresholding of analog values
0xD5 : synchronization byte to indicate start of a frame, breaks the regularity of the 0x55 pattern to be easily 
0x02 : STX start of frame
N times Effective data excluding command symbols)
0x03 : ETX end of frame
*/

//Start of what should be an include ...

#define ACC_LENGTH 16

struct myFifo{
  int data [ACC_LENGTH];
  unsigned int fifo_size ;
  unsigned int write_index ;
  unsigned int read_index ;
  unsigned int nb_available ; 
};

void init_fifo(struct myFifo * pf){
   pf -> fifo_size = ACC_LENGTH ;
   pf -> write_index = 0 ;
   pf -> read_index = 0 ;
   pf -> nb_available = 0 ;
 }


inline char read_fifo(struct myFifo * pf, int * token){
  if(pf->nb_available == 0) return -1 ;
  *token = pf->data[pf->read_index]  ;
  pf->read_index = pf->read_index + 1;
  if(pf->read_index >= pf->fifo_size) pf->read_index = 0 ;
  pf->nb_available = pf->nb_available -1 ;
  return 0 ;
  
}

inline char peek_fifo(struct myFifo * pf, int * token){
  if(pf->nb_available == 0) return -1 ;
  *token = pf->data[pf->read_index]  ;
  return 0 ;
  
}

inline char write_fifo(struct myFifo * pf, int  token){
  if(pf->nb_available >= pf->fifo_size) return -1 ;
  pf->data[pf->write_index] = token ;
  pf->write_index = pf->write_index + 1;
  if(pf->write_index >= pf->fifo_size) pf->write_index = 0 ;
  pf->nb_available = pf->nb_available + 1 ;
  return 0 ;
}
//end of fifo include

#define SENSOR_PIN A0
#define WORD_LENGTH 10
#define SYNC_SYMBOL 0xD5
#define ETX 0x03
#define STX 0x02

int sensorValue = 0;  // variable to store the value coming from the sensor


char cmd_symbols []= {0xD5, 0x02, 0x03};
int cmd_symbols_length = sizeof(cmd_symbols);


struct myFifo samples_fifo ;



//state variables of the thresholder
unsigned int signal_mean = 0 ;
unsigned long acc_sum = 0 ;
unsigned int acc_counter = 0 ;

//manechester decoder state variable
long shift_reg = 0;

//frame receiver state variables
int synced = -1 ;
int received_bit = 0 ;



//receiver interrupt
ISR(TIMER1_COMPA_vect){
  int sensorValue = analogRead(SENSOR_PIN);
  int i = 0;
  acc_sum = acc_sum +  sensorValue; 
  acc_counter = acc_counter + 1 ;
  if(acc_counter >= 32){
    signal_mean = acc_sum / 32 ;
    acc_counter = 0 ;
    acc_sum = 0 ;
  }
  if(signal_mean > 0){
 	 write_fifo(&samples_fifo, sensorValue);//wait for the thresholding to be stable
  }
  /*if(pin_state == 0){ // for debug purpose, only to make sur the sampling rate is the one expected
      digitalWrite(12, LOW);
      pin_state = 1 ;
  }else{
      digitalWrite(12, HIGH);
      pin_state = 0 ;
  }*/
}

void setupTimer1(unsigned char prescaler, unsigned int period){
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = period;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  switch(prescaler){
    case 0 :
      // Set CS20 bit for no prescaler
      TCCR1B |= (1 << CS10);
    case 1 :
      // Set CS21 bit for 8 prescaler
      TCCR1B |= (1 << CS11);
    case 2 :
      // Set CS21 bit for 64 prescaler
      TCCR1B |= (1 << CS11) | (1 << CS10);
      break ;   
    case 3 :
      // Set CS21 bit for 256 prescaler
      TCCR1B |= (1 << CS12) ;
      break ; 
    case 4 :
      // Set CS21 bit for 1024 prescaler
      TCCR1B |= (1 << CS12) | (1 << CS10);
      break ; 
    default :
      TCCR1B |= (1 << CS10);
      break ;
      
  } 
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
}


int detect_symbol(long * data_reg){
  unsigned char bit_val, prev_bit_val = 2 ;
  unsigned int same_counter = 0 ;
  int sample_val ;
  if(samples_fifo.nb_available < 10) return -1 ; // maximum symbol length is 10 (one symbol is ~4 samples, can have one sequence of two consecutive symbols so ~8 samples)
  while(same_counter < 10 ){
    peek_fifo(&samples_fifo, &sample_val);
    bit_val = sample_val > signal_mean ? 1 : 0 ;
    //Serial.print(bit_val , BIN);
    if(bit_val == prev_bit_val){
      read_fifo(&samples_fifo, &sample_val);
      same_counter ++ ;
    }else{
      if(same_counter >= 2 ){ // at least 2 bit with same val for a reliable data
        *data_reg = (*data_reg << 1) | prev_bit_val ;
         if(same_counter >= 5){ // at least 6 bit with same val for a reliable data
            *data_reg = (*data_reg << 1) | prev_bit_val ;
            return 2 ; // two symbol found
         }
         return 1 ; // one symbol found
      }
      read_fifo(&samples_fifo, &sample_val);
      same_counter ++ ;
    }
    prev_bit_val = bit_val ;
  }
  return -1 ;
}


char get_data(long data_reg, char *data){
  char received_data = 0;
  if((data_reg & 0x03) == 0x01){
       if(((data_reg >> 18) & 0x03) == 0x02){
           int i  ;
           received_data = 0 ;
           for(i = 2 ; i < 18 ; i = i + 2){
             received_data = received_data << 1 ;
             if(((data_reg >> i) & 0x03) == 0x01){
                 received_data |= 0x01 ;
             }else{
                 received_data &= ~0x01 ;
             } 
           }
           *data = received_data ;
          return 1 ;
       }else{
           return -1 ;
       }
     }else{
           return -1 ;
     }
     return -1 ;
}


// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 115200 bits per second:
  int i; 
  
  init_fifo(&samples_fifo);
  Serial.begin(115200);
  analogReference(INTERNAL); // internal reference is 1.1v, should give better accuracy for the mv range of the led output.
  cli();//stop interrupts
  setupTimer1(3, 12); // 4 time the transmitter clock
  sei();//allow interrupts

}


int detect_sync(long reg){
	int i  ;
        char data ;
	for(i = 0; i < 8 ; i ++){
	 if(get_data(reg>>i, &data)  >= 0){
	   if((data & 0xFF) == SYNC_SYMBOL){
	     Serial.println("\nsynced!");
	     return i ;
	   }
	 }
	}
	return -1 ;
}

int isCommandSymbol(char symbol){
    int i;
    for(i = 0; i < cmd_symbols_length; i ++){
       if(symbol == cmd_symbols[i]) return i ; 
    }
    return -1 ;  
}




// the loop routine runs over and over again forever:
void loop() {
  int sym ;
  int i; 
  char received_data ;
  sym = detect_symbol(&shift_reg);
  if(sym >= 0){
     if(synced < 0 ){
       synced = detect_sync(shift_reg);
	if(synced >= 0) received_bit = 20 ;
     }else{
       if(received_bit > 0 )received_bit =  received_bit - sym ;
       if(received_bit <= 0){
         if(received_bit < 0){
           if(get_data(shift_reg>>(synced+1), &received_data) < 0){
            synced = -1 ;  
           }else{
              if(received_data == ETX){
                synced = -1 ;
              }else{
                if(received_data != 0x02){  
                  Serial.print(received_data);
                }
                received_bit = 20 ;
              }
           }
         }else{
           if(get_data(shift_reg>>synced, &received_data) < 0){
            synced = -1 ;  
           }else{
              if(received_data == ETX){
                synced = -1 ;
              }else{
                if(received_data != 0x02){  
                  Serial.print(received_data);
                }
                received_bit = 20 ;
              }
           }
         }
         
       }
     }
  }

}
