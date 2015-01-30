
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
