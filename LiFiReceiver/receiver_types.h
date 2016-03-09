enum receiver_state {
  IDLE, //waiting for sync
  SYNC, //synced, waiting for STX
  START, //STX received
  DATA //receiving DATA
};
