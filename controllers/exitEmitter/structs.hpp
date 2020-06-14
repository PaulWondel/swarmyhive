// Signal struct with tag to see if it is an exit signal

struct exitSignal {
  int tag;
  char messageContent[8];
};