#include <webots/Supervisor.hpp>
#include <webots/Emitter.hpp>
#include "structs.hpp"

#define TIME_STEP 256

using namespace webots;
using namespace std;

// Error tag is 162
struct exitSignal noEntrance = {162,"exit"};

// Function to send exit signal through 
void sendMessage(exitSignal messageStruct, Emitter *device){
  device->send(&messageStruct,sizeof(messageStruct));
  return;
}

int main(){

  Supervisor *supervisor = new Supervisor();
  Emitter *emitter=supervisor->getEmitter("exitEmitter");
  emitter->setChannel(2);
  emitter->setRange(0.1);

  // timer for the delay loop
  int timer = 0;

  while (supervisor->step(TIME_STEP) != -1) {
    // delay loop for sending message
    if (timer >= 4){
      sendMessage(noEntrance,emitter);
      timer=0;
    }
    timer++;
  }

  delete emitter;
  delete supervisor;
  return EXIT_SUCCESS;
}