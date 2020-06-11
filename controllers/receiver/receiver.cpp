#include <webots/Supervisor.hpp>
#include <webots/Receiver.hpp>

#define TIME_STEP 96

using namespace webots;
using namespace std;

int main() {
  Supervisor *supervisor = new Supervisor();

  // do this once only
  //Node *robot_node = supervisor->getFromDef("Central");
  //Field *trans_field = robot_node->getField("translation");
  
  // receiver
  Receiver *receiver;
  receiver=supervisor->getReceiver("receiver");
  receiver->enable(TIME_STEP);
  //receiver->setChannel(1); 
  
  while (supervisor->step(TIME_STEP) != -1) {
    while(receiver->getQueueLength()>0){
      //char message = receiver->getData();
      cout << "Received message: "<<receiver->getData()<<endl;     
      receiver->nextPacket();
    }
  cout << "Queue length: "<<receiver->getQueueLength()<<endl;
  cout << "R Channel "<<receiver->getChannel()<<endl;
    // this is done repeatedly
    //const double *values = trans_field->getSFVec3f();
    //std::cout << "Slave is at position: " << values[0] << ' '
    //          << values[1] << ' ' << values[2] << std::endl;
              
    //receiver->getData();
  }
  receiver->disable();
  delete receiver;
  delete supervisor;
  return 0;
}