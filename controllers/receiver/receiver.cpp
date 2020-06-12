#include <webots/Supervisor.hpp>
#include <webots/Receiver.hpp>
#include "extra.hpp"
#include <vector> 

#define TIME_STEP 192

using namespace webots;
using namespace std;

// Dynamic array to save structs into
vector<Coordinates> unvisitedList;

int main(int argc, char **argv) {
  Supervisor *supervisor = new Supervisor();
  
  // Initialize receiver
  Receiver *receiver;
  receiver=supervisor->getReceiver("receiver");
  receiver->enable(TIME_STEP);
  receiver->setChannel(-1); 
  
  while (supervisor->step(TIME_STEP) != -1) {
    if(receiver->getQueueLength()>0){

      // Receive the Coordinate struct and keep the coordinates
      struct Coordinates *incomming = (struct Coordinates*)receiver->getData();
      double xcoord = incomming->xCoordinate;
      double zcoord = incomming->zCoordinate;
      cout << "Received X: "<<xcoord<<endl;
      cout << "Received Z: "<<zcoord<<endl;
      receiver->nextPacket();

      // Put the coordinates into a struct and 
      // push struct into an dynamic array (vector)
      unvisitedList.push_back(structTransport(xcoord,zcoord));

      // coordinateList.size() gives unsigned long int
      for (int i = (int)unvisitedList.size(); i == (int)unvisitedList.size();){
        cout<<i-1<<" Debug coordinate X: "<<unvisitedList[i-1].xCoordinate<<endl;
        cout<<i-1<<" Debug coordinate Z: "<<unvisitedList[i-1].zCoordinate<<endl;
        break;        
      }
    }
  }
  receiver->disable();
  delete receiver;
  delete supervisor;
  return 0;
}