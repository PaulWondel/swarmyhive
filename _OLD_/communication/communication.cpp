#include <webots/Supervisor.hpp>
#include <webots/Emitter.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include "communication.hpp"
#include <webots/Receiver.hpp>

#define TIME_STEP 256

using namespace webots;
using namespace std;

const char *exitMessage = "exit";
const int distanceValue=40;

struct Coordinates slave;
struct CoordinateWalls test;

// Error tag is 162
const int exitTag = 162;

// condition to stop robot from moving
bool stop = false;

void sendCoordinates(Coordinates message, Emitter *device){
  device->send(&message,sizeof(message));
  return;
}

int main() {

  // Robot motors initialized

  Robot *robot = new Robot();
  DistanceSensor *ds[2];
  char dsNames[2][10] = {"ds_right", "ds_left"};
  for (int i = 0; i < 2; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
  }
  Motor *wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (int i = 0; i < 4; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
  // Emitter initialized
  Emitter *emitter;
  emitter=robot->getEmitter("emitter");
  //char message[10] = "Turn";
  emitter->setChannel(1);

  // Receiver initialized
  Receiver *receiver;
  receiver=robot->getReceiver("receiver");
  receiver->enable(TIME_STEP);
  receiver->setChannel(2); 
  
  // Supervisor initialized
  Supervisor *supervisor;
  // bind the robot_node pointer to the DEF Slave in webots
  Node *robot_node = supervisor->getFromDef("Slave");
  // get the translation coordinates from webots
  Field *trans_field = robot_node->getField("translation");
  
  int avoidObstacleCounter = 0;
  while (robot->step(TIME_STEP) != -1 && stop == false) {
    double leftSpeed = 1.0;
    double rightSpeed = 1.0;
    
    const double *values = trans_field->getSFVec3f();
    double position[2]={values[0],values[2]};
    slave = {values[0],values[2]};
    
    // 
    if (avoidObstacleCounter > 0) {
      avoidObstacleCounter--;
      leftSpeed = 1.0;
      rightSpeed = -1.0;  
    }
    else
    {
      // read sensors
      for (int i = 0; i < 2; i++) {
        if (ds[i]->getValue() < 950.0)
          avoidObstacleCounter = distanceValue;
      }
    }
    if(avoidObstacleCounter == distanceValue){

      // sendCoordinates(slave,emitter);

      // Debug position print in commandline
      cout<<"Debug X: "<<position[0]<<endl;
      cout<<"Debug Z: "<<position[1]<<endl;
    }     

    // Receives message and stops bot from moving with message
    // if(receiver->getQueueLength()>0){
    //   char *message = (char*)receiver->getData();
    //   cout<<"Message received: "<<message<<endl;
    //   cout<<exitMessage<<" debug"<<endl;
    //   if(message[0] == exitMessage[0]){
    //     stop = true;
    //     leftSpeed = 0.0;
    //     rightSpeed = 0.0;
    //     cout<<"Reached exit: "<<stop<<endl;
    //   }
    //   receiver->nextPacket();
    // }

    // Receives message and stops bot from moving with struct
    if(receiver->getQueueLength()>0){
      struct exitSignal *message = (struct exitSignal*)receiver->getData();
      char *messageReceived = (char*)message->messageContent;
      cout<<"Message received: "<<messageReceived<<endl;
      cout<<"Message tag: "<<message->tag<<endl;
      if(message->tag == exitTag){
        stop = true;
        leftSpeed = 0.0;
        rightSpeed = 0.0;
        cout<<"Reached exit: "<<stop<<endl;
        sendCoordinates(slave,emitter);
      }
      receiver->nextPacket();
    }
    
    wheels[0]->setVelocity(leftSpeed);
    wheels[1]->setVelocity(rightSpeed);
    wheels[2]->setVelocity(leftSpeed);
    wheels[3]->setVelocity(rightSpeed);

  }
  receiver->disable();
  // delete receiver;
  // delete emitter;
  // delete robot;
  return 0;  // EXIT_SUCCESS
}