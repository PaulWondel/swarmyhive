#include <webots/Supervisor.hpp>
#include <webots/Emitter.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#define TIME_STEP 96

using namespace webots;
using namespace std;

//double pos[]; // value[x,y]

int main() {

  // Robot motors

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
  // Emitter Setup
  Emitter *emitter;
  emitter=robot->getEmitter("emitter");
  char message[10] = "Turn";
  emitter->setChannel(1);
  
  // Supervisor Setup
  Supervisor *supervisor;
  Node *robot_node = supervisor->getFromDef("Slave");
  Field *trans_field = robot_node->getField("translation");
  
  int avoidObstacleCounter = 0;
  while (robot->step(TIME_STEP) != -1) {
    double leftSpeed = 1.0;
    double rightSpeed = 1.0;
    
    const double *values = trans_field->getSFVec3f();
    //cout << "Slave is at position: " << values[0] << ' ' << ' ' << values[2] <<endl;
    //double pos[]={values[0],values[2]};
    cout << "E Channel "<<emitter->getChannel()<<endl;
    
              
    if (avoidObstacleCounter > 0) {
      avoidObstacleCounter--;
      leftSpeed = 1.0;
      rightSpeed = -1.0;  
      emitter->send(message,10);
      //cout<<"Sending message: "<<message<<endl;
    } else { // read sensors
      for (int i = 0; i < 2; i++) {
        if (ds[i]->getValue() < 950.0)
          avoidObstacleCounter = 100;
      }
    }
    wheels[0]->setVelocity(leftSpeed);
    wheels[1]->setVelocity(rightSpeed);
    wheels[2]->setVelocity(leftSpeed);
    wheels[3]->setVelocity(rightSpeed);
  }
  delete emitter;
  delete robot;
  return 0;  // EXIT_SUCCESS
}