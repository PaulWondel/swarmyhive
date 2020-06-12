#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/GPS.hpp>

#define TIME_STEP 100
using namespace webots;
using namespace std;

bool isTurning = false;
double leftSpeed1;
double rightSpeed1;
double leftSpeed2;
double rightSpeed2;
int counter;

void turnRight(){
      leftSpeed1 = 5;
      rightSpeed1 = -10;
      leftSpeed2 = 5;
      rightSpeed2 = -10;
}

void turnLeft() {
      leftSpeed1 = -10;
      rightSpeed1 = 5;
      leftSpeed2 = -10;
      rightSpeed2 = 5;
}

void adjustRight() {
  
      leftSpeed1 = 1;
      rightSpeed1 = -1;
      leftSpeed2 = 1;
      rightSpeed2 = -1;
}

void adjustLeft() {
      leftSpeed1 = -1;
      rightSpeed1 = 1;
      leftSpeed2 = -1;
      rightSpeed2 = 1;
}

void goForward() {
      leftSpeed1 = 6.00;
      rightSpeed1 = 6.00;
      leftSpeed2 = 6.00;
      rightSpeed2 = 6.00;
}
int main(int argc, char **argv) {
  Robot *robot = new Robot();
  GPS *gp;
  
  gp = robot->getGPS("global");
  gp->enable(TIME_STEP);
  
  DistanceSensor *ds[5];
  char dsNames[5][11] = {"ds_front", "ds_right1","ds_right2", "ds_left2", "ds_left1" };
  for (int i = 0; i < 5; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
  }
  Motor *wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (int i = 0; i < 4; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(1.0);
  }
  //int avoidObstacleCounter = 0;
  while (robot->step(TIME_STEP) != -1) {
    
    if(!isTurning){ 
      goForward();
      counter++;
    }
    
    // if(ds[0]->getValue() < 500) {
    
    // printf("dolo");
    // }
    if(ds[0]->getValue() < 500) {
      if(ds[1]->getValue() < 600 && ds[2]->getValue() <600) {
          isTurning = true;
          cout<<"turning left"<<endl;
          turnLeft();
      } else {
          isTurning = true;
          cout<<"turning right"<<endl;
          turnRight();
      }
   }else {
      isTurning = false;
    }
    
    
        if((ds[1]->getValue() > 300 && ds[1]->getValue() < 500) && (ds[2]->getValue() > 300 && ds[2]->getValue() < 500)) {
      goForward();
      
      
    }else if(counter > 2 && (ds[1]->getValue() <300 && ds[1]->getValue() >= ds[2]->getValue())){
    /*
    how to fix this, keep adjusting left until ds1 is bigger than ds2. need to fix if loops
    
    */
    cout<<"adjusting left"<<endl;
    adjustLeft();
    counter = 0;
    }
    
    if(counter > 2 && (ds[3]->getValue() <300 && ds[4]->getValue() < 300)) {
      adjustRight();
      cout<<"adjusting right"<<endl;
      counter = 0;
    }  
    
    
    
    
    
    
    
    
    
    
    // if(counter > 2 && (ds[1]->getValue() <300 && ds[2]->getValue() < 300)) {
      // adjustLeft();
      // cout<<"adjusting left"<<endl;
      // counter = 0;
    // }
    
    // if(counter > 2 && (ds[3]->getValue() <300 && ds[4]->getValue() < 300)) {
      // adjustRight();
      // cout<<"adjusting right"<<endl;
      // counter = 0;
    // }    
    
    wheels[0]->setVelocity(leftSpeed1);
    wheels[1]->setVelocity(rightSpeed1);
    wheels[2]->setVelocity(leftSpeed2);
    wheels[3]->setVelocity(rightSpeed2);
    
    //GPS PRINT COORDINATES
    cout<<"X : "<<fabs(round((gp->getValues()[0]-0.0624)/0.125))<<"  ||  Z : "<<fabs(round((gp->getValues()[2]-0.0624)/0.125))<<std::endl; 
  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}