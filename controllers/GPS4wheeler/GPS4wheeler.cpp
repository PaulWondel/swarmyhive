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
      rightSpeed1 = -2;
      leftSpeed2 = 1;
      rightSpeed2 = -2;
}

void adjustLeft() {
      leftSpeed1 = -2;
      rightSpeed1 = 1;
      leftSpeed2 = -2;
      rightSpeed2 = 1;
}
int main(int argc, char **argv) {
  Robot *robot = new Robot();
  GPS *gp;
  gp = robot->getGPS("global");
  gp->enable(TIME_STEP);
  
  DistanceSensor *ds[5];
  char dsNames[5][11] = {"ds_front", "ds_right","ds_right_2", "ds_left", "ds_left_2" };
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
      leftSpeed1 = 2.00;
      rightSpeed1 = 2.00;
      leftSpeed2 = 2.00;
      rightSpeed2 = 2.00;
    }
    
    
    if(ds[0]->getValue() < 800) {
      if(ds[1]->getValue() < 1000 && ds[2]->getValue() <1000) {
          isTurning = true;
          turnLeft();
      } else {
          isTurning = true;
          turnRight();
      }
   }else {
      isTurning = false;
    }
    
    if(ds[1]->getValue() <800 && ds[2]->getValue() > 800) {
      adjustLeft();
    }
    
    if(ds[3]->getValue() <800 && ds[4]->getValue() > 800) {
      adjustRight();
    }    
    
    wheels[0]->setVelocity(leftSpeed1);
    wheels[1]->setVelocity(rightSpeed1);
    wheels[2]->setVelocity(leftSpeed2);
    wheels[3]->setVelocity(rightSpeed2);
    
    //GPS PRINT COORDINATES
    /*std::cout<<"X : "<<gp->getValues()[0]<<std::endl;
    std::cout<<"Y : "<<gp->getValues()[1]<<std::endl;
    std::cout<<"Z : "<<gp->getValues()[2]<<std::endl;
    std::cout<<"##########################"<<std::endl;*/
  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}