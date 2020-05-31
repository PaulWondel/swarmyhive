#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/GPS.hpp>

#define TIME_STEP 100
using namespace webots;
//using namespace std;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  GPS *gp;
  gp = robot->getGPS("global");
  gp->enable(TIME_STEP);
  
  DistanceSensor *ds[3];
  char dsNames[3][11] = {"ds_front", "ds_right", "ds_left" };
  for (int i = 0; i < 3; i++) {
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
    double leftSpeed1 = 1.00;
    double rightSpeed1 = 1.00;
    double leftSpeed2 = 1.00;
    double rightSpeed2 = 1.00;
    
    std::cout<<"DS FRONT: "<<ds[0]->getValue()<<std::endl;
    std::cout<<"DS R: "<<ds[1]->getValue()<<std::endl;
    //std::cout<<"DS L: "<<ds[2]->getValue()<<std::endl;
    std::cout<<"##############"<<std::endl;
    
    
    if(ds[0]->getValue() < 900.0 && ds[1]->getValue() < 900.0) {
      //std::cout<<"DS FRONT: "<<ds[0]->getValue()<<std::endl;
      std::cout<<"SEES WALL AT FRONT AND RIGHT, TURNING LEFT"<<std::endl;
      leftSpeed1 = -10;
      rightSpeed1 = 10;
      leftSpeed2 = -5;
      rightSpeed2 = 5;
    } 
    else if(ds[0]->getValue() < 900.0 && ds[2]->getValue() < 900.0) {
      //std::cout<<"DS FRONT: "<<ds[0]->getValue()<<std::endl;
      std::cout<<"SEES WALL AT FRONT AND LEFT, TURNING RIGHT"<<std::endl;
      leftSpeed1 = 10;
      rightSpeed1 = -10;
      leftSpeed2 = 5;
      rightSpeed2 = -5;
    } 
    else if(ds[0]->getValue() < 900.0) {
      //std::cout<<"DS FRONT: "<<ds[0]->getValue()<<std::endl;
      std::cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<TURNING LEFT"<<std::endl;
      leftSpeed1 = -10;
      rightSpeed1 = 10;
      leftSpeed2 = -5;
      rightSpeed2 = 5;
    }
    else if(ds[1]->getValue() < 900.0){
      //std::cout<<"DS R: "<<ds[2]->getValue()<<std::endl;
      std::cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<ADJUSTING LEFT"<<std::endl;
      leftSpeed1 = -4;
      rightSpeed1 = 4;
      leftSpeed2 = 1;
      rightSpeed2 = 1;
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