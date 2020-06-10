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
bool robotCanMove;
int checkCoordCounter;



struct Coordinates {
    //current coordinate of the bot
    double xCoordinate;
    double zCoordinate;
};

struct CoordinateWalls {
    //what walls are available at said coordinate true=wall
    Coordinates pt;
    bool north;
    bool east;
    bool south;
    bool west;
    
};

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
  
  int frontSensorData = ds[0]->getValue();
  int frontRightSensorData = ds[1]->getValue();
  //int backRightSensorData = ds[2]->getValue();
  int frontLeftSensorData = ds[3]->getValue();
  //int backLeftSensorData = ds[4]->getValue();
      leftSpeed1 = 6.00;
      rightSpeed1 = 6.00;
      leftSpeed2 = 6.00;
      rightSpeed2 = 6.00;
      GPS test = fabs(round((gp->getValues()[0]-0.0624)/0.125));
      //double test1 = gp->getValues()[0];
      //int sensorXCoordinate = fabs(round((gp->getValues()[0]-0.0624)/0.125));
      int sensorZCoordinate = fabs(round((gp->getValues()[2]-0.0624)/0.125));
      Coordinates previousCoord = {0,0};
      Coordinates currentCoord = {fabs(round((gp->getValues()[0]-0.0624)/0.125)), sensorZCoordinate};
  
  // if(){};
  
  while(robot->step(TIME_STEP) != -1) {
  double huh;
    //cout<<test<<endl;
    cout<<typeid(gp->getValues()).name()<<endl;
    //cout<<typeid(fabs(round((gp->getValues()[2]-0.0624)/0.125))).name()<<endl;
    //cout<< "sensorx: " <<fabs(round((gp->getValues()[0]-0.0624)/0.125))<<"sensorZ: "<< sensorZCoordinate<<endl;
    if(robotCanMove){
    wheels[0]->setVelocity(leftSpeed1);
    wheels[1]->setVelocity(rightSpeed1);
    wheels[2]->setVelocity(leftSpeed2);
    wheels[3]->setVelocity(rightSpeed2);
    //cout<<"in move if"<<endl;
    //cout<<"coordcounter: "<<checkCoordCounter<<endl;
    
    }
    
    
    checkCoordCounter++;
  
  if(checkCoordCounter >= 3) {
  
    cout<<"in checkCoord if"<<endl;
  
    if((previousCoord.xCoordinate != currentCoord.xCoordinate ) || (previousCoord.zCoordinate != currentCoord.zCoordinate)){
    //cout<<"previoudCoord1: "<<previousCoord.xCoordinate <<", "<< previousCoord.zCoordinate<<endl;   
    //cout<<"currentCoord1: "<<currentCoord.xCoordinate <<", "<< currentCoord.zCoordinate<<endl;
    previousCoord = currentCoord;
    //cout<<"previoudCoord2: "<<previousCoord.xCoordinate <<", "<< previousCoord.zCoordinate<<endl;   
    //cout<<"currentCoord2: "<<currentCoord.xCoordinate <<", "<< currentCoord.zCoordinate<<endl;
    robotCanMove = false;
    //cout<<"previoudCoord: "<<previousCoord.xCoordinate <<", "<< previousCoord.zCoordinate<<endl;
    if(frontSensorData < 1000) {
      cout<<"front sensor triggered"<<endl;
    }
    
    
    }
    
    //cout<<"previoudCoord3: "<<previousCoord.xCoordinate <<", "<< previousCoord.zCoordinate<<endl;   
    //cout<<"currentCoord3: "<<currentCoord.xCoordinate <<", "<< currentCoord.zCoordinate<<endl;
    
    robotCanMove = true;
    checkCoordCounter = 0;
  }
checkCoordCounter++;    
}
  
  
  
  
  //int avoidObstacleCounter = 0;
  // while (robot->step(TIME_STEP) != -1) {
    // xCoordinate = fabs(round((gp->getValues()[0]-0.0624)/0.125));
    // zCoordinate = fabs(round((gp->getValues()[2]-0.0624)/0.125));
    
    
    // wheels[0]->setVelocity(leftSpeed1);
    // wheels[1]->setVelocity(rightSpeed1);
    // wheels[2]->setVelocity(leftSpeed2);
    // wheels[3]->setVelocity(rightSpeed2);
    
    // //GPS PRINT COORDINATES
    // cout<<"X : "<< xCoordinate<<"  ||  Z : "<<zCoordinate<<std::endl; 
  // }
  delete robot;
  return 0;  // EXIT_SUCCESS
}