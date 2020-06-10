#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Supervisor.hpp>
#include <cmath>


#define _USE_MATH_DEFINES
#define TIME_STEP 96

using namespace webots;
using namespace std;




//initializers

Supervisor *supervisor; 
DistanceSensor *ds[5];
char dsNames[5][11] = {"ds_front", "ds_right1","ds_right2", "ds_left2", "ds_left1" };
Motor *wheels[4];
char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
Node *robot_node;
Field *trans_field;
Field *rotation_field;
const double *trans_values;
const double *rotationValues;
double xValue;
double zValue;
double botAngle;


struct Coordinates {
    //current coordinate of the bot
    double xCoordinate;
    double zCoordinate;
};

struct CoordinateWalls {
    //what walls are available at said coordinate true=wall
    Coordinates pt;
    bool up;
    bool right;
    bool down;
    bool left;
    
};





void botMovement(bool status){
  if(!status){
    wheels[0]->setVelocity(0.0);
    wheels[1]->setVelocity(0.0);
    wheels[2]->setVelocity(0.0);
    wheels[3]->setVelocity(0.0);
    cout<<"MASSIVE"<<endl;
  }
   else {
    wheels[0]->setVelocity(5.0);
    wheels[1]->setVelocity(5.0);
    wheels[2]->setVelocity(5.0);
    wheels[3]->setVelocity(5.0);
  }
}

void botOrientation(double incomingAngle){

  // rad to degree = (rad*180)/pi
  
    // double angleInDeg = (incomingAngle * 180) / M_PI;
    // double degToRad; = (angleInDeg / 180) * M_PI;
    

   double UP = 1.5708; // 90 degrees
   double RIGHT = 0.0; // 0 degrees
   double DOWN = -1.5708; // 270/-90 degrees
   double LEFT = 3.1415; // 180 degrees
   double margin = 0.7854; // 45 degrees
   double directionValue [] = {0,1,0,0};
  
  //check if facing right
  if(incomingAngle <= (RIGHT+margin) && incomingAngle >= (RIGHT-margin)) {
  
  directionValue[3] = RIGHT; 
  rotation_field->setSFRotation(directionValue);
  //cout<<" new values: "<< directionValue<<endl;
  
  } else if (incomingAngle <= (DOWN+margin) && incomingAngle >= (DOWN-margin)) {
  
  directionValue[3] = DOWN; 
  rotation_field->setSFRotation(directionValue);
  
  } else if (incomingAngle <= (LEFT+margin) && incomingAngle >= (LEFT-margin)) {
  
  directionValue[3] = LEFT; 
  rotation_field->setSFRotation(directionValue);
  
  } else if (incomingAngle <= (UP+margin) && incomingAngle >= (UP-margin)) {
  
  directionValue[3] = UP; 
  rotation_field->setSFRotation(directionValue);
  
  } 
  
  

}

double leftSpeed1;
double rightSpeed1;
double leftSpeed2;
double rightSpeed2;
bool status = true;

void setup() {

  for (int i = 0; i < 5; i++) {
    ds[i] = supervisor->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
    
  }

  for (int i = 0; i < 4; i++) {
    wheels[i] = supervisor->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(5.0);
  }
  
  robot_node = supervisor->getFromDef("botJr");
  trans_field = robot_node->getField("translation");
  rotation_field = robot_node->getField("rotation");

}

int main() {
  supervisor = new Supervisor();
  setup();
  
  


  // do this once only
  

  Coordinates previousCoord = {0,0};

  while (supervisor->step(TIME_STEP) != -1) {
  
    // this is done repeatedly
    trans_values = trans_field->getSFVec3f();
    rotationValues = rotation_field->getSFRotation();
    botAngle = rotationValues[3];
    
    if(trans_values[0]-0.0625 >= 0) {
      xValue = floor((trans_values[0]-0.0625)/0.125);
    }
    
        if(trans_values[2]-0.0625 >= 0) {
      zValue = floor((trans_values[2]-0.0625)/0.125);
    }
    
    //Coordinates
    Coordinates currentCoord = {xValue, zValue};
    
    if((previousCoord.xCoordinate != currentCoord.xCoordinate ) || (previousCoord.zCoordinate != currentCoord.zCoordinate)){
     cout<<"we loopin "<<endl;
     previousCoord = currentCoord;
     botMovement(false);
     cout<<"old angle: "<<botAngle<<endl;
     botOrientation(botAngle);
     cout<<"new angle: "<<botAngle<<endl;
     botMovement(true);
     
     //in here check all sensors for walls, add them to the struct, check which direction we are facing before saving
    }
    
    cout<<" currentcoords x: "<< currentCoord.xCoordinate <<"   || z: "<< currentCoord.zCoordinate <<endl;
    cout<< "x: "<< xValue << "  ||  z: "<< zValue<<"   || angle: "<<botAngle<< endl;         
    //cout<< "x: "<< rotationValues[0] << "  ||  z: "<< rotationValues[2]<< "  ||   angle: "<<rotationValues[3]<<endl;
  }

  delete supervisor;
  return 0;
}