#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Supervisor.hpp>

#define TIME_STEP 96

using namespace webots;
using namespace std;

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
  }
   else {
    wheels[0]->setVelocity(5.0);
    wheels[1]->setVelocity(5.0);
    wheels[2]->setVelocity(5.0);
    wheels[3]->setVelocity(5.0);
  }
}

double leftSpeed1;
double rightSpeed1;
double leftSpeed2;
double rightSpeed2;

int main() {
  Supervisor *supervisor = new Supervisor();
  
    DistanceSensor *ds[5];
  char dsNames[5][11] = {"ds_front", "ds_right1","ds_right2", "ds_left2", "ds_left1" };
  for (int i = 0; i < 5; i++) {
    ds[i] = supervisor->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
    
  }
  Motor *wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (int i = 0; i < 4; i++) {
    wheels[i] = supervisor->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(5.0);
  }

  // do this once only
  Node *robot_node = supervisor->getFromDef("botJr");
  Field *trans_field = robot_node->getField("translation");
  Field *rotation_field = robot_node->getField("rotation");
  Coordinates previousCoord = {0,0};

  while (supervisor->step(TIME_STEP) != -1) {
    // this is done repeatedly
    const double *trans_values = trans_field->getSFVec3f();
    const double *rotationValues = rotation_field->getSFRotation();
    double xValue = fabs(floor((trans_values[0]-0.0625)/0.125));
    double zValue = fabs(floor((trans_values[2]-0.0625)/0.125));
    double botAngle = rotationValues[3];
    
    //Coordinates
    Coordinates currentCoord = {xValue, zValue};
    
    if((previousCoord.xCoordinate != currentCoord.xCoordinate ) || (previousCoord.zCoordinate != currentCoord.zCoordinate)){
     cout<<"we loopin "<<endl;
     previousCoord = currentCoord;
     botMovement(false);
     //in here check all sensors for walls, add them to the struct, check which direction we are facing before saving
    }
    
    cout<<" currentcoords x: "<< currentCoord.xCoordinate <<"   || z: "<< currentCoord.zCoordinate <<endl;
    cout<< "x: "<< xValue << "  ||  z: "<< zValue<<"   || angle: "<<botAngle<< endl;         
    //cout<< "x: "<< rotationValues[0] << "  ||  z: "<< rotationValues[2]<< "  ||   angle: "<<rotationValues[3]<<endl;
  }

  delete supervisor;
  return 0;
}