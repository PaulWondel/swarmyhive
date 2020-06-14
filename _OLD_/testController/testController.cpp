//file before merging everything together into slaveController.cpp



#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Supervisor.hpp>
#include <cmath>
#include <stack>

#define _USE_MATH_DEFINES
#define TIME_STEP 256

using namespace webots;
using namespace std;

//structs

struct Coordinates
{
  //current coordinate of the bot
  double xCoordinate;
  double zCoordinate;
};

struct CoordinateWalls
{
  //what walls are available at said coordinate true=wall
  Coordinates ptnPair;
  bool up;
  bool right;
  bool down;
  bool left;
  double direction;
};

//initializers

Supervisor *supervisor;
DistanceSensor *ds[3];
char dsNames[3][11] = {"ds_front", "ds_right", "ds_left"};
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
double leftSpeed1;
double rightSpeed1;
double leftSpeed2;
double rightSpeed2;
bool status = true;
Coordinates previousCoord = {0, 0};
Coordinates currentCoord = {0, 0};
stack<CoordinateWalls> coordStack;
double UP = 1.5708;     // 90 degrees
double RIGHT = 0.0;     // 0 degrees
double DOWN = -1.5708;  // 270/-90 degrees
double LEFT = 3.1415;   // 180 degrees
double margin = 0.7854; // 45 degrees
double _botDirection;
double directionValue[] = {0, 1, 0, 0};
double trans_center_values[] = {0.0625, 0.01, 0.0625}; // center of (0,0)


//methods

void botMovement(bool status)
{
  if (!status)
  {
    wheels[0]->setVelocity(0.0);
    wheels[1]->setVelocity(0.0);
    wheels[2]->setVelocity(0.0);
    wheels[3]->setVelocity(0.0);
  }
  else
  {
    wheels[0]->setVelocity(5.0);
    wheels[1]->setVelocity(5.0);
    wheels[2]->setVelocity(5.0);
    wheels[3]->setVelocity(5.0);
  }
}

void setRotationXYZ()
{

  directionValue[0] = 0;
  directionValue[1] = 1;
  directionValue[2] = 0;
}

void centerBot()
{

  trans_center_values[0] = 0.0625 + (0.125 * currentCoord.xCoordinate);
  trans_center_values[1] = 0.01;
  trans_center_values[2] = 0.0625 + (0.125 * currentCoord.zCoordinate);

  cout << "X: " << previousCoord.xCoordinate << "    ||    "
       << "Z: " << previousCoord.zCoordinate << "    ||    "
       << "Value[0]: " << trans_center_values[0] << "    ||    "
       << "Value[2]: " << trans_center_values[2] << endl;

  trans_field->setSFVec3f(trans_center_values);
}
void botOrientation(double incomingAngle)
{

  //check if facing right
  cout << "************************" << incomingAngle << endl;
  if (incomingAngle <= (RIGHT + margin) && incomingAngle >= (RIGHT - margin))
  {

    setRotationXYZ();
    centerBot();
    directionValue[3] = RIGHT;
    _botDirection = RIGHT;
    rotation_field->setSFRotation(directionValue);
  }
  else if (incomingAngle <= (DOWN + margin) && incomingAngle >= (DOWN - margin))
  {

    setRotationXYZ();
    centerBot();
    directionValue[3] = DOWN;
    rotation_field->setSFRotation(directionValue);

    _botDirection = DOWN;
  }
  else if ((incomingAngle <= (LEFT + margin) && incomingAngle >= (LEFT - margin)) || (incomingAngle <= (-LEFT + margin) && incomingAngle >= (-LEFT - margin)))
  {

    setRotationXYZ();
    centerBot();
    directionValue[3] = LEFT;
    rotation_field->setSFRotation(directionValue);

    _botDirection = LEFT;
  }
  else if (incomingAngle <= (UP + margin) && incomingAngle >= (UP - margin))
  {

    setRotationXYZ();
    centerBot();
    directionValue[3] = UP;
    _botDirection = UP;
    rotation_field->setSFRotation(directionValue);

    cout << "botDir" << _botDirection << endl;
  }
}

void wallDetection(Coordinates xzCoords, double currentDirection)
{

  //botMovement(false);
  botOrientation(currentDirection);

  int frontSensorData = ds[0]->getValue();
  int rightSensorData = ds[1]->getValue();
  int leftSensorData = ds[2]->getValue();
  bool _UP = false;
  bool _RIGHT = false;
  bool _DOWN = false;
  bool _LEFT = false;
  double _margin = 0.157;

  if (currentDirection <= (RIGHT + _margin) && currentDirection >= (RIGHT - _margin))
  {
    //GOING RIGHT

    if (frontSensorData < 1000)
      _RIGHT = true;
    if (rightSensorData < 1000)
      _DOWN = true;
    if (leftSensorData < 1000)
      _UP = true;
    _LEFT = true;
  }

  if (currentDirection <= (DOWN + _margin) && currentDirection >= (DOWN - _margin))
  {
    //GOING DOWN

    if (frontSensorData < 1000)
      _DOWN = true;
    if (rightSensorData < 1000)
      _LEFT = true;
    if (leftSensorData < 1000)
      _RIGHT = true;
    _UP = true;
  }

  if ((currentDirection <= (LEFT + _margin) && currentDirection >= (LEFT - _margin)) || (currentDirection <= (-LEFT + margin) && currentDirection >= (-LEFT - margin)))
  {
    //GOING LEFT

    if (frontSensorData < 1000)
      _LEFT = true;
    if (rightSensorData < 1000)
      _UP = true;
    if (leftSensorData < 1000)
      _DOWN = true;
    _RIGHT = true;
  }

  if (currentDirection <= (UP + _margin) && currentDirection >= (UP - _margin))
  {
    //GOING UP

    if (frontSensorData < 1000)
      _UP = true;
    if (rightSensorData < 1000)
      _RIGHT = true;
    if (leftSensorData < 1000)
      _LEFT = true;
    _DOWN = true;
  }
  //cout << "bot dir: " << _botDirection << endl;
  CoordinateWalls currentWalls = {xzCoords, _UP, _RIGHT, _DOWN, _LEFT, _botDirection};
  coordStack.push(currentWalls);

  _UP = false;
  _RIGHT = false;
  _DOWN = false;
  _LEFT = false;

  //cout<< coordStack.size()<<endl;
  //cout << "x : " << temp.ptnPair.xCoordinate<<"   || z: "  << temp.ptnPair.zCoordinate<< "  ||  up: "<<temp.up<< "  ||  right: "<<temp.right<< "  ||  down: "<<temp.down<< "  ||  left: "<<temp.left<< endl;
  //cout<<"============================================="<<endl;
}

void onlyPositives(double incomingAngle)
{
  // checks if it is out of bounds or not (0.0625 sets it to the middle of a square and *8 rounds it to a whole number)
  double scaledTransValueX = ((trans_values[0] - 0.0625) * 8);
  double scaledTransValueZ = ((trans_values[2] - 0.0625) * 8);

  cout << "Scaled X: " << scaledTransValueX << endl;
  cout << "Scaled Z: " << scaledTransValueZ << endl;
  cout << "==========================" << endl;

  if (scaledTransValueX >= 0)
  {

    if (incomingAngle <= (UP + margin) && incomingAngle >= (UP - margin)) //checks if its going right
    {

      //checks currentCoord with realtime location
      if (scaledTransValueX >= (currentCoord.xCoordinate + 0.5))
      {

        xValue = currentCoord.xCoordinate + 1;
      }
      else
      {
        xValue = currentCoord.xCoordinate;
      }
    }
    if (incomingAngle <= (DOWN + margin) && incomingAngle >= (DOWN - margin))
    {
      if (scaledTransValueX <= (currentCoord.xCoordinate - 0.5))
      {

        xValue = currentCoord.xCoordinate - 1;
      }
      else
      {
        xValue = currentCoord.xCoordinate;
      }
    }
  }

  if (scaledTransValueZ >= 0)
  {
    cout << "CCCCCCCCCCCC" << endl;

    if (incomingAngle <= (RIGHT + margin) && incomingAngle >= (RIGHT - margin)) //checks if its going right
    {
      cout << "DDDDDDDDD" << endl;

      //checks currentCoord with realtime location
      //currentCoord.zCoordinate + 0.5) >= scaledTransValueZ
      if (scaledTransValueZ >= (currentCoord.zCoordinate + 0.5))
      {

        zValue = currentCoord.zCoordinate + 1;
      }
      else
      {
        zValue = currentCoord.zCoordinate;
      }
    }

    cout << "EEEEAngle: " << incomingAngle << endl;

    if ((incomingAngle <= (LEFT + margin) && incomingAngle >= (LEFT - margin)) || (incomingAngle <= (-LEFT + margin) && incomingAngle >= (-LEFT - margin)))
    {
      cout << "EEEEEEEEEEE" << endl;
      cout << "scaledZZZZZ: " << scaledTransValueZ << endl;
      cout << "currentCoordZ: " << currentCoord.zCoordinate << endl;
      cout << "+++++++++++++++++++++++++++++++++++++++" << endl;
      if (scaledTransValueZ <= (currentCoord.zCoordinate - 0.5))
      {

        zValue = currentCoord.zCoordinate - 1;
      }
      else
      {
        zValue = currentCoord.zCoordinate;
      }
    }
  }
  botMovement(true);
}

void rotateBot(double changeDirection)
{

  if (changeDirection == UP)
  {

    setRotationXYZ();
    directionValue[3] = UP;
    rotation_field->setSFRotation(directionValue);
  }
  else if (changeDirection == RIGHT)
  {

    setRotationXYZ();
    directionValue[3] = RIGHT;
    rotation_field->setSFRotation(directionValue);
  }
  else if (changeDirection == DOWN)
  {

    setRotationXYZ();
    directionValue[3] = DOWN;
    rotation_field->setSFRotation(directionValue);
  }
  else if (changeDirection == LEFT)
  {

    setRotationXYZ();
    directionValue[3] = LEFT;
    rotation_field->setSFRotation(directionValue);
  }
}

void turnLogic(double incomingAngle)
{

  botMovement(false);
  int frontSensorData = ds[0]->getValue();
  int rightSensorData = ds[1]->getValue();
  int leftSensorData = ds[2]->getValue();

  //turn up

  if ((incomingAngle <= (UP + margin) && incomingAngle >= (UP - margin)) && frontSensorData < 1000)
  {

    //can't go forward check for available turn(s)

    if (rightSensorData < 1000)
    {

      if (leftSensorData < 1000)
      {
        //pop stack since this is a deadend, and recursive the function
        rotateBot(DOWN);
      }
      else
      {
        rotateBot(LEFT);
      }
    }
    else
    {
      rotateBot(RIGHT);
    }
  }

  //turn right

  if ((incomingAngle <= (RIGHT + margin) && incomingAngle >= (RIGHT - margin)) && frontSensorData < 1000)
  {

    if (rightSensorData < 1000)
    {

      if (leftSensorData < 1000)
      {
        //pop stack since this is a deadend, and recursive the function
        rotateBot(LEFT);
      }
      else
      {
        rotateBot(UP);
      }
    }
    else
    {
      rotateBot(DOWN);
    }
  }
  //turn down
  //turn left

  if ((incomingAngle <= (DOWN + margin) && incomingAngle >= (DOWN - margin)) && frontSensorData < 1000)
  {

    if (rightSensorData < 1000)
    {

      if (leftSensorData < 1000)
      {
        //pop stack since this is a deadend, and recursive the function
        rotateBot(RIGHT);
      }
      else
      {
        rotateBot(UP);
      }
    }
    else
    {
      rotateBot(LEFT);
    }
  }
  if ((incomingAngle <= (LEFT + margin) && incomingAngle >= (LEFT - margin) || (incomingAngle <= (-LEFT + margin) && incomingAngle >= (-LEFT - margin))) && frontSensorData < 1000)
  {
    //cout<<"wtf: #################"<<endl;

    if (rightSensorData < 1000)
    {

      if (leftSensorData < 1000)
      {
        //pop stack since this is a deadend, and recursive the function
        rotateBot(RIGHT);
        //180 turn
      }
      else
      {
        rotateBot(LEFT);
      }
    }
    else
    {
      rotateBot(RIGHT);
    }
  }
  
  //botMovement(true);
}
//}

void backTracking()
{

//bool cellVisited[][] = true;

}

void updateValues(double botAngle)
{ //updates angling, wall info when in new square

  turnLogic(botAngle);
  onlyPositives(botAngle);
  currentCoord = {xValue, zValue};


  cout << "px:" << previousCoord.xCoordinate << " z: " << previousCoord.zCoordinate << endl;
  cout << "cx:" << currentCoord.xCoordinate << " z: " << currentCoord.zCoordinate << endl;
  cout << "" << endl;

  if ((previousCoord.xCoordinate != currentCoord.xCoordinate) || (previousCoord.zCoordinate != currentCoord.zCoordinate)) //missing pathFound condition, disable if and use other if when found
  {
    previousCoord = currentCoord;
    cout << "x:" << previousCoord.xCoordinate << " z: " << previousCoord.zCoordinate << endl;
    wallDetection(currentCoord, botAngle);
    backTracking();
    //botMovement(true);
    // cout << " currentcoords x: " << currentCoord.xCoordinate << "   || z: " << currentCoord.zCoordinate << endl;

    // cout << " previouscoords x: " << previousCoord.xCoordinate << "   || z: " << previousCoord.zCoordinate << endl;

    //in here check all sensors for walls, add them to the struct, check which direction we are facing before saving
  }
}

void setup()
{

  for (int i = 0; i < 3; i++)
  {
    ds[i] = supervisor->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
  }

  for (int i = 0; i < 4; i++)
  {
    wheels[i] = supervisor->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(5.0);
  }

  robot_node = supervisor->getFromDef("botJr");
  trans_field = robot_node->getField("translation");
  rotation_field = robot_node->getField("rotation");
}

int main()
{

  supervisor = new Supervisor();
  setup();
  cout<<"test"<<endl;

  while (supervisor->step(TIME_STEP) != -1)
  {

    // this is done repeatedly

    trans_values = trans_field->getSFVec3f();
    rotationValues = rotation_field->getSFRotation();
    botAngle = rotationValues[3];
    updateValues(botAngle);
  }
  


  delete supervisor;
  return 0;
}