#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Compass.hpp>
//#include <webots/Robot.hpp>
//#include <webots/Emitter.hpp>
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

enum movementDirection
{
  NORTH,// 0
  EAST, // 1
  SOUTH, // 2
  WEST // 3

};

//initializers && variables
Compass *compass;
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
const double *compassDirection;
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
double trans_center_values[] = {0.0625, 0.02, 0.0625}; // center of (0,0)
// struct Coordinates slave;
// struct CoordinateWalls test;

//methods

//set coordinate values for transport
Coordinates structTransport(double xCoord, double zCoord)
{
  Coordinates transport = {(double)xCoord, (double)zCoord};
  return transport;
}

//used to send coordinates to the receiver
// void sendCoordinates(Coordinates message, Emitter *device){
// device->send(&message,sizeof(message));
// return;
// }

//stops to bot when called or restarts movement
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
    wheels[0]->setVelocity(10.0);
    wheels[1]->setVelocity(10.0);
    wheels[2]->setVelocity(10.0);
    wheels[3]->setVelocity(10.0);
  }
}

//set the values so the bot is correctly angled
void setRotationXYZ()
{

  directionValue[0] = 0;
  directionValue[1] = 1;
  directionValue[2] = 0;
}

//centers bot to the middle of a square
void centerBot()
{

  //scales incoming values from decimals to big numbers (1+)
  trans_center_values[0] = 0.0625 + (0.125 * currentCoord.xCoordinate);
  trans_center_values[1] = 0.05;
  trans_center_values[2] = 0.0625 + (0.125 * currentCoord.zCoordinate);

  // up for cleaning when program is done
  // cout << "X: " << previousCoord.xCoordinate << "    ||    "
  //      << "Z: " << previousCoord.zCoordinate << "    ||    "
  //      << "Value[0]: " << trans_center_values[0] << "    ||    "
  //      << "Value[2]: " << trans_center_values[2] << endl;

  //sets the values
  trans_field->setSFVec3f(trans_center_values);
}

//orientate the bot based on the incoming angle
void botOrientation(double incomingAngle)
{
  //cout << "************************" << incomingAngle << endl;

  //check if facing right
  if (incomingAngle <= (RIGHT + margin) && incomingAngle >= (RIGHT - margin))
  {

    setRotationXYZ();
    centerBot();
    directionValue[3] = RIGHT;                     // set angle field to the correct angle
    _botDirection = RIGHT;                         //sets value to be filled into struct
    rotation_field->setSFRotation(directionValue); // sets
  }
  else if (incomingAngle <= (DOWN + margin) && incomingAngle >= (DOWN - margin))
  {

    setRotationXYZ();
    centerBot();
    directionValue[3] = DOWN;
    _botDirection = DOWN;
    rotation_field->setSFRotation(directionValue);
  }
  else if ((incomingAngle <= (LEFT + margin) && incomingAngle >= (LEFT - margin)) || (incomingAngle <= (-LEFT + margin) && incomingAngle >= (-LEFT - margin)))
  {

    setRotationXYZ();
    centerBot();
    directionValue[3] = LEFT;
    _botDirection = LEFT;
    rotation_field->setSFRotation(directionValue);
  }
  else if (incomingAngle <= (UP + margin) && incomingAngle >= (UP - margin))
  {

    setRotationXYZ();
    centerBot();
    directionValue[3] = UP;
    _botDirection = UP;
    rotation_field->setSFRotation(directionValue);
  }
}

//function for detecting walls and saving them based on the current direction
void wallDetection(Coordinates xzCoords, double currentDirection)
{

  //botMovement(false);
  botOrientation(currentDirection);

  //local variables
  int frontSensorData = ds[0]->getValue();
  int rightSensorData = ds[1]->getValue();
  int leftSensorData = ds[2]->getValue();
  bool _UP = false;
  bool _RIGHT = false;
  bool _DOWN = false;
  bool _LEFT = false;
  double _margin = 0.157;

  //cycles through all 4 directions and update the walls values based on current angle
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

  //fills struct with coordinate pair, detected walls and current direction
  CoordinateWalls currentWalls = {xzCoords, _UP, _RIGHT, _DOWN, _LEFT, _botDirection};

  //push the struct to the stack
  coordStack.push(currentWalls);

  //send to stack to master

  //reset local variables for the next coordinate pair
  _UP = false;
  _RIGHT = false;
  _DOWN = false;
  _LEFT = false;
}

void onlyPositives(double incomingAngle)
{
  // checks if it is out of bounds or not (0.0625 sets it to the middle of a square and *8 scales it up)
  double scaledTransValueX = ((trans_values[0] - 0.0625) * 8);
  double scaledTransValueZ = ((trans_values[2] - 0.0625) * 8);

  //delete when done
  // cout << "Scaled X: " << scaledTransValueX << endl;
  // cout << "Scaled Z: " << scaledTransValueZ << endl;
  // cout << "==========================" << endl;

  //check if it is not out of bounds  X-AXIS
  if (scaledTransValueX >= 0)
  {

    if (incomingAngle <= (UP + margin) && incomingAngle >= (UP - margin)) //checks if its going right, also takes into account slight deviation possibility
    {

      //checks currentCoord with realtime location
      //if bot passed the connecter/threshold between two squares
      if (scaledTransValueX >= (currentCoord.xCoordinate + 0.5))
      {
        //move it to the center of the new square
        xValue = currentCoord.xCoordinate + 1;
      }
      else
      {
        //sets currentCoord back to what it was
        xValue = currentCoord.xCoordinate;
      }
    }
    //same as UP but for down instead
    if (incomingAngle <= (DOWN + margin) && incomingAngle >= (DOWN - margin))
    {
      //if bot passed the connecter/threshold between two squares
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
  // same as the if's above but for the Z-AXIS
  if (scaledTransValueZ >= 0)
  {

    if (incomingAngle <= (RIGHT + margin) && incomingAngle >= (RIGHT - margin)) //checks if its going right
    {
      if (scaledTransValueZ >= (currentCoord.zCoordinate + 0.5))
      {

        zValue = currentCoord.zCoordinate + 1;
      }
      else
      {
        zValue = currentCoord.zCoordinate;
      }
    }

    if ((incomingAngle <= (LEFT + margin) && incomingAngle >= (LEFT - margin)) || (incomingAngle <= (-LEFT + margin) && incomingAngle >= (-LEFT - margin)))
    {
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

//rotateBot depending on wall/sensor conditions
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

void testTurn()
{

  int frontSensorData = ds[0]->getValue();
  int rightSensorData = ds[1]->getValue();
  int leftSensorData = ds[2]->getValue();

  botMovement(false);
  if (frontSensorData < 500)
  {
    //cout<<"sees wall: " <<endl;
    if (rightSensorData > 600)
    {
      //cout<<"turn right: " <<rightSensorData<<endl;
      setRotationXYZ();
      directionValue[3] -= 1.5708;
      rotation_field->setSFRotation(directionValue);
    }
    else if (leftSensorData > 600)
    {
      //cout<<"turn left: " <<leftSensorData<<endl;
      setRotationXYZ();
      directionValue[3] += 1.5708;
      rotation_field->setSFRotation(directionValue);
    }
    else
    {
      //cout<<"180: " <<endl;
      setRotationXYZ();
      directionValue[3] += 3.1415;
      rotation_field->setSFRotation(directionValue);
    }
  }
  else
  {
    //cout<<"restart bot: " <<endl;
    botMovement(true);
  }
}

movementDirection setTrueDirection()
{

  // [0] == z , [2] == x

  double x = round(compassDirection[0]);
  double y = round(compassDirection[2]);
  //cout<< "x: "<<x<<"  y: "<<y<<endl;
  if (x == 1)
  {
    cout<<"EAST"<<endl;
    return EAST;
  }
  else if (x == -1)
  {
  cout<<"WEST"<<endl;
    return WEST;
  }
  else if (y == 1)
  {
  cout<<"NORTH"<<endl;
    return NORTH;
  }
  else
  {
  cout<<"SOUTH"<<endl;
    return SOUTH;
  }
}

//work in progress
void backTracking()
{

  //bool cellVisited[][] = true;
}

//function that calls all other functions that needs to update per cycle
void updateValues(double botAngle)
{

  //turnLogic(botAngle);
  testTurn();
  //onlyPositives(botAngle);
  //sets struct to values coming from onlyPositives(botAngle);
  currentCoord = {xValue, zValue};

  // only run the functions inside if the last location is different than the current location else ignore this and keeps moving
  if ((previousCoord.xCoordinate != currentCoord.xCoordinate) || (previousCoord.zCoordinate != currentCoord.zCoordinate)) //missing pathFound condition, disable if and use other if when found
  {
    //saves the currentCoord in another struct to be compared later
    previousCoord = currentCoord;
    wallDetection(currentCoord, botAngle);
    backTracking();
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

  // Emitter Setup
  // Emitter *emitter;
  // emitter=supervisor->getEmitter("emitter");

  // emitter->setChannel(1);

  robot_node = supervisor->getFromDef("botJr");
  trans_field = robot_node->getField("translation");
  rotation_field = robot_node->getField("rotation");
  compass = supervisor->getCompass("compass");
  compass->enable(TIME_STEP);
}

int main()
{

  supervisor = new Supervisor();

  setup();

  while (supervisor->step(TIME_STEP) != -1)
  {

    // this is done repeatedly
    trans_values = trans_field->getSFVec3f();
    rotationValues = rotation_field->getSFRotation();
    compassDirection = compass->getValues();
    botAngle = rotationValues[3];
    updateValues(botAngle);
    setTrueDirection();
    //cout << "compass points to: " << setTrueDirection() << endl;
    
  }

  delete supervisor;
  return 0;
}