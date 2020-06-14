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
enum movementDirection
{
  NORTH, // 0
  EAST,  // 1
  SOUTH, // 2
  WEST   // 3

};

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
  movementDirection direction;
  bool intersectionExists;
};

struct Walls
{
  //what walls are available at said coordinate true=wall
  bool up;
  bool right;
  bool down;
  bool left;
};

//initializers && variables
Compass *compass;
Supervisor *supervisor;
DistanceSensor *ds[4];

char dsNames[4][11] = {"ds_front", "ds_right", "ds_left", "ds_back"};
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
bool setStartCoordsOnce = true;
bool isBackTracking = false;
bool initialPop = true;
bool isVisited;
Coordinates previousCoord; //= {0, 0};
Coordinates currentCoord;  // {0,0};
stack<CoordinateWalls> coordStack;
double UP = 1.5708;     // 90 degrees
double RIGHT = 0.0;     // 0 degrees
double DOWN = -1.5708;  // 270/-90 degrees
double LEFT = 3.1415;   // 180 degrees
double margin = 0.7854; // 45 degrees
double _botDirection;
double directionValue[] = {0, 1, 0, 0};
double trans_center_values[] = {0.0625, 0.02, 0.0625}; // center of (0,0)
bool visitedSquares[15][15] = {{false}};
Walls visitedWalls[15][15] = {{false}};

// struct Coordinates slave;
// struct CoordinateWalls test;

// where we've been, where we haven't been too and correctpath

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

//function for detecting walls and saving them based on the current direction
void wallDetection(Coordinates xzCoords, movementDirection direction)
{

  //local variables
  int frontSensorData = ds[0]->getValue();
  int rightSensorData = ds[1]->getValue();
  int leftSensorData = ds[2]->getValue();
  int backSensorData = ds[3]->getValue();
  bool _UP = false;
  bool _RIGHT = false;
  bool _DOWN = false;
  bool _LEFT = false;

  //cycles through all 4 directions and update the walls values based on current angle
  //NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3
  if (direction == EAST)
  {
    //GOING RIGHT

    if (frontSensorData < 1000)
      _RIGHT = true;
    if (rightSensorData < 1000)
      _DOWN = true;
    if (leftSensorData < 1000)
      _UP = true;
    if (backSensorData < 1000)
      _LEFT = true;
  }

  if (direction == SOUTH)
  {
    //GOING DOWN

    if (frontSensorData < 1000)
      _DOWN = true;
    if (rightSensorData < 1000)
      _LEFT = true;
    if (leftSensorData < 1000)
      _RIGHT = true;
    if (backSensorData < 1000)
      _UP = true;
  }

  if (direction == WEST)
  {
    //GOING LEFT

    if (frontSensorData < 1000)
      _LEFT = true;
    if (rightSensorData < 1000)
      _UP = true;
    if (leftSensorData < 1000)
      _DOWN = true;
    if (backSensorData < 1000)
      _RIGHT = true;
  }

  if (direction == NORTH)
  {
    //GOING UP

    if (frontSensorData < 1000)
      _UP = true;
    if (rightSensorData < 1000)
      _RIGHT = true;
    if (leftSensorData < 1000)
      _LEFT = true;
    if (backSensorData < 1000)
      _DOWN = true;
  }

  //fills struct with coordinate pair, detected walls and current direction

  //if(visitedWalls[(int)xzCoords.xCoordinate][(int)xzCoords.zCoordinate]);
  if (!visitedSquares[(int)xzCoords.xCoordinate][(int)xzCoords.zCoordinate])
  {
    visitedWalls[(int)xzCoords.xCoordinate][(int)xzCoords.zCoordinate] = {_UP, _RIGHT, _DOWN, _LEFT};
  }
  {
  }
  visitedWalls[(int)xzCoords.xCoordinate][(int)xzCoords.zCoordinate] = {_UP, _RIGHT, _DOWN, _LEFT};

  //reset local variables for the next coordinate pair
  _UP = false;
  _RIGHT = false;
  _DOWN = false;
  _LEFT = false;
}

bool intersectionCheck(double x, double y)
{

  //PROBLEM HERE MAKE IT CHECK SENSORS TO DETERMINE INTERSECTION AND NOT X,

  int pathsFound = 0;
  Walls currentSpotWalls = visitedWalls[(int)x][(int)y];

  if (currentSpotWalls.up == false)
  {
    pathsFound++;
  }
  if (currentSpotWalls.down == false)
  {
    pathsFound++;
  }
  if (currentSpotWalls.right == false)
  {
    pathsFound++;
  }
  if (currentSpotWalls.left == false)
  {
    pathsFound++;
  }
  //cout << "pathfound: " << pathsFound << endl;
  if (pathsFound >= 3)
  {
    cout << "intersection found! " << endl;
    return true;
  }
  else
  {
    pathsFound = 0;
    return false;
  }
}

void getStartingCoordinates(double x, double z, bool state, movementDirection direction)
{

  Coordinates startCoord = {x, z};

  if (state)
  {
    //startCoord = {x,z};
    previousCoord = startCoord;
    currentCoord = startCoord;
    wallDetection(startCoord, direction);
    state = false;
  }
}

void onlyPositives(movementDirection direction) //double incomingAngle)//THIS IS ONE OF THEM
{
  // checks if it is out of bounds or not (0.0625 sets it to the middle of a square and *8 scales it up)
  double scaledTransValueX = ((trans_values[0] - 0.0625) * 8);
  double scaledTransValueZ = ((trans_values[2] - 0.0625) * 8);

  if (setStartCoordsOnce)
  {
    double tempx = round(((trans_values[0] - 0.0625) * 8));
    double tempz = round(((trans_values[2] - 0.0625) * 8));
    getStartingCoordinates(tempx, tempz, setStartCoordsOnce, direction);
    setStartCoordsOnce = false;
  }

  //check if it is not out of bounds  X-AXIS
  if (scaledTransValueX >= 0)
  {

    if (direction == NORTH)
    {

      //checks currentCoord with realtime location
      //if bot passed the connecter/threshold between two squares
      if (scaledTransValueX >= (currentCoord.xCoordinate + 0.75)) //THIS IS ONE OF THEM  "inboud bullshit"
      {

        //move it to the center of the new square
        xValue = round(scaledTransValueX);
        zValue = round(scaledTransValueZ);
      }
      else
      {
        //sets currentCoord back to what it was
        xValue = currentCoord.xCoordinate;
        zValue = currentCoord.zCoordinate;
      }
    }
    //same as UP but for down instead
    //if (incomingAngle <= (DOWN + margin) && incomingAngle >= (DOWN - margin))
    if (direction == SOUTH)
    {
      //if bot passed the connecter/threshold between two squares
      if (scaledTransValueX <= (currentCoord.xCoordinate - 0.75))
      {

        xValue = round(scaledTransValueX);
        zValue = round(scaledTransValueZ);
      }
      else
      {
        xValue = currentCoord.xCoordinate;
        zValue = currentCoord.zCoordinate;
      }
    }
  }
  // same as the if's above but for the Z-AXIS
  if (scaledTransValueZ >= 0)
  {

    //if (incomingAngle <= (RIGHT + margin) && incomingAngle >= (RIGHT - margin)) //checks if its going right
    if (direction == EAST)
    {

      if (scaledTransValueZ >= (currentCoord.zCoordinate + 0.75))
      {
        xValue = round(scaledTransValueX);
        zValue = round(scaledTransValueZ);
      }
      else
      {
        xValue = currentCoord.xCoordinate;
        zValue = currentCoord.zCoordinate;
      }
    }

    //if ((incomingAngle <= (LEFT + margin) && incomingAngle >= (LEFT - margin)) || (incomingAngle <= (-LEFT + margin) && incomingAngle >= (-LEFT - margin)))
    if (direction == WEST)
    {
      //cout<<"POSITIVE WEST:"<<endl;
      if (scaledTransValueZ <= (currentCoord.zCoordinate - 0.75))
      {
        xValue = round(scaledTransValueX);
        zValue = round(scaledTransValueZ);
      }
      else
      {
        xValue = currentCoord.xCoordinate;
        zValue = currentCoord.zCoordinate;
      }
    }
  }
}

movementDirection reverseDirection(movementDirection topStackDirection)
{
  switch (topStackDirection)
  {
  case WEST:
    return EAST;
    break;

  case EAST:
    return WEST;
    break;

  case NORTH:
    return SOUTH;
    break;

  case SOUTH:
    return NORTH;
    break;
  }
  return SOUTH;
}

// void testTurnAfterBacktracking(movementDirection direction)
// {

//   botMovement(false);
//   switch (direction)
//   {
//   case EAST:
//     directionValue[3] = 0;
//     break;

//   case WEST:
//     directionValue[3] = 3.1415;
//     break;

//   case NORTH:
//     directionValue[3] = 1.5708;
//     break;

//   case SOUTH:
//     directionValue[3] = -1.5708;
//     break;
//   }

//   //coordStack.pop();
//   setRotationXYZ();
//   rotation_field->setSFRotation(directionValue);
//   // might be here
//   // cout << " SETS isBackTracking to FALSE in testTurnAfterBacktracking " << endl;
//   // isBackTracking = false;
//   // botMovement(true);
// }

//work in progress
void backTracking() //THIS IS ONE OF THEM
{
  movementDirection topStackReverseDirection;
  Coordinates topStackPair = coordStack.top().ptnPair;
  Walls currentWalls = visitedWalls[curX][curZ];
  int curX = (int)currentCoord.xCoordinate;
  int curZ = (int)currentCoord.zCoordinate;

  if (coordStack.empty() == false)
  {

    botMovement(false);
    //missing out of bounds conditions
    if ((currentCoord.zCoordinate - topStackPair.zCoordinate) < 0)
    {
      topStackReverseDirection = EAST;
    }

    if ((currentCoord.zCoordinate - topStackPair.zCoordinate) > 0)
    {
      topStackReverseDirection = WEST;
    }

    if ((currentCoord.xCoordinate - topStackPair.xCoordinate) > 0)
    {
      topStackReverseDirection = SOUTH;
    }

    if ((currentCoord.xCoordinate - topStackPair.xCoordinate) < 0)
    {
      topStackReverseDirection = NORTH;
    }

    switch (topStackReverseDirection)
    {
    case EAST:
      directionValue[3] = 0;
      break;

    case WEST:
      directionValue[3] = 3.1415;
      break;

    case NORTH:
      directionValue[3] = 1.5708;
      break;

    case SOUTH:
      directionValue[3] = -1.5708;
      break;
    }

    if ((currentCoord.xCoordinate == topStackPair.xCoordinate) && (currentCoord.zCoordinate == topStackPair.zCoordinate))
    {

      if (coordStack.top().intersectionExists)
      {
        //if conditions turn right and pop
        if((currentCoord.zCoordinate - topStackPair.zCoordinate) < 0)){
          topStackReverseDirection = EAST;
        }
        //if conditions turn left and pop

        //if condityions turn up and pop

        //if condition turn down and pop
      }
      else // if no intersection found, pop and try again
      {
        coordStack.pop();
      }
    }

    setRotationXYZ();
    rotation_field->setSFRotation(directionValue);
    coordStack.pop();
    cout << "stack size after pop: " << coordStack.size() << endl;
    botMovement(true);
  }
}

void testTurn(movementDirection direction)
{
  int frontSensorData = ds[0]->getValue();
  int rightSensorData = ds[1]->getValue();
  int leftSensorData = ds[2]->getValue();
  movementDirection tempDirection;
  CoordinateWalls readyToPush = {};

  if (frontSensorData < 525)
  {
    //cout<<"front sensor : " << frontSensorData<<endl;

    botMovement(false);
    if (rightSensorData > 800)
    {
      //cout<<"right sensor sees no wall: " <<endl;

      switch (direction)
      {

      case EAST:
        directionValue[3] = -1.5708;
        tempDirection = SOUTH;
        break;

      case WEST:
        directionValue[3] = 1.5708;
        tempDirection = NORTH;
        break;

      case NORTH:
        directionValue[3] = 0;
        tempDirection = EAST;
        break;

      case SOUTH:
        directionValue[3] = 3.1415;
        tempDirection = WEST;
        break;
      }
      setRotationXYZ();
      rotation_field->setSFRotation(directionValue);
      readyToPush = {currentCoord, tempDirection, false};
      coordStack.push(readyToPush);
      botMovement(true);
      cout << "stack size after push: " << coordStack.size() << endl;
    }
    else if (leftSensorData > 800)
    {

      switch (direction)
      {

      case EAST:
        directionValue[3] = 1.5708;
        tempDirection = NORTH;
        break;

      case WEST:
        directionValue[3] = -1.5708;
        tempDirection = SOUTH;
        break;

      case NORTH:
        directionValue[3] = 3.1415;
        tempDirection = WEST;
        break;

      case SOUTH:
        directionValue[3] = 0;
        tempDirection = EAST;
        break;
      }
      setRotationXYZ();
      rotation_field->setSFRotation(directionValue);
      readyToPush = {currentCoord, tempDirection, false};
      coordStack.push(readyToPush);
      botMovement(true);
      cout << "stack size after push: " << coordStack.size() << endl;
    }
    else
    {
      directionValue[3] += 3.1415;
      setRotationXYZ();
      rotation_field->setSFRotation(directionValue);
      isBackTracking = true;
      botMovement(true);
    }
  }
  else
  {
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
    //cout<<"EAST"<<endl;
    return EAST;
  }
  else if (x == -1)
  {
    //cout<<"WEST"<<endl;
    return WEST;
  }
  else if (y == 1)
  {
    //cout<<"NORTH"<<endl;
    return NORTH;
  }
  else
  {
    //cout<<"SOUTH"<<endl;
    return SOUTH;
  }
}

//function that calls all other functions that needs to update per cycle
void updateValues()
{

  if (isBackTracking == false)
  {
    testTurn(setTrueDirection());
  }

  onlyPositives(setTrueDirection());

  currentCoord = {xValue, zValue};

  if (!visitedSquares[(int)xValue][(int)zValue]) //GET BACK TO THIS
  {
    visitedSquares[(int)xValue][(int)zValue] = true;
  }

  if (isBackTracking == false && ((previousCoord.xCoordinate != currentCoord.xCoordinate) || (previousCoord.zCoordinate != currentCoord.zCoordinate))) //missing pathFound condition, disable if and use other if when found
  {

    previousCoord = currentCoord;
    wallDetection(currentCoord, setTrueDirection());

    if (intersectionCheck(xValue, zValue))
    {
    }
  }
  else if (isBackTracking)
  {
    backTracking();
  }
}
void setup()
{

  for (int i = 0; i < 4; i++) //SENSOR INITIALIZERS
  {
    ds[i] = supervisor->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
  }

  for (int i = 0; i < 4; i++) //WHEEL INITIALIZERS
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
    setTrueDirection();
    updateValues();
  }

  delete supervisor;
  return 0;
}