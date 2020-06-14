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

  // cout << "UP WALL AT COORD " << xzCoords.xCoordinate << " & " << xzCoords.zCoordinate << " IS " << visitedWalls[(int)xzCoords.xCoordinate][(int)xzCoords.zCoordinate].up << endl;
  // cout << "DOWN WALL AT COORD " << xzCoords.xCoordinate << " & " << xzCoords.zCoordinate << " IS " << visitedWalls[(int)xzCoords.xCoordinate][(int)xzCoords.zCoordinate].down << endl;
  // cout << "LEFT WALL AT COORD " << xzCoords.xCoordinate << " & " << xzCoords.zCoordinate << " IS " << visitedWalls[(int)xzCoords.xCoordinate][(int)xzCoords.zCoordinate].left << endl;
  // cout << "RIGHT WALL AT COORD " << xzCoords.xCoordinate << " & " << xzCoords.zCoordinate << " IS " << visitedWalls[(int)xzCoords.xCoordinate][(int)xzCoords.zCoordinate].right << endl;

  //push the struct to the stack

  // so it figures out it needs to turn the other open path, otherwise it always gets stuck

  // ye that was the logic before walls shit so change it then
  //coordStack.push(currentWalls);

  // if (!coordStack.empty())
  // {
  //   cout << " xCoord: " << coordStack.top().ptnPair.xCoordinate << endl;
  //   cout << " zCoord: " << coordStack.top().ptnPair.zCoordinate << endl;
  //   cout << " UP: " << coordStack.top().up << endl;
  //   cout << " RIGHT: " << coordStack.top().right << endl;
  //   cout << " DOWN: " << coordStack.top().down << endl;
  //   cout << " LEFT: " << coordStack.top().left << endl;
  //   cout << " DIRECTION: " << coordStack.top().direction << endl;
  //   cout << "===================" << endl;
  // }

  //send to stack to master

  //reset local variables for the next coordinate pair
  _UP = false;
  _RIGHT = false;
  _DOWN = false;
  _LEFT = false;
}

bool intersectionCheck(double x, double y)
{

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
    //cout << "intersection found! " << endl;
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

/*

  TO FIX INSTEAD OF CHECKING FOR THE CURRENT COORD +1 CHANGE IT TO ACTUAL VALUE ROUNDED

*/
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

  //NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3

  //check if it is not out of bounds  X-AXIS
  if (scaledTransValueX >= 0)
  {

    //if (incomingAngle <= (UP + margin) && incomingAngle >= (UP - margin)) //checks if its going RIGHT?? ((I THINK UP INSTEAD)), also takes into account slight deviation possibility
    if (direction == NORTH)
    {

      //checks currentCoord with realtime location
      //if bot passed the connecter/threshold between two squares
      if (scaledTransValueX >= (currentCoord.xCoordinate + 0.5)) //THIS IS ONE OF THEM  "inboud bullshit"
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
      if (scaledTransValueX <= (currentCoord.xCoordinate - 0.5))
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

      if (scaledTransValueZ >= (currentCoord.zCoordinate + 0.5))
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
      if (scaledTransValueZ <= (currentCoord.zCoordinate - 0.5))
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
  botMovement(true);
}

void testTurnBackTrack(movementDirection reverseDirection) //THIS IS ONE OF THEM
{
  int frontSensorData = ds[0]->getValue();
  // int rightSensorData = ds[1]->getValue();
  // int leftSensorData = ds[2]->getValue();

  if (frontSensorData < 500)
  {

    switch (reverseDirection)
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
    botMovement(false);
    setRotationXYZ();
    rotation_field->setSFRotation(directionValue);
    botMovement(true);
  }
  else
  {
    botMovement(true);
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
  cout << "MAYDAY MAYDAY WE MALFUNCTIONED!!!!!!!!!!!!!!!!!!!" << endl;
  return SOUTH;
}

void testTurnAfterBacktracking(movementDirection direction)
{
  botMovement(false);
  switch (direction)
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
  setRotationXYZ();
  rotation_field->setSFRotation(directionValue);
  isBackTracking = false;
  botMovement(true);
}

//work in progress
void backTracking() //THIS IS ONE OF THEM
{

  if (!coordStack.empty())
  {

    Coordinates topStackPair = coordStack.top().ptnPair;
    movementDirection topStackReversedDirection = reverseDirection(coordStack.top().direction);

    testTurnBackTrack(topStackReversedDirection);

    int curX = (int)currentCoord.xCoordinate;
    int curZ = (int)currentCoord.zCoordinate;

    if (currentCoord.xCoordinate == topStackPair.xCoordinate && currentCoord.zCoordinate == topStackPair.zCoordinate)
    {
      if (coordStack.top().intersectionExists)
      {
        double botTranslationX = trans_values[0];
        double botTranslationZ = trans_values[0];
        // double tempx = round(((trans_values[0] - 0.0625) * 8));
        // double tempz = round(((trans_values[2] - 0.0625) * 8));
        Walls currentWalls = visitedWalls[curX][curZ];
        if (currentWalls.up == false)
        {

          if (visitedSquares[curX + 1][curZ] == false)
          {
            //if bot going left + 0.0625
            testTurnAfterBacktracking(NORTH);
          }
        }
        else if (currentWalls.down == false)
        {
          if (visitedSquares[curX - 1][curZ] == false)
          {
            testTurnAfterBacktracking(SOUTH);
          }
        }
        else if (currentWalls.left == false)
        {
          if (visitedSquares[curX][curZ - 1] == false)
          {
            testTurnAfterBacktracking(WEST);
          }
        }
        else if (currentWalls.right == false)
        {
          if (visitedSquares[curX][curZ + 1] == false)
          {
            testTurnAfterBacktracking(EAST);
          }
        }
      }
      else
      {
        cout << " xCoord: " << topStackPair.xCoordinate << endl;
        cout << " zCoord: " << topStackPair.zCoordinate << endl;
        cout << " DIRECTION: " << coordStack.top().direction << endl;
        cout << " stack size: " << coordStack.size() << endl;
        cout << "===================" << endl;
        cout << "stack popped" << endl;
        coordStack.pop();
      }
    }
  }
  else
  {
    cout << "backtrack else loop: " << endl;

    // CHECK FOR INTERSECTION AND TURN ACCORDINGLY

    // DOES NOT RETURN isBackTracking until

    //if intersection has a all visited nodes around it
    // pop that intersection and keep backtracking
    // if it has a unvisited node return false for backtracking and continue mapping

    isBackTracking = false;
  }
}

void testTurn(movementDirection direction)
{
  int frontSensorData = ds[0]->getValue();
  int rightSensorData = ds[1]->getValue();
  int leftSensorData = ds[2]->getValue();
  movementDirection tempDirection;

  if (frontSensorData < 500)
  {

    botMovement(false);
    if (rightSensorData > 600)
    {

      setRotationXYZ();

      switch (direction)
      {

      case EAST:
        directionValue[3] = -1.5708;
        tempDirection = EAST;
        break;

      case WEST:
        directionValue[3] = 1.5708;
        tempDirection = WEST;
        break;

      case NORTH:
        directionValue[3] = 0;
        tempDirection = NORTH;
        break;

      case SOUTH:
        directionValue[3] = 3.1415;
        tempDirection = SOUTH;
        break;
      }
      rotation_field->setSFRotation(directionValue);
      coordStack.push(CoordinateWalls{currentCoord, tempDirection, false});
      if (!coordStack.empty())
      {
        cout << " xCoord: " << coordStack.top().ptnPair.xCoordinate << endl;
        cout << " zCoord: " << coordStack.top().ptnPair.zCoordinate << endl;
        cout << " DIRECTION: " << coordStack.top().direction << endl;
        cout << " stack size: " << coordStack.size() << endl;
        cout << "===================" << endl;
      }
    }
    else if (leftSensorData > 600)
    {
      setRotationXYZ();

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
      rotation_field->setSFRotation(directionValue);
      coordStack.push(CoordinateWalls{currentCoord, tempDirection, false});
      if (!coordStack.empty())
      {
        cout << " xCoord: " << coordStack.top().ptnPair.xCoordinate << endl;
        cout << " zCoord: " << coordStack.top().ptnPair.zCoordinate << endl;
        cout << " DIRECTION: " << coordStack.top().direction << endl;
        cout << " stack size: " << coordStack.size() << endl;
        cout << "===================" << endl;
      }
    }
    else
    {

      // setRotationXYZ();
      // directionValue[3] += 3.1415;
      // rotation_field->setSFRotation(directionValue);
      // botMovement(false);
      isBackTracking = true; //?? need to fix ??
      backTracking();        //THIS IS ONE OF THEM
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

  //turnLogic(botAngle);
  if (!isBackTracking)
  {
    testTurn(setTrueDirection());
  }
  onlyPositives(setTrueDirection()); //THIS IS ONE OF THEM

  //sets struct to values coming from onlyPositives(botAngle);
  currentCoord = {xValue, zValue};

  if (!visitedSquares[(int)xValue][(int)zValue])
  {
    visitedSquares[(int)xValue][(int)zValue] = true;
    //cout << "Have i visited this square before at " << xValue << " & " << zValue << " = " << visitedSquares[(int)xValue][(int)zValue] << endl;
  }

  //cout << "x coord: " << xValue << endl;

  // only run the functions inside if the last location is different than the current location else ignore this and keeps moving
  if (!isBackTracking && ((previousCoord.xCoordinate != currentCoord.xCoordinate) || (previousCoord.zCoordinate != currentCoord.zCoordinate))) //missing pathFound condition, disable if and use other if when found
  {
    //saves the currentCoord in another struct to be compared later
    previousCoord = currentCoord;
    wallDetection(currentCoord, setTrueDirection());

    if (intersectionCheck(xValue, zValue))
    {
      CoordinateWalls xzDir = {currentCoord, setTrueDirection(), true};
      coordStack.push(xzDir);
      if (!coordStack.empty())
      {
        cout << " xCoord: " << coordStack.top().ptnPair.xCoordinate << endl;
        cout << " zCoord: " << coordStack.top().ptnPair.zCoordinate << endl;
        cout << " DIRECTION: " << coordStack.top().direction << endl;
        cout << " stack size: " << coordStack.size() << endl;
        cout << "===================" << endl;
      }
    }
  }
  else if (isBackTracking)
  {
    //cout<< "shadow is a bnish" << endl;
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
    //botAngle = rotationValues[3];
    updateValues();
    setTrueDirection();
    //cout << "compass points to: " << setTrueDirection() << endl;
  }

  delete supervisor;
  return 0;
}