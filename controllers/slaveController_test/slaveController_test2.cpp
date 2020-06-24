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

/*####### STRUCTS ########*/

enum movementDirection      //Struct used for declaring directions
{
  NORTH, // 0
  EAST,  // 1
  SOUTH, // 2
  WEST   // 3

};

struct Coordinates //Struct used for saving coordinates 
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


/*####### NODES ########*/

Compass *compass;
Supervisor *supervisor;
DistanceSensor *ds[4];

/*####### INITIALIZERS AND VARIABLES ########*/

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

/*####### METHODS ########*/

//set coordinate values for transport
Coordinates structTransport(double xCoord, double zCoord)
{
  Coordinates transport = {(double)xCoord, (double)zCoord};
  return transport;
}

//stops the bot when called or restarts movement
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

//set the values so the bot is correctly angled on the board
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
    //IF GOING RIGHT, CHECK ALL SENSORS
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

  //fills struct with detected walls 
  if (!visitedSquares[(int)xzCoords.xCoordinate][(int)xzCoords.zCoordinate])
  {
    visitedWalls[(int)xzCoords.xCoordinate][(int)xzCoords.zCoordinate] = {_UP, _RIGHT, _DOWN, _LEFT};
  }
  visitedWalls[(int)xzCoords.xCoordinate][(int)xzCoords.zCoordinate] = {_UP, _RIGHT, _DOWN, _LEFT};

  //reset local variables for the next coordinate pair
  _UP = false;
  _RIGHT = false;
  _DOWN = false;
  _LEFT = false;
}

bool intersectionCheck(double x, double z)    //CHECKS FOR INTERSECTIONS
{
  //PROBLEM HERE MAKE IT CHECK SENSORS TO DETERMINE INTERSECTION AND NOT X,
  int pathsFound = 0;
  Walls currentSpotWalls = visitedWalls[(int)x][(int)z];

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
 
  if (pathsFound >= 3)  //WHEN MORE THAN OR EQUAL TO 3 PATHS
  {
    return true;
  }
  else
  {
    pathsFound = 0;
    return false;
  }
}

void getStartingCoordinates(double x, double z, bool state, movementDirection direction)    //USED TO SET STARTING COORDINATES OF BOT
{

  Coordinates startCoord = {x, z};

  if (state)
  {
    previousCoord = startCoord;
    currentCoord = startCoord;
    wallDetection(startCoord, direction);
    state = false;
  }
}

void onlyPositives(movementDirection direction)   //NOT OUT OF BOUNDS 
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
    if (direction == NORTH)
    {
      //checks currentCoord with realtime location
      //if bot passed the connecter/threshold between two squares
      if (scaledTransValueX >= (currentCoord.xCoordinate + 0.75)) 
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
    if (direction == WEST)
    {
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
  botMovement(true);
}

void testTurnBackTrack(movementDirection reverseDirection)
{
  int frontSensorData = ds[0]->getValue();

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

    rotation_field->setSFRotation(directionValue);
    setRotationXYZ();
    botMovement(true);
  }
  else
  {
    botMovement(true);
  }
}

movementDirection reverseDirection(movementDirection topStackDirection)   //TAKES INCOMING DIRECTION AND SETS IT TO OPPOSITE. USED IN BACKTRACKING METHODS
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

void testTurnAfterBacktracking(movementDirection direction) //TAKES THE DIRECTION OF THE PREVIOUS COORD AND INVERTS IT
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

  double tempx = round(((trans_values[0] - 0.0625) * 8));
  double tempz = round(((trans_values[2] - 0.0625) * 8));

  coordStack.pop();

  setRotationXYZ();
  rotation_field->setSFRotation(directionValue);
  isBackTracking = false;
  botMovement(true);
}

void backTracking() //BACKTRACKING WHEN ENCOUNTERS A DEAD END
{

  if (!coordStack.empty())
  {

    Coordinates topStackPair = coordStack.top().ptnPair;
    movementDirection topStackReversedDirection = reverseDirection(coordStack.top().direction);

    testTurnBackTrack(topStackReversedDirection); // only 180 100% not the problem

    int curX = (int)currentCoord.xCoordinate;
    int curZ = (int)currentCoord.zCoordinate;

    if (currentCoord.xCoordinate == topStackPair.xCoordinate && currentCoord.zCoordinate == topStackPair.zCoordinate)
    {
      if (coordStack.top().intersectionExists)
      {
        double scaledTransValueX = ((trans_values[0] - 0.0625) * 8);
        double scaledTransValueZ = ((trans_values[2] - 0.0625) * 8);

        //PROBLEM IS IN HERE, EVEN IF INTERSECTION FOUND DOES NOT TURN LEFT

        Walls currentWalls = visitedWalls[curX][curZ];
        if (currentWalls.up == false)
        {
          if (visitedSquares[curX + 1][curZ] == false)
          {
            if (topStackReversedDirection == WEST)
            {
              if (scaledTransValueZ <= topStackPair.zCoordinate)
              {
                curX = round(scaledTransValueX);
                testTurnAfterBacktracking(NORTH);
              }
              if (topStackReversedDirection == EAST)
              {
                if (scaledTransValueZ >= (topStackPair.zCoordinate + 0.0625))
                {
                  curX = round(scaledTransValueX);
                  testTurnAfterBacktracking(NORTH);
                }
              }
            }
          }
        }
        if (currentWalls.down == false)
        {
          if (visitedSquares[curX - 1][curZ] == false)
          {
            if (topStackReversedDirection == WEST)
            {
              if (scaledTransValueZ <= topStackPair.zCoordinate)
              {
                testTurnAfterBacktracking(SOUTH);
              }
            }
            if (topStackReversedDirection == EAST)
            {
              if (scaledTransValueZ >= topStackPair.zCoordinate)
              {
                testTurnAfterBacktracking(SOUTH);
              }
            }
          }
        }
        if (currentWalls.left == false)
        {
          if (visitedSquares[curX][curZ - 1] == false)
          {
            if (topStackReversedDirection == NORTH)
            {
              if (scaledTransValueX >= topStackPair.xCoordinate)
              {
                testTurnAfterBacktracking(WEST);
              }
            }
            if (topStackReversedDirection == SOUTH)
            {
              if (scaledTransValueX <= (topStackPair.xCoordinate))
              {
                testTurnAfterBacktracking(WEST);
              }
            }
          }
        }
        if (currentWalls.right == false)
        {
          if (visitedSquares[curX][curZ + 1] == false)
          {
            if (topStackReversedDirection == NORTH)
            {
              if (scaledTransValueX >= topStackPair.zCoordinate)
              {
                testTurnAfterBacktracking(EAST);
              }
            }
            //cout<<"reversed direction : "<<topStackReversedDirection<<endl;
            if (topStackReversedDirection == SOUTH)
            {
              if (scaledTransValueX <= (topStackPair.xCoordinate))
              {
                testTurnAfterBacktracking(EAST);
              }
            }
            if (topStackReversedDirection == EAST)
            {
              if (scaledTransValueZ <= (topStackPair.zCoordinate))
              {
                testTurnAfterBacktracking(EAST);
              }
            }
          }
        }
      }
      else //all paths have been charted
      {
        cout << " x: " << topStackPair.xCoordinate << " z: " << topStackPair.zCoordinate << " DIR: " << coordStack.top().direction << " INT: " << coordStack.top().intersectionExists << " ss: " << coordStack.size() << endl;
        cout << " " << endl;
        coordStack.pop();
      }
    }
  }
  else
  {
    isBackTracking = false;
  }
}

void testTurn(movementDirection direction)  //TURNS TO SPECIFIED DIRECTION
{
  int frontSensorData = ds[0]->getValue();
  int rightSensorData = ds[1]->getValue();
  int leftSensorData = ds[2]->getValue();
  movementDirection tempDirection;

  if (frontSensorData < 525)
  {
    botMovement(false);
    if (rightSensorData > 800)
    {
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
      rotation_field->setSFRotation(directionValue);
      setRotationXYZ();
      coordStack.push(CoordinateWalls{currentCoord, tempDirection, false});
      if (!coordStack.empty())
      {
        cout << "TEST TURN RIGHT" << endl;
        cout << " x: " << coordStack.top().ptnPair.xCoordinate << " z: " << coordStack.top().ptnPair.zCoordinate << " DIR: " << coordStack.top().direction << " INT: " << coordStack.top().intersectionExists << " ss: " << coordStack.size() << endl;
        cout << " " << endl;
      }
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
      rotation_field->setSFRotation(directionValue);
      setRotationXYZ();
      coordStack.push(CoordinateWalls{currentCoord, tempDirection, false});
      if (!coordStack.empty())
      {
        cout << "TEST TURN LEFT" << endl;
        cout << " x: " << coordStack.top().ptnPair.xCoordinate << " z: " << coordStack.top().ptnPair.zCoordinate << " DIR: " << coordStack.top().direction << " INT: " << coordStack.top().intersectionExists << " ss: " << coordStack.size() << endl;
        cout << " " << endl;
      }
    }
    else
    {
      Walls tempWallLayout = visitedWalls[(int)coordStack.top().ptnPair.xCoordinate][(int)coordStack.top().ptnPair.zCoordinate];
      //compare vis walls at coord
      if (tempWallLayout.up && tempWallLayout.down && tempWallLayout.left)
      {
        //checking for right wall
        if (!tempWallLayout.right)
        {
          coordStack.push(CoordinateWalls{currentCoord, EAST, true});
        }
      }
      setRotationXYZ();
      // directionValue[3] += 3.1415;
      // rotation_field->setSFRotation(directionValue);
      // botMovement(false);
      isBackTracking = true;
      backTracking();
    }
  }
  else
  {
    botMovement(true);
  }
}

movementDirection setTrueDirection()    //COMPASS METHOD
{
  // [0] == z , [2] == x

  double x = round(compassDirection[0]);
  double y = round(compassDirection[2]);

  if (x == 1)
  {
    return EAST;
  }
  else if (x == -1)
  {
    return WEST;
  }
  else if (y == 1)
  {
    return NORTH;
  }
  else
  {
    return SOUTH;
  }
}

//function that calls all other functions that needs to update per cycle
void updateValues()
{

  if (!isBackTracking)
  {
    testTurn(setTrueDirection());
  }
  onlyPositives(setTrueDirection());

  //sets struct to values coming from onlyPositives(botAngle);
  currentCoord = {xValue, zValue};

  if (!visitedSquares[(int)xValue][(int)zValue])
  {
    visitedSquares[(int)xValue][(int)zValue] = true;
    //cout << "Have i visited this square before at " << xValue << " & " << zValue << " = " << visitedSquares[(int)xValue][(int)zValue] << endl;
  }

  // only run the functions inside if the last location is different than the current location else ignore this and keeps moving
  if (!isBackTracking && ((previousCoord.xCoordinate != currentCoord.xCoordinate) || (previousCoord.zCoordinate != currentCoord.zCoordinate)))
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
      }
    }
  }
  else if (isBackTracking)
  {
    backTracking();
  }
}
void setup()    //RUN SETUP ONCE FOR INITIALIZATION
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

  robot_node = supervisor->getFromDef("botJr2");
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
    trans_values = trans_field->getSFVec3f();
    rotationValues = rotation_field->getSFRotation();
    compassDirection = compass->getValues();
    updateValues();
    setTrueDirection();
  }

  delete supervisor;
  return 0;
}