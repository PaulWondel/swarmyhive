#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Compass.hpp>
#include <webots/Receiver.hpp>
#include <webots/Emitter.hpp>
#include <cmath>
#include <stack>

#define _USE_MATH_DEFINES
#define TIME_STEP 256

using namespace webots;
using namespace std;

/*####### STRUCTS ########*/

enum movementDirection //Struct used for declaring directions
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

struct sendPackage // save info in struct to send in a package
{
  CoordinateWalls botInfo;
  Walls wallInfo;
};

struct exitSignal {
  int tag;
  char messageContent[];
};

/*####### NODES ########*/

Compass *compass;
Supervisor *supervisor;
DistanceSensor *ds[4];
Emitter *emitter;
Receiver *receiver;

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
Walls intersectionWalls = {{false}};

// for exit Signal
const int exitTag = 162;

// where we've been, where we haven't been too and correctpath

/*####### METHODS ########*/

//set coordinate values for transport

// For sending messages
void sendCoordinates(Coordinates message, Emitter *device){
  device->send(&message,sizeof(message));
  return;
}

void sendIntersectionInfo(CoordinateWalls intersectionInfo){
  emitter->send(&intersectionInfo,sizeof(intersectionInfo));
  return;
}

void sendWalls(Walls message){
  emitter->send(&message,sizeof(message));
  return;
}
void sendPackageFuncton(sendPackage message){
  emitter->send(&message,sizeof(message));
  return;
}

//set coordinate values for transport
// Coordinates structTransport(double xCoord, double zCoord)
// {
//   Coordinates transport = {(double)xCoord, (double)zCoord};
//   return transport;
// }

// CoordinateWalls structTransport(double xCoord, double zCoord)
// {
//   CoordinateWalls transport = {(double)xCoord, (double)zCoord};
//   return transport;
// }


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

  if (_UP)
  {
  visitedSquares[(int)xzCoords.xCoordinate + 1][(int)xzCoords.zCoordinate] = true;
    intersectionWalls.up = true;
    // cout<<"DEBUG: WALL 0"<<endl;
  }
  if (_DOWN)
  {
    visitedSquares[(int)xzCoords.xCoordinate - 1][(int)xzCoords.zCoordinate] = true;
    intersectionWalls.down = true;
    // cout<<"DEBUG: WALL 1"<<endl;
  }
  if (_LEFT)
  {
    visitedSquares[(int)xzCoords.xCoordinate][(int)xzCoords.zCoordinate - 1] = true;
    intersectionWalls.left = true;
    // cout<<"DEBUG: WALL 2"<<endl;
  }
  if (_RIGHT)
  {
    visitedSquares[(int)xzCoords.xCoordinate][(int)xzCoords.zCoordinate + 1] = true;
    intersectionWalls.right = true;
    // cout<<"DEBUG: WALL 3"<<endl;
  }

  //reset local variables for the next coordinate pair
  _UP = false;
  _RIGHT = false;
  _DOWN = false;
  _LEFT = false;
}

bool intersectionCheck(double x, double z, movementDirection botHeading) //CHECKS FOR INTERSECTIONS
{
  // to fix if it is an intersection ignore if loop depending which direction you're coming from
  //PROBLEM HERE MAKE IT CHECK SENSORS TO DETERMINE INTERSECTION AND NOT X,
  int pathsFound = 0;
  cout << "checking if it is a intersection!: " <<endl;
  if (visitedSquares[(int)x + 1][(int)z] == false && botHeading != SOUTH)
  {
    pathsFound++;
  }
  if (visitedSquares[(int)x - 1][(int)z] == false && botHeading != NORTH)
  {
    pathsFound++;
  }
  if (visitedSquares[(int)x][(int)z + 1] == false && botHeading != WEST)
  {
    pathsFound++;
  }
  if (visitedSquares[(int)x][(int)z - 1] == false && botHeading != EAST)
  {
    pathsFound++;
  }

  if (pathsFound >= 2) //WHEN MORE THAN OR EQUAL TO 3 PATHS
  {
    cout <<"yes"<<endl;
    return true;
  }
  else
  {
    cout << "no "<<endl;
    cout << "X : " << x << "            Z : " << z << endl;
    pathsFound = 0;
    return false;
  }
}

bool intersectionCheckAvailablePaths(double x, double z) //CHECKS FOR AVAILABLE PATHS
{
  int pathsFound = 4;
  cout << "===================================" << endl;
  cout << "X : " << x << "            Z : " << z << endl;
  cout << "===================================" << endl;

  if (visitedSquares[(int)x + 1][(int)z] == true)
  {
    cout << "JOKES ON YOU, PATH ABOVE YOU HAS BEEN CHECKED" << endl;
    pathsFound--;
  }
  if (visitedSquares[(int)x - 1][(int)z] == true)
  {
    cout << "JOKES ON YOU, PATH BELOW YOU HAS BEEN CHECKED" << endl;
    pathsFound--;
  }
  if (visitedSquares[(int)x][(int)z + 1] == true)
  {
    cout << "JOKES ON YOU, PATH TO YOUR RIGHT HAS BEEN CHECKED" << endl;
    pathsFound--;
  }
  if (visitedSquares[(int)x][(int)z - 1] == true)
  {
    cout << "JOKES ON YOU, PATH TO YOUR LEFT HAS BEEN CHECKED" << endl;
    pathsFound--;
  }

  if (pathsFound > 0) //WHEN MORE THAN 0 paths
  {
    cout << "PATHS ARE STILL AVAILABLE TO BE CHARTED" << endl;
    return true; //still paths available to be charted
  }
  else
  {
    cout << "GO FUCK YOURSELF" << endl;
    pathsFound = 4;
    return false;
  }
}

void getStartingCoordinates(double x, double z, bool state, movementDirection direction) //USED TO SET STARTING COORDINATES OF BOT
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

void onlyPositives(movementDirection direction) //NOT OUT OF BOUNDS
{
  // checks if it is out of bounds or not (0.0625 sets it to the middle of a square and *8 scales it up)
  double halfSquare = 0.06665; //size of half a square
  double upScaler = 7.5;       //upscale value
  double scaledTransValueX = ((trans_values[0] - halfSquare) * upScaler);
  double scaledTransValueZ = ((trans_values[2] - halfSquare) * upScaler);

  if (setStartCoordsOnce)
  {
    double tempx = round(((trans_values[0] - halfSquare) * upScaler));
    double tempz = round(((trans_values[2] - halfSquare) * upScaler));
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

    setRotationXYZ();
    rotation_field->setSFRotation(directionValue);

    botMovement(true);
  }
  else
  {
    botMovement(true);
  }
}

movementDirection reverseDirection(movementDirection topStackDirection) //TAKES INCOMING DIRECTION AND SETS IT TO OPPOSITE. USED IN BACKTRACKING METHODS
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

void testTurnAfterBacktracking(movementDirection direction) //TAKES THE DIRECTION OF THE PREVIOUS COORD
{
  botMovement(false); //stops bot movement
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

  //double tempx = round(((trans_values[0] - 0.0625) * 8));   //NOT USED
  //double tempz = round(((trans_values[2] - 0.0625) * 8));
  cout << "pop? : " <<endl;
  coordStack.pop();
  setRotationXYZ();
  rotation_field->setSFRotation(directionValue);
  isBackTracking = false;
  botMovement(true);
}

void testTurn(movementDirection direction) //TURNS TO SPECIFIED DIRECTION
{
  int frontSensorData = ds[0]->getValue();
  int rightSensorData = ds[1]->getValue();
  int leftSensorData = ds[2]->getValue();
  movementDirection tempDirection;

  if (frontSensorData < 550) //if front sensor triggered
  {
    botMovement(false);        //stop robot
    if (rightSensorData > 800) // if right path is open
    {
      switch (direction) //change direction
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
      setRotationXYZ();                              //fix rotation values
      rotation_field->setSFRotation(directionValue); //rotate bot      
    }
    else if (leftSensorData > 800) //if left path is open
    {
      switch (direction) //change dir
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
    }
    else //if dead end
    {

      setRotationXYZ();
      directionValue[3] = directionValue[3] + 3.1415;
      rotation_field->setSFRotation(directionValue);
      cout << " trying to 180: "<<endl;
      // isBackTracking = true;
      // backTracking();
    }
  }
  else //if path is clear, keep moving forward
  {
    botMovement(true);
  }
}

movementDirection setTrueDirection() //COMPASS METHOD
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

void exitSeeker() // SEARCH FOR EXIT
{
  cout<<"DEBUG: now in exitSeeker"<<endl;
  struct exitSignal *message = (struct exitSignal*)receiver->getData();
  char *messageReceived = (char*)message->messageContent;
  cout<<"Message received: "<<messageReceived<<endl;
  cout<<"Message tag: "<<message->tag<<endl;
  // if exit is found, robot must stop
  if(message->tag == exitTag){

    cout<<"DEBUG: stoping robot"<<endl;
    botMovement(false);
    cout<<"DEBUG: Robot stopped"<<endl;
    
    cout<<"Reached exit"<<endl;
    sendCoordinates(currentCoord,emitter);
  }
  receiver->nextPacket();  
}

//function that calls all other functions that needs to update per cycle
void updateValues()
{

  testTurn(setTrueDirection());
  onlyPositives(setTrueDirection());

  //sets struct to values coming from onlyPositives(botAngle);
  currentCoord = {xValue, zValue};

  if (!visitedSquares[(int)xValue][(int)zValue])
  {
    visitedSquares[(int)xValue][(int)zValue] = true;
    //cout << "Have i visited this square before at " << xValue << " & " << zValue << " = " << visitedSquares[(int)xValue][(int)zValue] << endl;
  }

  // only run the functions inside if the last location is different than the current location else ignore this and keeps moving
  if (((previousCoord.xCoordinate != currentCoord.xCoordinate) || (previousCoord.zCoordinate != currentCoord.zCoordinate)))
  {
    //saves the currentCoord in another struct to be compared later
    previousCoord = currentCoord;
    wallDetection(currentCoord, setTrueDirection());

    if (intersectionCheck(xValue, zValue, setTrueDirection())) //here needs to check if its actually an intersection
    {
      cout << " intersection found or whatever" << endl;
      CoordinateWalls xzDir = {currentCoord, setTrueDirection(), true};
      // coordStack.push(xzDir);

      // send info new intersection to server
      // sendCoordinates(currentCoord,emitter);
      // sendIntersectionInfo(xzDir);
      // send values of the walls at the intersection
      // sendWalls(intersectionWalls);

      struct sendPackage packageInfo = {xzDir,intersectionWalls};
      sendPackageFuncton(packageInfo);

      cout << "  x: " << xzDir.ptnPair.xCoordinate <<  "  z: " << xzDir.ptnPair.zCoordinate << "  Dir:   " << xzDir.direction <<endl;

      // if (!coordStack.empty())
      // {
      //   cout << "  x: " << xzDir.ptnPair.xCoordinate <<  "  z: " << xzDir.ptnPair.zCoordinate << "  Dir:   " << xzDir.direction <<endl;
      // }      
    }
    // resets the values of the struct of intersection Walls
    intersectionWalls = {false};
  }  
    // send coordinates to server
    // sendCoordinates(currentCoord,emitter);
    // if(receiver->getQueueLength()>0){
    //   cout<<"DEBUG: got to exitSeeker()"<<endl;
    //   exitSeeker();
    // }
  
}

void setup() //RUN SETUP ONCE FOR INITIALIZATION
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

  robot_node = supervisor->getFromDef("botJr");
  trans_field = robot_node->getField("translation");
  rotation_field = robot_node->getField("rotation");
  compass = supervisor->getCompass("compass");
  compass->enable(TIME_STEP);
  
  receiver=supervisor->getReceiver("receiver");
  receiver->enable(TIME_STEP);
  receiver->setChannel(2); 
  emitter=supervisor->getEmitter("emitter");
  emitter->setChannel(1);
  
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

  
  receiver->disable();
  delete receiver;
  delete emitter;

  delete supervisor;
  return 0;
}