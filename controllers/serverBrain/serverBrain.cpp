//last edit before making it into master

#include <webots/Supervisor.hpp>
#include <webots/Receiver.hpp>
#include "extra.hpp"
#include <vector>
#include <webots/Compass.hpp>

#define TIME_STEP 256

using namespace webots;
using namespace std;

// Dynamic array to save structs into
bool visitedSquares[15][15] = {{false}};
double tempX;
double tempZ;

/*####### NODES ########*/
Receiver *receiver;
Supervisor *supervisor;

void wallSetter(double x, double z, Walls intersectionWall) // CHECKS FOR INTERSECTIONS
{
  // to fix if it is an intersection ignore if loop depending which direction you're coming from
  // PROBLEM HERE MAKE IT CHECK SENSORS TO DETERMINE INTERSECTION AND NOT X,
  cout << "SERVER DEBUG: checking if it is a intersection!: " << endl;
  if (intersectionWall.up)
  {
    // it goes into this loop if path NORTH of it is blocked
    visitedSquares[(int)x + 1][(int)z] = true;
    cout << "SERVER DEBUG: NORTH wall set to true" << endl;
  }
  if (intersectionWall.down)
  {
    // it goes into this loop if path SOUTH of it is blocked
    visitedSquares[(int)x - 1][(int)z] = true;
    cout << "SERVER DEBUG: SOUTH wall set to true" << endl;
  }
  if (intersectionWall.right)
  {
    // it goes into this loop if path EAST of it is blocked
    visitedSquares[(int)x][(int)z + 1] = true;
    cout << "SERVER DEBUG: EAST wall set to true" << endl;
  }
  if (intersectionWall.left)
  {
    // it goes into this loop if path WEST of it is blocked
    visitedSquares[(int)x][(int)z - 1] = true;
    cout << "SERVER DEBUG: WEST wall set to true" << endl;
  }
}

// checking if a square has already been visited and tells which way the bot should be going to. It returns a direction
movementDirection squareChecker(double x, double z, movementDirection botDirection) {
  if (botDirection == NORTH)
  {
    if (visitedSquares[(int)x + 1][(int)z] == false)
    {
      // go NORTH
      return NORTH;
    }
    else if (visitedSquares[(int)x][(int)z + 1] == false)
    {
      // go EAST
      return EAST;
    }
    else if (visitedSquares[(int)x][(int)z - 1] == false)
    {
      // go WEST
      return WEST;
    }
  }
  if (botDirection == SOUTH)
  {
    if (visitedSquares[(int)x - 1][(int)z] == false)
    {
      // go SOUTH
      return  SOUTH;
    }
    else if (visitedSquares[(int)x][(int)z - 1] == false)
    {
      // go WEST
      return WEST;
    }
    else if (visitedSquares[(int)x][(int)z + 1] == false)
    {
      // go EAST
      return EAST;
    }    
  }
  if (botDirection == EAST)
  {
    if (visitedSquares[(int)x][(int)z + 1] == false)
    {
      // go EAST
      return EAST;
    }
    else if (visitedSquares[(int)x - 1][(int)z] == false)
    {
      // go SOUTH
      return SOUTH;
    }
    else if (visitedSquares[(int)x + 1][(int)z] == false)
    {
      // go NORTH
      return NORTH;
    }    
  }
  if (botDirection == WEST)
  {
    if (visitedSquares[(int)x][(int)z - 1] == false)
    {
      // go WEST
      return WEST;
    }
    else if (visitedSquares[(int)x + 1][(int)z] == false)
    {
      // go NORTH
      return NORTH;
    }
    else if (visitedSquares[(int)x - 1][(int)z] == false)
    {
      // go SOUTH
      return SOUTH;
    }     
  }
  cout<<"SERVER DEBUG: return NORTH eventhough direction selected"<<endl;
  return NORTH;
}

void setup()
{
  supervisor = new Supervisor();
  receiver = supervisor->getReceiver("receiver");
  receiver->enable(TIME_STEP);
  receiver->setChannel(-1);
}

void readDirection(movementDirection x)
{
  switch (x){
    case 0:
      cout<<"SERVER DEBUG: GO direction NORTH"<<endl;
      break;
    case 1:
      cout<<"SERVER DEBUG: GO direction EAST"<<endl;
      break;
    case 2:
      cout<<"SERVER DEBUG: GO direction SOUTH"<<endl;
      break;
    case 3:
      cout<<"SERVER DEBUG: GO direction WEST"<<endl;
      break;
    default:
      cout<<"No direction"<<endl;
      break;
  }
}

int main(int argc, char **argv)
{
  setup();

  while (supervisor->step(TIME_STEP) != -1)
  {
    if (receiver->getQueueLength() > 0)
    {
      struct receivePackage *incomingPackage = (struct receivePackage *)receiver->getData();
      struct receivePackage savedPackage = {incomingPackage->botInfo, incomingPackage->wallInfo};
      receiver->nextPacket();
      wallSetter(savedPackage.botInfo.ptnPair.xCoordinate, savedPackage.botInfo.ptnPair.zCoordinate, savedPackage.wallInfo);
      // if square already visited send info back to bot
      readDirection(
        squareChecker(savedPackage.botInfo.ptnPair.xCoordinate, savedPackage.botInfo.ptnPair.zCoordinate, savedPackage.botInfo.direction)
      );
    }
  }
  receiver->disable();
  delete receiver;
  delete supervisor;
  return 0;
}