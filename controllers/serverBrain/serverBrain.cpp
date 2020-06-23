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
// vector<Coordinates> unvisitedList;
bool visitedSquares[15][15] = {{false}};


void wallSetter(double x, double z, Walls intersectionWall) //CHECKS FOR INTERSECTIONS
{
  // to fix if it is an intersection ignore if loop depending which direction you're coming from
  // PROBLEM HERE MAKE IT CHECK SENSORS TO DETERMINE INTERSECTION AND NOT X,
  // int pathsFound = 0;
  cout << "SERVER DEBUG: checking if it is a intersection!: " <<endl;
  // if (visitedSquares[(int)x + 1][(int)z] == false && botHeading != SOUTH)
  if(intersectionWall.up)
  {
    // it goes into this loop if path NORTH of it is unvisited
    // pathsFound++;
    visitedSquares[(int)x + 1][(int)z] = true;
    cout<<"SERVER DEBUG: NORTH wall set to true"<<endl;
  }
  // if (visitedSquares[(int)x - 1][(int)z] == false && botHeading != NORTH)
  if(intersectionWall.down)
  {
    // it goes into this loop if path SOUTH of it is unvisited
    // pathsFound++;
    visitedSquares[(int)x - 1][(int)z] = true;
    cout<<"SERVER DEBUG: SOUTH wall set to true"<<endl;
  }
  // if (visitedSquares[(int)x][(int)z + 1] == false && botHeading != WEST)
  if(intersectionWall.right)
  {
    // it goes into this loop if path EAST of it is unvisited
    // pathsFound++;
    visitedSquares[(int)x][(int)z + 1] = true;
    cout<<"SERVER DEBUG: EAST wall set to true"<<endl;
  }
  // if (visitedSquares[(int)x][(int)z - 1] == false && botHeading != EAST)
  if(intersectionWall.left)
  {
    // it goes into this loop if path WEST of it is unvisited
    // pathsFound++;
    visitedSquares[(int)x][(int)z - 1] = true;
    cout<<"SERVER DEBUG: WEST wall set to true"<<endl;
  }

  // if (pathsFound >= 2) //WHEN MORE THAN OR EQUAL TO 3 PATHS
  // {
  //   cout <<"yes"<<endl;
  //   return true;
  // }
  // else
  // {
  //   cout << "no "<<endl;
  //   cout << "X : " << x << "            Z : " << z << endl;
  //   pathsFound = 0;
  //   return false;
  // }
}

void saveIntersectionInfo(){
  
}

int main(int argc, char **argv) {
  Supervisor *supervisor = new Supervisor();
  
  // Initialize receiver
  Receiver *receiver;
  receiver=supervisor->getReceiver("receiver");
  receiver->enable(TIME_STEP);
  receiver->setChannel(-1); 
  
  while (supervisor->step(TIME_STEP) != -1) {
    // if(receiver->getQueueLength()>0){

    //   // Receive the Coordinate struct and keep the coordinates
    //   struct Coordinates *incomming = (struct Coordinates*)receiver->getData();
    //   double xcoord = incomming->xCoordinate;
    //   double zcoord = incomming->zCoordinate;
    //   cout << "Received X: "<<xcoord<<endl;
    //   cout << "Received Z: "<<zcoord<<endl;
    //   receiver->nextPacket();

    //   // Put the coordinates into a struct and 
    //   // push struct into an dynamic array (vector)
    //   unvisitedList.push_back(structTransport(xcoord,zcoord));

    //   // coordinateList.size() gives unsigned long int
    //   for (int i = (int)unvisitedList.size(); i == (int)unvisitedList.size();){
    //     cout<<i-1<<" Debug coordinate X: "<<unvisitedList[i-1].xCoordinate<<endl;
    //     cout<<i-1<<" Debug coordinate Z: "<<unvisitedList[i-1].zCoordinate<<endl;
    //     break;        
    //   }
    // }

    // cout<<"DEBUG: QUEUE LENGTH "<<receiver->getQueueLength()<<endl;
    if(receiver->getQueueLength()>0){
      // struct Intersection *incoming = (struct Intersection*)receiver->getData();
      // struct Intersection savedCoord = {incoming->ptnPair.xCoordinate, incoming->ptnPair.zCoordinate, incoming->direction,incoming->intersectionExists};
      // cout<< "Received X: "<<savedCoord.ptnPair.xCoordinate<<endl;
      // cout<< "Received Z: "<<savedCoord.ptnPair.zCoordinate<<endl;
      // cout<< "Received Direction: "<<savedCoord.direction<<endl;

      // double xcoord = incoming->ptnPair.xCoordinate;
      // double zcoord = incoming->ptnPair.zCoordinate;
      // movementDirection currentDirection = incoming->direction; // the direction that the bot is facing
      // mark square that robot direction is in
      // mark square where the bot came from   
      // cout<< "Received X: "<<incoming->ptnPair.xCoordinate<<endl;
      // cout<< "Received Z: "<<incoming->ptnPair.zCoordinate<<endl;
      // cout<< "Received Direction: "<<incoming->direction<<endl;
      
      // intersectionCheck(xcoord, zcoord, currentDirection);  
      // save coordinates into array or vector
      // receiver->nextPacket();      

      // we receive the walls from coordinates of xcoord and zcoord
      // struct Walls *incomingWalls = (struct Walls*)receiver->getData();
      // struct Walls savedWalls = {incomingWalls->up,incomingWalls->right,incomingWalls->down,incomingWalls->left};

      // cout<< "Received NORTH: "<<savedWalls.up<<endl;
      // cout<< "Received EAST: "<<savedWalls.right<<endl;
      // cout<< "Received SOUTH: "<<savedWalls.down<<endl;
      // cout<< "Received WEST: "<<savedWalls.left<<endl;

      // receiver->nextPacket();
      // wallSetter(savedCoord.ptnPair.xCoordinate,savedCoord.ptnPair.zCoordinate,savedWalls);

      struct receivePackage *incomingPackage = (struct receivePackage*)receiver->getData();
      struct receivePackage savedPackage = {incomingPackage->botInfo,incomingPackage->wallInfo};
      // cout<< "Received X: "<<savedPackage.botInfo.ptnPair.xCoordinate<<endl;
      receiver->nextPacket();
      wallSetter(savedPackage.botInfo.ptnPair.xCoordinate,savedPackage.botInfo.ptnPair.zCoordinate,savedPackage.wallInfo);
    }
  }
  receiver->disable();
  delete receiver;
  delete supervisor;
  return 0;
}