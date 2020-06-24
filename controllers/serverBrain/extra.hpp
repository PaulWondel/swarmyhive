#include <webots/Compass.hpp>

struct Coordinates // current coordinate of the bot
{
  double xCoordinate;
  double zCoordinate;
};

enum movementDirection //Struct used for declaring directions
{
  NORTH, // 0
  EAST,  // 1
  SOUTH, // 2
  WEST   // 3
};

struct Intersection // what walls are available at said coordinate true=wall
{
  Coordinates ptnPair;
  movementDirection direction;
  bool intersectionExists;
};

struct Walls // what walls are available at said coordinate true=wall
{
  bool up;
  bool right;
  bool down;
  bool left;
};

struct receivePackage // save info in struct to send in a package
{
  Intersection botInfo;
  Walls wallInfo;
  const int botNr;
};

// Function to place coordinates into a struct
Coordinates structTransport(double xCoord, double zCoord){
  Coordinates transport = {(double)xCoord,(double)zCoord};
  return transport;
}