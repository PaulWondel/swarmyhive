struct Coordinates{
  //current coordinate of the bot
  double xCoordinate;
  double zCoordinate;
};

struct CoordinateWalls {
  //what walls are available at said coordinate true=wall
  Coordinates ptnPair;
  bool up;
  bool right;
  bool down;
  bool left;
  double direction;
};

// Function to place coordinates into a struct
Coordinates structTransport(double xCoord, double zCoord){
  Coordinates transport = {(double)xCoord,(double)zCoord};
  return transport;
}