struct Coordinates {
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

CoordinateWalls createStruct(Coordinates structure, bool wall_up, bool wall_right, bool wall_down, bool wall_left){
  CoordinateWalls currentWalls = {xzCoords, _UP, _RIGHT, _DOWN, _LEFT, _botDirection};
  coordStack.push(currentWalls);
}