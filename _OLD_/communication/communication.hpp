//Struct with the current coordinates
struct Coordinates {
  double xCoordinate;
  double zCoordinate;
};

// Struct that gives the information of walls if present at certain coordinates
struct CoordinateWalls {
  Coordinates ptnPair;
  bool up;
  bool right;
  bool down;
  bool left;
  double direction;
};

struct exitSignal {
  int tag;
  char messageContent[];
};