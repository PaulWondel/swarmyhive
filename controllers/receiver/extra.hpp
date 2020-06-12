struct Coordinates{
  //current coordinate of the bot
  double xCoordinate;
  double zCoordinate;
};

// Function to place coordinates into a struct
Coordinates structTransport(double xCoord, double zCoord){
  Coordinates transport = {(double)xCoord,(double)zCoord};
  return transport;
}