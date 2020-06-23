#include <iostream>
#include <vector>
using namespace std;

bool run_state = true;

struct Coordinates //Struct used for saving coordinates 
{
  double xCoordinate;
  double zCoordinate;
};

struct Checkcoord
{
  double x;
  double z;
};

vector<Coordinates> unvisitedList;
struct Coordinates looper;
int vectorSize = 20;


int main() {
  double x = 50;
  double z = 0;
  struct Checkcoord visited = {37,13};
  

  for (int i = 0; i < vectorSize; i++){
    looper = {x,z};
    unvisitedList.push_back(looper);
    x--;
    z++;
  }
  
  while(run_state == true){
    for (int i = 0; i < (int)unvisitedList.size(); i++){
      // cout<<i<<" Debug coordinate X: "<<unvisitedList[i].xCoordinate<<endl;
      // cout<<i<<" Debug coordinate Z: "<<unvisitedList[i].zCoordinate<<endl;

      double checkX = unvisitedList[i].xCoordinate;
      double checkZ = unvisitedList[i].zCoordinate;

      // if(checkX == 37 && checkZ == 13){
      //   cout<<"["<<checkX<<","<<checkZ<<"]"<<" has been visited"<<endl;
      // }
      // else{
      //   cout<<"["<<checkX<<","<<checkZ<<"]"<<" has not been visited"<<endl;
      // }

      if(visited.x == checkX && visited.z == checkZ){
        cout<<"["<<checkX<<","<<checkZ<<"]"<<" has been visited"<<endl;
      }
      else{
        cout<<"["<<checkX<<","<<checkZ<<"]"<<" has not been visited"<<endl;
      }
    }
    //run
    run_state = false;
  }



}