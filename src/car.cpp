
#include "json.hpp"
#include <iostream>  

using namespace std;
using json = nlohmann::json;

//[car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

/*
 * This class handles the operations related to car
 */
class Car{
public:
  int id = -1;
  double x;
  double y;
  double vx = -1;
  double vy = -1;
  double s = 99999999999;
  double d;
  double yaw = 0;
  double speed = 0;
  int lane = -1;

  //friend function prints the details of car
  friend ostream& operator<<(ostream& os, const Car& car);

  /*
   * A constructor which accepts a car's json as input and inits objects accordingly
   * But, this needs more work to be enabled, skipping this for now
  void car(const json& data){
    id = data[0];
    x  = data[1];
    y  = data[2];
    vx = data[3];
    vy = data[4];
    s = data[5];
    d = data[6];
  }
  */

  /*
   * Calculates the speed of a given car, from the vx and vy component
   */
  double speed_calc(){
    if ((vx == -1) && (vy == -1)){
      //cout << "vx and vx is 0, cannot calculate speed\n";
      //invalid vx and vy component in current class' context, do not do anything
      ;
    }else{
      return sqrt(vx*vx + vy*vy);
    }
  }

  /*
   * From the "d" of car given in frenet, calculate the lane
   */
  int get_lane(){
    if (( d > 0 ) &&( d <= 4 ) ){
      return 0;
    }else if (( d > 4 ) && ( d <= 8 )){
      return 1;
    }else if (( d > 8 ) && (d <= 12)){
      return 2;
    }else{
      // invalid lane in current context
      //cout << "Lane for car_id " <<id << " not found, its d is " <<d << "\n";
      return -1;
    }
  }

  /*
   * This opetor tells if the car is smaller than, meaning is behind the "this" car
   * Is used in sorting cars
   */
  bool operator < (const Car& car) const
  {
    return (s < car.s);
  }

  /*
   * Check if lane1 and lane2 are adjacent to each other
   */
  bool isAdjacentLane(int lane1,int lane2){
    if (abs(lane1 - lane2) == 1 ){
      return true;
    }else{
      return false;
    }
  }
  
  /*
   * Check if the car is in adjacent lane of "this" car
   */
  bool inAdjacentLaneOf(Car& car){
    return isAdjacentLane(get_lane(), car.get_lane());
  }
};


/*
 * Help print out details of car easily using cout
 */
ostream& operator<<(ostream& os, Car& car)  
{  
  os << "id = "<< car.id << " s = " << car.s << " d = " <<car.d << " Lane = "<< car.get_lane() <<"\n";  
  return os;  
}
