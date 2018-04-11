#include "car.cpp"
#include <vector>
#include <list>
#include "json.hpp"

using namespace std;

//Counter for number of lanes to be considered
#define NO_LANES 3
//Maximum desired speed for the car
#define DESIRED_SPEED 49.5

/*
 * This class defines the behaviour of "main car" when compared to car in current and other lanes
 * Decides the speed of the car, if to reduce or increase, and if or not it is safe to switch lanes
 */
class BehaviourPlanner{

public:
  //array of vector of cars in each lane
  vector<Car> cars_lane[NO_LANES];
  // main car
  Car main_car;
  // target speed of the main_car. This is desired to be always at DESIRED_SPEED.
  // The behaviour planner decides this speed to be set to the car
	double target_speed;

  // information from the simulator about the previous path
	json previous_path_y;
	json previous_path_x;
	double end_path_s;
	double end_path_d;
	// size of the previous path
  int prev_size;

  /*
   * setter for previous path x
   */
	int setPreviousPathX(json prev_path_x){
		previous_path_x = prev_path_x;
		prev_size = previous_path_x.size();
	}

  /*
   * setter for previous path y
   */
	int setPreviousPathY(json prev_path_y){
		previous_path_y = prev_path_y;
	}

  /*
   * Clears the cars in each of the lanes
   */
  void clear(){
    for (int i = 0; i<NO_LANES; i++){
      cars_lane[i].clear();
    }
  }

  /*
   * Helper function to display cars in a vector of cars
   */
  void displayCars(std::vector<Car> cars){
		for (std::vector<Car>::iterator it = cars.begin() ; it != cars.end(); ++it)
						cout << *it;
  }

  /*
   * Helper function to display cars in all the lanes
   */
	void displayCarsInLanes(){
		cout << "**********status*********************\n";
		for (std::vector<Car>::iterator it = cars_lane[0].begin() ; it != cars_lane[0].end(); ++it)
						cout << *it;
		for (std::vector<Car>::iterator it = cars_lane[1].begin() ; it != cars_lane[1].end(); ++it)
						cout << *it;
		for (std::vector<Car>::iterator it = cars_lane[2].begin() ; it != cars_lane[2].end(); ++it)
						cout << *it;
		cout << "**********status end*****************\n";
	}

  /*
   * Sort the cars in each of the lanes based on the "s"
   * Uses the overloaded "<" operator in the Car class
   */
	void sort_cars_lanes(){
    for (int i = 0; i<NO_LANES; i++){
  	  std::sort(cars_lane[i].begin(), cars_lane[i].end());
    }
	}

  /*
   * This function returns the "Car" nearest-behind to the "main_car" in "lane"
   */
	Car getNearestCarBehind(int lane){
    Car nearest;
		for (std::vector<Car>::const_iterator it = cars_lane[lane].begin() ; it != cars_lane[lane].end(); ++it){
			if ( (*it).s > main_car.s){
        break;
			}
      nearest.lane = lane;
			nearest = *(it);
		}
		return nearest;
	}

  /*
   * This function returns the "Car" nearest-ahead to the "main_car" in "lane"
   */
	Car getNearestCarAhead(int lane){
    Car nearest;
		for (std::vector<Car>::const_iterator it = cars_lane[lane].begin() ; it != cars_lane[lane].end(); ++it){
			if ( (*it).s > main_car.s){
        nearest.lane = lane;
				nearest = *it;
        break;
			}
		}
		return nearest;
	}

  /*
   *This function sets the cars just ahead of the main car in each lane, lanewise
   */
  void get_next_cars(std::vector<Car> &cars){
    for (int i = 0; i < NO_LANES; i++){
     Car nearest_ahead = getNearestCarAhead(i);
       cars.push_back(nearest_ahead);
    }
  }
  
  /*
   *This function sets the cars just behind of the main car in each lane, lanewise
   */
  void get_prev_cars(std::vector<Car> &cars){
    for (int i = 0; i < NO_LANES; i++){
     Car nearest_behind = getNearestCarBehind(i);
     cars.push_back(nearest_behind);
    }
  }

  /*
   * This function returns int values which define how close the main car is to the 
   * next car in its lane.
   * 0 - it is far
   * 1 - it is getting close, we should check to change lane
   * 2 - emergency, reduce speed steeply
   */  
  int howClose(std::vector<Car> next_cars){
    Car next_car = next_cars[main_car.get_lane()];
    if (next_car.id == -1){
      return 0;
    }else{
      double delta_s = next_car.s - main_car.s;
      if (delta_s < 30){
        if (delta_s < 15){
          //emergency, we should brake now
          return 2;
        }else{
          //We are getting close, we should try and change lane
          return 1; 
        }
      }else{
        //We are good, no need to change lane
        return 0;
      }
    }
  }

  /*
   * This functions inspects the cars in all the lanes which are ahead of the main_car
   * Output of the function is a list of lanes, where the car can switch, ordered in priority, 
   * making it easy for iteration
   * The decision is maded based on gain for changing to lane
   */
  void getBestLanesAhead(std::vector<int> &best_lanes, std::vector<Car> next_cars){
    // gains for each of the lanes
    int gain[3] = {0,0,0};
    
    //iterate over lanes
    for (int i = 0; i < NO_LANES; i++){
      cout << "checking lane " << i << "\n";
      // if lane is not adjacent to current lane, we cannot change to it.
      // Drop gain and continue to inspect next lane
      if (!main_car.isAdjacentLane(main_car.get_lane(), i)){
        cout << "Lane is not adjacent" << "\n";
        gain[i] -= 1000000;
        continue;
      }

      // Main car is ahead of the lane we inspecting, no use changing to it.
      // Drop gain and continue to inspect next lane
      if (main_car.s > next_cars[i].s){
        cout << "main car is ahead of the next car, no need to change lane\n";
        gain[i] -= 1000000;
        continue;
      }

      // When no car is found in sensor range, the id is set to -1, we can safely change to that lane
      // increase gain a lot, and continue to inspect next lane
      if (next_cars[i].id == -1){
        cout << "Car id is -1, is probably out of range" << "\n";
        gain[i] += 1000000;
        continue;
      }else{
        //This is a valid car, to inspect further
        if (next_cars[i].s - main_car.s > 30){
          // add gain relative to s
          // Also, add gain relative to the next car's speed.
          // There can be a lot of additional logic here to identify which is a better lane
          cout << "Valid car ahead in adjacent lane, and further than 30s, adding gain relative to s" << "\n";
          gain[i] += 10*(next_cars[i].s - main_car.s);
          gain[i] += next_cars[i].speed_calc();
        }else{
          gain[i] -= 1000000;
          cout << "Reducing gain, car is within 30s\n";
        }
      }
    }
    // Sort and push lanes into the lane priorrity array as per the determined weights
    cout << "Gains == 0:" << gain[0] << " 1:" << gain[1] << " 2:" << gain[2] << "\n";
    for (int i = 0; i < NO_LANES; i++){
      if (gain[i] < 0){
        cout << "Negative gain, not useful, for lane " << i << "\n";
        continue;
      }else{
        if (best_lanes.empty()){
          best_lanes.push_back(i);
          continue;
        }
        bool added=false;
        for (std::vector<int> :: iterator it = best_lanes.begin() ; it != best_lanes.end(); ++it){
          if(gain[*it] < gain[i]){
            best_lanes.insert(it, i);
            added = true;
            break;
          }
        }
        if (!added){
          best_lanes.push_back(i);
        }
      }
    }

    // print out the lanes prioriity wise
    cout << "Lanes =";
    for (std::vector<int> :: iterator it = best_lanes.begin() ; it != best_lanes.end(); ++it){
      cout << (*it) << " ";
    }
    cout << "\n";
  }

  /*
   * In spect the lanes which we recieve to as changable for more checks.
   * This function primarily checks for where the previosu car is and takes a call to change lane accordingly
   */
  bool changeLane(int lane, std::vector<Car> prev_cars, std::vector<Car> next_cars){
    bool retval = false;
    double main_car_speed = main_car.speed_calc();
    double next_car_speed = next_cars[lane].speed_calc();
    double prev_car_speed = prev_cars[lane].speed_calc();
    double next_car_main_lane = next_cars[main_car.get_lane()].speed_calc();
    // a special condition where the previous car in the target lane is invalde ( id = -1 )
    // set desired speed, and okay to change lane
    if (prev_cars[lane].id == -1){
      cout << "Previous car is probably invalid, good to change lane" << "\n";
      if (next_car_main_lane < DESIRED_SPEED){
        target_speed = next_car_main_lane;
      }else{
        target_speed = DESIRED_SPEED;
      }
      main_car.lane = lane;
      return true;
    }

    // Logic to check how close/far/safe we are w.r.t. previous car in "target" lane
    if ((main_car.s - prev_cars[lane].s > 50) || (prev_car_speed - main_car_speed < 10 && main_car.s - prev_cars[lane].s > 30)){
      cout << "Car " << prev_cars[lane] << " is further behind than 30s, okay to change lane, checking further conditions \n";
      if (( main_car_speed > next_car_speed) && (next_cars[lane].s - main_car.s < 50 )){
        cout << "Car " << next_cars[lane] << " is slower than main car, and is too close, no use in changing lane\n";
        retval = false;
      }else{
        cout << "good to change lane to "<< lane << "\n";
        main_car.lane = lane;
        retval = true;
      }
    }else{
      cout << "Car " << prev_cars[lane] << " is within range, could collide, not okay to change lane\n";
      retval = false;
    }

    // also set the target speed for the car
    if ( next_car_main_lane > prev_car_speed ){
      target_speed = next_car_main_lane;
    }else{
      target_speed = prev_car_speed;
    }
    return retval;
  }

  //main function for behavioru planning
	void updateBehaviour(){
    // vectors for the car ahead and behind the main_car in each of the lanes
    std::vector<Car> next_cars;
    std::vector<Car> prev_cars;
    get_next_cars(next_cars);
    get_prev_cars(prev_cars);
    //display debug info
    cout << "Main car : " << main_car;
    cout << "Prev cars : \n";
    displayCars(prev_cars);
    cout << "Next cars : \n";
    displayCars(next_cars);

    int closeness = howClose(next_cars);
    if (closeness == 0){
      cout << "not to close\n";
      target_speed = DESIRED_SPEED;
    }else{
      if (closeness == 1){
        std::vector<int> best_lanes;
        getBestLanesAhead(best_lanes, next_cars);
        for(std::vector<int>::iterator it = best_lanes.begin(); it != best_lanes.end(); ++it){
          cout << "Checking to change lane to : " << *it << "\n";
          if(changeLane( *it ,prev_cars,next_cars)){
            cout << "Changing lane to " << main_car.lane << "\n";
            break;
          }
        }
      }else if(closeness == 2){
        cout << "emergency\n";
        target_speed = next_cars[main_car.get_lane()].speed_calc(); 
      }
    } 
  }
};
