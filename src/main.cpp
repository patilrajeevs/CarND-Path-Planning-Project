#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <list>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "behaviour.cpp"
using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
  //TODO  changed in aaron's file. Do I need ot change this ?
	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

Car getNearestCarBehind(double main_car_s, const std::vector<Car> &cars_lane){
  Car nearest;
  for (std::vector<Car>::const_iterator it = cars_lane.begin() ; it != cars_lane.end(); ++it){
    if ( (*it).s > main_car_s){
      nearest = *prev(it);
    }
  }
  return nearest;
}

Car getNearestCarAhead(double main_car_s, const std::vector<Car> &cars_lane){
  Car nearest;
  for (std::vector<Car>::const_iterator it = cars_lane.begin() ; it != cars_lane.end(); ++it){
    if ( (*it).s > main_car_s){
      nearest = *it;
    }
  }
  return nearest;
}


bool changePossible(int target_lane, const Car &main_car, const std::vector<Car> &cars_lane){
  bool retval = false;
  Car nearest_ahead = getNearestCarAhead( main_car.s, cars_lane);
  Car nearest_behind = getNearestCarBehind( main_car.s, cars_lane);
  
  if (main_car.s - nearest_behind.s > 30){
      retval = true;
  }else{
    cout << "Car behind is closer than 30m cannot change lane";
  }
  if (nearest_ahead.s - main_car.s > 50 ){
    retval &= true;
  }else{
    cout << "Car ahead is closer than 50m cannot change lane";
  }
  return retval;
}

int changeLane(Car &main_car, const std::vector<Car> &cars_lane0,\
               const std::vector<Car> &cars_lane1, const std::vector<Car> &cars_lane2){
  int current_lane = main_car.get_lane();
  int target_lane = -1;
  if(current_lane == 1){
    if (changePossible(2, main_car, cars_lane2)){
      target_lane = 2;  
    }else if (changePossible(0, main_car, cars_lane2)){
      target_lane = 0;
    }
  }else if (current_lane == 0 || current_lane == 1){
    if (changePossible(1, main_car, cars_lane2)){
      target_lane = 1;
    }
  }
  cout << "DEBUG : Change lane to " << target_lane << "\n" ;
  return target_lane; 
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  //Create an object of the behaviour planner.
  //seet initial values for a few parameters
  BehaviourPlanner bp;
  bp.main_car.speed = 0;
  bp.main_car.lane = 1;
	bp.target_speed = 0;
  h.onMessage([&bp,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          // Pass on the main car's data to the behavior planner
          bp.main_car.x = j[1]["x"];
         	bp.main_car.y = j[1]["y"];
          bp.main_car.s = j[1]["s"];
         	bp.main_car.d = j[1]["d"];
          bp.main_car.yaw = j[1]["yaw"];
          // telemetry data is slow probably, do not feed that into the main car's data
          //main_car.speed = j[1]["speed"];

          // Previous path data given to the Planner
         	bp.setPreviousPathX(j[1]["previous_path_x"]);
         	bp.setPreviousPathY(j[1]["previous_path_y"]);
         	// Previous path's end s and d values 
         	bp.end_path_s = j[1]["end_path_s"];
         	bp.end_path_d = j[1]["end_path_d"];

         	// Sensor Fusion Data, a list of all other cars on the same side of the road.
         	auto cars = j[1]["sensor_fusion"];
          //START
          //sensor fusion part starts
          if (bp.prev_size > 0){
            bp.main_car.s = bp.end_path_s;
          }
            
          // clear car data from previous iteration from the behaviour planner
          bp.clear();
          //iterate over the sensor fusion data
          for( int i = 0; i < cars.size(); i++){
            auto data = cars[i];
            Car tracking_car = Car();
				    
            // populate data for the tracking car in an object
            // find a way to put below inside a constructor for car class
            tracking_car.id = data[0];
						tracking_car.x  = data[1];
						tracking_car.y  = data[2];
						tracking_car.vx = data[3];
						tracking_car.vy = data[4];
						tracking_car.s = data[5];
						tracking_car.d = data[6];
            int lane = tracking_car.get_lane();
            if (lane == 0 || lane ==1 || lane == 2){
              // push the car info to behaviour planning object
              bp.cars_lane[lane].push_back(Car(tracking_car));
            }else{
              // some cars were found to have an invalid "d". error in sensor fusion?
              cout << "incorrect lane for " << tracking_car;
            }
          }
          cout << "*******************ITERATION STARTED ******************************\n";
          //debug info
          //cout << "#################cars before sortig below : #####################\n";
          //bp.displayCarsInLanes();
          
          // Sort the cars in each of the lanes based on their respective "s"
          bp.sort_cars_lanes(); 
          cout << "#################cars after sortig below : ######################\n";
          bp.displayCarsInLanes();
          cout << "#################cars display over: #####################\n";
          // update behavioru using the behaviour planner class.
          // This sets the target speed and the lane after inspecting all conditions
          bp.updateBehaviour(); 
          cout << "*******************ITERATION ENDED ******************************\n";
          //sensor fusion part ends
          
          // Implementation below is as shown in the project walk through, and no changes in the same.

          vector <double> ptsx;
          vector <double> ptsy;

          double ref_x = bp.main_car.x;
          double ref_y = bp.main_car.y;
          double ref_yaw = deg2rad(bp.main_car.yaw);

          if (bp.prev_size < 2){
            double prev_car_x = bp.main_car.x - cos(bp.main_car.yaw);
            double prev_car_y = bp.main_car.y - sin(bp.main_car.yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(bp.main_car.x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(bp.main_car.y);

          }else{
            ref_x = bp.previous_path_x[bp.prev_size-1];  
            ref_y = bp.previous_path_y[bp.prev_size-1];

            double ref_x_prev = bp.previous_path_x[bp.prev_size - 2];
            double ref_y_prev = bp.previous_path_y[bp.prev_size - 2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          vector<double> next_wp0 = getXY(bp.main_car.s+30, (2+4*bp.main_car.lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(bp.main_car.s+60, (2+4*bp.main_car.lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(bp.main_car.s+90, (2+4*bp.main_car.lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i = 0; i < ptsx.size(); i++){
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] =( shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
            ptsy[i] =( shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
          }

          tk::spline s;
          s.set_points(ptsx,ptsy);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for(int i=0; i<bp.prev_size; i++){
            next_x_vals.push_back(bp.previous_path_x[i]);
            next_y_vals.push_back(bp.previous_path_y[i]);
          }

          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;
          double vel_change = 0.224;
          for(int i=0; i<= 50 - bp.prev_size;i++){
            if (bp.main_car.speed < bp.target_speed ) {  
              // Accelerate if under target speed
              //cout << "DEBUG : Increasing velocity by " << vel_change << "\n";
              bp.main_car.speed += vel_change;
            }else if (bp.main_car.speed > bp.target_speed) { 
              // Brake if below target
              //cout << "DEBUG : Dropping velocity by " << vel_change << "\n";
              bp.main_car.speed -= vel_change;
            }
              
            double N = (target_dist/(.02*bp.main_car.speed/2.24));
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;

            x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          } 
          //END
          json msgJson;
          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
