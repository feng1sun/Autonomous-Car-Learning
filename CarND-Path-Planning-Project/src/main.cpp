#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <thread>
#include <vector>
#include "utils/Eigen-3.3/Eigen/Core"
#include "utils/Eigen-3.3/Eigen/QR"
#include "utils/json.hpp"
#include "utils/utils.h"
#include "map/map.h"
#include "vehicle/vehicle.h"

using namespace std;

// for convenience
using json = nlohmann::json;

int main() {
  uWS::Hub h;
  ofstream log;
  log.open("../src/log.txt");
  log.clear();

  // Load up map values for waypoint's x,y,s and d normalized normal vectors

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map(map_file_.c_str(), ifstream::in);
  MAP Map (in_map);

  Vehicle egoCar(Map, log);
  h.onMessage(
    [&egoCar, &log](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // auto sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {

        auto j = json::parse(s);

        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	egoCar.ego.s = j[1]["s"];
          	egoCar.ego.y = j[1]["y"];
          	egoCar.ego.d = j[1]["d"];
          	egoCar.ego.x = j[1]["x"];
          	egoCar.ego.yaw = deg2rad(j[1]["yaw"]);
          	egoCar.ego.speed = mph2mps(j[1]["speed"]);

          	// Previous path data given to the Planner
          	vector<double> previous_path_x = j[1]["previous_path_x"];
          	vector<double> previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

            //TODO:
            log << "============ NEW ============" << endl;

            egoCar.ego.previous_path_x = previous_path_x;
            egoCar.ego.previous_path_y = previous_path_y;
            egoCar.ego.end_s = end_path_s;
            egoCar.ego.end_d = end_path_d;
            egoCar.ego_sensor_fusion = sensor_fusion;
            
            egoCar.pipeline();

          	vector<double> next_x_vals = egoCar.ego_next_x_vals;
          	vector<double> next_y_vals = egoCar.ego_next_y_vals;

          	json msgJson;
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

            log << endl;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } 
      else {
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
