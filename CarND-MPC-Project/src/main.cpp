#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <csignal>
#include <iostream>
#include <fstream>
#include <thread>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "./tools/json.hpp"
#include "./tools/tools.h"
#include "./msg/message.h"
#include "./mpc/mpc.h"

// for convenience
using json = nlohmann::json;

// Create json logger.
json json_log;

// SIGINT handler
void int_handler(int x)
{
    cout <<"In interrupt handler"<<endl; // after pressing CTRL+C you'll see this message
    ofstream writer("./log.json", ios_base::trunc);
    writer << json_log << endl;
    writer.close();
    cout << "Write Done." << endl;
    exit(1);
}

int main() {
  signal(SIGINT,int_handler);
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double a = j[1]["throttle"];
					double delta = j[1]["steering_angle"];
          /*
          * Both are in between [-1, 1].
          */
          json_log["ptsx"].push_back(ptsx);
          json_log["ptsy"].push_back(ptsy);
          json_log["px"].push_back(px);
          json_log["py"].push_back(py);
          json_log["psi"].push_back(psi);
          json_log["v"].push_back(v);
          json_log["a"].push_back(a);
          json_log["delta"].push_back(delta);
          double steer_value;
          double throttle_value;

          message state;
          state.ptsx = ptsx;
          state.ptsy = ptsy;
          state.px = px;
          state.py = py;
          state.psi = psi;
          state.v = v;
          state.a = a;
          state.delta = delta;

          mpc.Model(state);

          vector<double> Bx_ref = mpc.Bx_ref;
          vector<double> By_ref = mpc.By_ref;

					double latency = 0.1;
					double Lf = 2.67;
					delta *= -1;
					v *= 0.44704;
			
          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
					
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals = Bx_ref;
          vector<double> next_y_vals = By_ref;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;

          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
