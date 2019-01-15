#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

PID pid_st;
PID pid_th;

// Gain scheduling target values for steering PID
double target_kp_0 = 0.15;
double target_kp_max = 0.05;

double target_kd_0 = 0.04;
double target_kd_max = 0.01;

// Target throttle
double target_th = 0.45;

int main() {
  uWS::Hub h;

  pid_st.Init(target_kp_0, 0.001, target_kd_0);
  pid_st.SetTarget(0.0);
  pid_st.SetMin(-1.0);
  pid_st.SetMax(1.0);

  pid_th.Init(0.12, 0.01, 0.00);
  pid_th.SetTarget(target_th);
  pid_th.SetMin(0.0);
  pid_th.SetMax(1.0);

  h.onMessage([&pid_st](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());                    
          
          pid_st.UpdateError(cte);
          double steer_value = pid_st.GetOutput(); 
          
          pid_th.UpdateError(abs(cte));
          double throttle = pid_th.GetOutput();

          // Linear gain scheduling with car speed
          double newKp = (target_kp_max - target_kp_0)/40.0 * speed + target_kp_0; 
          double newKd = (target_kd_max - target_kd_0)/40.0 * speed + target_kd_0; 

          pid_st.SetKp(newKp);
          pid_st.SetKd(newKd);

          std::cout << std::fixed;
          std::cout << std::setprecision(3);
          std::cout << "CTE: " << cte << "\t Steering: " << steer_value << "\t throttle: " << throttle << "\t Kp st: " << newKp << "\t Kd st: " << newKd << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

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