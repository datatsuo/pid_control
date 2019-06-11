#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;
  // Initialization of variables for PID control of steering value
  double Kp_ = 0.13;
  double Ki_ = 0.001;
  double Kd_ = 3.1;
  bool do_twiddle = false; // NOTE: after the PID coeffs are tuned with twiddle, set this to false
  pid.Init(Kp_, Ki_, Kd_, do_twiddle);

  PID pid_speed;
  /* use below in case one wants to do PID control for throttle independently */
  // // Initialization of variables for PID control of throttle value
  // double Kp_speed = 0.02;
  // double Ki_speed = 0.00;
  // double Kd_speed = 0.8;
  // bool do_twiddle_speed = false;
  // pid_speed.Init(Kp_speed, Ki_speed, Kd_speed, do_twiddle_speed);

  h.onMessage([&pid, &pid_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          // double speed = std::stod(j[1]["speed"].get<std::string>());
          // double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double throttle_value;

          // update PID errors for steering value
          if(pid.twiddle == true){ // do the PID coefficients tuning with twiddle
            pid.UpdateErrorTwiddle(cte);
          }
          else{ // do not carry out the coefficients tuning
            pid.UpdateError(cte);
          }
          // compute the steering value
          steer_value = - pid.TotalError();
          /* in case one wants to add a bias to the steer_value use below instead */
          // steer_value = 0.4 - pid.TotalError();

          // compute the throttle value (since the speed should be decreased
          // when the steer_value is away from 0, a term proportional to abs(steer_value) is subtracted)
          throttle_value = 0.5 - 0.1 * abs(steer_value);

          /* use below in case one wants to PID control for throttle independently */
          // // update PID errors for throttle value
          // if(pid_speed.twiddle == true){ // do the PID coefficients tuning with twiddle
          //   pid_speed.UpdateErrorTwiddle(cte);
          // }
          // else{ // do not carry out the coefficients tuning
          //   pid_speed.UpdateError(cte);
          // }
          // // compute the throttle value (since the speed should be decreased
          // // when the steer_value is away from 0, a term proportional to abs(steer_value) is also subtracted)
          // throttle_value = 0.5 - pid_speed.TotalError() - 0.1 * abs(steer_value);

          /* use below in case one wants to use a fixed throttle_value */
          // throttle_value = 0.3;

          // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          // std::cout << "CTE: " << cte << " Throttle Value: " << throttle_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
