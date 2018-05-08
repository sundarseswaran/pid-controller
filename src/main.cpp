#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <valarray>     // std::valarray

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

// added for twiddler
double p[] = {0, 0, 0};
double dp[] = {1, 1, 1};
double tolerance = 0.001;
std::valarray<double> dpSum (dp, 3);

int num_params = 3;
int twAdjustingParamIndex  = -1;
int twBatchSize = 10;
int twRunCounter = 0;
int itTwOpt = 0;

double best_err = 100;

bool twResetParams = true;
bool twOptimized = false;

//int atParamIndex = -1;

void twiddleOptimizer(PID &pidcontrol) {

  double paramsDelta = 0;
  itTwOpt++;
  twRunCounter = 0;

  std::cout << "Optimizer: " << itTwOpt << ", Error: " << pidcontrol.TotalError() << std::endl;
  std::cout << "Params: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;

  if (dpSum.sum() > tolerance) {

    // pick the correct parameter that need to be adjusted.
    if(!twResetParams) { // if the twiddler is going through the reset cycle just continue to verifying error again
                         // without moving to next index.
      twAdjustingParamIndex++;
      if(twAdjustingParamIndex >= num_params) {
        twAdjustingParamIndex = 0;
      }
    }

    if (twResetParams) {
      // reset the adjustments made to the params, if the change is increasing the error
      if(fabs(pidcontrol.TotalError()) < best_err) {
        best_err = fabs(pidcontrol.TotalError());
        dp[twAdjustingParamIndex] *= 1.1;
      } else {
        p[twAdjustingParamIndex] += dp[twAdjustingParamIndex];
        dp[twAdjustingParamIndex] *= 0.9;
      }

      twResetParams = false;
      return;
    }

    if (fabs(pidcontrol.TotalError()) < best_err) {
      // when a better error is found
      //  // set that as a best error
      //  // and adjust dParams

      best_err = fabs(pidcontrol.TotalError());
      paramsDelta = 1.1 * dp[twAdjustingParamIndex];
    } else {
      // if the newly found error is worse than the best_err error - reset the value of params
      paramsDelta = -2 * dp[twAdjustingParamIndex];
      twResetParams = true;
    }

    // update params with estimated change.
    p[twAdjustingParamIndex] += paramsDelta; // dp[twAdjustingParamIndex];

  } else {
    twOptimized = true;
  }

  return;
}

int main()
{
  uWS::Hub h;

  PID pidSteering, pidThrottle;
  // TODO: Initialize the pid variable.

//  pidSteering.Init(0.134611, 0.000270736, 3.05349);
//  pidThrottle.Init(0.316731, 0.0000, 0.0226185);

//  pidSteering.Init(0, 0, 0);
  pidSteering.Init(0.1, 0.001, 2);

  h.onMessage([&pidSteering, &pidThrottle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value, throttle_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          pidSteering.UpdateError(cte);
          steer_value = pidSteering.TotalError();

          if(!twOptimized && twRunCounter >= twBatchSize) {
            twiddleOptimizer(pidSteering);
          }

          if(!twOptimized) {
            twRunCounter++;
          }

//          // applicable only when a 2nd pid controller is used or throttling
//          pidThrottle.UpdateError(0.5-speed);
//          throttle_value = pidThrottle.TotalError();

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
//          std::cout << "CTE: " << cte << " Throttle Value: " << throttle_value << std::endl;
//          std::cout << "CTE: " << cte << " speed Value: " << speed << std::endl;

//          // applicable only when a 2nd pid controller is used or throttling
//          json msgJson;
//          msgJson["steering_angle"] = steer_value;
//          if(speed > 0.1) {
//            msgJson["throttle"] = speed + throttle_value;
//          } else {
//            msgJson["throttle"] = 0.3;
//          }

          // model with constant throttle.
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//          std::cout << msg << std::endl;
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
