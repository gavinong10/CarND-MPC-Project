#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

int POLYDEGREE = 3;

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
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
          double delta = j[1]["steering_angle"];
          delta *= -1.0; // convert as per World->Unity conventions

          // Print out the points that we will be fitting to
          std::cout << "ptsx: " << endl;
          for (double ptx : ptsx) {
            std:: cout << ptx << ", " << std::endl;
          }
          std::cout << "\n" << endl;

          std::cout << "ptsy: " << endl;
          for (double pty : ptsy) {
            std:: cout << pty << ", " << std::endl;
          }
          std::cout << "\n" << endl;

          std::cout << "px: " << px << "\n" << std::endl;
          std::cout << "py: " << py << "\n" << std::endl;
          std::cout << "psi: " << psi << "\n" << std::endl;
          std::cout << "v: " << v << "\n" << std::endl;

          // Transform telemetry to car coordinate frame
          size_t num_pts = ptsx.size();
          Eigen::VectorXd car_ptsx = Eigen::VectorXd(num_pts);
          Eigen::VectorXd car_ptsy = Eigen::VectorXd(num_pts);
          for(size_t i=0; i<num_pts; i++) {
            car_ptsx(i) =  (ptsx[i]-px)*cos(psi) + (ptsy[i]-py)*sin(psi);
            car_ptsy(i) = -(ptsx[i]-px)*sin(psi) + (ptsy[i]-py)*cos(psi);
          }

          // Fit a polynomial to the points representing the ideal path the vehicle should follow.
          auto coeffs = polyfit(car_ptsx, car_ptsy, POLYDEGREE);

          // Calculate the cross track error

          // In car's coordinate system, it is at (0,0)
          // The cross track error needs to be calculated at a point in time
          // caused by the latency effect

          // Update state accounding for latency
          double v_ms = v * 0.44704; // convert from mph to m/s
          double dt_latency = 0.1; // in seconds;
          double delta_psi =  v_ms*delta/Lf*dt_latency;
          // Use delta_psi/2 to account for fact that psi a change of angle across dt (non-constant)
          double px_car_latency = v_ms*cos(delta_psi/2)*dt_latency;
          double py_car_latency = v_ms*sin(delta_psi/2)*dt_latency;

          // Now we can evaluate cte
          double cte = polyeval(coeffs, px_car_latency) - py_car_latency;
          std::cout << "cte: " << cte << "\n" << std::endl;

          // Calculate orientation error
          // We need to find the derivative of the polyfit function and get it's psi value transformed back.
          double dpy = coeffs[1] + 2*coeffs[2]*px_car_latency + 3*coeffs[3]*px_car_latency*px_car_latency;
          double ref_psi = atan(dpy) + psi;
          double epsi = psi - ref_psi; // i.e. epsi = -atan(dpy)
          
          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value = 0;
          double throttle_value = 0.01;

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
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          for(size_t i=0; i<num_pts; i++) {
            next_x_vals.push_back(car_ptsx(i));
            next_y_vals.push_back(car_ptsy(i));
          }

          std::cout << "next_x_vals: " << endl;
          for (double next_x_val : next_x_vals) {
            std:: cout << next_x_val << ", " << std::endl;
          }
          std::cout << "\n" << endl;

          std::cout << "next_y_vals: " << endl;
          for (double next_y_val : next_y_vals) {
            std:: cout << next_y_val << ", " << std::endl;
          }
          std::cout << "\n" << endl;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
