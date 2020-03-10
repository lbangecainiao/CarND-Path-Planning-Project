#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          double ref_x;
          double ref_y;
          double previous_x;
          double previous_y;
          double ref_yaw;
          double dist = 0.5;
          double lane = 1;
          double tar_vel = 80; //km/h
          vector<double> pts_x;
          vector<double> pts_y;
          int prev_size = previous_path_x.size();
          int size = 50;
          // Derive the ref points and the ref yaw in the anchor points set
          if(previous_path_x.size()<2) {
            ref_x = car_x;
            ref_y = car_y;
            ref_yaw = deg2rad(car_yaw);
            previous_x = car_x - cos(car_yaw); //
            previous_y = car_y - sin(car_yaw); //
            pts_x.push_back(previous_x);
            pts_y.push_back(previous_y);
            pts_x.push_back(ref_x);
            pts_y.push_back(ref_y);
          } else {
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            previous_x = previous_path_x[prev_size-2];
            previous_y = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-previous_y,ref_x-previous_x);
            pts_x.push_back(previous_x);
            pts_y.push_back(previous_y);
            pts_x.push_back(ref_x);
            pts_y.push_back(ref_y);
          }
          //Shift the lane if a slow vehicle is presented in the from
          for(int i = 0; i < sensor_fusion.size(); ++i) {
            double front_d = sensor_fusion[i][6];
            double front_s = sensor_fusion[i][5];
            double front_vx = sensor_fusion[i][3];
            double front_vy = sensor_fusion[i][4];
            double front_s_dot; 
            vector<double> checked_lane;
            if(front_d < 4+4*lane && front_d > 4*lane && (front_s - car_s) < 30 && front_s > car_s) {  //To check if there is a vehicle ahead
              checked_lane = adjacent_lane (lane);
            }
            cout<<"Front s"<<front_s<<'\n';
            cout<<"Front d"<<front_d<<'\n';
            cout<<"Distance"<<front_s-car_s<<'\n';
            cout<<"Lane"<<lane<<'\n';
          }
   
   
          // Derive the anchor points to define the spline
          vector<double> pts_spline0 = getXY(car_s+30, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> pts_spline1 = getXY(car_s+60, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> pts_spline2 = getXY(car_s+90, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          pts_x.push_back(pts_spline0[0]);
          pts_x.push_back(pts_spline1[0]);
          pts_x.push_back(pts_spline2[0]);
          pts_y.push_back(pts_spline0[1]);
          pts_y.push_back(pts_spline1[1]);
          pts_y.push_back(pts_spline2[1]);
          
          //Perform the coordinate transformation
          double new_x;
          double new_y;
          for(int i = 0; i<pts_x.size(); ++i) {
            new_x = pts_x[i]-ref_x;
            new_y = pts_y[i]-ref_y;
            pts_x[i] = new_x*cos(0-ref_yaw) - new_y*sin(0-ref_yaw);
            pts_y[i] = new_x*sin(0-ref_yaw) + new_y*cos(0-ref_yaw);
          }
          
          //Load the remaining previous path
          for(int i = 0; i<prev_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          //Create the spline
          tk::spline s;
          s.set_points(pts_x,pts_y);
          
          //Add the interpolated points
          double horizon_x = 30;
          double horizon_y = s(horizon_x);
          double horizon_dist = sqrt(horizon_x*horizon_x + horizon_y*horizon_y);
          double step_x = 0.02*(tar_vel/3.6)/horizon_dist * horizon_x;
          double add_on_x = 0;
          double add_on_y = 0;
          double next_x;
          double next_y;
          for(int i=1; i<=size-prev_size; i++) {
            add_on_x += step_x;
            add_on_y = s(add_on_x);
            next_x = add_on_x*cos(ref_yaw) - add_on_y*sin(ref_yaw);
            next_y = add_on_x*sin(ref_yaw) + add_on_y*cos(ref_yaw);
            next_x += ref_x;
            next_y += ref_y;
            next_x_vals.push_back(next_x);
            next_y_vals.push_back(next_y);
          }
             
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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