#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

#define MIN_SPEED 5
#define TYPICAL_SPEED 49.5 // a bit less than max in case the controler overshoots
#define ACCEL_DELTA_MAX 0.224
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

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
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

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  //string map_file_ = "../data/highway_map.csv";
  string map_file_ = "C:/Users/uidt5589/Documents/00_Udacity/Term_3/Path-Planning/data/highway_map.csv";
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

  // start in lane 1
  int my_ref_lane = 1;
  // have a reference velocity to target
  double my_ref_vel = 0; // mph

#ifdef UWS_VCPKG
  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &my_ref_lane, &my_ref_vel](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length,
    uWS::OpCode opCode) {
#else
  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &my_ref_lane, &my_ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
    uWS::OpCode opCode) {
#endif
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

          double ego_x = j[1]["x"];
          double ego_y = j[1]["y"];
          double ego_s = j[1]["s"];
          double ego_d = j[1]["d"];
          double ego_yaw = j[1]["yaw"];
          double ego_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          int previous_path_size = previous_path_x.size();
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];


          // if previous path does contain points, assume car is at
          // end position of previous path
          if (previous_path_size > 0) {
            ego_s = end_path_s;
          }

          // Read sensor fusion data and decide what best to do next: overtake, slow down or accelerate
          bool change_lanes_or_slow_down = false;
          bool lane_change_left_blocked = (my_ref_lane == 0);
          bool lane_change_right_blocked = (my_ref_lane == 2);
          // changing lanes -> no double-lane-changes == too high accell!
          short my_lane = ((short)floor(ego_d / 4));
          bool changing_lanes = (my_lane != my_ref_lane);
          // distance of leading vehicle ahead
          double min_dist_left = 999.0;
          double min_dist_here = 999.0;
          double min_dist_right = 999.0;

          // find unit normal vector at currernt position
          int other_waypoint_idx = NextWaypoint(ego_x, ego_y, ego_yaw, map_waypoints_x, map_waypoints_y);
          double other_waypoint_dx = map_waypoints_dx[other_waypoint_idx];
          double other_waypoint_dy = map_waypoints_dy[other_waypoint_idx];

          // Go through object list
          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            // get object data
            float obj_d         = sensor_fusion[i][6];
            double obj_vx       = sensor_fusion[i][3];
            double obj_vx_sqr   = obj_vx * obj_vx;
            double obj_vy       = sensor_fusion[i][4];
            double obj_vy_sqr   = obj_vy * obj_vy;
            double obj_v        = sqrt(obj_vx_sqr + obj_vy_sqr);
            double obj_s        = sensor_fusion[i][5];
            // calculate vd and vs
            double obj_vd = (obj_vx * other_waypoint_dx) + (obj_vy * other_waypoint_dy);
            double obj_vs = sqrt(obj_vx_sqr + obj_vy_sqr - (obj_vd * obj_vd));
            // Predict object behavior
            obj_s                 += ((double)previous_path_size*.02*obj_vs);
            short obj_lane        = ((short)floor(obj_d / 4));
            bool obj_moving_right = (obj_vd > 2.0);
            bool obj_moving_left  = (obj_vd < -2.0);
            short obj_direction   = obj_moving_right ? 1 : (obj_moving_left ? -1 : 0);
            short obj_merge       = obj_lane + obj_direction;
            // Check object lane
            bool obj_in_egolane   = (obj_lane == my_ref_lane) || (obj_merge == my_ref_lane);
            bool obj_in_leftlane  = (obj_lane == my_ref_lane - 1) || (obj_merge == my_ref_lane - 1);
            bool obj_in_rightlane = (obj_lane == my_ref_lane + 1) || (obj_merge == my_ref_lane + 1);
            // In order to avoid collisions, make sure that when changing lanes the cars on the side lanes are far away enough/slower than the ego vehicle
            // determine if car is 30m ahead, 15m behind or closer than 10m
            double long_displacement = obj_s - ego_s;
            bool obj_ahead  = (long_displacement > 0.0) && (long_displacement < 30.0);
            bool obj_close  = abs(obj_s - ego_s) < 15;
            bool obj_behind = (long_displacement < 0.0) && (long_displacement > -20.0);
            // update leading vehicle distance
            if (long_displacement > 0.0)
            {
              if (obj_in_egolane)
              {
                if (long_displacement < min_dist_here)
                {
                  min_dist_here = long_displacement;
                }
              }
              else if (obj_in_leftlane)
              {
                if (long_displacement < min_dist_left)
                {
                  min_dist_left = long_displacement;
                }
              }
              else if (obj_in_rightlane)
              {
                if (long_displacement < min_dist_right)
                {
                  min_dist_right = long_displacement;
                }
              }
              else
              {
              }
            }
            // Get velocity difference between ego and object
            bool delta_v = obj_v - my_ref_vel < 0;
            // Is there an object on the ego lane?
            change_lanes_or_slow_down = change_lanes_or_slow_down || (obj_ahead && obj_in_egolane);
            // Is it safe to change lanes ?
            bool obj_is_obstacle      = ((obj_ahead && delta_v) || (obj_behind && !delta_v) || obj_close);
            lane_change_left_blocked  = lane_change_left_blocked || (obj_in_leftlane && obj_is_obstacle);
            lane_change_right_blocked = lane_change_right_blocked || (obj_in_rightlane && obj_is_obstacle);
          }

          // Can we change lanes?
          lane_change_left_blocked = lane_change_left_blocked || (min_dist_left < min_dist_here);
          lane_change_right_blocked = lane_change_right_blocked || (min_dist_right < min_dist_here);

          // Adjust speed and lane positioning
          if (change_lanes_or_slow_down)
          {
            // Path blocked, if we change lanes or cannot change lanes then we need to slow down
            if (changing_lanes || (lane_change_left_blocked && lane_change_right_blocked) || my_ref_vel < 20)
            {
              if (my_ref_vel > MIN_SPEED)
              {
                my_ref_vel -= ACCEL_DELTA_MAX;
              }
            }
          // We can change lanes
            else if (!lane_change_left_blocked)
            {
              my_ref_lane = (my_ref_lane - 1);
            }
            else if (!lane_change_right_blocked)
            {
              my_ref_lane = (my_ref_lane + 1);
            }
          }
          // No obstacle, accelerate if speed is below set speed
          else if (my_ref_vel < TYPICAL_SPEED)
          {
            my_ref_vel += ACCEL_DELTA_MAX;
          }

          // Trajectory generation

          // List of waypoints
          vector<double> control_x;
          vector<double> control_y;

          double my_ref_x = ego_x;
          double my_ref_y = ego_y;
          double ref_yaw = deg2rad(ego_yaw);

          if (previous_path_size < 2)
          {
            // No history, add virtual point for smoothing
            double my_prev_x = ego_x - cos(ego_yaw);
            double my_prev_y = ego_y - sin(ego_yaw);
            control_x.push_back(my_prev_x);
            control_x.push_back(ego_x);
            control_y.push_back(my_prev_y);
            control_y.push_back(ego_y);
          }
          else
          {
            // Use last 2 points from previous trajectory in order to ensure smoothness
            my_ref_x = previous_path_x[previous_path_size - 1];
            my_ref_y = previous_path_y[previous_path_size - 1];
            double my_prev_x = previous_path_x[previous_path_size - 2];
            double my_prev_y = previous_path_y[previous_path_size - 2];
            ref_yaw = atan2(my_ref_y - my_prev_y, my_ref_x - my_prev_x);
            control_x.push_back(my_prev_x);
            control_x.push_back(my_ref_x);
            control_y.push_back(my_prev_y);
            control_y.push_back(my_ref_y);
          }
          // In Frenet add evenly 30m spaced points ahead of the starting ref
          for (int i = 30; i <= 90; i += 30)
          {
            vector<double> controlpoint = getXY(ego_s + i, (2 + 4 * my_ref_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            control_x.push_back(controlpoint[0]);
            control_y.push_back(controlpoint[1]);
          }

          // Transform coordinates
          vector<double> control_x_car;
          vector<double> control_y_car;

          for (int i = 0; i < control_x.size(); i++)
          {
            double x_translate = control_x[i] - my_ref_x;
            double y_translate = control_y[i] - my_ref_y;
            control_x_car.push_back(x_translate*cos(ref_yaw) + y_translate * sin(ref_yaw));
            control_y_car.push_back(-x_translate*sin(ref_yaw) + y_translate * cos(ref_yaw));
          }

          // create spline and sample waypoints
          tk::spline spline_car;
          spline_car.set_points(control_x_car, control_y_car);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // use all previous points
          for (int i = 0; i < previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // set interpolation distance such as to ensure drivin at the desired speed
          double target_x_car = 30.0;
          double target_x_car_sqr = target_x_car* target_x_car;
          double target_y_car = spline_car(target_x_car);
          double target_y_car_sqr = target_y_car* target_y_car;
          double target_dist = sqrt(target_x_car_sqr + target_y_car_sqr);
          double n_points = (2.24*target_dist) / (0.02*my_ref_vel);

          // fill up the rest of our path planner so we have 50 points
          double recent_x_car = 0;
          for (int i = 1; i <= 50 - previous_path_x.size(); i++)
          {
            double x_car = recent_x_car + (target_x_car / n_points);
            double y_car = spline_car(x_car);
            recent_x_car = x_car;
            // go to world coordinates
            double x = (x_car*cos(ref_yaw) - y_car*sin(ref_yaw));
            double y = (x_car*sin(ref_yaw) + y_car*cos(ref_yaw));
            // translate
            x += my_ref_x;
            y += my_ref_y;
            next_x_vals.push_back(x);
            next_y_vals.push_back(y);
          }

          json msgJson;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

#ifdef UWS_VCPKG
          ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif

        }
      }
      else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
#ifdef UWS_VCPKG
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
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
    }
    else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

#ifdef UWS_VCPKG
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code,
    char *message, size_t length) {
    ws->close();
#else
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
    char *message, size_t length) {
    ws.close();
#endif
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
#ifdef UWS_VCPKG // has nothing to do with vcpkg, but avoids adding a new preprocessor define
  if (h.listen("127.0.0.1", port)) {
#else
  if (h.listen(port)) {
#endif
    std::cout << "Listening to port " << port << std::endl;
  }
  else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
  }
