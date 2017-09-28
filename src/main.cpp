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

int lane = 1;
int prevLane = 1;
long laneChangeTicker = 50;
double ref_vel = 0.0;


using namespace std;

const double safeDistFront=30.0;  //Buffer distance in front of car in next lane
const double safeDistBack=10.0;   //Buffer distance in back of car in next lane
const double safeDistBackFuture=36.0;  //Buffer for cars comming from behind in next lane

const double safeDistDetectionVal=30.0;  //s distance to start detecting cars in front of my car so to slow down and consider lane change
const double safeDistDetectionVal2 = safeDistDetectionVal * .7;  //s distance to slow down even more
const double safeDistDetectionVal3 = safeDistDetectionVal * .5;  //s distance to slow down a lot

const double sideTooCloseBuffer=1.0;  //d distance that will detect if a car is comming in to my lane

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

int lane_changer(vector<vector<double>> sensor_fusion, double current_s, double car_speed, int current_lane)
{
	
	//Might want to change lane so check out our options
	long kcost = 0; //keep lane cost
	long rcost = 0; //right lane cost
	long lcost = 0; //left lane cost
	int othercar_lane;
	int lane_to_return = current_lane;
	double lclosestcar_s = 99999;
	double rclosestcar_s = 99999;
	double lclosestcar_s_back = -99999;
	double rclosestcar_s_back = -99999;
	double lclosestcar_v_back = 0.0;
	double rclosestcar_v_back = 0.0;


	//For each othercar...
	for( int i = 0; i < sensor_fusion.size(); i++) {
		//get lane the this other car is in
		float d = sensor_fusion[i][6];
		double check_car_s = sensor_fusion[i][5]; //s position of car in my lane
		double check_car_vx = sensor_fusion[i][3];//velocity x
		double check_car_vy = sensor_fusion[i][4];//velocity y

		double check_vel = sqrt(check_car_vx*check_car_vx + check_car_vy*check_car_vy);

		if( d < 4.0 ) 
			othercar_lane = 0; //left lane
		else if(d >= 4.0 && d < 8.0 )
			othercar_lane = 1; //center lane
		else
			othercar_lane = 2; //right lane

		//In left most lane already
		if( current_lane == 0 )
			lcost+=999;

		//In right most lane already
		if( current_lane == 2 )
			rcost+=999;

		//Score relative left lane only...
		if( othercar_lane == current_lane - 1 ) {
			if( check_car_s > (current_s - safeDistBack) && check_car_s < (current_s + safeDistFront) ) { //Unsafe range
				lcost += 999;
				//cout << "not free to switch left" << endl;
			}
			else {				
				kcost += 100;
				
			}
						        
		}
		//Score relative right lane only...
		else if( othercar_lane == current_lane + 1 ) {
			if( (check_car_s > (current_s - safeDistBack) && check_car_s < (current_s + safeDistFront)) ) {//Unsafe range
				rcost += 999;
				//cout << "not free to switch right" << endl;
			}
			else {
				kcost += 100;				
			}
						        
		}

		//Find closest distances of nearest-car-in-front and for left and right lanes which may be used to determine which lane to go to		
		if( othercar_lane == (current_lane - 1) )  { //left
			if( current_s < check_car_s && check_car_s < lclosestcar_s )
				lclosestcar_s = check_car_s;

		}
		else if( othercar_lane == (current_lane + 1) )  { //right
			if( current_s < check_car_s && check_car_s < rclosestcar_s )
				rclosestcar_s = check_car_s;
		}

		//Find closest distances and velocities of nearest-car-in-back and for left and right lanes which may be used to determine which lane to go to	
		if( othercar_lane == (current_lane - 1) )  { //left
			if( current_s > check_car_s && check_car_s > lclosestcar_s_back ) {
				lclosestcar_s_back = check_car_s;
				lclosestcar_v_back = check_vel;
			}
		}
		else if( othercar_lane == (current_lane + 1) )  { //right
			if( current_s > check_car_s && check_car_s > rclosestcar_s_back ) {
				rclosestcar_s_back = check_car_s;
			    rclosestcar_v_back = check_vel;
			}
		}

		

	}//for

	//cout << "keep=" << kcost << " lcost=" << lcost << " rcost=" << rcost << endl;

	//Check for best if have a choice of left or right...
	if( current_lane == 1 ) { //Only revelent for center lane
	 
      if( rclosestcar_s > lclosestcar_s ) {
		  lcost+=200;
		  //cout << "Best to go right" << endl;
	  }
	  else {
		  rcost+=200;
		  //cout << "Best to go left" << endl;
	  }
	}

	//Check to see cars in other lanes are on a collision course from behind
	double future_s = current_s + (30.0*.02*car_speed); //project s 30 points in future in left lane
	if( lclosestcar_s_back >  0 ) { //left
		//project closest car in left lane
		double check_future_s = lclosestcar_s_back + (30.0*.02*lclosestcar_v_back); //project s 30 points in future in left lane
		
		//cout << "L future s=" << future_s << " future check s=" << check_future_s << "back speed=" << lclosestcar_v_back << "myspeed=" << car_speed <<endl;

		if( check_future_s > (future_s - safeDistBackFuture) && lclosestcar_v_back > car_speed) { //Unsafe range
			lcost += 999; //dangerous
			//cout << "Accelerating carin left lane behind" << endl;
		}
		else
			kcost += 100;					        
	}
	if( rclosestcar_s_back > 0 ) { //right
		//project closest car in right lane
		double check_future_s = rclosestcar_s_back + (30.0*.02*rclosestcar_v_back); //project s 30 points in future in left lane
		
		//cout << "R future s=" << future_s << " future check s=" << check_future_s <<  " back speed=" << lclosestcar_v_back << " myspeed=" << car_speed <<endl;

		if( check_future_s > (future_s - safeDistBackFuture) && lclosestcar_v_back > car_speed) { //Unsafe range
			rcost += 999; //dangerous
		    //cout << "Accelerating car in right lane behind" << endl;
		}
		else
			kcost += 100;					        
	}



	//Determine which lane is best based on cost...
	if( kcost > rcost || kcost > lcost ) {
		if( lcost <= rcost )  								
			lane_to_return--; //Change to left lane
		else
			lane_to_return++; //Change to right lane
	}

	//else leave lanes as is	

	return lane_to_return;
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

  //int lane = 1;
  


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
			double car_s_immed = car_s;
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

			car_speed/=2.27273; //convert to m/s

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

			int prev_size = previous_path_x.size();

			if( prev_size > 0 ) {
				car_s = end_path_s;
			}

			int too_close = 0;			

			//Go through each object in sensor fusion list...
			for( int i = 0; i < sensor_fusion.size(); i++) {

				//car is in my lane
				float d = sensor_fusion[i][6];
				double check_car_s = sensor_fusion[i][5]; //s position of other car
				double check_car_s_immed = check_car_s;   //s position of other car
				

				//boundry of lane the ego is in
				float lbound = 2+4*lane-2;
				float rbound = 2+4*lane+2;

				//Only process if in my lane range
				if( d < rbound && d > lbound ) {
					double vx = sensor_fusion[i][3];
					double vy = sensor_fusion[i][4];
					double check_speed = sqrt(vx*vx+vy*vy);   //speed of car in m lane
					
					
				    check_car_s+=((double)prev_size*.02*check_speed); //project s points out of car in my lane
					
					double s_diff = check_car_s-car_s;
					
					//See if too close in same lane					
					if( (check_car_s_immed > car_s_immed) && s_diff < safeDistDetectionVal)  {
						//ref_vel = 29.5;
						too_close = 1;        //deceleration level 1
						//cout << "s diff 0=" << s_diff << endl;
						if( s_diff < safeDistDetectionVal3 )
							too_close = 3;    //deceleration level 3
						else if( s_diff < safeDistDetectionVal2 )
							too_close = 2;    //deceleration level 2
					}
				}//if

				//If is too close from side...
				if( (fabs(lbound - d) < sideTooCloseBuffer || fabs(rbound - d) < sideTooCloseBuffer) && 
					check_car_s_immed > car_s_immed - 5.0 && check_car_s_immed < car_s_immed + 20.0 ) {
					too_close = 4;
					//cout << "too close to side" << endl;
				}


				//Detect car rigth beside me
				//if( (fabs(lbound - d) < 4.0 || fabs(rbound - d) < 4.0) && 
				//	check_car_s_immed > car_s_immed - 5.0 && check_car_s_immed < car_s_immed + 5.0 ) {
				//		double vx = sensor_fusion[i][3];
				//	double vy = sensor_fusion[i][4];
				//	double check_speed = sqrt(vx*vx+vy*vy);   //speed of car in m lane
				//	cout << "car beside me " << laneChangeTicker << "my speed=" << car_speed << " other speed=" << check_speed <<endl;
				//}

			}//for

			//Increment lane change ticker used to delay lane change if too soon
			laneChangeTicker++;
			if( laneChangeTicker > 50000 )
				laneChangeTicker = 50;


			//random lane change
			//if( laneChangeTicker > 0 && laneChangeTicker % 300 == 0 ) {
			//	lane = lane_changer(sensor_fusion, car_s_immed, car_speed, lane);
			//	cout << "random lane cost" << endl;
			//}

			if( too_close > 0 ) { //decelerate and maybe change lanes
				switch( too_close ) {
				case 1:
					ref_vel -= .224; //normal deceleration
					//cout << "level 1" << endl;
					break;
				case 2:
					ref_vel -= .70;  //medium deceleration
					//cout << "level 2" << endl;
					break;
				case 3:
					ref_vel -= 1.00; //high deceleration
					//cout << "level 3" << endl;
					break;
                case 4:
					ref_vel -= 2.50; //high deceleration
					//cout << "level 4" << endl;
					break;
				default:
					ref_vel -= .224; //normal deceleration
					break;
				}

				//Do not decelerate too much.
				if( ref_vel < 0.0 ) ref_vel = 1.0;
				
				
				
				if( laneChangeTicker > 15 ) //delay consecutive lane changes to avoid jerk 
					lane = lane_changer(sensor_fusion, car_s_immed, car_speed, lane);
				//else cout << "lane ticker tick" << endl;

				if( prevLane != lane ) {
					//cout << "lane change----" << endl;
					laneChangeTicker = 0;
					prevLane = lane;
				}

				//cout << "lane to go in to=" << lane << endl;				

			}
			else if( ref_vel < 49.5) { //Accelerate
				ref_vel += .33; //.224
			}


			//Figure out what points to send back to the simulator....

			vector<double> ptsx;
			vector<double> ptsy;

			double ref_x = car_x;
			double ref_y = car_y;
			double ref_yaw = deg2rad(car_yaw);


			//Create or use points to create spline...
			
			if(prev_size < 2) //Create a couple previous points then "create" some based on current car_yaw and position
			{
				double prev_car_x = car_x - cos(car_yaw);
				double prev_car_y = car_y - sin(car_yaw);

				ptsx.push_back(prev_car_x);
				ptsx.push_back(car_x);

				ptsy.push_back(prev_car_y);
				ptsy.push_back(car_y);
			}
			else  //use previous path points
			{
				ref_x = previous_path_x[prev_size-1];
				ref_y = previous_path_y[prev_size-1];

				double ref_x_prev = previous_path_x[prev_size-2];
				double ref_y_prev = previous_path_y[prev_size-2];
				ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

                ptsx.push_back(ref_x_prev);
				ptsx.push_back(ref_x);

				ptsy.push_back(ref_y_prev);
				ptsy.push_back(ref_y);
				
			}

			//Make some points out in 30,60, and 90 meters in the distance
			vector<double> next_mp0 = getXY(car_s+30,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> next_mp1 = getXY(car_s+60,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> next_mp2 = getXY(car_s+90,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

			ptsx.push_back(next_mp0[0]);
			ptsx.push_back(next_mp1[0]);
			ptsx.push_back(next_mp2[0]);

			ptsy.push_back(next_mp0[1]);
			ptsy.push_back(next_mp1[1]);
			ptsy.push_back(next_mp2[1]);

			//Shift and rotate to origin...
			for( int i = 0; i < ptsx.size(); i++ ) {
				double shift_x = ptsx[i] - ref_x;
				double shift_y = ptsy[i] - ref_y;

				ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
				ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
			}

			tk::spline s;

			s.set_points(ptsx,ptsy);  //Set spline points



			//Set up points to send back to simulator...

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

			//Init with unused previous path points
			for(int i = 0; i < previous_path_x.size(); i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

			//Break up spline points to get to the desired velocity.  This is mainly taken from code walk through.
			double target_x = 30;
			double target_y = s(target_x); //y value from spline
			double target_dist = sqrt((target_x*target_x)+(target_y*target_y));

			double x_add_on = 0;

			//Add some new points to end derived from the spline...
			for( int i = 1; i <= 50-previous_path_x.size(); i++ ) {
				double N = (target_dist/(.02*ref_vel/2.24));  //will help us get right spacing
				double x_point = x_add_on+(target_x)/N;
				double y_point = s(x_point);

				x_add_on = x_point;

				//Shift and rotate back
				double x_ref = x_point;
				double y_ref = y_point;

				x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
				y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

				x_point += ref_x;
				y_point += ref_y;

				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);

			}



		//Drive in a circle
		 /* double pos_x;
          double pos_y;
          double angle;
          int path_size = previous_path_x.size();

          for(int i = 0; i < path_size; i++)
          {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
          }

          if(path_size == 0)
          {
              pos_x = car_x;
              pos_y = car_y;
              angle = deg2rad(car_yaw);
          }
          else
          {
              pos_x = previous_path_x[path_size-1];
              pos_y = previous_path_y[path_size-1];

              double pos_x2 = previous_path_x[path_size-2];
              double pos_y2 = previous_path_y[path_size-2];
              angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
          }

          double dist_inc = 0.5;
          for(int i = 0; i < 50-path_size; i++)
          {    
              next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
              next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
              pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
              pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
          }*/


          	
			json msgJson;
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
