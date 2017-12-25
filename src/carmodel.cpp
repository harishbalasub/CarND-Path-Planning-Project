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
#include "util.h"
#include "carmodel.h"

using namespace std;
// for convenience
using json = nlohmann::json;



int CarModel::_getTargetLane(double car_s, double car_d, double max_speed, int current_lane, int prev_points){
        vector<double> traffic_speed_min;
        vector<double> traffic_speed_ahead;
        vector<double> traffic_speed_behind;
        vector<double> clearance_ahead;
        vector<double> clearance_behind;
        vector<double> lane_positions;
        lane_positions.push_back (2.0);
        lane_positions.push_back (6.0);
        lane_positions.push_back (10.0);

        int chosen_lane = current_lane;

        if (isLaneChangeNeeded()==false || isSafeSpeedForLaneChange()==false)
                return chosen_lane;

        for (int j=0; j<lane_positions.size(); j++)
        {

                traffic_speed_min.push_back(100.0);
                traffic_speed_ahead.push_back(100.0);
                traffic_speed_behind.push_back(100.0);
                clearance_ahead.push_back(1000.0);
                clearance_behind.push_back(1000.0);
                for(int i=0; i<sensor_fusion.size(); i++)
                {
                        //determine if car is directly ahead
                        double sensor_d = sensor_fusion[i].d;
                        if (abs(sensor_d - lane_positions[j]) < 2.7)
                        {
                                double vx = sensor_fusion[i].vx;
                                double vy = sensor_fusion[i].vy;
                                double check_speed = sqrt(vx*vx + vy*vy);
                                double check_car_s = sensor_fusion[i].s;

                                /*
                                   check_car_s += (double)prev_points*.02*check_speed;

                                   double check_distance = check_car_s-(car_s+prev_points*.02*max_speed);

                                //adjust check_distance when wrapping around max_s to zero
                                 */
                                check_car_s+=((double)prev_points*.02*check_speed);
                                double check_distance = (check_car_s-car_s);
                                if (check_distance > (max_s/2.0))
                                        check_distance -= max_s;
                                if (check_distance < (-1.0*max_s/2.0))
                                        check_distance += max_s;

                                // check s values greater than mine and s gap

                                if (check_distance > 0 && check_speed < traffic_speed_min[j])
                                        traffic_speed_min[j] = check_speed;

                                if( check_distance > 0 && abs(check_distance) < clearance_ahead[j])
                                {
                                        clearance_ahead[j] = check_distance;
                                        traffic_speed_ahead[j] = check_speed;
                                }

                                if( check_distance < 0 && abs(check_distance) < clearance_behind[j])
                                {
                                        clearance_behind[j] = check_distance*-1.0;
                                        traffic_speed_behind[j] = check_speed;            
                                }
                        }
                }

        }


        //look at lane to the left unless car is in leftmost lane
        if (current_lane > 0)
        {
                if ((clearance_ahead[current_lane-1] > min_clearance_ahead && clearance_behind[current_lane-1] > min_clearance_behind) && (traffic_speed_min[current_lane-1] > max_speed)
                                && (clearance_ahead[current_lane] > 3.0*min_clearance_ahead/4.0) 
                   )
                {
                        chosen_lane = current_lane-1;
                        //ref_vel = traffic_speed[current_lane-1];
                        too_close_count = 0;
                }
                if ((clearance_ahead[current_lane-1] > (min_clearance_ahead/2) &&
                    ((traffic_speed_ahead[current_lane-1]+2) > max_speed)&& 
                    clearance_behind[current_lane-1] > (min_clearance_behind/2)) &&
                    ((traffic_speed_behind[current_lane-1] <= (max_speed+2)) || (traffic_speed_behind[current_lane-1]==100)) && 
                    (clearance_ahead[current_lane] > 2.0*min_clearance_ahead/4.0)
                   )
                {
                        chosen_lane = current_lane-1;
                        //ref_vel = traffic_speed[current_lane-1];
                        too_close_count = 0;
                }
        }

        //look at lane to the right unless car is in rightmost lane
        if (current_lane < (lane_positions.size()-1))
        {
                if ((clearance_ahead[current_lane+1] > min_clearance_ahead && clearance_behind[current_lane+1] > min_clearance_behind) && (traffic_speed_min[current_lane+1] > max_speed)
                                && (clearance_ahead[current_lane] > 3.0*min_clearance_ahead/4.0) 
                   )
                {
                        chosen_lane = current_lane+1;
                        //ref_vel = traffic_speed[current_lane+1];
                        too_close_count = 0;
                }
                if ((clearance_ahead[current_lane+1] > (min_clearance_ahead/2) &&
                    ((traffic_speed_ahead[current_lane+1]+2) > max_speed)&& 
                    clearance_behind[current_lane+1] > (min_clearance_behind/2)) &&
                    ((traffic_speed_behind[current_lane+1] <= (max_speed+2)) || (traffic_speed_behind[current_lane+1]==100)) && 
                    (clearance_ahead[current_lane] > 2.0*min_clearance_ahead/4.0)
                   )
                {
                        chosen_lane = current_lane+1;
                        //ref_vel = traffic_speed[current_lane-1];
                        too_close_count = 0;
                }

        }

        //if ( (chosen_lane == current_lane) && too_close==false)
        //    too_close_count = 0;

        return chosen_lane;

}

void CarModel::setTelemetryData(char * data)
{
        auto s = hasData(data);
        auto j = json::parse(s);

        car_x = j[1]["x"];
        car_y = j[1]["y"];
        car_s = j[1]["s"];
        car_d = j[1]["d"];
        car_yaw = j[1]["yaw"];
        car_speed = j[1]["speed"];

        // Previous path data given to the Planner
        previous_path_x.clear();
        previous_path_y.clear();
        auto p_x = j[1]["previous_path_x"];
        for (auto it = p_x.begin(); it != p_x.end(); it++) {
                previous_path_x.push_back(*it);
        }
        auto p_y = j[1]["previous_path_y"];
        for (auto it = p_y.begin(); it != p_y.end(); it++) {
                previous_path_y.push_back(*it);
        }
        // Previous path's end s and d values 
        end_path_s = j[1]["end_path_s"];
        end_path_d = j[1]["end_path_d"];

        // Sensor Fusion Data, a list of all other cars on the same side of the road.
        auto s_f = j[1]["sensor_fusion"];
        sensor_fusion.clear();
        // ["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.
        for (auto it = s_f.begin(); it != s_f.end(); it++) {
                OtherCar car;
                car.id = (*it)[0];
                car.x = (*it)[1];
                car.y = (*it)[2];
                car.vx = (*it)[3];
                car.vy = (*it)[4];
                car.s = (*it)[5];
                car.d = (*it)[6];
                sensor_fusion.push_back(car);
        }
        prev_size = previous_path_x.size();

        next_x_vals.clear();
        next_y_vals.clear();
        ptsx.clear();
        ptsy.clear();

        ref_x = car_x;
        ref_y = car_y;
        ref_yaw = deg2rad(car_yaw);
        if (prev_size>0)
        {
                car_s = end_path_s;
        }
}

int CarModel::findLane(int lane)
{
        // find ref_v to use
        for(int i= 0;i < sensor_fusion.size(); i++)
        {
                // car is in my lane
                float d = sensor_fusion[i].d;
                if (d< (2+4*lane+3) && d>(2+4*lane-3))
                {
                        double vx = sensor_fusion[i].vx;
                        double vy = sensor_fusion[i].vy;
                        double check_speed = sqrt(vx*vx + vy*vy);
                        double check_car_s = sensor_fusion[i].s;
                        double distance;

                        check_car_s+=((double)prev_size*.02*check_speed);
                        distance = check_car_s - car_s;
                        if (distance > (max_s/2.0))
                                distance -= max_s;
                        if (distance < (-1.0*max_s/2.0))
                                distance += max_s;
                        // check s values greater than mine and s gap
                        //if ((check_car_s > car_s) && ((check_car_s-car_s)<model.getSpeedLimit()))
                        if ((distance>0) && (distance<getSpeedLimit()))
                        {
                                // flag to try to change lanes
                                //too_close = true;
                                enableTooClose((check_car_s-car_s), check_speed);
                        }
                }

        }

        updateSpeed();
        lane = _getTargetLane(car_s, car_d, getSpeed(), lane, prev_size);
        std::cout << "lane " << lane << " ref_vel " << getSpeed() << " too_close " << isTooClose() << " prev_size " << prev_size << endl;
        return lane;

}

void CarModel::calcRoughPath(int lane, vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s)
{
        if(prev_size < 2)
        {
                //create two points defining a path tangent to the car
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);

                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
        }
        //use end of previous path as starting reference
        else
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

        //define rough path in Frenet coordinates, convert to XY
        //vector<double> next_wp0 = getXY(car_s+15, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        //vector<double> next_wp1 = getXY(car_s+30, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        vector<double> next_wp2 = getXY(car_s+45, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        vector<double> next_wp3 = getXY(car_s+60, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        vector<double> next_wp4 = getXY(car_s+75, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        vector<double> next_wp5 = getXY(car_s+90, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

        //ptsx.push_back(next_wp0[0]);
        //ptsx.push_back(next_wp1[0]);
        ptsx.push_back(next_wp2[0]);
        ptsx.push_back(next_wp3[0]);
        ptsx.push_back(next_wp4[0]);
        ptsx.push_back(next_wp5[0]);

        //ptsy.push_back(next_wp0[1]);
        //ptsy.push_back(next_wp1[1]);
        ptsy.push_back(next_wp2[1]);
        ptsy.push_back(next_wp3[1]);
        ptsy.push_back(next_wp4[1]);
        ptsy.push_back(next_wp5[1]);

        //convert global XY coordinates to car's reference frame to allow for polynomial smoothing
        for (int i=0; i<ptsx.size(); i++)
        {
                double shift_x = ptsx[i]-ref_x;
                double shift_y = ptsy[i]-ref_y;

                ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
                ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
        }
}

void CarModel::calcSplinePath()
{
        //create spline for smooth path
        tk::spline sp;

        sp.set_points(ptsx,ptsy);

        int prev_points=prev_size;

        //load previous points into path planner
        for (int i=0; i<prev_points; i++)
        {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
        }

        //calculate remaining points for path planner based on spline
        double target_x = 30.0;
        double target_y = sp(target_x);
        double target_dist = sqrt(target_x*target_x + target_y*target_y);


        double x_add_on = 0;


        for(int i = 0; i <= 50-prev_points; i++)
        {
                double N = target_dist/(.02*getSpeed()/2.24);
                double x_point = x_add_on+target_x/N;
                double y_point = sp(x_point);


                x_add_on = x_point;

                double x_ref = x_point;
                double y_ref = y_point;

                //convert points back to global coordinates
                x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
                y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

                x_point += ref_x;
                y_point += ref_y;

                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
        }

}

