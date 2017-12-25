#ifndef __CARMODEL_H
#define __CARMODEL_H
using namespace std;
typedef struct _OtherCar{
   unsigned int id;
   double x;
   double y;
   double vx;
   double vy;
   double s;
   double d;
} OtherCar;

class CarModel {
private:
   enum CarModelState { KEEP_LANE, CHANGING_LANE };
  
   unsigned int state;
   bool too_close;
   unsigned int too_close_count;
   unsigned int too_close_thresh;
   double ref_vel;
   double speed_limit; 
   double speed_dec;
   double min_clearance_ahead;
   double min_clearance_behind;
   double speed_min;
   double max_s;

   double car_x;
   double car_y;
   double car_s;
   double car_d;
   double car_yaw;
   double car_speed;

   // Previous path data given to the Planner
   vector<double> previous_path_x;
   vector<double> previous_path_y;
   // Previous path's end s and d values 
   double end_path_s;
   double end_path_d;

   // Sensor Fusion Data, a list of all other cars on the same side of the road.
   vector<OtherCar> sensor_fusion;
   // ["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.
   int prev_size;


   vector<double> ptsx;
   vector<double> ptsy;

   double ref_x;
   double ref_y;
   double ref_yaw;

public:
   // output to simulation
   vector<double> next_x_vals;
   vector<double> next_y_vals;

   CarModel(){
      state = KEEP_LANE; 
      too_close = false;
      too_close_count = 0;
      too_close_thresh = 1;
      speed_min = 2.24;
      speed_limit = 50.0;
      ref_vel = speed_min; 
      speed_dec = .224;
      min_clearance_ahead = 12;
      min_clearance_behind = 12;
      max_s = 6945.554;
   }
   bool isTooClose(){
      return too_close;
   }
   void enableTooClose(double dist, double other_car_speed){
      too_close = true;
      too_close_count += 1;
/*
      if (dist< (speed_limit/2)){
          speed_dec = min(speed_dec+.1, 2.0);
      }
      else  
*/
      if (dist< (2*speed_limit/3))
      {
          speed_dec=1.224;
      }
      else if (dist< (4*speed_limit/3))
          speed_dec = .224;  
   }
   void disableTooClose(){
      too_close = false;
   }   
   bool isLaneChangeNeeded(){
      return (too_close_count>=too_close_thresh)?true:false;
   }
   double getSpeed(){
      return ref_vel;
   }
   double getSpeedLimit(){
      return speed_limit;  
   }
   void updateSpeed(){
      if (isTooClose())
      {
         ref_vel -= speed_dec;
         if (ref_vel<speed_min)
             ref_vel = speed_min;
      }
      else if (ref_vel<(speed_limit-1.0))
      {
         if (speed_dec>=(1-.224))
         {
            ref_vel -= .224;
            speed_dec = .224;
         }
         else
            ref_vel += .224;
      }
   } 
   bool isSafeSpeedForLaneChange(){
      if (ref_vel> (3*speed_limit/5))
          return true;
      return false;
   }

   int _getTargetLane(double car_s, double car_d, double max_speed, int current_lane, int prev_points);
   void setTelemetryData(char * data);
   int findLane(int lane);
   void calcRoughPath(int lane, vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s);
   void calcSplinePath();
   
};

#endif 

