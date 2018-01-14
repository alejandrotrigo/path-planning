#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <math.h>
#include <string>
#include <iterator>
//#include "cost.h"

/**
 * Initializes Vehicle
 */

// TODO Hacer las helper functions aparte, pi, distance, closestwaypoints, nextwaypoints y frenet


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const vector<double> &maps_y)
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

int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
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
std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
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

Vehicle::Vehicle(int id){
    this->id = id;
}

Vehicle::Vehicle(int id, double x, double y, double vx, double vy, double s, double d) {

    this->id = id;
    this->lane = (int) d / 4;
    this->x = x;
    this->y = y;
    this->vx = vx;
    this->vy = vy;
    this->ax = 0;
    this->ay = 0;
    this->s = s;
    this->d = d;

}

Vehicle::~Vehicle() {}

double Vehicle::get_velocity(){
    return sqrt(vx*vx + vy*vy);
}

bool Vehicle::is_visible(){
    if ((d > 0) && (s < MAX_S)) return true;
    else return false;
}

void Vehicle::get_accel(double vx, double vy, double diff){
    this->ax = (vx - this->vx) / diff;
    this->ay = (vy - this->vy) / diff;
    this->ax > 0.01 ? this->ax : 0;
    this->ay > 0.01 ? this->ay : 0;
    this->vx = vx;
    this->vy = vy;
}

void Vehicle::increment(int dt = 1) {

    this->x += this->vx * dt + this->ax * dt * dt / 2;
    this->y += this->vy * dt + this->ay * dt * dt / 2;
    this->vx += this->ax * dt;
    this->vy += this->ay * dt;
    double angle = atan2(this->vy, this->vx);
    double yaw = (angle > 0.1) ? angle : 0;
    std::vector<double> frenet = getFrenet(this->x, this->y, yaw);
    this->s = frenet[0];
    this->d = frenet[1];
    this->lane = (int) this->d / 4;
}

Vehicle Vehicle::position_at(double t) {
    
    double pred_x = this->x + this->vx * t + this->ax * t * t / 2;
    double pred_y = this->y + this->vy * t + this->ay * t * t / 2;
    double pred_vx = this->vx + this->ax * t;
    double pred_vy = this->vy + this->ay * t;
    double angle = atan2(pred_vy, pred_vx);
    double yaw = (angle > 0.1) ? angle : 0;
    std::vector<double> frenet = getFrenet(pred_x, pred_y, yaw);
    double pred_s = frenet[0];
    double pred_d = frenet[1];

    return new Vehicle(this->id, pred_x, pred_y, pred_vx, pred_vy, pred_s, pred_d);
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
    /*
    Generates predictions for non-ego vehicles to be used
    in trajectory generation for the ego vehicle.
    */
    vector<Vehicle> predictions;
    for(int i = 0; i < horizon; i++) {
      predictions.push_back(position_at(horizon));
    }
    return predictions;
}
