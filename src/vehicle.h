#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

  double MAX_S = 6945.554;

  struct collider{

    bool collision ; // is there a collision?
    int  time; // time collision happens

  };

  int id, lane;
  double x, y, vx, vy, ax, ay, s, d;


  /**
  * Constructor
  */
  Vehicle(){};
  Vehicle(int id);
  Vehicle(int id, double x, double y, double vx, double vy, double s, double d);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  double get_velocity();

  bool is_visible();

  void get_accel(double vx, double vy, double diff);

  void increment(int dt);

  Vehicle position_at(double t);

  vector<Vehicle> generate_predictions(int horizon=2);

};

#endif