#ifndef FSM_H
#define FSM_H

#include <fstream>
#include <math.h>
#include <iostream>
#include <vector>
#include <tuple>
#include <map>
#include "vehicle.h"

const double MAX_ACCELERATION = 0.224;
const double MAX_DIST = 10000;
const double DISTANCE = 66;
const double SECURITY_DIST = 20.0;
const double MIN_DIST = SECURITY_DIST/2;
const double TARGET_SPEED = 49.7;
const double SPEED_LIMIT = 50.0;
const double PREDICTION_INTERVAL = 0.15, 

enum State {KL, PL, PR, TL, TR};

class FSM
{
private:
  struct collider
  {
  	bool collision = false;
  	int time = 100;
  }

  struct trajectory
  {
  	int target_lane, current_lane;
  	double avg_speed;
  	double prop_closest_approach;
  	double actual_closest_approach;
  	collider collides;
  };

  int const COLLISION = 10e6;
  int const DANGER = 3*10e5;
  int const COMFORT = 10e4;
  int const EFFICIENCY = 10e3;

  double change_lane_cost(trajectory data);
  double inefficiency_cost(trajectory data);
  double collision_cost(vector<_prediction> trajectory, map<int, vector<_prediction> > predicitons, trajectory data);
  double buffer_cost(vector<_prediction> trajectory, map<int, vector<_prediction> > predicitons, trajectory data);
  double free_line_cost(vector<_prediction> trajectory, map<int, vector<_prediction> > predicitons, trajectory data);

  bool check_collision(double car_s, double ref_vel, _prediction snap, _prediction pred, State state, bool space);


  std::vector<State> successor_states(int curr_l);

  double get_total_cost(double car_s, double ref_vel, vector<_prediction> trajectory, map<int, vector<prediction> > predictions, State state);
  std::vector<double> closest_vehicle(int lane, double s, std::vector<std::vector<double> > sensor);

public:
  State curr_s = State::KL;
  FSM();
  virtual ~FSM(){};
  State transition_function( int curr_lane, double s_frenet, double ego_speed, std::vector<std::vector<double> > sensor);
};

#endif  /*FSM_H*/
