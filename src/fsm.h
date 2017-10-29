#ifndef FSM_H
#define FSM_H

#include <fstream>
#include <math.h>
#include <iostream>
#include <vector>
#include <tuple>
#include <map>

const double MAX_ACCELERATION = 0.224;
const double TARGET_SPEED = 49.7;
const double SPEED_LIMIT = 50.0;
const double SECURITY_DIST = 20.0;
const double MIN_DIST = SECURITY_DIST/2;

enum State {KL, PL, PR, TL, TR};

class FSM
{
private:
  std::vector<State> successor_states(int curr_l);
  double cost_function(State s, int curr_lane, double s_frenet, double ego_speed, std::vector<std::vector<double> > sensor);
  std::vector<double> closest_vehicle(int lane, double s, std::vector<std::vector<double> > sensor);

public:
  State curr_s = State::KL;
  FSM();
  virtual ~FSM(){};
  State transition_function( int curr_lane, double s_frenet, double ego_speed, std::vector<std::vector<double> > sensor);
};

#endif  /*FSM_H*/
