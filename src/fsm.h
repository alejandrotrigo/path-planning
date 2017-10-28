#ifndef FSM_H
#define FSM_H

#include <fstream>
#include <math.h>
#include <iostream>
#include <vector>
#include <tuple>

const double MAX_ACCELERATION = 0.224;
const double TARGET_SPEED = 49.7;
const double SPEED_LIMIT = 50.0;

enum State {KL, PL, PR, TL, TR};

class FSM
{
private:
  std::vector<State> successor_states(int curr_l);
  double cost_function(State s, std::vector<int> predictions);
public:
  State curr_s = State::KL;
  double curr_speed = 0.0;
  struct collider
  {
      bool collision;
      int time;
  };
  FSM(){  };
  virtual ~FSM(){};
  State transition_function(std::vector<int> predictions, State curr_state, int curr_lane, std::vector<double> weights);
};

#endif  /*FSM_H*/
