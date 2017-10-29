#include "fsm.h"

FSM::FSM()
{

}

void delete_elem(std::vector<State> &v, State st)
{
  std::vector<State> aux;
  for(State s : v)
  {
    if (st == s) continue;
    else aux.push_back(s);
    v = aux;
  }
}

std::vector<State> FSM::successor_states(int curr_l)
{
  State s[] = {State::KL, State::PL, State::PR, State::TL, State::TR};
  //State s[] = {State::KL, State::TL, State::TR};
  std::vector<State> states (s, s + sizeof(s) / sizeof(State));


  if (curr_l == 0)
  {
    delete_elem(states, State::TL);
    delete_elem(states, State::PL);

  }
  if (curr_l == 2)
  {
    delete_elem(states, State::TR);
    delete_elem(states, State::PR);
  }

  if (this->curr_s == State::KL)
  {
    delete_elem(states, State::TL);
    delete_elem(states, State::TR);
  }

  if (this->curr_s == State::TL)
  {
    delete_elem(states, State::PR);
    delete_elem(states, State::TR);
    delete_elem(states, State::TL);
  }

  if (this->curr_s == State::TR)
  {
    delete_elem(states, State::PL);
    delete_elem(states, State::TL);
    delete_elem(states, State::TR);
  }

  if (this->curr_s == State::PL)
  {
    delete_elem(states, State::TR);
    delete_elem(states, State::PL);
  }
  if (this->curr_s == State::PR)
  {
    delete_elem(states, State::TL);
    delete_elem(states, State::PR);
  }

  return states;
}

std::vector<double> FSM::closest_vehicle(int lane, double s, std::vector<std::vector<double> > sensor)
{
  double dist = 10e99;
  double vel = 0.5;
  double v_s, v_d, v_v; // vehicle s pos, d pos and velocity
  int v_lane;
  double front = -1;

  for (unsigned int i = 0; i < sensor.size(); i++)
  {
    v_s = sensor[i][5];
    v_d = sensor[i][6];
    v_v = sqrt(pow(sensor[i][3], 2) + pow(sensor[i][4], 2));
    if (v_d < 4) v_lane = 0;
    else if (v_d < 8) v_lane = 1;
    else v_lane = 2;

    if (lane == v_lane)
    {
        if (v_s > s and (v_s -s) < dist) //in front
        {
          dist = v_s - s;
          vel = v_v;
          front = 1;
        }else if (s >= v_s and (s-v_s) < dist) // behind
        {
          dist = s - v_s;
          vel = v_v;
          front = -1;
        }
      }
   }
   if (fabs(dist) < 0.001) dist = 1;
   return {dist, vel, front};
}

double FSM::cost_function(State s, int curr_lane, double s_frenet, double ego_speed, std::vector<std::vector<double> > sensor)
{
  //Speed_cost

  //if (curr_speed < target_speed) cost = STOP_COST * ((target_speed - curr_speed / target_speed);
  //if (curr_speed > SPEED_LIMIT) cost = 1;
  //if (curr_speed > target_speed && curr_speed < speed_limit) cost = (curr_speed - target_speed) / (SPEED_LIMIT - target_speed);
  int target_lane;
  std::vector<double> closest_v;
  double cost = 0.0;
  double s_pos = s_frenet;

  if ((s == this->curr_s) && (ego_speed < TARGET_SPEED)) cost +=0.5;

  if (ego_speed == TARGET_SPEED)
  {
    if (s == (State::TL)) target_lane = curr_lane - 1;
    if (s == (State::TR)) target_lane = curr_lane + 1;
  }

  closest_v = closest_vehicle(target_lane, s_pos, sensor);
  double dist = closest_v[0];
  double vel = closest_v[1];
  bool front = closest_v[2] > 0;

  if ((front) && (dist > MIN_DIST) && (ego_speed > vel)) cost += 0;
  if ((front) && (dist < SECURITY_DIST) && (ego_speed > vel)) cost += 1;
  if ((front) && (dist > MIN_DIST) && (ego_speed < vel)) cost += 0.5;
  if ((front) && (dist < SECURITY_DIST) && (ego_speed < vel)) cost += 0.75;

  if ((!front) && (dist > MIN_DIST) && (ego_speed > vel)) cost += 0;
  if ((!front) && (dist < MIN_DIST) && (ego_speed > vel)) cost += 0.5;
  if ((!front) && (dist > MIN_DIST) && (ego_speed < vel)) cost += 0.75;
  if ((!front) && (dist < MIN_DIST) && (ego_speed < vel)) cost += 1;
  // TODO COSTE DE CADA CARRIL Y NO DE LA SITUACIÓN

  return cost;
}

State FSM::transition_function( int curr_lane, double s_frenet, double ego_speed, std::vector<std::vector<double> > sensor)
{
  std::vector<State> possible_successor_states = successor_states(curr_lane);
  std::vector<std::tuple<State, double> > costs;

  std::vector<double> closest_v = closest_vehicle(curr_lane, s_frenet, sensor);

  if ((closest_v[0] > SECURITY_DIST) || (closest_v[0] > SECURITY_DIST) && (closest_v[2] > 0))
  {
    this->curr_s = State::KL;
    return State::KL;
  }

  for (unsigned int i = 0; i < possible_successor_states.size(); i++)
  {
    State s = possible_successor_states[i];
    double cost_for_state = cost_function(s, curr_lane, s_frenet, ego_speed, sensor);
    auto tup = std::make_tuple(s,cost_for_state);
    costs.push_back(tup);
  }

  State best_s;
  double min_cost = 10e99;
  double cost = 0;

  for (std::tuple<State, double> t : costs)
  {
    cost = std::get<1>(t);
    std::cout << "Cost: " << cost << " state: " << std::get<0>(t) << std::endl;
    if (cost < min_cost)
    {
      min_cost = cost;
      best_s = std::get<0>(t);

    }
  }

  this->curr_s = best_s;
  return best_s;
}

/*
int main()
{
  std::vector< std::map<std::string, double> > j;
  std::map<std::string, double> first;
  first["x"] = .1;
  first["y"] = .2;
  first["z"] = .3;

  int lane = 0;
  j.push_back(first);

  std::vector<std::vector<double> > sensor = {{1.0, 2.0, 3.0, 4.0}, {2.0, 1.0, 4.0, 3.0}};
  FSM fsm;

  State final_state = fsm.transition_function(lane, j[0], sensor);

  std::cout << "state: " << final_state << std::endl;

}
*/
