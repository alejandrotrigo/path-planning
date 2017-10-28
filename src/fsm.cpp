#include "fsm.h"

void delete_elem(std::vector<State> v, State st)
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
  }

  if (this->curr_s == State::TR)
  {
    delete_elem(states, State::PL);
    delete_elem(states, State::TL);
  }

  if (this->curr_s == State::PL) delete_elem(states, State::TR);
  if (this->curr_s == State::PR) delete_elem(states, State::TL);

  return states;
}

vector<int> FSM::state_at(std::vector<double> j, int lane, int t)
{
  int s = j["s"] + j["v"] * t + j["a"] * t * t/2;
  int v = j["v"] + j["a"] * t;
  return {lane, s, v, j["a"]}
}

bool FSM::collides_with(std::vector<double>j, std::vector<double> sensor, int at_time)
{
  vector<int> check1 = state_at(j, lane, at_time);
  vector<int> check2 = state_at(sensor, lane, at_time);
  return (check1[0] == check2[0]) && (abs(check1[1] - check2[1]) <= 2);
}

FSM::collider FSM::will_collide_with(std::vector<double> j, std::vector<double> sensor, int lane, int time)
{
    FSM::collider collider_temp;
    collider_tem.collision = false;
    collider_temp.time = -1;

    for (unsigned int i = 0; i<time+1; i++)
    {
      if(collides_with(other, i))
      {
        collider_temp.collision = true;
        collider_temp.time = t;
        return collider_temp;
      }
    }

    return collider_temp;
}

double FSM::cost_function(State s, std::vector<int> predictions)
{
  //Speed_cost

  if (curr_speed < target_speed) cost = STOP_COST * ((target_speed - curr_speed / target_speed);
  if (curr_speed > SPEED_LIMIT) cost = 1;
  if (curr_speed > target_speed && curr_speed < speed_limit) cost = (curr_speed - target_speed) / (SPEED_LIMIT - target_speed);


  return 0.0;
}

double FSM::get_weights(State s, double speed, double acceleration)
{
  
}

State FSM::transition_function(std::vector<int> predictions, State curr_state, int curr_lane, std::vector<double> weights)
{
  std::vector<State> possible_successor_states = successor_states(curr_lane);
  std::vector<std::tuple<State, double> > costs;

  for (State s : possible_successor_states)
  {
    double weight = weights[0];
    double cost_for_state = weight * cost_function(s, predictions);
    auto tup = std::make_tuple(s,cost_for_state);
    costs.push_back(tup);
  }

  State best_s;
  double min_cost = 10e99;
  double cost = 0;

  for (std::tuple<State, double> t : costs)
  {
    if (std::get<1>(t) < min_cost)
    {
      min_cost = cost;
      best_s = std::get<0>(t);
    }
  }

  return best_s;
}

//////////////////////////////777////////////////////////TODO

void Vehicle::realize_state(map<int,vector < vector<int> > > predictions) {

	/*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
    string state = this->state;
    if(state.compare("CS") == 0)
    {
    	realize_constant_speed();
    }
    else if(state.compare("KL") == 0)
    {
    	realize_keep_lane(predictions);
    }
    else if(state.compare("LCL") == 0)
    {
    	realize_lane_change(predictions, "L");
    }
    else if(state.compare("LCR") == 0)
    {
    	realize_lane_change(predictions, "R");
    }
    else if(state.compare("PLCL") == 0)
    {
    	realize_prep_lane_change(predictions, "L");
    }
    else if(state.compare("PLCR") == 0)
    {
    	realize_prep_lane_change(predictions, "R");
    }

}

void Vehicle::realize_constant_speed() {
	a = 0;
}

int Vehicle::_max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s) {

	int delta_v_til_target = target_speed - v;
    int max_acc = min(max_acceleration, delta_v_til_target);

    map<int, vector<vector<int> > >::iterator it = predictions.begin();
    vector<vector<vector<int> > > in_front;
    while(it != predictions.end())
    {

    	int v_id = it->first;

        vector<vector<int> > v = it->second;

        if((v[0][0] == lane) && (v[0][1] > s))
        {
        	in_front.push_back(v);

        }
        it++;
    }

    if(in_front.size() > 0)
    {
    	int min_s = 1000;
    	vector<vector<int>> leading = {};
    	for(int i = 0; i < in_front.size(); i++)
    	{
    		if((in_front[i][0][1]-s) < min_s)
    		{
    			min_s = (in_front[i][0][1]-s);
    			leading = in_front[i];
    		}
    	}

    	int next_pos = leading[1][1];
    	int my_next = s + this->v;
    	int separation_next = next_pos - my_next;
    	int available_room = separation_next - preferred_buffer;
    	max_acc = min(max_acc, available_room);
    }

    return max_acc;

}

void Vehicle::realize_keep_lane(map<int,vector< vector<int> > > predictions) {
	this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int,vector< vector<int> > > predictions, string direction) {
	int delta = -1;
    if (direction.compare("L") == 0)
    {
    	delta = 1;
    }
    this->lane += delta;
    int lane = this->lane;
    int s = this->s;
    this->a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_prep_lane_change(map<int,vector<vector<int> > > predictions, string direction) {
	int delta = -1;
    if (direction.compare("L") == 0)
    {
    	delta = 1;
    }
    int lane = this->lane + delta;

    map<int, vector<vector<int> > >::iterator it = predictions.begin();
    vector<vector<vector<int> > > at_behind;
    while(it != predictions.end())
    {
    	int v_id = it->first;
        vector<vector<int> > v = it->second;

        if((v[0][0] == lane) && (v[0][1] <= this->s))
        {
        	at_behind.push_back(v);

        }
        it++;
    }
    if(at_behind.size() > 0)
    {

    	int max_s = -1000;
    	vector<vector<int> > nearest_behind = {};
    	for(int i = 0; i < at_behind.size(); i++)
    	{
    		if((at_behind[i][0][1]) > max_s)
    		{
    			max_s = at_behind[i][0][1];
    			nearest_behind = at_behind[i];
    		}
    	}
    	int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
    	int delta_v = this->v - target_vel;
    	int delta_s = this->s - nearest_behind[0][1];
    	if(delta_v != 0)
    	{

    		int time = -2 * delta_s/delta_v;
    		int a;
    		if (time == 0)
    		{
    			a = this->a;
    		}
    		else
    		{
    			a = delta_v/time;
    		}
    		if(a > this->max_acceleration)
    		{
    			a = this->max_acceleration;
    		}
    		if(a < -this->max_acceleration)
    		{
    			a = -this->max_acceleration;
    		}
    		this->a = a;
    	}
    	else
    	{
    		int my_min_acc = max(-this->max_acceleration,-delta_s);
    		this->a = my_min_acc;
    	}

    }

}









int main()
{
  std::cout << "LISTO " << std::endl;
}
