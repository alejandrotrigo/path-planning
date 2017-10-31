
#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <chrono>

#include "vehicle.h"
#include "fsm.h"

class PathPlanner
{
public:
	PathPlanner();
	virtual ~PathPlanner(){};

	void update_vehicles_state(std::vector<std::vector<double> > sensor);
	void update_ego_state(double car_s, double x, double y, double yaw, double s, double d, double speed);

private:

	std::vector <Vehicle> vehicles;
	map<int, vector<prediction> > predictions;
	Vehicle ego = Vehicle(-1);
	FSM fsm = fsm(ego);

	double get_time();
};

#endif /*PATH_PLANNER_H*/