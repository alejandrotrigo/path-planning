#include "path_planner.h"

Vehicle PathPlanner::find_by_id(std::vector<Vehicle> vehicles, int id)
{
	for (Vehicle v : vehicles)
		if (v[0] == id) return Vehicle;

	return NULL;
}

PathPlanner::PathPlanner()
{
	this->ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()); 
}

double PathPlanner::get_time()
{
	milliseconds t = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
	t = (double)(new_time - ms).count() / 1000;
	ms = new_time;
	return diff;
}

void PathPlanner::update_vehicle_state(std::vector<std::vector<double> >sensor)
{
	double t = get_time();

	for (unsigned int i = 0; i < sensor.size(); i++)
	{
		std::vector data = sensor[i];
		Vehicle v = NULL;
		if ((data[5] <= MAX_S) && (data[6] >= 0))
		{
			if((v = find_by_id(this->vehicles, data[0])) == NULL)
			{
				v = new Vehicle(data[0], data[1], data[2], data[3], data[4], data[5], data[6], t);
				vehicles.push_back(v);
			}
			else
			{
				v.update_other(data[1], data[2], data[3], data[4], data[5], data[6], t)
			}
		} 
	}
}