#pragma once
#include "vehicle.h"
#include <memory>
#include <map>
#include "vehicle.h"

typedef std::shared_ptr<vehicle> vehicle_ptr;
typedef std::map<long, vehicle_ptr> map_vehs;

#define LEFT  1
#define RIGHT -1
class Lane_change
{

public:
	void initiate_lane_change(int veh_ID, int Target_lane_pos, double veh_length, map_vehs &map_vehicles);
	Lane_change();
	~Lane_change();
};

