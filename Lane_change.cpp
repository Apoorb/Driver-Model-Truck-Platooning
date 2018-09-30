#include "Lane_change.h"
#include<iostream>
using namespace std;


void Lane_change::initiate_lane_change(int veh_ID, int Target_lane_pos, double veh_length, map_vehs &map_vehicles) {
	if (Target_lane_pos == -1) {
		// There is a CACC equipped vehicle on the right lane 
		// check right front 
		bool is_ego_front_spacing_enough = false;
		is_ego_front_spacing_enough = (map_vehicles[veh_ID]->spacing[1][0] - map_vehicles[veh_ID]->right_lead_veh_length) > 20; // at least 20m clear at the front
		bool is_ego_front_ttc_enough = false;
		if (map_vehicles[veh_ID]->spd_diff[1][0] <= 0)
			is_ego_front_ttc_enough = true;
		else {
			map_vehicles[veh_ID]->ttc[1][0] = (map_vehicles[veh_ID]->spacing[1][0] - map_vehicles[veh_ID]->right_lead_veh_length) / map_vehicles[veh_ID]->spd_diff[1][0];
			is_ego_front_ttc_enough = map_vehicles[veh_ID]->ttc[1][0] > 5.0; // at least 5 sec front TTC
		}
		// check right rear (rear spacing is negative)
		bool is_ego_rear_spacing_enough = false;
		is_ego_rear_spacing_enough = (abs(map_vehicles[veh_ID]->spacing[1][1]) - veh_length) > 20; // at least 20m clear at the rear
		bool is_ego_rear_ttc_enough = false;
		if (map_vehicles[veh_ID]->spd_diff[1][1] >= 0)
			is_ego_rear_ttc_enough = true;
		else {
			map_vehicles[veh_ID]->ttc[1][1] = -1 * (abs(map_vehicles[veh_ID]->spacing[1][1]) - veh_length) / map_vehicles[veh_ID]->spd_diff[1][1];
			is_ego_rear_ttc_enough = map_vehicles[veh_ID]->ttc[1][1] > 5.0; // at least 5 sec rear TTC
		}
		// change if all pass
		if (is_ego_front_spacing_enough && is_ego_front_ttc_enough && is_ego_rear_spacing_enough && is_ego_rear_ttc_enough){
			map_vehicles[veh_ID]->active_lane_change = -1;
			//cout <<"VEh Id "<<veh_ID <<"F Space " << is_ego_front_spacing_enough << "F TTC " << is_ego_front_ttc_enough << "Rear S" << is_ego_rear_spacing_enough << "Rear ttc" << is_ego_rear_ttc_enough << endl;

		}
	}


	else {
		// There is a CACC equipped vehicle on the left lane 
		// check left front 
		bool is_ego_front_spacing_enough = false;
		is_ego_front_spacing_enough = ((map_vehicles[veh_ID]->spacing[0][0] - map_vehicles[veh_ID]->left_lead_veh_length) > 20); // at least 12m clear at the front
		bool is_ego_front_ttc_enough = false;
		if (map_vehicles[veh_ID]->spd_diff[0][0] <= 0) {
			is_ego_front_ttc_enough = true;
		}
		else {
			map_vehicles[veh_ID]->ttc[0][0] = (map_vehicles[veh_ID]->spacing[0][0] - map_vehicles[veh_ID]->left_lead_veh_length) / map_vehicles[veh_ID]->spd_diff[0][0];
			is_ego_front_ttc_enough = map_vehicles[veh_ID]->ttc[0][0] > 5.0; // at least 5 sec front TTC
		}
		// check left rear (rear spacing is negative)
		bool is_ego_rear_spacing_enough = false;
		is_ego_rear_spacing_enough = (abs(map_vehicles[veh_ID]->spacing[0][1]) - veh_length) > 20; // at least 12m clear at the rear
		bool is_ego_rear_ttc_enough = false;
		if (map_vehicles[veh_ID]->spd_diff[0][1] >= 0)
			is_ego_rear_ttc_enough = true;
		else {
			map_vehicles[veh_ID]->ttc[0][1] = -1 * (abs(map_vehicles[veh_ID]->spacing[0][1]) - veh_length) / map_vehicles[veh_ID]->spd_diff[0][1];
			is_ego_rear_ttc_enough = map_vehicles[veh_ID]->ttc[0][1] > 5.0; // at least 5 sec rear TTC
		}
		// change if all pass
		if (is_ego_front_spacing_enough && is_ego_front_ttc_enough && is_ego_rear_spacing_enough && is_ego_rear_ttc_enough){
			map_vehicles[veh_ID]->active_lane_change = 1;
			//cout << "VEh Id " << veh_ID << "F Space " << is_ego_front_spacing_enough << "F TTC " << is_ego_front_ttc_enough << "Rear S" << is_ego_rear_spacing_enough << "Rear ttc" << is_ego_rear_ttc_enough << endl;
		}
	}
 }




Lane_change::Lane_change()
{
}


Lane_change::~Lane_change()
{
}
