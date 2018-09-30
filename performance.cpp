#include "performance.h"

void Performance::update_values(map_vehs &map_vehicles, double &sim_time) {
	// call only when needed to calculate performance measures
	// updating values for vehicles currently in the network 
	
	// reset values
	n_net_veh = 0;
	t_platoon_net_veh = 0;
	total_t_net_veh = 0;
	n_net_platoon_veh = 0;

	n_net_veh = map_vehicles.size();
	for (auto it = map_vehicles.begin(); it != map_vehicles.end(); ++it) {
		auto veh_ptr = it->second;
		t_platoon_net_veh += (veh_ptr->t_lead + veh_ptr->t_follow + veh_ptr->t_member) ;
		total_t_net_veh += (sim_time - veh_ptr->entry_time);
		if (veh_ptr->CACC_status == CACC_LEADER || veh_ptr->CACC_status == CACC_FOLLOWER || veh_ptr->CACC_status== CACC_MEMBER)
			n_net_platoon_veh += 1;
	}
}

void Performance::calculate(map_vehs &map_vehicles, double &sim_time) {
	update_values(map_vehicles, sim_time);
	connected_trucks_in_network = n_net_veh;
	platooning_trucks_in_network = n_net_platoon_veh;
	pct_platooning_in_network = (double)n_net_platoon_veh/(double)n_net_veh*100; // % of trucks in currently platooning mode
	pct_time_platooning_in_network = t_platoon_net_veh/total_t_net_veh*100; // % of time in platooning mode averaged for connected trucks currently in network

	all_connected_trucks = n_net_veh + n_exit_veh; // total connected trucks generated (including those already left)
	// % of time in platooning mode averaged for all connected trucks generated
	pct_time_all_platooning = (t_platoon_net_veh+t_platoon_exit_veh)/(total_t_net_veh+total_t_exit_veh)*100; 

	
}

long Performance::platoon_len(map_vehs & map_vehicles, long Id)
{	
	long ix;
				ix = 2;
				bool err = false;
				long Buffer_ID = Id;
				while (err == false && (map_vehicles[Buffer_ID]->leader_CACC_status != CACC_LEADER)) {
							if (map_vehicles[Buffer_ID]->leader_ID == -1 || map_vehicles[Buffer_ID]->leader_CACC_status != CACC_MEMBER)
											err = true;
							
							if (err == false) {
								++ix;
								Buffer_ID = map_vehicles[Buffer_ID]->leader_ID;
							}
										}
				return ix;
}


void Performance::update_values_at_exit(vehicle_ptr &veh_ptr) {
	
	// call at the KILL driver
	n_exit_veh += 1;
	t_platoon_exit_veh += (veh_ptr->t_lead + veh_ptr->t_follow);
	total_t_exit_veh += (veh_ptr->exit_time - veh_ptr->entry_time);
}

