#pragma once
#ifndef PERFORMANCE_H
#define PERFORMANCE_H

#include <memory>
#include <map>
#include "vehicle.h"
#include<string>

typedef std::shared_ptr<vehicle> vehicle_ptr;
typedef std::map<long, vehicle_ptr> map_vehs;

class Performance
{
private:
	double timestep = 0.1;
	long n_net_veh = 0; // count of trucks currently in the network
	long n_net_platoon_veh = 0; // count of trucks currently in platooning mode
	long n_exit_veh = 0; // count of trucks already left the network
	double t_platoon_net_veh = 0; // total platoon time of vehicles currently in the network
	double t_platoon_exit_veh = 0; // total platoon time of exited vehicles
	double total_t_net_veh = 0; // total time of vehicles in network
	double total_t_exit_veh = 0; // total time of vehicles of exited vehicles

	void update_values(map_vehs &map_vehicles, double &sim_time); // call by "calculate" to update member variable values for vehicles in network

public:

	Performance() {}
	Performance(double timestep_) : timestep(timestep_) {}

	void update_values_at_exit(vehicle_ptr &veh_ptr); // log the value of exiting vehicle before erasing it
	void calculate(map_vehs &map_vehicles, double &sim_time); // call to calculate performance measures
	long platoon_len(map_vehs &map_vehicles, long Id); // calculate the patoon lengths and output it 




	// statistics
	long connected_trucks_in_network; 
	long platooning_trucks_in_network;
	double pct_platooning_in_network; // % of trucks in currently platooning mode
	double pct_time_platooning_in_network; // % of time in platooning mode averaged for connected trucks currently in network

	long all_connected_trucks;
	double pct_time_all_platooning; // % of time in platooning mode averaged for all connected trucks generated


	std::string Ti = "Title";// for outputinng the title
	std::string Ti1 = "Title";// for outputinng the title
	std::string Ti2 = "Title";// for outputinng the title
};

#endif