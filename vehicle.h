#pragma once
#ifndef VEHICLE_H
#define VEHICLE_H

#include <string>

#define C_BLUE -16776961	// equipped CACC truck (non-platoon)
#define C_BLACK -16777216	// regular vehicles
#define C_YELLOW -10240		// platoon follower
#define C_RED -65536		// platoon leader
#define C_PINK -65426		// not used
#define C_WHITE -1			// platoon member

// ctrl_mode
#define SPD_CONTROL		100
#define GAP_CONTROL		101

// CACC_status
#define NON_CACC		200
#define CACC_OFF		201
#define CACC_LEADER		202
#define CACC_FOLLOWER	203
#define CACC_MEMBER		204

class vehicle
{

private:
	double lead_veh_distance;
	double prev_lead_veh_distance;

	

	long prev_leader_ID = -1; // leader_ID of the previous time step

public:
	double des_speed = 0; //Apoorba edit 03/14/17 
	// for keeping track of time spent following cacc-capable truck
	double time_following_cv = 0.0;
	void update_time_following_cv(); // called by update_lead_distance


	static double timestep;
	long ID;
	long veh_type;
	long veh_color;
	long leader_ID;
	long follower_ID;
	double cur_speed;
	//double orig_desired_speed = -1; // permanent for the driver
	double desired_speed_non_platoon = -1; // used in the calculation (could adopt the leader's desired speed) [m/s]
	double timegap_desired;
	double driver_desired_timegap =2;
	double timegap_cacc_following;
	double lead_veh_length;
	double acc;
	double des_acc;


	int ctrl_mode; // gap or speed regulation mode
	int CACC_status; // off or leader or follower
	int leader_CACC_status; // -1 is no leader
	int follower_CACC_status; // -1 is no follower

	// for lane change
	double lane_end_dist = -1;
	

	// keep track of time entering and exiting the network
	double entry_time;
	double exit_time;
	double t_lead = 0;
	double t_follow = 0;
	double t_member = 0;




	//Platoon formation
	long veh_lead_left_ID=-1;
	long veh_lead_right_ID=-1;

	//Lane Change 09/05/18
	long DummyLead = -1;
	long veh_lane = 0;
	// for lane change ttc calculation
	// first index: 0 = left, 1 = right
	// second index: 0 = front, 1 = rear
	double spacing[2][2] = { 1000,1000,1000,1000 };
	double spd_diff[2][2] = { 0,0,0,0 };
	double ttc[2][2] = { 99, 99, 99, 99 };
	double left_lead_veh_length = -1;
	double right_lead_veh_length = -1;

	long active_lane_change = 0; // lane change trigger 1=left, -1=right
	long rel_target_lane = 0;

	


	/*****************************************************************************************/
	/*Variables to get the distance travelled by the vehicle on the current link**************/
	/*****************************************************************************************/
	long nextLink = -1;// next link of the vehicle 

	//double distance_in_network; //odometer reading of total distance in the network
	//double distance_currentlink; // total distance spend on the current link
	//	long origin_Link;// 
	//	double distance_previous_Links;
	//	int i; 
	//	long tempLink;

	vehicle() {};
	vehicle(long ID_, long color_) : 
		ID(ID_), veh_color(color_), leader_ID(-1), follower_ID(-1), lead_veh_distance(-1), prev_lead_veh_distance(-1), 
		timegap_desired(2.0), timegap_cacc_following(1.0), ctrl_mode(SPD_CONTROL), CACC_status(CACC_OFF), leader_CACC_status(-1), follower_CACC_status(-1){}

	void update_lead_distance(double lead_dist); // copy existing lead_veh_distance & update with new value
	void update_veh_stats(); // update individual vehicle stats

	double calculate_acc(double sim_timestep,int link_no, int veh_lane_);
	double get_lead_dist();
	double get_prev_lead_dist();

	// get the time spent following CACC-equipped vehicle before activating platooning
	double get_time_following_cv();

	//get the distance travelled on the current link
	//double get_dist_currentLink(); 

	void reinitialize();


	long platoon_pos;

};

//
//std::string str_ctrl_mode(int ctrl_mode_) {
//	std::string str_out;
//	switch (ctrl_mode_) {
//	case SPD_CONTROL:
//		str_out = "Spd_Control";
//		break;
//	case GAP_CONTROL:
//		str_out = "Gap_Control";
//		break;
//	}
//	return str_out;
//}
//
//std::string str_CACC_status(int CACC_status_) {
//	std::string str_out;
//	switch (CACC_status_) {
//	case NON_CACC:
//		str_out = "Unequipped";
//		break;
//	case CACC_OFF:
//		str_out = "CACC_Off";
//		break;
//	case CACC_LEADER:
//		str_out = "CACC_Leader";
//		break;
//	case CACC_FOLLOWER:
//		str_out = "CACC_Follower";
//		break;
//	}
//	return str_out;
//}

#endif