#include "vehicle.h"
#include <cmath>
#include <iostream>

double vehicle::timestep = 0.1;

void vehicle::update_lead_distance(double lead_dist) {
	prev_lead_veh_distance = lead_veh_distance; // copy to current value
	if (leader_ID != -1) {		
		lead_veh_distance = lead_dist - lead_veh_length; // update with new value
	} 
	else {
		// no leader, default lead_veh_distance to -1
		lead_veh_distance = -1;
	}
	update_time_following_cv();
}

double vehicle::get_lead_dist() {
	return lead_veh_distance;
}

double vehicle::get_prev_lead_dist() {
	return prev_lead_veh_distance;
}

double vehicle::calculate_acc(double sim_timestep, int link_no, int veh_lane_) {

	
	//AB - Assigning a fixed desired speed to CACC equipped vehicles 
	if (CACC_status == CACC_FOLLOWER || CACC_status == CACC_MEMBER) {
		 des_speed = 31.29;
	}
	else if(CACC_status == CACC_LEADER){
		//des_speed = 29.0576;   // Leader should behave like a manual truck (David,Kevin,Mike) - 10/27/17

		des_speed=desired_speed_non_platoon;
	}
	else {
		des_speed = desired_speed_non_platoon;
	}
	double output_acc;

	// acceleration during gap regulation is calculable only 
	// when there is a leader at least one time step (lead_veh_distance != -1 & prev_lead_veh_distance != -1)
	bool is_calculable = (lead_veh_distance != -1) && (prev_lead_veh_distance != -1);
	double speed_error = cur_speed - des_speed;
	double asc = -0.4*speed_error;
	
	//Mike: Assume CACC has no braking limit- 10/27/17
	//if (asc < -3.4){
	//	asc = -3.4; //The acceleration required is greater that the CACC capabilities, Use -3.4
	//}
	if (asc > 2)
		asc = 2;


	
	// control mode determination -- no change if not either

	if ((leader_ID == -1) || (lead_veh_distance > 120)) {
		//330 is the SSD for a comfortable acceleration of 3.4 m/s2 and freeway speed of 75 MPH
		// speed regulation if spacing > 120 m or no leader (leader_ID == -1)
		ctrl_mode = SPD_CONTROL;
	}
	
	if ((lead_veh_distance < 100) && is_calculable) {
		// 300 is choosen arbitrarily based on the 330 value 
		// gap regulation if spacing < 100 m 
		// calculable if both lead_veh_distance and prev_lead_veh_distance are available

		ctrl_mode = GAP_CONTROL;
	}



	if (ctrl_mode == SPD_CONTROL) {
		// speed regulation if spacing > 120 m or no leader (leader_ID == -1)
		output_acc = asc;
		// CACC vehicle in speed control mode can be either CACC_LEADER or CACC_OFF
			
	}

	if (ctrl_mode == GAP_CONTROL) {
		// gap regulation if spacing < 100 m and both lead_veh_distance and prev_lead_veh_distance are available

		// choosing timegap_desired based on CACC status
		if (CACC_status == CACC_FOLLOWER|| CACC_status == CACC_MEMBER) {
			timegap_desired = timegap_cacc_following;
			//CACC_status = CACC_FOLLOWER;
		}
		else
		{//Check if the vehicle in on a link containing the ramps. Increase the time gap (Edit-03/14/2017) 
			//Not using this edit- 10/27/17 
			if (veh_lane_==1&& ((link_no >= 501 && link_no <= 600) || (link_no >= 801 && link_no <= 900))) {
					//	timegap_desired = 3;  //check this later
				timegap_desired = driver_desired_timegap;  //Edit-10/27/17
			}
			else {
				timegap_desired = driver_desired_timegap;
				// this vehicle can be CACC_LEADER of another platoon or CACC_OFF
			}
		}
		double desired_spacing = timegap_desired*cur_speed;
		if (cur_speed <= 11.17) { //11.17 m/s = 25 Mph
			desired_spacing = timegap_desired*cur_speed + 3; //take care of very low speed //Edit-10/27/17
		}
		double spacing_error = lead_veh_distance - desired_spacing;
		double spacing_change = (lead_veh_distance - prev_lead_veh_distance) / sim_timestep;
		double agap = spacing_change + 0.25*spacing_error;



	if (CACC_status == CACC_FOLLOWER || CACC_status == CACC_MEMBER) {
			//if (ID == 19) {
			//	std::cout << "desired spacing: " << desired_spacing << "; spacing_error: " << spacing_error << "; spacing change: " << spacing_change << "; agap: " << agap << std::endl;
			//}

			//if (agap > asc) {
			//	output_acc = asc;    //This is causing problem in platooning
			//}
			//else
			
			//Mike- Consider CACC can deccelerate at higher rate- 10/27/17
			 if (agap > 2) {
				output_acc = 2;
			}
			else {
				output_acc = agap;
			}

		}
	else { //CACC_LEADER
		   //Mike - Lead veh act like manual - 10/27/17
		output_acc = des_acc;
		if (agap > 2) {
			output_acc = 2;
		}
		
			//if (agap > asc) {
			//	output_acc = asc;
			//}
			//else if (agap < -3.4) {
			//	output_acc = -3.4;
			//	///The acceleration required is greater that the CACC capabilities, Use the value provided by VISSIM
			//	//CACC_status = CACC_OFF;
			//}
			//else {
			//	output_acc = agap;
			//}
	}
		
		// implement lane change following the leader when in gap control mode
	}


	// color control

	if (CACC_status == CACC_LEADER) {
		veh_color = C_RED;
	}
	else if (CACC_status == CACC_FOLLOWER) {
		// platoon follower status
		veh_color = C_YELLOW;
	}
	else if (CACC_status == CACC_MEMBER) {
		//platoon member status
		//veh_color = C_WHITE;
		//edit -03/09/17
		veh_color = C_YELLOW;
	}
	else {
		// CACC_status = CACC_OFF;
		veh_color = C_BLUE;
	}


	if (abs(output_acc) < 0.0001)
		output_acc = 0;

	acc = output_acc;
	return output_acc;
}

void vehicle::update_time_following_cv() {

	if ((leader_ID != -1) && (leader_ID == prev_leader_ID) && (lead_veh_distance < 100) &&
		((leader_CACC_status == CACC_OFF) || (leader_CACC_status == CACC_LEADER) || (leader_CACC_status==CACC_FOLLOWER)
		|| (leader_CACC_status == CACC_MEMBER)))
		// add to time counter
		time_following_cv += timestep;
	else
		// reset time counter
		time_following_cv = 0.0;
	
	prev_leader_ID = leader_ID;
}

double vehicle::get_time_following_cv() {
	return time_following_cv;
}

void vehicle::reinitialize()
{
	lead_veh_length = -1;
	left_lead_veh_length = -1;
	right_lead_veh_length = -1;
	veh_lead_right_ID = -1;
	veh_lead_left_ID = -1;
		for (auto i = 0; i < 2; i++) {
			for (auto j = 0; i < 2; i++) {
				spd_diff[i][j] = 0;
				ttc[i][j] = 99;
				spacing[i][j] = 1000;
			}
		}
}

void vehicle::update_veh_stats() {

	switch (CACC_status) {
	case CACC_LEADER:
		t_lead += timestep;
		break;
	case CACC_FOLLOWER:
		t_follow += timestep;
		break;
	case CACC_MEMBER:
		t_member += timestep;
	default:
		break;

	}

}










/*
double vehicle::get_dist_currentLink()
{
	if (i == 0)
	{
		origin_Link = currentLink;
		i++;
	}
	if (currentLink==origin_Link)
	{
		tempLink = currentLink;
		//current link is the starting link of the vehicle for this condition
		distance_currentlink = distance_in_network;
		distance_previous_Links = 0;
	}
	else
	{
		if (tempLink != currentLink)
		{
			tempLink = currentLink;
			distance_previous_Links = distance_in_network;
		}
		distance_currentlink = distance_in_network - distance_previous_Links; 
	}
	return distance_currentlink;
}

*/