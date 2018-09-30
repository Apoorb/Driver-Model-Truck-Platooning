/*==========================================================================*/
/*  DriverModel.cpp                                  DLL Module for VISSIM  */
/*                                                                          */
/* Praprut Songchitruksa													*/		
/* Created October 4, 2015													*/		
/*Last Update  03/20/2017 -- Apoorba Bibeka										*/					
/* Note: DLL units are in metric (1 m = 3.28 ft)							*/		
/*==========================================================================*/



#include <stdio.h>
#include <iostream>
#include<fstream>
#include <iomanip>
#include <map>
#include <set>
#include <memory>
#include "DriverModel.h"
#include "vehicle.h"
#include "settings.h"
#include "performance.h"
#include <direct.h>
#include"Lane_change.h"
using namespace std;
        
// sleep
#include <chrono>
#include <thread>

//Nakagami dist
#include "nakagami.h"
#include <random>
/*==========================================================================*/

double rel_spd_front=-99;




// longitudinal control output
double  desired_acceleration = 0.0;

//double  desired_speed     = 0.0;
long    turning_indicator    = 0;
long    veh_color        = RGB(0,0,0);

// PS CACC modifications


settings cfg; // .json config file
char *config_file;

vehicle veh; // vehicle object
typedef std::shared_ptr<vehicle> vehicle_ptr;



double sim_time = -1;
double sim_timestep = -1;
long veh_ID = -1;
long lead_veh_ID = -1;
long follow_veh_ID = -1;
double lead_veh_distance = -1; // meters
double follow_veh_distance = -1; // meters
double desired_speed = -1; // [m/s]
double cur_speed = -1; // current speed
double cur_acc = 0.0; // current acceleration
double lead_veh_length = -1;
double veh_length = -1;

double processed_time = 0.0;

map<long, vehicle_ptr> map_vehicles;
set<long> set_cv_IDs;

// CACC parameters
const double timegap_desired = 1.0; // sec

//link no
int veh_link_no=0; 
// lane change control
long veh_lane = 0; // 1 is rightmost lane
double  desired_lane_angle = 0.0;

long veh_active_lane_change = 0; // lane change status
long veh_rel_target_lane = 0;
double veh_lane_angle = 0.0;

long no_lanes = 0;




std::string path; // Path of the file
std::string platoon_len_file; //Path for platoon length file
std::string perf_mes; // Path for performance meausre file 
std::string moves_process; // Path for emission output
// performance measures
typedef std::shared_ptr<Performance> pm_ptr;
pm_ptr pm;

std::shared_ptr<Lane_change> lane_change_ptr;

double odometer;



struct moves {

	long V_ID;
	double sim_t;
	double c_spd;
	double c_acc;
	long sta;
	double frn_gap;
	double re_gap;
	long recpt;
};


vector<moves> m;
moves temp;
//**********************************************************************************
//code for cut-in. Bad hack! Try to make the code better sometime
bool is_first_time=TRUE;

//###################################################################################################
//Modifying the code to model manual vehicles along with CACC vehicles
//###################################################################################################

int lead_left_ID =-1;
int lead_right_ID = -1;
double spacing[2][2] = { 1000,1000,1000,1000 };
double spd_diff[2][2] = { 0,0,0,0 };
double ttc[2][2] = { 99, 99, 99, 99 };
double left_lead_veh_length = -1;
double right_lead_veh_length = -1;
long veh_type=-1;  //201 are CACC equipped trucks 
long active_lane_change = 0; // lane change status
long rel_target_lane = 0;
//###################################################################################################


/*****************************************************************************************/
/*Apoorba edit - 03/24/17 Entrance ramp no aux lane -  Lane change                         */
/*****************************************************************************************/
vector<pair<double, bool>> is_veh_present_on_ramp = { {0,0}, {0,0} };
//check if we are in the the first iteration of the next time step
double next_time_step = 0;
bool is_next_time_step = false;


/*==========================================================================*/

BOOL APIENTRY DllMain (HANDLE  hModule, 
                       DWORD   ul_reason_for_call, 
                       LPVOID  lpReserved)
{
  switch (ul_reason_for_call) {
      case DLL_PROCESS_ATTACH:
      case DLL_THREAD_ATTACH:
      case DLL_THREAD_DETACH:
      case DLL_PROCESS_DETACH:
         break;
  }
  return TRUE;
}

/*==========================================================================*/

DRIVERMODEL_API  int  DriverModelSetValue (long   type, 
                                           long   index1,
                                           long   index2,
                                           long   long_value,
                                           double double_value,
                                           char   *string_value)
{
  /* Sets the value of a data object of type <type>, selected by <index1> */
  /* and possibly <index2>, to <long_value>, <double_value> or            */
  /* <*string_value> (object and value selection depending on <type>).    */
  /* Return value is 1 on success, otherwise 0.                           */

  switch (type) {
    case DRIVER_DATA_PATH                   :
	case DRIVER_DATA_PARAMETERFILE:
		config_file = string_value;
		return 1;
    case DRIVER_DATA_TIMESTEP               :
		sim_timestep = double_value;
		return 1;
    case DRIVER_DATA_TIME                   :
		sim_time = double_value;
		return 1;
    case DRIVER_DATA_VEH_ID                 :
		veh_ID = long_value;
		return 1;
    case DRIVER_DATA_VEH_LANE               :
		veh_lane = long_value;
		return 1;
    case DRIVER_DATA_VEH_ODOMETER           :
		odometer = double_value;
		return 1; 
    case DRIVER_DATA_VEH_LANE_ANGLE         :
		veh_lane_angle = double_value;
		return 1;
    case DRIVER_DATA_VEH_LATERAL_POSITION   :
    case DRIVER_DATA_VEH_VELOCITY           :
		cur_speed = double_value;
		return 1;
    case DRIVER_DATA_VEH_ACCELERATION       :
		cur_acc = double_value;
		return 1;
    case DRIVER_DATA_VEH_LENGTH             :
		veh_length = double_value;
		return 1;
    case DRIVER_DATA_VEH_WIDTH              :
    case DRIVER_DATA_VEH_WEIGHT             :
    case DRIVER_DATA_VEH_MAX_ACCELERATION   :
      return 1;
    case DRIVER_DATA_VEH_TURNING_INDICATOR  :
      turning_indicator = long_value;
      return 1;
    case DRIVER_DATA_VEH_CATEGORY           :
    case DRIVER_DATA_VEH_PREFERRED_REL_LANE :
    case DRIVER_DATA_VEH_USE_PREFERRED_LANE :
      return 1;
    case DRIVER_DATA_VEH_DESIRED_VELOCITY   :
	{
		desired_speed = double_value;
		return 1;
	}
    case DRIVER_DATA_VEH_X_COORDINATE       :
    case DRIVER_DATA_VEH_Y_COORDINATE       :
    case DRIVER_DATA_VEH_TYPE               :
		veh_type = long_value;
      return 1;
    case DRIVER_DATA_VEH_COLOR              :
      veh_color = long_value;
      return 1;
    case DRIVER_DATA_VEH_CURRENT_LINK       :
		veh_link_no = long_value;
      return 1; /* (To avoid getting sent lots of DRIVER_DATA_VEH_NEXT_LINKS messages) */
                /* Must return 1 if these messages are to be sent from VISSIM!         */
    case DRIVER_DATA_VEH_NEXT_LINKS         :
    case DRIVER_DATA_VEH_ACTIVE_LANE_CHANGE :
		veh_active_lane_change = long_value;
		return 1;
    case DRIVER_DATA_VEH_REL_TARGET_LANE    :
		veh_rel_target_lane = long_value;
		return 1;
    case DRIVER_DATA_NVEH_ID                :
	{
		if ((index1 == 0) & (index2 == 1)) {
			// get ID of the leader
			lead_veh_ID = long_value;
		}
		if ((index1 == 0) & (index2 == -1)) {
			// get ID of the follower
			follow_veh_ID = long_value;
		}

		/* (index1 = relative lane: +2 = second to the left, +1 = next one to the left,   */
		/*                           0 = current lane,                                    */
		/*                          -1 = next one to the right, -2 = second to the right) */
		/* (index2 = relative position: positive = downstream (+1 next, +2 second next)   */
		/*                              negative = upstream (-1 next, -2 second next))    */
		
		if ((index1 == 1) & (index2 == 1)) {
			// get ID of the vehicle in the left lane downstream 
			lead_left_ID = long_value;
		}
		if ((index1 == -1) & (index2 == 1)) {
			// get ID of the vehicle in the right lane downstream 
			lead_right_ID = long_value;
		}
		return 1;
	}
    case DRIVER_DATA_NVEH_LANE_ANGLE        :
    case DRIVER_DATA_NVEH_LATERAL_POSITION  :
    case DRIVER_DATA_NVEH_DISTANCE          :
	{
		if ((index1 == 0) & (index2 == 1)) {
			// get distance to the front of the leader (defaults to -1 if no leader)
			lead_veh_distance = double_value; // meters
		}
		if ((index1 == 0) & (index2 == -1)) {
			// get distance from ego veh's front bumper to the front bumper of the follower
			follow_veh_distance = double_value; // meters
		}
		// for spacing and spd_diff
		// first index: 0 = left, 1 = right
		// second index: 0 = front, 1 = rear
		if (index1 == 1 && index2 == 1) {
			// left front
			spacing[0][0] = double_value;
		}
		if (index1 == 1 && index2 == -1) {
			// left rear
			spacing[0][1] = double_value;
		}
		if (index1 == -1 && index2 == 1) {
			// right front
			spacing[1][0] = double_value;
		}
		if (index1 == -1 && index2 == -1) {
			// right rear
			spacing[1][1] = double_value;
		}

		return 1;
	}
    case DRIVER_DATA_NVEH_REL_VELOCITY      :
	{

		if ((index1 == 0) & (index2 == 1)) {
			// get distance to the front of the leader (defaults to -1 if no leader)
			rel_spd_front = double_value; // meters
		}
		// veh_spd - n_veh_spd
		// first index: 0 = left, 1 = right
		// second index: 0 = front, 1 = rear
		if (index1 == 1 && index2 == 1) {
			// left front
			spd_diff[0][0] = double_value;
		}
		if (index1 == 1 && index2 == -1) {
			// left rear
			spd_diff[0][1] = double_value;
		}
		if (index1 == -1 && index2 == 1) {
			// right front
			spd_diff[1][0] = double_value;
		}
		if (index1 == -1 && index2 == -1) {
			// right rear
			spd_diff[1][1] = double_value;
		}
		return 1;
	}
    case DRIVER_DATA_NVEH_ACCELERATION      :
    case DRIVER_DATA_NVEH_LENGTH            :
	{
		if ((index1 == 0) & (index2 == 1)) {
			// get the length of the leader (defaults to -1 if no leader)
			lead_veh_length = double_value; // meters
		}
		if (index1 == 1 && index2 == 1) {
			// left front
			left_lead_veh_length = double_value;
		}
		if (index1 == -1 && index2 == 1) {
			// right front
			right_lead_veh_length = double_value;
		}
		return 1;
	}
    case DRIVER_DATA_NVEH_WIDTH             :
    case DRIVER_DATA_NVEH_WEIGHT            :
    case DRIVER_DATA_NVEH_TURNING_INDICATOR :
    case DRIVER_DATA_NVEH_CATEGORY          :
    case DRIVER_DATA_NVEH_LANE_CHANGE       :
    case DRIVER_DATA_NO_OF_LANES            :
		no_lanes = long_value;
		return 1;

    case DRIVER_DATA_LANE_WIDTH             :
    case DRIVER_DATA_LANE_END_DISTANCE      :
	case DRIVER_DATA_RADIUS                 :
    case DRIVER_DATA_MIN_RADIUS             :
    case DRIVER_DATA_DIST_TO_MIN_RADIUS     :
    case DRIVER_DATA_SLOPE                  :
    case DRIVER_DATA_SLOPE_AHEAD            :
    case DRIVER_DATA_SIGNAL_DISTANCE        :
    case DRIVER_DATA_SIGNAL_STATE           :
    case DRIVER_DATA_SIGNAL_STATE_START     :
    case DRIVER_DATA_SPEED_LIMIT_DISTANCE   :
    case DRIVER_DATA_SPEED_LIMIT_VALUE      :
      return 1;
    case DRIVER_DATA_DESIRED_ACCELERATION :
      desired_acceleration = double_value;
      return 1;
    case DRIVER_DATA_DESIRED_LANE_ANGLE :
		desired_lane_angle = double_value;
	    return 1;
    case DRIVER_DATA_ACTIVE_LANE_CHANGE :
	//	if (map_vehicles[veh_ID]->CACC_status != CACC_FOLLOWER || map_vehicles[veh_ID]->CACC_status != CACC_MEMBER)
		active_lane_change = long_value;
      return 1;
    case DRIVER_DATA_REL_TARGET_LANE :
		rel_target_lane = long_value;
      return 1;
    default :
      return 0;
  }
}

/*--------------------------------------------------------------------------*/

DRIVERMODEL_API  int  DriverModelGetValue (long   type, 
                                           long   index1,
                                           long   index2,
                                           long   *long_value,
                                           double *double_value,
                                           char   **string_value)
{
  /* Gets the value of a data object of type <type>, selected by <index1> */
  /* and possibly <index2>, and writes that value to <*double_value>,     */
  /* <*float_value> or <**string_value> (object and value selection       */
  /* depending on <type>).                                                */
  /* Return value is 1 on success, otherwise 0.                           */

  switch (type) {
    case DRIVER_DATA_STATUS :
      *long_value = 0;
      return 1;
    case DRIVER_DATA_VEH_TURNING_INDICATOR :
      *long_value = turning_indicator;
      return 1;
    case DRIVER_DATA_VEH_DESIRED_VELOCITY   :
		*double_value = desired_speed; 
      return 1;
    case DRIVER_DATA_VEH_COLOR :
		*long_value = veh_color; 
      return 1;
    case DRIVER_DATA_WANTS_SUGGESTION :
      *long_value = 1;
      return 1;
    case DRIVER_DATA_DESIRED_ACCELERATION :
			*double_value = desired_acceleration;
     return 1;

    case DRIVER_DATA_DESIRED_LANE_ANGLE :
	{
		if (veh_active_lane_change == 0) {
			// no active lane change
			return 0; // no suggestion needed
		}

		*double_value = desired_lane_angle;
		return 1;
	}
    case DRIVER_DATA_ACTIVE_LANE_CHANGE :
	{

		// free lane change (no control otherwise)
		*long_value = active_lane_change;

		return 1;
	}

    case DRIVER_DATA_REL_TARGET_LANE :
      *long_value = rel_target_lane;
      return 1;
    case DRIVER_DATA_SIMPLE_LANECHANGE :
      *long_value = 1;
      return 1;
    default :
      return 0;
  }
}

/*==========================================================================*/

DRIVERMODEL_API  int  DriverModelExecuteCommand (long number)
{
  /* Executes the command <number> if that is available in the driver */
  /* module. Return value is 1 on success, otherwise 0.               */
  
	switch (number) {
	case DRIVER_COMMAND_INIT:
	{
		try {
			// redirect IO to console
			FILE *conin, *conout;
			AllocConsole();
			freopen_s(&conin, "conin$", "r", stdin);
			freopen_s(&conout, "conout$", "w", stdout);
			freopen_s(&conout, "conout$", "w", stderr);

			cout << "\n*****************************************\n" << endl;
			cout << "Connected Vehicle Platooning Simulation v 1.0" << endl;
			cout << "\n*****************************************\n" << endl;


			// load configuration for the DLL
			cout << endl << "Loading configuration file: " << config_file << endl;
			cfg.load(config_file);
			std::string processed_json = cfg.m_saved_cfg_file;
			cout << "Saving configuration file as: " << processed_json << endl;
			cfg.save(processed_json);

			cout << "CACC simulation starting..." << endl;
			cout << "Simulation time step size (sec): " << sim_timestep << " second" << endl;

			// create object for tracking performance measures
			pm = std::make_shared<Performance>(Performance(sim_timestep));

			//Create smart pointer for lane change 
			lane_change_ptr = std::make_shared<Lane_change>(Lane_change());


			//Make a directory for Performance measures
			const char * p = cfg.m_pm_output_dir.c_str();
			_mkdir(p);
			path = cfg.m_pm_output_dir;
			path += "\\";
			path += cfg.m_run_spec;

		}
		catch (...) {
			cout << "Exception: Unknown" << endl;
		}

		// set timestep for all vehicles
		vehicle::timestep = sim_timestep;

		return 1;
	}
	case DRIVER_COMMAND_CREATE_DRIVER:
	{
		// store a set of veh IDs for connected vehicles
		//Vehicles with iniial desired speed less than 74 Mph will be connected vehicles
		if (veh_ID != -1  &&(desired_speed*2.23694)<74) {

			// set for quick lookup
			set_cv_IDs.insert(veh_ID);

			// map for full info
			vehicle_ptr veh = std::make_shared<vehicle>(vehicle(veh_ID, veh_color));
			map_vehicles[veh_ID] = veh;

			// set timegap for cacc following mode for each vehicle
			map_vehicles[veh_ID]->timegap_cacc_following = cfg.m_desired_cacc_time_gap;

			// set desired speeds
			map_vehicles[veh_ID]->desired_speed_non_platoon = desired_speed;
			//map_vehicles[veh_ID]->orig_desired_speed = desired_speed;

			// set entry time
			map_vehicles[veh_ID]->entry_time = sim_time;
			//	map_vehicles[veh_ID]->i = 0;// Helps finding the orgin link for routing decisions


			//set desired timegap for the driver 

		//	random_device device;
		//	mt19937 generator(device());
		//  std::normal_distribution<double> ndistribution(2, 0.4);
		//	map_vehicles[veh_ID]->driver_desired_timegap= ndistribution(generator);
			map_vehicles[veh_ID]->driver_desired_timegap = 1.2;
			
			//cout << "Time: " << sim_time << "; New Veh ID: " << veh_ID <<"Desired Speed"<<desired_speed*2.23694 << "; Connected Trucks in Network: " << set_cv_IDs.size() << endl;

		}

		return 1;
	}
	case DRIVER_COMMAND_KILL_DRIVER:
	{
		//Set_cv_IDs contains set of all connected vehicles 
		if (set_cv_IDs.find(veh_ID) != set_cv_IDs.end()) {
		// store exit time for the vehicle
		// cout << "Veh_ID " << veh_ID << " exiting at time " << sim_time << endl;
		map_vehicles[veh_ID]->exit_time = sim_time;
		// update perf measure object
		pm->update_values_at_exit(map_vehicles[veh_ID]);
		// delete veh_ID upon exiting the network
		set_cv_IDs.erase(veh_ID);
		// delete map for exiting vehicle
		map_vehicles.erase(veh_ID);
		/*
		// don't delete immediately since there could be a map referring to leader
		// delay at least 30 secs before deleting
		if ((sim_time - map_vehicles[veh_ID]->exit_time) >= 30.0) {
			cout << "Sim_time " << sim_time << "; Deleting veh_ID: " << veh_ID << endl;
			map_vehicles.erase(veh_ID);
		}
		*/
		}
		return 1;
	}

	case DRIVER_COMMAND_MOVE_DRIVER:
	{
		/*Following code is for both manual and connected vehicle*/
		/****************************************************************************************/
		//check if we are in the the first iteration of the next time step
		is_next_time_step = false;
		if (sim_time > next_time_step) {
			is_next_time_step = true;
			next_time_step = sim_time;
		}

		if (is_next_time_step) {
			is_veh_present_on_ramp[1] = is_veh_present_on_ramp[0];
			//if we are in the first iteration of the next time step
			//transfer information of veh on ramp 1201 to is_veh_present_on_ramp[1]
		}
		
		//Debugging
		/*if(is_next_time_step){
		cout << "*************************************************************************" << endl;
		}
		cout << "sim time = "<<sim_time<<" Is this next time step = " << is_next_time_step << endl;
		cout << "previous time step = " << is_veh_present_on_ramp[1].first << " is veh on ramp = " << is_veh_present_on_ramp[1].second << endl;*/
		
		/****************************************************************************************/


		//Set_cv_IDs contains set of all connected vehicles
		//We are checking if veh_ID is not in set_cv_IDs. If not, then it is a manual veh
		if (set_cv_IDs.find(veh_ID) == set_cv_IDs.end()) {
		/*****************************************************************************************************************************************************************/
		/******************************************************Manual vehicles**************************************************************************************************/
		/*****************************************************************************************************************************************************************/
			//cout << "sim time ramp " << is_veh_present_on_ramp[0].first<< endl;
			// link 1201 to 1300 are entrance ramps merging onto freeway with no auxilary lane
						if (is_veh_present_on_ramp[0].first==sim_time) {
							if (veh_link_no == 1201) {
								is_veh_present_on_ramp[0].second = 1;
							}	
						}
						else {
							//This condition is called only once per time step
							//if we are in the first iteration of the next time step
					/*set*/	is_veh_present_on_ramp[0].first = sim_time;  //This cond is used to check 
																		//if we are in the 1st iter of the
																		//next time step
							is_veh_present_on_ramp[0].second = 0;		//At 1st iter set vehicles on ramp to 0
							if (veh_link_no == 1201) {
								is_veh_present_on_ramp[0].second = 1;
							}
						}
			}
				
		else{
		/*******************************************************************************************************************************************************/
		/***********************************Assigning values to members of vehicle class************************************************************************/
		/*******************************************************************************************************************************************************/
		map_vehicles[veh_ID]->veh_lane = veh_lane;
		map_vehicles[veh_ID]->veh_color = veh_color;
		map_vehicles[veh_ID]->leader_ID = lead_veh_ID;
		map_vehicles[veh_ID]->follower_ID = follow_veh_ID;
		map_vehicles[veh_ID]->cur_speed = cur_speed;
		map_vehicles[veh_ID]->lead_veh_length = lead_veh_length;
		// calculate lead veh distance by substracting lead_veh_length & update time spent following equipped vehicle
		map_vehicles[veh_ID]->update_lead_distance(lead_veh_distance);
		// check if lead_veh_ID is an equipped veh (regular veh IDs also included)
		bool is_lead_veh_equipped = set_cv_IDs.find(lead_veh_ID) != set_cv_IDs.end();
		// check if follow_veh_ID is an equipped veh
		bool is_follow_veh_equipped = set_cv_IDs.find(follow_veh_ID) != set_cv_IDs.end();
		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		//Edits 03/21/17- To enable modeling manual vehicles //////////////////////////////////////
		map_vehicles[veh_ID]->veh_lead_left_ID = lead_left_ID;//////////////////////////////////////
		map_vehicles[veh_ID]->veh_lead_right_ID = lead_right_ID;//////////////////////////////////////
		map_vehicles[veh_ID]->spacing[0][0] = spacing[0][0];//////////////////////////////////////
		map_vehicles[veh_ID]->spacing[0][1] = spacing[0][1];//////////////////////////////////////
		map_vehicles[veh_ID]->spacing[1][0] = spacing[1][0];//////////////////////////////////////
		map_vehicles[veh_ID]->spacing[1][1] = spacing[1][1];//////////////////////////////////////
		map_vehicles[veh_ID]->spd_diff[0][0] = spd_diff[0][0];//////////////////////////////////////
		map_vehicles[veh_ID]->spd_diff[0][1] = spd_diff[0][1];//////////////////////////////////////
		map_vehicles[veh_ID]->spd_diff[1][0] = spd_diff[1][0];//////////////////////////////////////
		map_vehicles[veh_ID]->spd_diff[1][1] = spd_diff[1][1];//////////////////////////////////////
		map_vehicles[veh_ID]->left_lead_veh_length = left_lead_veh_length;//////////////////////////////////////
		map_vehicles[veh_ID]->right_lead_veh_length = right_lead_veh_length;//////////////////////////////////////
		map_vehicles[veh_ID]->des_acc = desired_acceleration;//////////////////////////////////////
		map_vehicles[veh_ID]->active_lane_change = active_lane_change; //////////////////////////////////////
		map_vehicles[veh_ID]->rel_target_lane = rel_target_lane;//////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		/*******************************************************************************************************************************************************/

		/*******************************************************************************************************************************************************/
		/*************************************************************Vehicle clustering - CACC status**********************************************************/
		/*******************************************************************************************************************************************************/
		if (lead_veh_ID != -1) {
			if (is_lead_veh_equipped) {
				// leader is a CACC-equipped vehicle
				map_vehicles[veh_ID]->leader_CACC_status = map_vehicles[lead_veh_ID]->CACC_status;
			}
			else {
				// leader is unequipped vehicle
				map_vehicles[veh_ID]->leader_CACC_status = NON_CACC;
			}
		}
		
				//We can add the code for outputing the information regarding cutins
				//simSec, cutin,lead_veh_len,spacing_platooning,distance_cut_in, speed,acceleration
		if ((map_vehicles[veh_ID]->CACC_status == CACC_FOLLOWER || map_vehicles[veh_ID]->CACC_status == CACC_MEMBER) && (lead_veh_ID!=-1 && map_vehicles[veh_ID]->leader_CACC_status== NON_CACC)) {
					ofstream fout_hack;
					string cutInFile;
					//if (is_first_time) {
					//	//cutInFile = cfg.m_run_spec;
					//	//cutInFile += "cutIn.csv";
					//	cutInFile = "cutIn.csv";
					//	fout_hack.open(cutInFile, std::fstream::app);
					//	//fout_hack <<"scenario,"<< "simSec," << "cutIn," << "CutVLen," << "PlatDist," << "CutVehDist," << "Speed," << "Acc" << endl;
					//	//speed from m/s to Mph 2.23694* speed
					//	//acc from m/s2 to ft/s2 3.28084* acc
					//	//cout << "scenario," << "simSec," << "cutIn," << "CutVLen," << "PlatDist," << "CutVehDist," << "Speed," << "Acc" << endl;
					//	fout_hack.close();
					//	is_first_time = FALSE;
					//}
					cutInFile = "cutIn.csv";
					fout_hack.open(cutInFile, std::fstream::app);
					long p_length;
					
					if (fout_hack.is_open() &&sim_time>=300 &&odometer>= 4000) {
						std::this_thread::sleep_for(100ms); // slow to real-time when this condition met.
					/*	cout << "scenario,"<<"VehID," << "simSec," << "cutIn," << "CutVLen," << "PlatDist," << "CutVehDist," << "Speed," << "Acc," << "Leader Status,"<<"Odometer truck (m)" << endl;
						cout << cfg.m_run_spec << "," << veh_ID<<"," << sim_time << "," << 1 << "," << lead_veh_length << "," << map_vehicles[veh_ID]->get_prev_lead_dist() << "," << map_vehicles[veh_ID]->get_lead_dist() << "," << cur_speed << "," << cur_acc << "," << map_vehicles[veh_ID]->leader_CACC_status << ","<<odometer << endl;
						cout << endl;*/
							fout_hack<<cfg.m_run_spec <<","<< sim_time << "," << 1 << "," << lead_veh_length << "," << map_vehicles[veh_ID]->get_prev_lead_dist() << "," << map_vehicles[veh_ID]->get_lead_dist() << "," << cur_speed << "," << cur_acc << "," << map_vehicles[veh_ID]->leader_CACC_status << "," << odometer << endl;
								
					}
				}
		

		if (follow_veh_ID != -1) {
			if (is_follow_veh_equipped) {
				// follower is a CACC-equipped truck
				map_vehicles[veh_ID]->follower_CACC_status = map_vehicles[follow_veh_ID]->CACC_status;
			}
			else {
				// follower is unequipped vehicle
				map_vehicles[veh_ID]->follower_CACC_status = NON_CACC;
			}
		}

		
		double time_spent_following = map_vehicles[veh_ID]->get_time_following_cv();


	
	
	if(map_vehicles[veh_ID]->CACC_status==CACC_FOLLOWER || map_vehicles[veh_ID]->CACC_status == CACC_MEMBER){
	long plat_len = pm->platoon_len(map_vehicles, veh_ID);
		if (plat_len == (cfg.m_max_plat_len+1)) {
			map_vehicles[veh_ID]->time_following_cv = -100;
		}
	}

		double follower_time_spent_following = 0.0;
		if (is_follow_veh_equipped)
		{
			follower_time_spent_following = map_vehicles[follow_veh_ID]->get_time_following_cv();
		}

		 if (map_vehicles[veh_ID]->CACC_status!=CACC_LEADER&& is_lead_veh_equipped  && (time_spent_following >= cfg.m_min_time_following_to_form_platoon)) {
			if (follower_time_spent_following >= cfg.m_min_time_following_to_form_platoon) {

				map_vehicles[veh_ID]->CACC_status = CACC_MEMBER;
			}
			else {
				map_vehicles[veh_ID]->CACC_status = CACC_FOLLOWER;

			}
		}


		 else if (((map_vehicles[veh_ID]->follower_CACC_status == CACC_FOLLOWER)
			 || (map_vehicles[veh_ID]->follower_CACC_status == CACC_MEMBER))) {
			 map_vehicles[veh_ID]->CACC_status = CACC_LEADER;
		 }

		 else { map_vehicles[veh_ID]->CACC_status = CACC_OFF; }


		 if(cfg.m_base){
		 map_vehicles[veh_ID]->CACC_status = CACC_OFF;
		 }
		/*******************************************************************************************************************************************************/

		/******************************************************************************************************************************************************/
		/**************************************		CACC -Platoon desired time gap************************************************************************************/
		/******************************************************************************************************************************************************/
		
	
		if ((time_spent_following >10  && time_spent_following < 10.15) && (map_vehicles[veh_ID]->CACC_status == CACC_FOLLOWER || map_vehicles[veh_ID]->CACC_status == CACC_MEMBER)) {
			//At 10 sec is the min following dist to form a platoon at 10.1 sec the desired CACC timegap is fixed

		/*	random_device device;
			mt19937 generator(device());
			discrete_distribution<int> distribution; */
			if (cfg.m_gap_setting == "Conservative"){
				//70%- 1.1    20% -0.9   7% 0.7    3% 0.6
				//distribution = { 70,20,7,3 };
				map_vehicles[veh_ID]->timegap_cacc_following = 1.2;
			}

			else if (cfg.m_gap_setting == "Aggressive"){
			//	distribution = { 3,7,20,70 };  // Aggressive 
				map_vehicles[veh_ID]->timegap_cacc_following = 0.6;
			}
			else {}

			/*int gap_set = distribution(generator);
			if (gap_set == 0){
				map_vehicles[veh_ID]->timegap_cacc_following = 1.1;
			}
			else if (gap_set == 1){
				map_vehicles[veh_ID]->timegap_cacc_following = 0.9;
			}
			else if (gap_set == 2){
				map_vehicles[veh_ID]->timegap_cacc_following = 0.7;
			}
			else{
				map_vehicles[veh_ID]->timegap_cacc_following = 0.6;
			}
			*/

		//	cout << "veh ID is   " << veh_ID << endl;
		//	cout << "veh distance from the start of the link is" << endl;
		//	cout << "Time spend following" << time_spent_following << endl;
		//	cout << "gap setting used    " << gap_set << endl;
		//	cout << "CACC timegap    " << map_vehicles[veh_ID]->timegap_cacc_following<<endl;
			
		}






		/*******************************************************************************************************************************************************/
		/*************************************************************Packet loss - Nakagami dist***************************************************************/
		/*******************************************************************************************************************************************************/

		
		int rx1 = -99;
	
if(!cfg.m_no_drop){

		if (map_vehicles[veh_ID]->CACC_status == CACC_FOLLOWER || map_vehicles[veh_ID]->CACC_status == CACC_MEMBER) {

			bool rx;
			double rx_prob = -99;
			
			double trans_power = cfg.m_trans_power;

			// calculates the prob of message reception using Nakagami m=3
			// distance and transmission power is in meters (or in the same units)
			double dist_ratio = map_vehicles[veh_ID]->get_lead_dist() / trans_power;
			rx_prob = exp(-3 * pow(dist_ratio, 2))*(1 + 3 * pow(dist_ratio, 2) + 4.5*pow(dist_ratio, 4));


		
			if(rx_prob<=1){
			random_device device;
			mt19937 generator(device());
			bernoulli_distribution dist(rx_prob);
			rx = dist(generator);
			rx1 = rx;
			if (!rx) {
			
				map_vehicles[veh_ID]->CACC_status = CACC_OFF;
				map_vehicles[veh_ID]->time_following_cv = 0;
				

			}
		}
	}
}

	
	// Code for generation random no from uniform distribution 
	/*std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> dis(0, 1);
	double x = dis(gen);



	if (x > rx_prob)
	map_vehicles[veh_ID]->CACC_status = CACC_OFF;

	*/
/*******************************************************************************************************************************************************/
if (!cfg.m_base) {
	// calculate acceleration
	desired_acceleration = map_vehicles[veh_ID]->calculate_acc(sim_timestep, veh_link_no,veh_lane);
}

		try {


			/*******************************************************************************************************************************************************/
			/**********************Performace Measures *************************************************************************************************************/
			/*******************************************************************************************************************************************************/

			if (sim_time > 299 &&  sim_time <4501 && sim_time > processed_time && fmod(sim_time, cfg.m_pm_calc_freq) == 0 && cfg.m_debug) {

				ofstream fout;

				if (pm->Ti == "Title") {
					platoon_len_file = path + "_";
					platoon_len_file += "platoon_length.csv";
					fout.open(platoon_len_file, std::fstream::trunc);
					fout.close();
				}
				fout.open(platoon_len_file, std::fstream::app);
				long p_length;

				if (fout.is_open()) {
					for (auto it = map_vehicles.begin(); it != map_vehicles.end(); ++it) {
						vehicle_ptr veh_ptr = it->second;
						if (veh_ptr->CACC_status == CACC_FOLLOWER) {
							p_length = pm->platoon_len(map_vehicles, veh_ptr->ID);

							if (pm->Ti == "Title") {
								fout << "******************************" << cfg.m_run_spec << "*********************************" << endl;
								fout << "sim_time" << "," << "Platoon_length" << endl;
								pm->Ti = "N";
							}
							if (p_length != 1) {
								fout << setprecision(0) << sim_time << ",";
								fout << p_length << endl;
							}
						}


					}
					fout.close();
				}
				else {
					cout << "error opening the file" << endl;
				}


			}

			if (sim_time > 4498 &&  sim_time <4501 && sim_time > processed_time && fmod(sim_time, cfg.m_pm_calc_freq) == 0 && cfg.m_debug) {

				ofstream fout1;

				pm->calculate(map_vehicles, sim_time);

				if (pm->Ti1 == "Title") {
					perf_mes = path + "_";
					perf_mes += "perf_mes_cv.csv";
					fout1.open(perf_mes, std::fstream::trunc);
					fout1.close();
				}

				fout1.open(perf_mes, std::fstream::app);

				if (fout1.is_open()) {

					if (pm->Ti1 == "Title") {
						fout1 << "******************************" << cfg.m_run_spec << "*********************************" << endl;
						fout1 /*<< "sim_time" << "," << "per_CV_Platoon_mode" << "," << "Avg_per_time_plat_network" << "," */ << "Avg_per_time_plat_all" << endl;
						pm->Ti1 = "N";
					}

					/*fout1 << setprecision(0) << sim_time << ",";
					fout1 << setprecision(4) << pm->pct_platooning_in_network << ",";
					fout1 << setprecision(4) << pm->pct_time_platooning_in_network << ","; */
					fout1 << setprecision(4) << pm->pct_time_all_platooning << endl;
					fout1.close();
				}
				else
					cout << "can't open the file" << endl;
			}

			if (sim_time > 299 && odometer > 1000 && map_vehicles[veh_ID]->entry_time > 299 && sim_time < 4501 && fmod(sim_time, 1) == 0 && cfg.m_debug) {

				temp.V_ID = veh_ID;
				temp.sim_t = sim_time;
				temp.c_spd = cur_speed;
				temp.c_acc = cur_acc;
				temp.sta = map_vehicles[veh_ID]->CACC_status;
				temp.frn_gap = 3.28*(lead_veh_distance - lead_veh_length);
				temp.re_gap = -3.28*(follow_veh_distance - veh_length);
				temp.recpt = rx1;

				m.push_back(temp);

			}


			if (sim_time > 299 && sim_time <4501 && sim_time > processed_time && fmod(sim_time, 300) == 0 && cfg.m_debug) {


				ofstream fout2;


				if (pm->Ti2 == "Title") {
					moves_process = path + "_";
					moves_process += "moves_process.csv";
					fout2.open(moves_process, std::fstream::trunc);
					fout2.close();
				}

				fout2.open(moves_process, std::fstream::app);
				if (fout2.is_open()) {
					if (pm->Ti2 == "Title") {
						fout2 << "******************************" << cfg.m_run_spec << "*********************************" << endl;
						fout2 << "vehID" << "," << "simtime_sec" << "," << "speed_mph" << "," << "acc_fps2" << "," << "status" << "," << "front_gap_ft" << "," << "rear_gap_ft" << "," << "Reception" << endl;
						pm->Ti2 = "N";
					}

					for (auto i = m.begin(); i < m.end(); i++) {

						fout2 << i->V_ID << ",";
						fout2 << i->sim_t << ",";
						fout2 << 2.23694* i->c_spd << ",";
						fout2 << setprecision(2) << 3.28084*i->c_acc << ",";
						fout2 << i->sta << ",";
						fout2 << setprecision(5) << i->frn_gap << ",";
						fout2 << i->re_gap << ",";
						fout2 << i->recpt;
						fout2 << endl;
						
					}
					m.clear();
					fout2.close();
				}
			}


			/*******************************************************************************************************************************************************/
		}
		catch (...) {
			cout << "Exception:  Output files" << endl;
		}
/******************************************************************************************************************************************************/
/**************************************		lane change monitoring ************************************************************************************/
/******************************************************************************************************************************************************/
				// lane change monitoring for any
				/*
				if ((active_lane_change != 0) | (veh_active_lane_change != 0))) {
					std::this_thread::sleep_for(100ms); // slow to real-time when this condition met.
					cout << "Ego Veh ID: " << veh_ID << "; Active Lane Change: " << active_lane_change << "; Veh Active: " << veh_active_lane_change <<
						"; Desired Angle: " << desired_lane_angle << endl;
				}
				*/

				// lane change monitoring for platoon leader
				/*
				if ((map_vehicles[veh_ID]->CACC_status == CACC_LEADER) & ((active_lane_change != 0) | (veh_active_lane_change != 0))) {
					std::this_thread::sleep_for(100ms); // slow to real-time when this condition met.
					cout << "Ego Veh ID: " << veh_ID << "; Active Lane Change: " << active_lane_change << "; Veh Active: " << veh_active_lane_change <<
						"; Desired Angle: " << desired_lane_angle << endl;
				}
				*/
/******************************************************************************************************************************************************/

/*******************************************************************************************************************************************************/
/**********************LANE CHANGE TO JOIN PLATOON *****************************************************************************************************/
/*******************************************************************************************************************************************************/
		if (!cfg.m_CACC_right_lane&& !cfg.m_CACC_left_lane && !cfg.m_base) {

			if ((is_lead_veh_equipped || is_follow_veh_equipped) && (map_vehicles[veh_ID]->CACC_status == CACC_OFF)) {

				//If the their is the possibility of a platoon formation, do not change lanes 
				map_vehicles[veh_ID]->active_lane_change = 0;
			}

			if (!is_lead_veh_equipped && !is_follow_veh_equipped && (map_vehicles[veh_ID]->CACC_status == CACC_OFF)) {
				//Check if the vehicle in the front left is equipped
				bool is_left_lead_veh_equipped = set_cv_IDs.find(map_vehicles[veh_ID]->veh_lead_left_ID) != set_cv_IDs.end();
				//Check if the vehicle in the front right is equipped	
				bool is_right_lead_veh_equipped = set_cv_IDs.find(map_vehicles[veh_ID]->veh_lead_right_ID) != set_cv_IDs.end();

				if (is_left_lead_veh_equipped) {
					if ((map_vehicles[map_vehicles[veh_ID]->veh_lead_left_ID]->CACC_status == CACC_FOLLOWER || map_vehicles[map_vehicles[veh_ID]->veh_lead_left_ID]->CACC_status == CACC_OFF) && (map_vehicles[veh_ID]->spacing[0][0] - map_vehicles[veh_ID]->left_lead_veh_length) > 6) {
						lane_change_ptr->initiate_lane_change(veh_ID, 1, veh_length, map_vehicles);
						}
				}

				else if (is_right_lead_veh_equipped) {
					if ((map_vehicles[map_vehicles[veh_ID]->veh_lead_right_ID]->CACC_status == CACC_FOLLOWER || map_vehicles[map_vehicles[veh_ID]->veh_lead_right_ID]->CACC_status == CACC_OFF) && (map_vehicles[veh_ID]->spacing[1][0] - map_vehicles[veh_ID]->right_lead_veh_length) > 6) {
						lane_change_ptr->initiate_lane_change(veh_ID, -1, veh_length, map_vehicles);
					}
				}
			}
			else {
				//No action
			}


}
/*******************************************************************************************************************************************************/

/*******************************************************************************************************************************************************/
/**********************LANE CHANGE *********************************************************************************************************************/
/*******************************************************************************************************************************************************/
		if (cfg.m_lane_ctrl_free) {
			//cout << "free lane change" << endl;
			// no lane change control - free mode
		}
		else if (cfg.m_no_lane_change) {			
			//cout << "No lane change when in CACC mode..." << endl;
			if (map_vehicles[veh_ID]->CACC_status == CACC_FOLLOWER || map_vehicles[veh_ID]->CACC_status == CACC_MEMBER
				) {
				map_vehicles[veh_ID]->active_lane_change = 0;
			}

			if (map_vehicles[veh_ID]->CACC_status == CACC_LEADER)
				map_vehicles[veh_ID]->active_lane_change = 0;
			//Assuming that we have only one OD pair. The next link of leader would be on freeway so there is no condition for that 	
		}

		else if (cfg.m_CACC_left_lane) {
			//cout << "left lane change" << endl;

			map_vehicles[veh_ID]->active_lane_change = 0;
			if (veh_lane != no_lanes) 
				lane_change_ptr->initiate_lane_change(veh_ID, 1, veh_length, map_vehicles);
		}

			else if (cfg.m_CACC_right_lane) {
				//cout << "right lane change" << endl;
				//cout << cfg.m_CACC_right_lane << endl;

				//start with no lane change
				map_vehicles[veh_ID]->active_lane_change = 0;
				int right_lane; //variable storing the location of the right lane
				//If the link number is between 1 to 100 then it is a basic freeway section - right lane =1
				//If the link number is between 500 and 600 then the link has an auxilary lane: right lane=2 
				if((veh_link_no>=501 && veh_link_no<=600)|| (veh_link_no >= 901 && veh_link_no <= 1000)) {
					right_lane = 2; 
				}
				else {
					right_lane = 1;
				}
				//cout << "vehicle link no is" << veh_link_no << endl;
				if (veh_lane != right_lane) 
					lane_change_ptr->initiate_lane_change(veh_ID, -1, veh_length, map_vehicles);
				}
/*******************************************************************************************************************************************************/
			else{
				//cout << "Default lane change" << endl;
				map_vehicles[veh_ID]->active_lane_change = 0;

		}
/*******************************************************************************************************************************************************/
		        
/*******************************************************************************************************************************************************/
/**********************Lane Change Link 1- Don't care about collisions********************************************************************************************/
/*******************************************************************************************************************************************************/
		//These lane changes are done on the 1st link so that all trucks can move to there respective links
	/*		if (cfg.m_CACC_left_lane&& veh_link_no == 1&& veh_lane!=3) {
				map_vehicles[veh_ID]->active_lane_change = 1;
			}

			else if (cfg.m_CACC_right_lane&& veh_link_no == 1&&veh_lane!= 1) {
				map_vehicles[veh_ID]->active_lane_change = -1;
			}
			
			else if (cfg.m_CACC_left_lane&& veh_link_no == 1 && veh_lane == 3) {
				map_vehicles[veh_ID]->active_lane_change = 0;

			}
			else if (cfg.m_CACC_right_lane&& veh_link_no == 1 && veh_lane == 1) {
				map_vehicles[veh_ID]->active_lane_change = 0;
			}
			else {

			}
		*/
/*******************************************************************************************************************************************************/

	if(cfg.m_ramp_lane_change){
/******************************************************************
*************************************************************************************/
/**********************Lane Change CACC equip vehicles are on Link 801-900 and manual veh on link 1201 :Don't be on the right lane**********************/
/*******************************************************************************************************************************************************/
		//is_veh_present_on_ramp[1].second checks if a vehicle was present in the last time step
		//vehicle should be on the second lane from the right. We don't want to facilitate lane change 
			if (veh_link_no >= 801 && veh_link_no <= 900&& veh_lane==1 && is_veh_present_on_ramp[1].second) {
				lane_change_ptr->initiate_lane_change(veh_ID, 1, veh_length, map_vehicles);
			}
			else if (veh_link_no >= 801 && veh_link_no <= 900 && veh_lane != 1) {
				map_vehicles[veh_ID]->active_lane_change = 0;
			}
/*******************************************************************************************************************************************************/
/**********************Lane Change CACC equip vehicles are on Link 901 to 1000- Be on the 3rd lane from the right **********************/
/*******************************************************************************************************************************************************/
		//is_veh_present_on_ramp[1].second checks if a vehicle was present in the last time step
		//Vehicles should be on the 3rd lane from right as this is the auxilary lane case and we want to facilitate lane change 
			if (veh_link_no >= 901 && veh_link_no <= 1000 && veh_lane == 2 && is_veh_present_on_ramp[1].second) {
				lane_change_ptr->initiate_lane_change(veh_ID, 1, veh_length, map_vehicles);
			}
			else if (veh_link_no >= 901 && veh_link_no <= 1000 && veh_lane == 3) {
				map_vehicles[veh_ID]->active_lane_change = 0;
			}

			if (veh_link_no >= 10000) {
				map_vehicles[veh_ID]->active_lane_change = 0;
			}
	}
	else if (cfg.m_ramp_lane_change_entirePlatoon) {
		if (map_vehicles[veh_ID]->CACC_status == CACC_LEADER && follow_veh_ID >= 1 && veh_link_no >= 801 && veh_link_no <= 900 && veh_lane == 1 && is_veh_present_on_ramp[1].second) {
			lane_change_ptr->initiate_lane_change(veh_ID, 1, veh_length, map_vehicles);
			lane_change_ptr->initiate_lane_change(follow_veh_ID, 1, veh_length, map_vehicles);
			if (map_vehicles[veh_ID]->active_lane_change == 1 && map_vehicles[follow_veh_ID]->active_lane_change == 1) {
				map_vehicles[veh_ID]->active_lane_change = 1;
				map_vehicles[follow_veh_ID]->DummyLead = veh_ID;
				/*cout << "sim sec" << sim_time << "  The lead veh id is : " << veh_ID << endl;
				cout << "The follower veh id is : " << follow_veh_ID << endl;*/
			}
			else {
				map_vehicles[veh_ID]->active_lane_change = 0;
				map_vehicles[follow_veh_ID]->active_lane_change = 0;
			}
		}
			else if (map_vehicles[veh_ID]->CACC_status == CACC_OFF && veh_link_no >= 801 && veh_link_no <= 900 && veh_lane == 1 && is_veh_present_on_ramp[1].second) {
				lane_change_ptr->initiate_lane_change(veh_ID, 1, veh_length, map_vehicles);
			}
			else if (veh_link_no >= 801 && veh_link_no <= 900 && veh_lane != 1) {
				map_vehicles[veh_ID]->active_lane_change = 0;
			}
/*******************************************************************************************************************************************************/
/**********************Lane Change CACC equip vehicles are on Link 901 to 1000- Be on the 3rd lane from the right **********************/
/*******************************************************************************************************************************************************/
			//is_veh_present_on_ramp[1].second checks if a vehicle was present in the last time step
			//Vehicles should be on the 3rd lane from right as this is the auxilary lane case and we want to facilitate lane change 
			if (map_vehicles[veh_ID]->CACC_status == CACC_LEADER  && follow_veh_ID >= 1 && veh_link_no >= 901 && veh_link_no <= 1000 && veh_lane == 2 && is_veh_present_on_ramp[1].second) {
				lane_change_ptr->initiate_lane_change(veh_ID, 1, veh_length, map_vehicles);
				lane_change_ptr->initiate_lane_change(follow_veh_ID, 1, veh_length, map_vehicles);
				if (map_vehicles[veh_ID]->active_lane_change == 1 && map_vehicles[follow_veh_ID]->active_lane_change == 1) {
					map_vehicles[veh_ID]->active_lane_change = 1;
					map_vehicles[follow_veh_ID]->DummyLead = veh_ID;
					/*cout <<"sim sec"<<sim_time<< "  The lead veh id is : " << veh_ID << endl;
					cout << "The follower veh id is : " << follow_veh_ID << endl;*/

				}
				else {
					map_vehicles[veh_ID]->active_lane_change = 0;
					map_vehicles[follow_veh_ID]->active_lane_change = 0;
				}
			}
			else if ( map_vehicles[veh_ID]->DummyLead >= 1) {
				if ((map_vehicles[map_vehicles[veh_ID]->DummyLead]->active_lane_change == 1)|| (map_vehicles[map_vehicles[veh_ID]->DummyLead]->veh_lane !=map_vehicles[veh_ID]->veh_lane)) {
					map_vehicles[veh_ID]->active_lane_change = 1;
					/*cout << "2 The follower veh id is : " << veh_ID << endl;
					cout << "2 The leader veh id is : " << map_vehicles[veh_ID]->DummyLead << endl;
					cout << "2 Active lane change follower : " << map_vehicles[veh_ID]->active_lane_change << endl;*/
					map_vehicles[veh_ID]->DummyLead = -1;
				}	
				else {
					map_vehicles[veh_ID]->active_lane_change = 0;
					//cout << "Follower is not changing lane" << endl;

				}
			}
			else if (map_vehicles[veh_ID]->CACC_status == CACC_OFF  && veh_link_no >= 901 && veh_link_no <= 1000 && veh_lane == 2 && is_veh_present_on_ramp[1].second){
				lane_change_ptr->initiate_lane_change(veh_ID, 1, veh_length, map_vehicles);
			}
			else if (veh_link_no >= 901 && veh_link_no <= 1000 && veh_lane == 3) {
				map_vehicles[veh_ID]->active_lane_change = 0;
			}
           else if (veh_link_no >= 10000) {
				map_vehicles[veh_ID]->active_lane_change = 0;
			}
}
			
/*******************************************************************************************************************************************************/
		



			//Edit 03/21/17 = Edits to enable modelling manual vehicles using dll

			active_lane_change=map_vehicles[veh_ID]->active_lane_change;
			veh_color = map_vehicles[veh_ID]->veh_color;
			desired_speed = map_vehicles[veh_ID]->des_speed;

/*****************************************************************************************************************************************************************/
/******************************************************Debugging**************************************************************************************************/
/*****************************************************************************************************************************************************************/
		// update vehicle platoon stats
		map_vehicles[veh_ID]->update_veh_stats();


		//if (is_lead_veh_equipped) 
		//if (time_spent_following > 0) // candidate for cacc activation
		if( cfg.m_debug && map_vehicles[veh_ID]->rel_target_lane != 0) /* Tracking ego vehicle lane change  */
		{
			/*
			try{
			std::this_thread::sleep_for(100ms); // slow to real-time when this condition met.
		
			// monitor those vehicles about to turn on CACC or already in CACC
			cout << endl;
			cout << "Ego Veh ID: " << map_vehicles[veh_ID]->ID <<  endl;
			cout << "Ego Ctrl Mode: " << (map_vehicles[veh_ID]->ctrl_mode);

			cout << "veh length" << veh_length << endl;
			if(lead_veh_ID>0){
				cout<<"; Lead Distance (m): " << map_vehicles[veh_ID]->get_lead_dist() << "; Acceleration (m/s2): " << map_vehicles[veh_ID]->acc << endl;
			cout << "Ego Status: " << (map_vehicles[veh_ID]->CACC_status) << "; Leader Status: " << (map_vehicles[veh_ID]->leader_CACC_status) << endl;
			cout << "Time Spent Following (s): " << time_spent_following << endl;		
			}
			// lane change debugging
			cout << "Rel Target Lane: " << map_vehicles[veh_ID]->rel_target_lane << "; Active Lane Change: " << map_vehicles[veh_ID]->active_lane_change << "; Desired Lane Angle " << desired_lane_angle << endl;
			if (map_vehicles[veh_ID]->rel_target_lane == 1)
				cout << "Front Left Spacing (m): " << map_vehicles[veh_ID]->spacing[0][0]- map_vehicles[veh_ID]->left_lead_veh_length << "; Front Left Spd Diff (m/s): " << map_vehicles[veh_ID]->spd_diff[0][0] << "; Front Left TTC (s): " << map_vehicles[veh_ID]->ttc[0][0] <<
				"; Rear Left Spacing (m): " << abs(map_vehicles[veh_ID]->spacing[0][1])-veh_length << "; Rear Left Spd Diff (m/s): " << map_vehicles[veh_ID]->spd_diff[0][1] << "; Rear Left TTC(s) : " << map_vehicles[veh_ID]->ttc[0][1] << endl;
			else if (map_vehicles[veh_ID]->rel_target_lane == -1)
				cout << "Front Right Spacing (m): " << map_vehicles[veh_ID]->spacing[1][0] - map_vehicles[veh_ID]->right_lead_veh_length << "; Front Right Spd Diff (m/s): " << map_vehicles[veh_ID]->spd_diff[1][0] << "; Front Right TTC (s): " << map_vehicles[veh_ID]->ttc[1][0] <<
				"; Rear Right Spacing (m): " << abs(map_vehicles[veh_ID]->spacing[1][1]) - veh_length << "; Rear Right Spd Diff (m/s): " << map_vehicles[veh_ID]->spd_diff[1][1] << "; Rear Right TTC(s) : " << map_vehicles[veh_ID]->ttc[1][1] << endl;
			}
			catch (...) { cout << "Error in debugging" << endl; }
			*/
		}
		
		map_vehicles[veh_ID]->reinitialize();
		processed_time = sim_time;
/***************************************************************Code for connected vehicles end********************************************************************/
		}  //for the first if condition (set_cv_IDs==...)




		return 1;
	} //for case 
    default :
		return 0;
		
  }     //for switch
}    //for the driver model execute function

/*==========================================================================*/
/*  End of DriverModel.cpp                                                 */
/*==========================================================================*/