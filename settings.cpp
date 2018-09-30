#include "settings.h"

// property tree & json parser
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <boost/foreach.hpp>
#include <exception>
#include <iostream>


void settings::load(const std::string &filename)
{
	// Create empty property tree object
	using boost::property_tree::ptree;
	ptree pt;

	// Load json file and put its contents in property tree. 
	// No namespace qualification is needed, because of Koenig 
	// lookup on the second argument. If reading fails, exception
	// is thrown.
	read_json(filename, pt);

	/* File Management */
	m_saved_cfg_file = pt.get<std::string>("File_Management.Saved_Config_File", "Config_Saved.json");

	/* Performance Measures */

	// output frequency
	m_pm_calc_freq = pt.get("Perf_Measures.Output_Freq_Sec", 60); // every 60 sec by default
	// file output directory
	m_pm_output_dir = pt.get<std::string>("Perf_Measures.Output_Dir", ".\\PerfMeasures");

	/* CACC Algorithm */

	// time spent following
	m_min_time_following_to_form_platoon = pt.get("CACC.Min_Time_Spent_Following_Sec", 10.0);
	// desired time gap for cacc mode
	m_desired_cacc_time_gap = pt.get("CACC.Desired_CACC_Time_Gap_Sec", 0.8);

	/* lane control */
	m_lane_ctrl_free = pt.get("Lane_Ctrl_Mode.Free", false); // free lane change - default
	m_lane_ctrl_follow_leader = pt.get("Lane_Ctrl_Mode.Follow_Leader", false); // change lane with a leader when possible
	m_no_lane_change = pt.get("Lane_Ctrl_Mode.No_Lane_Change_During_Platooning", false); // no lane change for both leader and follower in platoon mode
	m_CACC_left_lane = pt.get("Lane_Ctrl_Mode.CACC_left_lane", false); // All CACC equipped vehicles on the left lane
	m_CACC_right_lane=pt.get("Lane_Ctrl_Mode.CACC_right_lane", true); // All CACC equipped vehicles on the right lane
	/* miscellaneous */
	m_debug = pt.get("Misc.Debug", false); // debug mode printout

	m_trans_power = pt.get("Transmission_Power.Trans_con_power", 100); // Transmission power for Nakagami distribution
	m_no_drop = pt.get("Transmission_Power.no_drop", true);


	m_gap_setting = pt.get("Gap_set.type", "Conservative"); // get gap setting 
	
	m_run_spec = pt.get("Run_specification.Senario", "Senario_1"); // get gap setting

	m_base = pt.get("Case.Base", true);// check if the scenario is base case

	m_max_plat_len = pt.get("Platoon.maxLen", 0); // Ask for maximum platoon length 
	m_ramp_lane_change = pt.get("Ramp.lane_change", false);// Check if we want to initaite lane 
															//change for vehicles in right
															//lane for merge section 
	m_ramp_lane_change_entirePlatoon = pt.get("Ramp.lane_change_EntirePlatoon", false);
}


void settings::save(const std::string &filename)
{

	// Create empty property tree object
	using boost::property_tree::ptree;
	ptree pt;

	// Put
	pt.put("File_Management.Saved_Config_File", m_saved_cfg_file);

	pt.put("Perf_Measures.Output_Freq_Sec", m_pm_calc_freq);
	pt.put("Perf_Measures.Output_Dir", m_pm_output_dir);

	pt.put("CACC.Min_Time_Spent_Following_Sec", m_min_time_following_to_form_platoon);
	pt.put("CACC.Desired_CACC_Time_Gap_Sec", m_desired_cacc_time_gap);

	pt.put("Lane_Ctrl_Mode.Free", m_lane_ctrl_free);
	pt.put("Lane_Ctrl_Mode.No_Lane_Change_During_Platooning", m_no_lane_change);
	pt.put("Lane_Ctrl_Mode.Follow_Leader", m_lane_ctrl_follow_leader);
	pt.put("Lane_Ctrl_Mode.CACC_left_lane", m_CACC_left_lane);
	pt.put("Lane_Ctrl_Mode.CACC_right_lane", m_CACC_right_lane);

	pt.put("Misc.Debug", m_debug);
	pt.put("Transmission_Power.Trans_con_power", m_trans_power);
	pt.put("Transmission_Power.no_drop", m_no_drop);
	pt.put("Gap_set.type", m_gap_setting);
	pt.put("Run_specification.Senario", m_run_spec);
	pt.put("Base.Case", m_base);
	pt.put("Platoon.maxLen", m_max_plat_len);
	pt.put("Ramp.lane_change", m_ramp_lane_change);
	pt.put("Ramp.lane_change_EntirePlatoon", m_ramp_lane_change_entirePlatoon);

	// Write property tree to json file
	write_json(filename, pt);

}

/*
// for debugging

int main()
{
	try
	{
		settings cfg;
		cfg.load("Driver_Parameters.json");
		cfg.save("Driver_Parameters_Out.json");
		std::cout << "Success\n";
		std::getchar(); // pause console before exit
	}
	catch (std::exception &e)
	{
		std::cout << "Error: " << e.what() << "\n";
	}
	return 0;
}
*/