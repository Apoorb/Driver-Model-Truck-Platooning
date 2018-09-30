#ifndef SETTINGS_H
#define SETTINGS_H

#include <string>
#include <map>

class settings
{
	// read .json data file
public:
	/************ File Management *********************************/
	std::string m_saved_cfg_file;
	
	/************ Performance Measures *********************************/
	// performance measures calculation and output frequency (in seconds)
	int m_pm_calc_freq;
	// file output directory
	std::string m_pm_output_dir;

	/************ CACC Algorithm *********************************/

	// minimum time required in seconds for a follower to follow a CACC-enabled vehicle in a non-platooning state
	// before it can couple with a leader.
	double m_min_time_following_to_form_platoon;
	double m_desired_cacc_time_gap;
	
	/************ Lane Control Modes *********************************/
	bool m_lane_ctrl_free; // free lane change
	bool m_no_lane_change; // no lane change when in platoon
	bool m_lane_ctrl_follow_leader; // change lane with a leader when possible
	bool m_CACC_left_lane;  // All CACC if equipped vehicles in the left lane
	bool m_CACC_right_lane; //All CACC equipped vehicles on the right lane 
	/************* Miscellaneous **********************************/
	bool m_debug; // debug mode print out

	/************* Nakagami distribution ***************************/
	double m_trans_power;
	bool m_no_drop;

	/************* gap setting  *************************************/
	std::string m_gap_setting;

	/************* Run Specification **********************************/
	std::string m_run_spec;

	/************* Base Case		 **********************************/
	bool m_base;

	/************* Max Platoon Length ***********************************/
	int m_max_plat_len;

	/************ Load & Save json file*********************************/

	void load(const std::string &filename);
	void save(const std::string &filename);

	/************* Ramp lane change		 **********************************/
	bool m_ramp_lane_change;
	bool m_ramp_lane_change_entirePlatoon; //Entire platoon changes lane. 
};

#endif