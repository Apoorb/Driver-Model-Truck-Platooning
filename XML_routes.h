#pragma once
#ifndef XML_routes_h
#define XML_routes_h
#include <string>
#include<map>
#include <memory>

using namespace std;

//Stores the attributes of routing decisions
struct Routes
{
	
	double routeDec_Dist; //distance of routing decision from the starting point of the link it is on (dist in mtr)
};

class XML_routes
{
public:
	long routeNo;
	long routeDecLink; // Link on which routing decision is present
	multimap<pair<long,long>, shared_ptr<Routes>> map_Routes;// map for storing the location of all routing decisions
	XML_routes();
	~XML_routes();

	void saveRoutes(const std::string &);// function for saving the routing decisions from the VISSIM file
	void display();// function for debugging 
};

#endif 

