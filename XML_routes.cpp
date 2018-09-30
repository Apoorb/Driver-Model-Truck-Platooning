#include<iostream>
#include "XML_routes.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>


using namespace std;
namespace pt = boost::property_tree;

XML_routes::XML_routes()
{
}


XML_routes::~XML_routes()
{
}

void XML_routes::saveRoutes(const std::string & filename)
{

	// Create empty property tree object
	pt::ptree tree;

	// Parse the XML into the property tree.
	pt::read_xml(filename, tree);

	BOOST_FOREACH(pt::ptree::value_type const&  v, tree.get_child("network.vehicleRoutingDecisionsStatic")) {

		shared_ptr<Routes> r = make_shared<Routes>(Routes());
		routeNo = v.second.get<long>("<xmlattr>.no", 200);
		routeDecLink = v.second.get<long>("<xmlattr>.link", 200);
		r->routeDec_Dist= v.second.get<double>("<xmlattr>.pos", 200);
		map_Routes.insert(make_pair(make_pair(routeDecLink, routeNo), r));
	}
}

void XML_routes::display()
{
	for (auto it = map_Routes.begin(); it != map_Routes.end(); it++)
	{
		cout << it->second->routeDec_Dist << endl;
	}
}