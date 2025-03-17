#pragma once
#include "BasicGraph.h"


class KivaGrid :
	public BasicGraph
{
public:
	vector<int> endpoints;
	vector<int> agent_home_locations;
	vector<pair<int, int>> aisle_locations;  // Store aisle locations as (row, col) pairs
	// vector<int> initial_locations;

    bool load_map(string fname);
	bool load_Minghua_map(string fname);
    void preprocessing(bool consider_rotation); // compute heuristics
    void set_aisle_locations(const vector<pair<int, int>>& locations) { aisle_locations = locations; }
    bool is_location_full(int loc) const { return std::find(endpoints.begin(), endpoints.end(), loc) != endpoints.end(); }
};
