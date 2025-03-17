#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include "KivaGraph.h"
#include "../inc/TasksLoader.h"
#include "../inc/AgentsLoader.h"
#include "States.h"
#include "../inc/LNS.h"

tuple<vector<tuple<int, vector<int>>>, vector<pair<double, double>>> pymain(string& map_file, vector<tuple<int, tuple<int, int>, tuple<int, int>>> unassigned_tasks, vector<tuple<int, tuple<int, int>>> agents, 
    vector<vector<int>> task_sequences, vector<tuple<int, int>> aisle_locations, int gaussian_method)
{
    namespace py=pybind11;

    KivaGrid G;
    if (!G.load_Minghua_map(map_file))
    {   
        std::cout << "Failed to load map." << std::endl;
        tuple<vector<tuple<int, vector<int>>>, vector<pair<double, double>>> empty;
        return empty;
    }

    // Set aisle locations
    vector<pair<int, int>> aisle_pairs;
    for (const auto& loc : aisle_locations) {
        aisle_pairs.push_back(make_pair(std::get<0>(loc), std::get<1>(loc)));
    }
    G.set_aisle_locations(aisle_pairs);

    G.preprocessing(0);
    TasksLoader tl(G, unassigned_tasks);

    vector<tuple<int, int>> agent_ids;
    for (const auto& agent : agents)
    {
        int id = std::get<0>(agent);
        int x = std::get<0>(std::get<1>(agent));
        int y = std::get<1>(std::get<1>(agent));
        int new_id = G.cols*x + y;
        agent_ids.push_back(make_tuple(id, new_id));
    }

    // Init Starts
    vector<State> starts;
    for (const auto& agent : agent_ids)
    {
        int id = std::get<0>(agent);
        int loc = std::get<1>(agent);
        starts.push_back(State(loc, 0));
    }
    std::map<int, vector<int>> delivering_agents;

    AgentsLoader al(G, starts, delivering_agents, task_sequences);

    for (size_t i = 0; i < al.agents_all.size(); ++i)
    {
        if (i < task_sequences.size())
        {
            al.agents_all[i].task_sequence = task_sequences[i];
        }
    }

    LNS lns(G, tl, al, 2, 1, 2, 2);
    lns.set_gaussian_method(gaussian_method);  // Set the Gaussian method
    lns.run(1); // 1 second time limit

    // Extract just the sequences from the tuple return
    auto [sequences, weights] = lns.getTaskSequence();
    return make_tuple(sequences, weights);
}

double distance(string& map_file, tuple<int, int> start_loc, tuple<int, int> goal_loc)
{
    namespace py=pybind11;

    KivaGrid G;
    if (!G.load_Minghua_map(map_file))
    {   
        std::cout << "Failed to load map." << std::endl;
        double empty;
        return empty;
    }

    G.preprocessing(0);

    int start_id = G.cols*std::get<0>(start_loc) + std::get<1>(start_loc);
    int goal_id = G.cols*std::get<0>(goal_loc) + std::get<1>(goal_loc);

    if (start_id == goal_id)
    {
        return 0;
    }

    return G.heuristics.at(goal_id)[start_id];
}