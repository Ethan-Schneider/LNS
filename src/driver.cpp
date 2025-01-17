#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include "KivaGraph.h"
#include "TasksLoader.h"
#include "AgentsLoader.h"
#include "States.h"
#include "LNS.h"

int pymain(string& map_file, vector<tuple<int, tuple<int, int>, tuple<int, int>>> assigned_tasks, vector<tuple<int, tuple<int, int>, tuple<int, int>>> unassigned_tasks, vector<tuple<int, tuple<int, int>>> agents, 
    vector<vector<int>> task_sequences)
{
    namespace py=pybind11;

    KivaGrid G;
    if (!G.load_Minghua_map(map_file))
    {
        std::cout << "Failed to load map." << std::endl;
        return 1;
    }

    G.print_map();
    G.preprocessing(0);
    TasksLoader tl(G, assigned_tasks, unassigned_tasks);
    tl.print_tasks();

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

    std::cout << "Task sequences:" << std::endl;
    for (const auto& sequence : task_sequences)
    {
        for (const auto& task : sequence)
        {
            std::cout << task << " ";
        }
        std::cout << std::endl;
    }

    AgentsLoader al(G, starts, delivering_agents, task_sequences);

    for (size_t i = 0; i < al.agents_all.size(); ++i)
    {
        if (i < task_sequences.size())
        {
            al.agents_all[i].task_sequence = task_sequences[i];
        }
    }

    for (const auto& agent : al.agents_all)
    {
        std::cout << "Agent " << agent.agent_id << " task sequence: ";
        for (const auto& task : agent.task_sequence)
        {
            std::cout << task << " ";
        }
        std::cout << std::endl;
    }

    LNS lns(G, tl, al, 2, 1, 2, 2);
    lns.run(1); // 1 second time limit

    std::cout << "Finished LNS Execution" << std::endl;
    lns.printTaskSequence();
    return 0;
}