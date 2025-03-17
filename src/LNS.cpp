/*
 * This code is modified from https://github.com/Jiaoyang-Li/Flatland
*/

#include "../inc/LNS.h"

bool LNS::run_repeat_Hungarian_greedy()
{
    this->num_of_agents = al.agents_all.size();
    this->num_of_tasks = tl.tasks_all.size();

    vector<int> all_task_id;
    vector<int> remain_task_id;
    for (auto t : tl.tasks_all)
    {
        all_task_id.push_back(t.task_id);
    }
    remain_task_id = all_task_id;
    int remain_tasks = num_of_tasks;
    while (remain_tasks > 0)
    {
        assigned_tasks.clear();
        int row = max(num_of_agents, remain_tasks);
        dlib::matrix<int> cost(row, row);
	    for (int i = 0; i < row; i++)
        {
            if (i >= num_of_agents)
            {
                for (int j = 0; j < row; j++)
                    cost(i, j) = -2147483;
            }
            else
            {
                for (int j = 0; j < row; j++)
                {
                    if (j >= remain_tasks)
                    {
                        cost(i, j) = -2147483;
                    }
                    else
                    {
                        Task& task = tl.tasks_all[tl.tasks_table[remain_task_id[j]]];
                        Agent& agent = al.agents_all[i];
                        int temp_cost = 0;
                        if (agent.task_sequence.size() > 0)
                        {
                            temp_cost = calculateMakespan(agent, agent.task_sequence);

                            Task& last_task = tl.tasks_all[tl.tasks_table[agent.task_sequence[agent.task_sequence.size()-1]]];
                            temp_cost += G.heuristics.at(last_task.goal_arr[last_task.goal_arr.size()-1])[task.goal_arr[0]];
                        }
                        else
                        {
                            temp_cost = agent.start_timestep + G.heuristics.at(task.goal_arr[0])[agent.start_location];      
                        }
                        
                        for (int k = 0; k < task.goal_arr.size()-1; k++)
                            temp_cost += G.heuristics.at(task.goal_arr[k])[task.goal_arr[k+1]];
                            
                        // Apply Gaussian-based weight to the cost
                        double weight = computeGaussianWeight(i, remain_task_id[j]);
                        // double ignore = computeAllLocationsGaussianWeight(i, 0);
                        // temp_cost = static_cast<int>(temp_cost * weight);
                        temp_cost = weight;
                        
                        cost(i, j) = -temp_cost;
                    }
                }
            }
        }
        vector<long> assignment = max_cost_assignment(cost);
        int curr_remain_tasks = remain_tasks;
        for (int i = 0; i < al.agents_all.size(); i++)
	    {
            Agent& ag = al.agents_all[i];
            if (assignment[i] < curr_remain_tasks)
            {
                assigned_tasks.push_back(remain_task_id[assignment[i]]);
                ag.task_sequence.push_back(remain_task_id[assignment[i]]);
            }
        }
        for (int i : assigned_tasks)
            remain_task_id.erase(std::remove(remain_task_id.begin(), remain_task_id.end(), i), remain_task_id.end());
        remain_tasks = remain_tasks - assigned_tasks.size();
    }
    return true;
}

bool LNS::run_Hungarian_greedy()
{
    this->num_of_agents = al.agents_all.size();
    this->num_of_tasks = tl.tasks_all.size();
    int row = max(num_of_agents, num_of_tasks);
    dlib::matrix<int> cost(row, row);
	for (int i = 0; i < row; i++)
    {
        if (i >= num_of_agents)
        {
            for (int j = 0; j < row; j++)
                cost(i, j) = -2147483;
        }
        else
        {
            for (int j = 0; j < row; j++)
            {
                if (j >= num_of_tasks)
                    cost(i, j) = -2147483;
                else
                {
                    Task& task = tl.tasks_all[j];
                    Agent& agent = al.agents_all[i];
                    int temp_cost = 0;
                    int waiting_time = agent.start_timestep + G.heuristics.at(task.goal_arr[0])[agent.start_location];
                    temp_cost = waiting_time;
                    for (int k = 0; k < task.goal_arr.size()-1; k++)
                        temp_cost += G.heuristics.at(task.goal_arr[k])[task.goal_arr[k+1]];
                    
                    // Apply Gaussian-based weight to the cost
                    double weight = computeGaussianWeight(i, task.task_id);
                    temp_cost = static_cast<int>(temp_cost * weight);
                    
                    cost(i, j) = -temp_cost;
                }
            }
        }
    }
    vector<long> assignment = max_cost_assignment(cost);
    for (int i = 0; i < al.agents_all.size(); i++)
	{
        Agent& ag = al.agents_all[i];
        if (assignment[i] < num_of_tasks)
        {
            assigned_tasks.push_back(tl.tasks_all[assignment[i]].task_id);
            (*ag.new_task_sequence).push_back(tl.tasks_all[assignment[i]].task_id);
            ag.task_sequence.push_back(tl.tasks_all[assignment[i]].task_id);
        }
    }
    return true;
}

bool LNS::run_HBH_greedy()
{
    clock_t t = clock();
    this->num_of_agents = al.agents_all.size();
    this->num_of_tasks = tl.tasks_all.size();
    if (num_of_tasks == 0)
        return true;
    vector<tuple<int, int, int>> task_agent_pair;
    for (int i = 0; i < num_of_tasks; i++)
    {
        Task& task = tl.tasks_all[i];
        for (int j = 0; j < num_of_agents; j++)
        {
            Agent& agent = al.agents_all[j];
            int cost = 0;
            // cost = agent.start_timestep + G.get_Manhattan_distance(agent.start_location, task.goal_arr[0]);
            cost = agent.start_timestep + G.heuristics.at(task.goal_arr[0])[agent.start_location];
            for (int k = 0; k < task.goal_arr.size()-1; k++)
                cost += G.heuristics.at(task.goal_arr[k])[task.goal_arr[k+1]];
                // cost += G.get_Manhattan_distance(task.goal_arr[k], task.goal_arr[k+1]);
            task_agent_pair.push_back(make_tuple(task.task_id-1, agent.agent_id-1, cost));
        }
    }
    sort(task_agent_pair.begin(), task_agent_pair.end(), [ ]( const tuple<int, int, int>& lhs, const tuple<int, int, int>& rhs )
    {
        return std::get<2>(lhs) < std::get<2>(rhs);
    });
    vector<int> visited_task;
    vector<int> visited_agent;
    for (auto p : task_agent_pair)
    {
        if (visited_agent.size() == num_of_agents)
            break;
        Agent& ag = al.agents_all[std::get<1>(p)];
        if (find(visited_agent.begin(), visited_agent.end(), std::get<1>(p)) == visited_agent.end() 
            && find(visited_task.begin(), visited_task.end(), std::get<0>(p)) == visited_task.end())
        {
            (*ag.new_task_sequence).clear();
            (*ag.new_task_sequence).push_back(std::get<0>(p)+1);
            visited_task.push_back(std::get<0>(p));
            visited_agent.push_back(std::get<1>(p));
        }
    }
    return true;
}

bool LNS::run(int time_limit)
{
    // start_time = Time::now();
    srand(time(NULL));
    this->num_of_agents = al.agents_all.size();
    this->num_of_tasks = tl.tasks_all.size();
    if (num_of_tasks == 0)
        return true;
    if (num_of_agents == 1)
    {
        this->insertion_strategy = 1;
        this->lns_insertion_strategy = 1;
    }
    // cout << "LNS runtime is " << initial_runtime << endl;
    if (!getInitialSolution()) {
        return false;
    }

    initial_makespan = getMakespan();
    initial_flowtime = getFlowtime();

    // initial_runtime = ((fsec)(Time::now() - start_time)).count();
    // double runtime = (std::clock() - t) * 1.0/ CLOCKS_PER_SEC;

    // cout << "LNS runtime is " << initial_runtime << endl;
    // cout << "Initial makespan = " << initial_makespan << ", "
    //     << "Initial flowtime = " << (double)initial_flowtime/num_of_tasks << ", "
    //     << "runtime = " << initial_runtime << endl;

    int iterations = 0;
    int best_makespan = initial_makespan;
    int best_flowtime = initial_flowtime;
    clock_t t = clock();
    // while (((fsec)(Time::now()- start_time)).count() < time_limit) {
    while ((std::clock() - t) * 1.0/ CLOCKS_PER_SEC < time_limit)
    {
        // high_resolution_clock::time_point curr_time = Time::now();
        iterations++;
        neighbors.clear();
        for (int i = 0; i < (int)tl.tasks_all.size(); i++) {
            neighbors.push_back(tl.tasks_all[i].task_id);
        }
        std::default_random_engine generator(rand());
        vector<int> sorted_neighbors;
        sorted_neighbors.clear();
        switch (removal_strategy)
        {
            case 0:
                std::shuffle(neighbors.begin(), neighbors.end(), std::default_random_engine(rand()));
                neighbors.resize(neighborhood_size);
                break;
            case 1:
                while (sorted_neighbors.size() < neighborhood_size) {
                    generateNeighborsByShawRemoval();
                    for (int i = 0; i < neighborhood_size; i ++) {
                        sorted_neighbors.push_back(neighbors.back());
                        neighbors.pop_back();
                    }
                }
                sorted_neighbors.resize(neighborhood_size);
                neighbors = sorted_neighbors;
                break;
            case 2:
                for (Agent& agent : al.agents_all) {
                    curr_task_sequence[agent.agent_id-1] = agent.task_sequence;
                }
                while (sorted_neighbors.size() < neighborhood_size) {
                    generateNeighborsByWorstRemoval();
                    int n = (int)(neighbors.size() * pow((double)rand()/ (RAND_MAX), p));
                    int neighbor = neighbors[n];
                    neighbors.erase(neighbors.begin()+n);
                    sorted_neighbors.push_back(neighbor);
                    for (Agent& agent : al.agents_all) {
                        agent.task_sequence.erase(std::remove(agent.task_sequence.begin(), agent.task_sequence.end(), neighbor), agent.task_sequence.end());
                    }
                }
                sorted_neighbors.resize(neighborhood_size);
                neighbors = sorted_neighbors;
                break;
            default:
                cout << "Wrong removal strategy" << endl;
                exit(0);
        }
        
        if (removal_strategy != 2) {
            for (Agent& agent : al.agents_all) {
                curr_task_sequence[agent.agent_id-1] = agent.task_sequence;
                for (int i : neighbors) {
                    agent.task_sequence.erase(std::remove(agent.task_sequence.begin(), agent.task_sequence.end(), i), agent.task_sequence.end());
                }
            }
        }
        initializeAssignmentHeap();
        while (neighbors.size())
        {
            sortNeighborsByStrategy(lns_insertion_strategy);
            addTaskAssignment();
            updateAssignmentHeap();
        }
        int makespan = 0;
        int flowtime = 0;
        makespan = getMakespan();
        flowtime = getFlowtime();
        if (flowtime < best_flowtime) {
            best_makespan = makespan;
            best_flowtime = flowtime;
        }
        else {
            for (Agent& agent : al.agents_all) {
                agent.task_sequence.clear();
                agent.task_sequence = curr_task_sequence[agent.agent_id-1];
            }
        }
    }

    // printTaskSequence();

    // cout << "Return Task Sequence" << endl;
    // // return task sequence
    // int agent_num = al.agents_all.size();
    // for (int i = 0; i < agent_num; i++) {
    //     cout << "Creating new Agent" << endl;
    //     Agent& ag = al.agents_all[i];
    //     cout << "Clearing new task sequence" << endl;
    //     (*ag.new_task_sequence).clear();
    //     for (int e : ag.task_sequence) {
    //         cout << "Pushing back new task sequence" << endl;
    //         cout << e << endl;
    //         cout << "New Task Sequence Size: " << (*ag.new_task_sequence).size() << endl;
    //         (*ag.new_task_sequence).push_back(e);
    //         cout << "New Task Sequence Size: " << (*ag.new_task_sequence).size() << endl;
    //         cout << "Pushed back new task sequence" << endl;
    //     }
    //     cout << "Exited nested for loop" << endl;
    // }
    cout << "Returning from LNS" << endl;
    return true;
}

void LNS::generateNeighborsByShawRemoval()
{   
    int idx = rand() % neighbors.size();
    Task& random_task = tl.tasks_all[tl.tasks_table[neighbors[idx]]];

    // Calculate delivery and pickup time for each task
    for (Agent& agent : al.agents_all) {
        int delivery_time = 0;
        int pick_up_time = 0;
        for (int i = 0; i < agent.task_sequence.size(); i++) {
            Task& task = tl.tasks_all[tl.tasks_table[agent.task_sequence[i]]];
            if (i == 0) {
                // pick_up_time += agent.start_timestep + G.get_Manhattan_distance(agent.start_location, task.goal_arr[0]);
                pick_up_time += agent.start_timestep + G.heuristics.at(task.goal_arr[0])[agent.start_location];
            }
            task.pick_up_time = pick_up_time;
            task.delivery_time = task.pick_up_time;
            for (int k = 1; k <= task.goal_arr.size()-1; k++)
                task.delivery_time += G.heuristics.at(task.goal_arr[k])[task.goal_arr[k-1]];
                // task.delivery_time += G.get_Manhattan_distance(task.goal_arr[k], task.goal_arr[k-1]);
            if (i != agent.task_sequence.size()-1) {
                Task& next_task = tl.tasks_all[tl.tasks_table[agent.task_sequence[i+1]]];
                // pick_up_time = task.delivery_time + G.get_Manhattan_distance(task.goal_arr.back(), next_task.goal_arr[0]);
                pick_up_time = task.delivery_time + G.heuristics.at(task.goal_arr.back())[next_task.goal_arr[0]];
            }
        }
    }
    for (int i = 0; i < tl.tasks_all.size(); i++) {
        Task& task = tl.tasks_all[i];
        // task.relatedness = relatedness_weight1 * (G.get_Manhattan_distance(task.goal_arr.back(), random_task.goal_arr.back())
        //     + G.get_Manhattan_distance(task.goal_arr[0], random_task.goal_arr[0])) + 
        //     + relatedness_weight2 * (std::abs(task.pick_up_time - random_task.pick_up_time) + std::abs(task.delivery_time - random_task.delivery_time));
        task.relatedness = relatedness_weight1 * (G.heuristics.at(task.goal_arr.back())[random_task.goal_arr.back()])
            + G.heuristics.at(task.goal_arr[0])[random_task.goal_arr[0]] + 
            + relatedness_weight2 * (std::abs(task.pick_up_time - random_task.pick_up_time) + std::abs(task.delivery_time - random_task.delivery_time));
    }
    quickSort(neighbors, 0, neighbors.size()-1, false, insertion_strategy, removal_strategy);
                        
}

void LNS::generateNeighborsByWorstRemoval()
{   
    for (Agent& agent : al.agents_all) {
        int curr_cost = calculateFlowtime(agent, agent.task_sequence);
        for (int i = 0; i < agent.task_sequence.size(); i++) {
            Task& task = tl.tasks_all[agent.task_sequence[i]];
            vector<int> new_task_sequence = agent.task_sequence;
            new_task_sequence.erase(new_task_sequence.begin()+i);
            task.delta_cost = curr_cost - calculateFlowtime(agent, new_task_sequence);
        }
    }
    quickSort(neighbors, 0, neighbors.size()-1, false, insertion_strategy, removal_strategy);
}

int LNS::calculateMakespan(Agent agent, vector<int> task_sequence)
{
    int makespan = 0;
    for (int i = 0; i < task_sequence.size(); i++) {
        Task& task = tl.tasks_all[tl.tasks_table[task_sequence[i]]];
        if (i == 0) {
            // makespan = agent.start_timestep + G.get_Manhattan_distance(agent.start_location, task.goal_arr[0]);
            makespan = agent.start_timestep + G.heuristics.at(task.goal_arr[0])[agent.start_location];
        }
        if (task.goal_arr.size() > 1)
        {
            for (int j = 0; j < task.goal_arr.size()-1; j++)
                makespan += G.heuristics.at(task.goal_arr[j])[task.goal_arr[j+1]];
                // makespan += G.get_Manhattan_distance(task.goal_arr[j], task.goal_arr[j+1]);
        }
        if (i != task_sequence.size()-1 ) {
            Task& next_task = tl.tasks_all[tl.tasks_table[task_sequence[i+1]]];
            // makespan += G.get_Manhattan_distance(task.goal_arr.back(), next_task.goal_arr[0]);
            makespan += G.heuristics.at(task.goal_arr.back())[next_task.goal_arr[0]];
        }
    }
    return makespan;
}

int LNS::calculateFlowtime(Agent agent, vector<int> task_sequence)
{
    int sum_of_release_time = 0;
    int sum_of_delivery_time = 0;
    int delivery_time = 0;
    if (task_sequence.size() == 0) {
        return 0;
    }
    
    if (task_sequence.size() == 1) {
        Task& task = tl.tasks_all[tl.tasks_table[task_sequence[0]]];
        delivery_time = agent.start_timestep + G.heuristics.at(task.goal_arr[0])[agent.start_location];
        
        if (task.goal_arr.size() > 1)
        {
            for (int i = 0; i < task.goal_arr.size()-1; i++)
                delivery_time += G.heuristics.at(task.goal_arr[i])[task.goal_arr[i+1]];
        }
        
        // Apply Gaussian-based weight
        double weight = computeGaussianWeight(agent.agent_id-1, task_sequence[0]);
        // delivery_time = static_cast<int>(delivery_time * weight);
        delivery_time = weight;
        
        sum_of_delivery_time += delivery_time;
    }
    
    for (int i = 0; i < task_sequence.size()-1; i++) {
        Task& task = tl.tasks_all[tl.tasks_table[task_sequence[i]]];
        Task& next_task = tl.tasks_all[tl.tasks_table[task_sequence[i+1]]];
        if (i == 0) {
            delivery_time = agent.start_timestep + G.heuristics.at(task.goal_arr[0])[agent.start_location];
        }
        if (task.goal_arr.size() > 1)
        {
            for (int k = 0; k < task.goal_arr.size()-1; k++)
                delivery_time += G.heuristics.at(task.goal_arr[k])[task.goal_arr[k+1]];
        }
        
        // Apply Gaussian-based weight for current task
        double weight = computeGaussianWeight(agent.agent_id-1, task_sequence[i]);
        delivery_time = static_cast<int>(delivery_time * weight);
        
        sum_of_delivery_time += delivery_time;
        delivery_time += G.heuristics.at(task.goal_arr[task.goal_arr.size()-1])[next_task.goal_arr[0]];
        
        if (i == task_sequence.size()-2) {
            if (next_task.goal_arr.size() > 1)
            {
                for (int k = 0; k < next_task.goal_arr.size()-1; k++)
                    delivery_time += G.heuristics.at(next_task.goal_arr[k])[next_task.goal_arr[k+1]];
            }
            
            // Apply Gaussian-based weight for last task
            weight = computeGaussianWeight(agent.agent_id-1, task_sequence[i+1]);
            delivery_time = static_cast<int>(delivery_time * weight);
            
            sum_of_delivery_time += delivery_time;
        }
    }
    return sum_of_delivery_time - sum_of_release_time;
}

bool LNS::getInitialSolution()
{
    // neighbors.resize(num_of_tasks);
    // for (int i = 0; i < num_of_tasks; i++) {
    //     neighbors[i] = tl.tasks_all[i].task_id;
    // }
    // initializeAssignmentHeap();
    // while (neighbors.size())
    // {
    //     sortNeighborsByStrategy(insertion_strategy);
    //     addTaskAssignment();
    //     updateAssignmentHeap();
    // }



    // assigned_tasks.clear();
    run_repeat_Hungarian_greedy();
    // neighbors.clear();
    // for (int i = 0; i < num_of_tasks; i++) {
    //     if (find(assigned_tasks.begin(), assigned_tasks.end(), tl.tasks_all[i].task_id) == assigned_tasks.end())
    //         neighbors.push_back(tl.tasks_all[i].task_id);
    // }
    return true;
}

void LNS::initializeAssignmentHeap()
{
    for (int task : neighbors) {
        int index = tl.tasks_table[task];
        tl.tasks_all[index].ta.clear();
        tl.tasks_all[index].assignment_heap.clear();
        for (auto& agent : al.agents_all) {
            int curr_cost = calculateFlowtime(agent, agent.task_sequence);
            for (int i = 0; i < agent.task_sequence.size()+1; i++) {
                vector<int> new_task_sequence = agent.task_sequence;
                new_task_sequence.insert(new_task_sequence.begin()+i, task);
                int temp_cost = calculateFlowtime(agent, new_task_sequence) - curr_cost;
                Key agent_pos_pair(agent.agent_id, i);
                TaskAssignment task_assignment(agent.agent_id, i, temp_cost);
                handle_t handle = tl.tasks_all[index].assignment_heap.push(task_assignment);
                tl.tasks_all[index].ta.insert(std::make_pair(agent_pos_pair, handle));
            }
        }
    }
}

void LNS::sortNeighborsByStrategy(int curr_insertion_strategy)
{
    if (curr_insertion_strategy == 0) {
        std::shuffle(neighbors.begin(), neighbors.end(), std::default_random_engine(rand()));
    }
    else if (curr_insertion_strategy == 1 || curr_insertion_strategy == 2) {
        quickSort(neighbors, 0, neighbors.size()-1, true, curr_insertion_strategy, removal_strategy);
    }
    else {
        cout << "Wrong insertion strategy" << endl;
        exit(0);
    }
}

void LNS::quickSort(vector<int>& task_order, int low, int high, bool insert, int curr_insertion_strategy, int removal_strategy)
{
    if (low >= high) {
        return;
    }
    int pivot = task_order[high];
    int i = low;
    for (int j = low; j <= high - 1; j++) {
        if (tl.compareTask(tl.tasks_all[tl.tasks_table[task_order[j]]], tl.tasks_all[tl.tasks_table[pivot]], insert, curr_insertion_strategy, removal_strategy)) {
            std::swap(task_order[i], task_order[j]);
            i++;
        }
    }
    std::swap(task_order[i], task_order[high]);
    quickSort(task_order, low, i-1, insert, curr_insertion_strategy, removal_strategy);
    quickSort(task_order, i+1, high, insert, curr_insertion_strategy, removal_strategy);
}

void LNS::addTaskAssignment()
{
    Task& t = tl.tasks_all[tl.tasks_table[neighbors[0]]];
    removed_task = t.task_id;
    int pos = t.assignment_heap.top().pos;
    updated_agent = t.assignment_heap.top().agent;
    for (auto& agent : al.agents_all) {
        if (agent.agent_id == updated_agent) {
            agent.task_sequence.insert(agent.task_sequence.begin()+pos, t.task_id);
            break;
        }
    }
    neighbors.erase(neighbors.begin());
}

void LNS::updateAssignmentHeap()
{
    for (int task : neighbors) {
        int index = tl.tasks_table[task];
        for (auto& agent : al.agents_all) {
            if (agent.agent_id == updated_agent) {
                // int curr_cost = getFlowtime();
                int curr_cost = calculateFlowtime(agent, agent.task_sequence);
                for (int i = 0; i < agent.task_sequence.size()+1; i++) {
                    vector<int> new_task_sequence = agent.task_sequence;
                    new_task_sequence.insert(new_task_sequence.begin()+i, task);
                    int temp_cost = calculateFlowtime(agent, new_task_sequence) - curr_cost;
                    // int temp_cost = getFlowtime(agent, new_task_sequence) - curr_cost;
                    Key agent_pos_pair(agent.agent_id, i);
                    if (tl.tasks_all[index].ta.find(agent_pos_pair) != tl.tasks_all[index].ta.end()) {
                        handle_t handle = tl.tasks_all[index].ta[agent_pos_pair];
                        if ((*handle).insertion_cost != temp_cost) {
                            (*handle).insertion_cost = temp_cost;
                            tl.tasks_all[index].assignment_heap.update(handle);
                        }
                    } else {
                        TaskAssignment task_assignment(updated_agent, i, temp_cost);
                        handle_t handle = tl.tasks_all[index].assignment_heap.push(task_assignment);
                        tl.tasks_all[index].ta.insert(std::make_pair(agent_pos_pair, handle));
                    }
                }
            }
        }
    }
}

int LNS::getFlowtime()
{
    int flowtime = 0;
    for (auto& agent : al.agents_all) {
        if (agent.task_sequence.size() > 0) {
            int curr_cost = calculateFlowtime(agent, agent.task_sequence);
            flowtime += curr_cost;
        }
    }
    return flowtime;
}

int LNS::getFlowtime(Agent agent, vector<int> task_sequence)
{
    int flowtime = 0;
    for (auto& ag : al.agents_all) {
        if (ag.agent_id == agent.agent_id) {
            int curr_cost = calculateFlowtime(agent, task_sequence);
            flowtime += curr_cost;
        }
        else
            flowtime += calculateFlowtime(agent, agent.task_sequence);
    }
    return flowtime;
}

int LNS::getMakespan()
{
    int makepsan = 0;
    for (auto& agent : al.agents_all) {
        if (agent.task_sequence.size() > 0) {
            int curr_cost = calculateMakespan(agent, agent.task_sequence);
            makepsan = std::max(makepsan, curr_cost);
        }
    }
    return makepsan;
}

void LNS::printTaskSequence()
{
    for (auto& agent : al.agents_all)
    {
        // cout << "after 2 agent.task_sequence.size() " << agent.task_sequence.size() << endl;
        // cout << " == Flowtime : " << calculateFlowtime(agent, agent.task_sequence)/agent.task_sequence.size() << " ";
        cout << "Agent " << agent.agent_id << " task sequence: ";
        for (auto i : agent.task_sequence)
            cout << i << " ";
        cout << endl;
    }
}

tuple<vector<tuple<int, vector<int>>>, vector<pair<double, double>>> LNS::getTaskSequence() {
    vector<tuple<int, vector<int>>> sequences;
    vector<pair<double, double>> weights;
    
    for (const auto& agent : al.agents_all) {
        if (!agent.task_sequence.empty()) {
            sequences.push_back(make_tuple(agent.agent_id, agent.task_sequence));
            
            // Get the last task in the sequence
            int last_task_id = agent.task_sequence.back();
            
            // Compute both Gaussian weights
            double warehouse_weight = computeWarehouseGaussianWeight(agent.agent_id-1, last_task_id);
            double method_weight = computeMethodGaussianWeight(agent.agent_id-1, last_task_id);
            
            weights.push_back(make_pair(warehouse_weight, method_weight));
        }
    }
    
    return make_tuple(sequences, weights);
}

vector<int> LNS::getUnallocatedTaskLocations() {
    vector<int> unallocated_locations;
    
    // Get all unallocated task locations
    for (const auto& task : tl.tasks_all) {
        // Check if this task is not in any agent's sequence
        bool is_allocated = false;
        for (const auto& agent : al.agents_all) {
            if (std::find(agent.task_sequence.begin(), agent.task_sequence.end(), task.task_id) != agent.task_sequence.end()) {
                is_allocated = true;
                break;
            }
        }
        
        // If task is not allocated, add its locations
        if (!is_allocated) {
            // Add both start and goal locations
            unallocated_locations.push_back(task.goal_arr[0]);  // Start location
            // unallocated_locations.push_back(task.goal_arr.back());  // Goal location
        }
    }
    
    return unallocated_locations;
}

vector<double> LNS::computeDistancesToUnallocatedTasks(int agent_id) {
    vector<double> distances;
    Agent& agent = al.agents_all[agent_id];
    
    // Get the agent's current location
    int agent_loc = agent.task_sequence.empty() ? agent.start_location : 
                    tl.tasks_all[tl.tasks_table[agent.task_sequence.back()]].goal_arr.back();
    
    // Get all unallocated task locations
    vector<int> unallocated_locations = getUnallocatedTaskLocations();
    
    // Calculate distances to all unallocated task locations
    for (int task_loc : unallocated_locations) {
        // Skip if this is the agent's own current location
        if (task_loc == agent_loc) continue;
        
        // Check if both locations exist in the heuristics map
        auto it = G.heuristics.find(agent_loc);
        if (it != G.heuristics.end() && task_loc < it->second.size()) {
            double dist = it->second[task_loc];
            distances.push_back(dist);
        }
    }
    
    return distances;
}

double LNS::computeMeanDistance(const vector<double>& distances) {
    if (distances.empty()) return 0.0;
    double sum = 0.0;
    for (double dist : distances) {
        sum += dist;
    }
    return sum / distances.size();
}

double LNS::computeStdDevDistance(const vector<double>& distances, double mean) {
    if (distances.empty()) return 0.0;
    double sum_sq_diff = 0.0;
    for (double dist : distances) {
        double diff = dist - mean;
        sum_sq_diff += diff * diff;
    }
    return sqrt(sum_sq_diff / distances.size());
}

double LNS::computeGaussianCDF(double x, double mean, double stddev) {
    if (stddev == 0) return 1.0;
    return 0.5 * (1 + erf((x - mean) / (stddev * sqrt(2))));
}

double LNS::computeWarehouseGaussianWeight(int agent_id, int task_id) {
    // Get distances to all warehouse locations
    vector<double> distances = computeDistancesToWarehouseLocations(agent_id);
    // cout << distances.size() << endl;
    if (distances.empty()) return 1.0;  // Default weight if no warehouse locations found
    
    // Compute mean and standard deviation
    double mean = computeMeanDistance(distances);
    double stddev = computeStdDevDistance(distances, mean);

    // cout << "Gaussian weight for the items in the warehouse: " << mean << " " << stddev << endl;
    
    // Get the distance to the current task
    Task& task = tl.tasks_all[tl.tasks_table[task_id]];
    Agent& agent = al.agents_all[agent_id];
    int agent_final_loc = agent.task_sequence.empty() ? agent.start_location : 
                         tl.tasks_all[tl.tasks_table[agent.task_sequence.back()]].goal_arr.back();
    
    // Check if both locations exist in the heuristics map
    double task_dist = 0.0;
    auto it = G.heuristics.find(agent_final_loc);
    if (it != G.heuristics.end() && task.goal_arr[0] < it->second.size()) {
        task_dist = it->second[task.goal_arr[0]];
    } else {
        // If locations not found, use a default large distance
        task_dist = mean + 2 * stddev;
    }
    
    // Use computeGaussianCDF to get a weight between 0 and 1
    double cdf = computeGaussianCDF(task_dist, mean, stddev);
    
    // Normalize the weight to be between 0.5 and 1.5
    return 0.5 + cdf;
}

double LNS::computeMethodGaussianWeight(int agent_id, int task_id) {
    if (gaussian_method == 0) {
        // Use unallocated tasks method
        vector<double> distances = computeDistancesToUnallocatedTasks(agent_id);
        if (distances.empty()) return 1.0;
        
        double mean = computeMeanDistance(distances);
        double stddev = computeStdDevDistance(distances, mean);

        // cout << "Gaussian weight for unallocated tasks: " << mean << " " << stddev << endl;
        
        Task& task = tl.tasks_all[tl.tasks_table[task_id]];
        Agent& agent = al.agents_all[agent_id];
        int agent_final_loc = agent.task_sequence.empty() ? agent.start_location : 
                             tl.tasks_all[tl.tasks_table[agent.task_sequence.back()]].goal_arr.back();
        
        double task_dist = 0.0;
        auto it = G.heuristics.find(agent_final_loc);
        if (it != G.heuristics.end() && task.goal_arr[0] < it->second.size()) {
            task_dist = it->second[task.goal_arr[0]];
        } else {
            task_dist = mean + 2 * stddev;
        }
        
        double cdf = computeGaussianCDF(task_dist, mean, stddev);
        return 0.5 + cdf;
    } else {
        // Use warehouse items method
        return computeWarehouseGaussianWeight(agent_id, task_id);
    }
}

double LNS::computeGaussianWeight(int agent_id, int task_id) {
    // Compute both weights
    // double warehouse_weight = computeWarehouseGaussianWeight(agent_id, task_id);
    double method_weight = computeMethodGaussianWeight(agent_id, task_id);
    
    // Log both weights to statistics
    // Note: This will be handled by the Python code that calls this function
    
    // Return the weight based on the chosen method
    return method_weight;
}

vector<int> LNS::getWarehouseLocations() {
    vector<int> locations;
    // Get all locations that have items in the warehouse
    for (const auto& loc : G.aisle_locations) {
        int loc_id = G.cols * loc.first + loc.second;
        if (G.is_location_full(loc_id)) {
            locations.push_back(loc_id);
        }
    }
    return locations;
}

vector<double> LNS::computeDistancesToWarehouseLocations(int agent_id) {
    vector<double> distances;
    Agent& agent = al.agents_all[agent_id];
    
    // Get the agent's current location
    int agent_loc = agent.task_sequence.empty() ? agent.start_location : 
                    tl.tasks_all[tl.tasks_table[agent.task_sequence.back()]].goal_arr.back();
    
    // Get all warehouse locations with items
    vector<int> warehouse_locations = getWarehouseLocations();
    
    // Calculate distances to all warehouse locations
    for (int loc : warehouse_locations) {
        // Skip if this is the agent's own current location
        if (loc == agent_loc) continue;
        
        // Check if both locations exist in the heuristics map
        auto it = G.heuristics.find(agent_loc);
        if (it != G.heuristics.end() && loc < it->second.size()) {
            double dist = it->second[loc];
            distances.push_back(dist);
        }
    }
    
    return distances;
}

vector<double> LNS::computeDistancesToAllLocations(int agent_id) {
    vector<double> distances;
    Agent& agent = al.agents_all[agent_id];
    
    // Get the agent's current location
    int agent_loc = agent.task_sequence.empty() ? agent.start_location : 
                    tl.tasks_all[tl.tasks_table[agent.task_sequence.back()]].goal_arr.back();
    
    // Calculate distances to all locations in the heuristics map
    for (const auto& [loc, heuristic_row] : G.heuristics) {
        // Skip if this is the agent's own current location
        if (loc == agent_loc) continue;
        
        // Check if the location exists in the heuristics map
        if (loc < heuristic_row.size()) {
            double dist = heuristic_row[loc];
            distances.push_back(dist);
        }
    }
    
    return distances;
}

double LNS::computeAllLocationsGaussianWeight(int agent_id, int target_loc) {
    // Get all distances from agent's last location
    vector<double> distances = computeDistancesToAllLocations(agent_id);
    
    // Compute mean and standard deviation
    double mean = computeMeanDistance(distances);
    double stddev = computeStdDevDistance(distances, mean);

    // cout << "Gaussian weight for all locations: " << mean << " " << stddev << endl;
    
    // Get the distance to the target location
    Agent& agent = al.agents_all[agent_id];
    int agent_loc = agent.task_sequence.empty() ? agent.start_location : 
                    tl.tasks_all[tl.tasks_table[agent.task_sequence.back()]].goal_arr.back();
    
    double target_dist = 0.0;
    auto it = G.heuristics.find(agent_loc);
    if (it != G.heuristics.end() && target_loc < it->second.size()) {
        target_dist = it->second[target_loc];
    }
    
    // Compute Gaussian weight
    double z = (target_dist - mean) / stddev;
    return exp(-0.5 * z * z);
}