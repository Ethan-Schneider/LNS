#pragma once
#include <vector>
#include <queue>
#include <map>
#include <boost/heap/priority_queue.hpp>
#include <boost/heap/pairing_heap.hpp>
#include <boost/unordered_map.hpp>
#include <iostream>

#include "KivaGraph.h"

// using namespace std;
using boost::unordered_map;
using std::vector;
using std::priority_queue;
using std::size_t;
using std::string;
using std::hash;
typedef std::pair<int, int> Key;

struct TaskAssignment
{
    int agent;
    int pos;
    int insertion_cost;
    TaskAssignment(int agent, int pos, int insertion_cost)
        : agent(agent), pos(pos), insertion_cost(insertion_cost)
        {};
};

struct CompareTaskAssignment {
    bool operator()(const TaskAssignment& ta1, const TaskAssignment& ta2) const
    {
        return ta1.insertion_cost > ta2.insertion_cost;
    }
};
typedef boost::heap::pairing_heap<TaskAssignment, boost::heap::compare<CompareTaskAssignment>>::handle_type handle_t;

class Task{
public:
    int task_id;
    vector<int> goal_arr;
    float relatedness;  
    int delta_cost;
    int pick_up_time = 0, delivery_time = 0;

    std::map<Key, handle_t> ta; 
    boost::heap::pairing_heap<TaskAssignment, boost::heap::compare<CompareTaskAssignment>> assignment_heap;
    
    Task(int id, vector<int>& goal_arr)
        : task_id(id), goal_arr(goal_arr)
        {};
    // Task Constructor
    Task(){};
    // ~Task();
};
typedef boost::heap::pairing_heap<TaskAssignment, boost::heap::compare<CompareTaskAssignment>>::handle_type handle_t;


class TasksLoader{
public:
    int num_of_tasks;
    vector<Task> tasks_all;
    std::map<int, int> tasks_table;
    string frequency;
    static inline bool compareTask(Task& t1, Task& t2, bool insert, int insertion_strategy, int removal_strategy)
    {
        if (insert && insertion_strategy == 1) {
            return t1.assignment_heap.top().insertion_cost <= t2.assignment_heap.top().insertion_cost;
        }
        else if (insert && insertion_strategy == 2) {
            TaskAssignment ta1 = t1.assignment_heap.top();
            Key agent_pos_pair_1(ta1.agent, ta1.pos);
            int t1_first_best = ta1.insertion_cost;
            t1.assignment_heap.pop();
            int t1_second_best = t1.assignment_heap.top().insertion_cost;
            handle_t handle_1 = t1.assignment_heap.push(ta1);
            t1.ta[agent_pos_pair_1] = handle_1;

            TaskAssignment ta2 = t2.assignment_heap.top();
            Key agent_pos_pair_2(ta2.agent, ta2.pos);
            int t2_first_best = ta2.insertion_cost;
            t2.assignment_heap.pop();
            int t2_second_best = t2.assignment_heap.top().insertion_cost;
            handle_t handle_2 = t2.assignment_heap.push(ta2);
            t2.ta[agent_pos_pair_2] = handle_2;

            return t1_second_best - t1_first_best >= t2_second_best - t2_first_best;
        }
        else if (removal_strategy == 1) {
            return t1.relatedness >= t2.relatedness;
        }
        else if (removal_strategy == 2) {
            return t1.delta_cost >= t2.delta_cost;
        }
        return true;
    }
    TasksLoader(const KivaGrid& G, const std::vector<std::tuple<int, std::tuple<int, int>, std::tuple<int, int>>>& current_tasks, const std::vector<std::tuple<int, std::tuple<int, int>, std::tuple<int, int>>>& undelivered_tasks);
    TasksLoader(const std::map<int, Task>& current_tasks, const std::map<int, Task>& unasigned_tasks);
    TasksLoader();
    void print_tasks();
};
typedef boost::heap::pairing_heap<TaskAssignment, boost::heap::compare<CompareTaskAssignment>>::handle_type handle_t;