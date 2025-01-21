#include "../inc/TasksLoader.h"
#include <map>
#include <vector>

TasksLoader::TasksLoader(const std::map<int, Task>& current_tasks, const std::map<int, Task>& unasigned_tasks)
{
    // int idx = 0;
    // for (auto itr = current_tasks.begin();itr != current_tasks.end(); itr++)
    // {
    //     Task task = itr->second;
    //     int task_id = itr->first;
    //     if (find(undelivered_tasks.begin(), undelivered_tasks.end(), task_id) != undelivered_tasks.end())
    //         continue;
    //     int i = 0;
    //     for (; i < task.goal_arr.size(); i++)
    //     {
    //         if (find(assigned_endpoints.begin(), assigned_endpoints.end(), task.goal_arr[i])!=assigned_endpoints.end()) {
    //             break;
    //         }
    //     }
    //     if (i != task.goal_arr.size())
    //         continue;
    //     tasks_all.push_back(task);
    //     tasks_table.insert(std::make_pair(task_id, idx));
    //     idx++;
    // }
}

TasksLoader::TasksLoader(const KivaGrid& G, const std::vector<std::tuple<int, std::tuple<int, int>, std::tuple<int, int>>>& current_tasks, const std::vector<std::tuple<int, std::tuple<int, int>, std::tuple<int, int>>>& unassigned_tasks)
{
    // for (int i = 0; i < current_tasks.size(); i++)
    // {
    //     int start_id = G.cols*std::get<0>(std::get<1>(current_tasks[i])) + std::get<1>(std::get<1>(current_tasks[i]));
    //     int goal_id = G.cols*std::get<0>(std::get<2>(current_tasks[i])) + std::get<1>(std::get<2>(current_tasks[i]));

    //     vector<int> goal_arr ={start_id, goal_id};

    //     Task task = Task(std::get<0>(current_tasks[i]), goal_arr);
    //     tasks_all.push_back(task);
    //     tasks_table.insert(std::make_pair(task.task_id, i));
    // }
    for (int i = 0; i < unassigned_tasks.size(); i++)
    {
        int start_id = G.cols*std::get<0>(std::get<1>(unassigned_tasks[i])) + std::get<1>(std::get<1>(unassigned_tasks[i]));
        int goal_id = G.cols*std::get<0>(std::get<2>(unassigned_tasks[i])) + std::get<1>(std::get<2>(unassigned_tasks[i]));
        std::vector<int> goal_arr ={start_id, goal_id};

        Task task = Task(std::get<0>(unassigned_tasks[i]), goal_arr);
        tasks_all.push_back(task);
        tasks_table.insert(std::make_pair(task.task_id, i));
    }
}

void TasksLoader::print_tasks()
{
    for (int i = 0; i < tasks_all.size(); i++)
    {
        std::cout << "Task ID: " << tasks_all[i].task_id << " ";
        std::cout << "Goal Arr: ";
        for (int j = 0; j < tasks_all[i].goal_arr.size(); j++)
        {
            std::cout << tasks_all[i].goal_arr[j] << " ";
        }
        std::cout << std::endl;
    }
}