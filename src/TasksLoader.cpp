#include "TasksLoader.h"
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