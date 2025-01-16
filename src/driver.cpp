#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include "KivaGraph.h"
#include "TasksLoader.h"

int pymain(string& map_file, vector<tuple<int, tuple<int, int>, tuple<int, int>>> assigned_tasks, vector<tuple<int, tuple<int, int>, tuple<int, int>>> unassigned_tasks)
{
    namespace py=pybind11;

    KivaGrid G;
    if (!G.load_Minghua_map(map_file))
    {
        std::cout << "Failed to load map." << std::endl;
        return 1;
    }

    G.print_map();
    TasksLoader tl(G, assigned_tasks, unassigned_tasks);
    tl.print_tasks();

    return 0;
}