#include "KivaGraph.h"

#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

int pymain()
{
    namespace py=pybind11;

    std::string map-file = "maps/symbotic_small.map";
    KivaGrid G;
    if (!G.load_map(map-file))
    {
        std::cout << "Failed to load map." << std::endl;
        return 1;
    }

    G.print_map();

    return 0;
}