// pybind_wrapper.cpp
// Python bindings for TSP solver

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "tsp_solver.h"

namespace py = pybind11;

PYBIND11_MODULE(_tsp_solver_cpp, m) {
    m.doc() = "C++ TSP solver using OR-Tools with Python bindings";
    
    py::class_<TSPSolver>(m, "TSPSolver")
        .def(py::init<>(), "Create a new TSP solver")
        .def("solve", &TSPSolver::solve,
             py::arg("distance_matrix"),
             py::arg("depot") = 0,
             R"pbdoc(
                Solve TSP problem.
                
                Args:
                    distance_matrix: NxN integer distance matrix (list of lists)
                    depot: Starting node index (default: 0)
                
                Returns:
                    tuple: (solution_path, total_cost)
                        - solution_path: List of node indices in visit order
                        - total_cost: Total path cost
             )pbdoc");
    
#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
