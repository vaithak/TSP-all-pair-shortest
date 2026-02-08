// tsp_solver.cpp
// TSP solver implementation using OR-Tools

#include "tsp_solver.h"
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"
#include <iostream>
#include <memory>

using namespace operations_research;

TSPSolver::TSPSolver() {
    // Constructor
}

std::pair<std::vector<int>, double> TSPSolver::solve(
    const std::vector<std::vector<int>>& distance_matrix,
    int depot) {
    
    const int num_nodes = distance_matrix.size();
    const int num_vehicles = 1;
    
    // Create routing index manager
    RoutingIndexManager manager(num_nodes, num_vehicles, depot);
    
    // Create routing model
    RoutingModel routing(manager);
    
    // Create distance callback
    const int transit_callback_index = routing.RegisterTransitCallback(
        [&distance_matrix, &manager](int64_t from_index, int64_t to_index) -> int64_t {
            // Convert from routing variable index to distance matrix node index
            auto from_node = manager.IndexToNode(from_index).value();
            auto to_node = manager.IndexToNode(to_index).value();
            return distance_matrix[from_node][to_node];
        }
    );
    
    // Define cost of each arc
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);
    
    // Setting first solution heuristic
    RoutingSearchParameters search_parameters = DefaultRoutingSearchParameters();
    search_parameters.set_first_solution_strategy(
        FirstSolutionStrategy::PATH_CHEAPEST_ARC);
    
    // Set time limit (optional, 30 seconds)
    search_parameters.set_time_limit(absl::Seconds(30));
    
    // Solve the problem
    const Assignment* solution = routing.SolveWithParameters(search_parameters);
    
    // Extract solution
    std::vector<int> path;
    double total_cost = 0.0;
    
    if (solution != nullptr) {
        // Extract path
        int64_t index = routing.Start(0);
        while (!routing.IsEnd(index)) {
            int node = manager.IndexToNode(index).value();
            path.push_back(node);
            
            int64_t previous_index = index;
            index = solution->Value(routing.NextVar(index));
        }
        
        // Get total cost
        total_cost = static_cast<double>(solution->ObjectiveValue());
        
        std::cout << "TSP Solution found with cost: " << total_cost << std::endl;
        std::cout << "Path: ";
        for (int node : path) {
            std::cout << node << " ";
        }
        std::cout << std::endl;
    } else {
        std::cerr << "No solution found!" << std::endl;
    }
    
    return {path, total_cost};
}
