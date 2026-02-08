// tsp_solver.h
// TSP solver using OR-Tools

#ifndef TSP_SOLVER_H
#define TSP_SOLVER_H

#include <vector>
#include <utility>

class TSPSolver {
public:
    TSPSolver();
    ~TSPSolver() = default;
    
    /**
     * Solve TSP problem using OR-Tools.
     * 
     * @param distance_matrix NxN integer distance matrix
     * @param depot Starting node index
     * @return pair of (solution_path, total_cost)
     */
    std::pair<std::vector<int>, double> solve(
        const std::vector<std::vector<int>>& distance_matrix,
        int depot = 0
    );
    
private:
    // Internal helper methods if needed
};

#endif // TSP_SOLVER_H
