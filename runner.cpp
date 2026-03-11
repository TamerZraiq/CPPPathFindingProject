/*
* Tamer Zraiq (G00425053)
* C++ Programming Path Finding Project - 4th Year 2026
* runner.cpp
*
* Planner run modes — single heuristic and 3-way comparison.
* Keeps main.cpp clean by owning all the setup and output logic.
*/

#include <iostream>
#include <iomanip>
#include <string>
#include "runner.h"
#include "config.h"
#include "pathFinding.h"

// Run all 3 heuristics on the same grid and print a comparison table.
void runHeuristicComparison(const std::vector<std::vector<int>>& grid)
{
    using H = PathPlanning::Heuristic;

    const H          heuristics[3] = { H::MANHATTAN, H::EUCLIDEAN, H::CHEBYSHEV };
    const char* names[3] = { "Manhattan",  "Euclidean",  "Chebyshev" };
    PathPlanning::PlannerResult results[3];

    for (int i = 0; i < 3; ++i) {
        std::cout << "\n\n================================================\n";
        std::cout << "  Running: " << names[i] << "\n";
        std::cout << "================================================\n";

        PathPlanning planner;
        planner.setGrid(grid);
        planner.setStart(START_ROW, START_COL);
        planner.setGoal(GOAL_ROW, GOAL_COL);
        planner.setHeuristic(heuristics[i]);
        planner.setVerbose(SHOW_ITERATION_TRACE);
        results[i] = planner.AStar_Planner();
    }

    // Print comparison table
    std::cout << "\n\n================================================\n";
    std::cout << "  HEURISTIC COMPARISON TABLE\n";
    std::cout << "================================================\n";
    std::cout << std::left
        << std::setw(12) << "Heuristic"
        << std::setw(14) << "Path Found"
        << std::setw(14) << "Iterations"
        << std::setw(18) << "Nodes Expanded"
        << std::setw(14) << "Path Length"
        << std::setw(12) << "Time (ms)"
        << "\n";
    std::cout << std::string(84, '-') << "\n";

    for (int i = 0; i < 3; ++i) {
        const auto& r = results[i];
        std::cout << std::left
            << std::setw(12) << names[i]
            << std::setw(14) << (r.pathFound ? "Yes" : "No")
            << std::setw(14) << r.iterations
            << std::setw(18) << r.nodesExpanded
            << std::setw(14) << r.pathLength
            << std::setw(12) << std::fixed << std::setprecision(4) << r.timeMs
            << "\n";
    }
    std::cout << "================================================\n";
}

// Run a single heuristic as defined by ACTIVE_HEURISTIC in config.h.
void runSingleHeuristic(const std::vector<std::vector<int>>& grid)
{
    PathPlanning::Heuristic h = PathPlanning::Heuristic::MANHATTAN;
    std::string active = ACTIVE_HEURISTIC;
    if (active == "EUCLIDEAN") h = PathPlanning::Heuristic::EUCLIDEAN;
    else if (active == "CHEBYSHEV") h = PathPlanning::Heuristic::CHEBYSHEV;

    PathPlanning planner;
    planner.setGrid(grid);
    planner.setStart(START_ROW, START_COL);
    planner.setGoal(GOAL_ROW, GOAL_COL);
    planner.setHeuristic(h);
    planner.setVerbose(SHOW_ITERATION_TRACE);
    planner.AStar_Planner();
}