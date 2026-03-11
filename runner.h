#pragma once
/*
* Tamer Zraiq (G00425053)
* C++ Programming Path Finding Project - 4th Year 2026
* runner.h
*
* Declares the planner run modes.
* Defined in runner.cpp — called from main.cpp.
*/

#pragma once
#ifndef RUNNER_H
#define RUNNER_H

#include <vector>

// Run all 3 heuristics on the same grid and print a comparison table.
void runHeuristicComparison(const std::vector<std::vector<int>>& grid);

// Run a single heuristic as defined by ACTIVE_HEURISTIC in config.h.
void runSingleHeuristic(const std::vector<std::vector<int>>& grid);

#endif // RUNNER_H