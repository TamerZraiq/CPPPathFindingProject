/*
* Tamer Zraiq (G00425053)
* C++ Programming Path Finding Project - 4th Year 2026
* main.cpp
*/

#include <iostream>
#include "config.h"
#include "gridGen.h"
#include "runner.h"
#include "tests.h"

int main()
{
    std::cout << "================================\n";
    std::cout << "   A* Pathfinding - Tamer Z     \n";
    std::cout << "================================\n";
    std::cout << "  1. Run A* planner\n";
    std::cout << "  2. Run test suite\n";
    std::cout << "================================\n";
    std::cout << "Choice: ";

    int choice = 0;
    std::cin >> choice;

    if (choice == 1) {
        unsigned int seed = 0;
        auto grid = RANDOMISE_GRID ? generateGrid(seed) : getDefaultGrid();
        COMPARE_HEURISTICS ? runHeuristicComparison(grid) : runSingleHeuristic(grid);
    }
    else if (choice == 2) {
        runAllTests();
    }
    else {
        std::cout << "Invalid choice.\n";
    }

    return 0;
}