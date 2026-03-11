/*
* Tamer Zraiq (G00425053)
* C++ Programming Path Finding Project - 4th Year 2026
* tests.cpp
*
* Manual test cases for the A* planner.
* Each test sets up a grid, runs the planner, and prints the full trace
* so the result can be read directly from the terminal output.
*
* Call runAllTests() from main.cpp to run all cases in sequence.
*/

#include <iostream>
#include <vector>
#include "pathFinding.h"
#include "tests.h"

// Prints a labelled banner so each test is easy to spot in the output.
static void printTestHeader(int number, const std::string& description)
{
    std::cout << "\n\n";
    std::cout << "################################################\n";
    std::cout << "  TEST " << number << ": " << description << "\n";
    std::cout << "################################################\n";
}

// ----------------------------------------------------------------
// Test 1 ? Normal case
// The default 5x5 grid with a clear path from (0,0) to (4,4).
// Expected: path found, trace shows full expansion.
// ----------------------------------------------------------------
static void test_NormalCase()
{
    printTestHeader(1, "Normal case - 5x5 grid, path exists");

    PathPlanning planner;
    planner.setGrid({
        {0, 0, 0, 0, 0},
        {0, 1, 1, 0, 0},
        {1, 1, 0, 0, 1},
        {1, 1, 0, 1, 1},
        {1, 0, 0, 0, 0}
        });
    planner.setStart(0, 0);
    planner.setGoal(4, 4);
    planner.AStar_Planner();
}

// ----------------------------------------------------------------
// Test 2 ? No path
// The goal at (3,3) is completely enclosed by obstacles.
// Expected: open list exhausts, "No path found" message.
// ----------------------------------------------------------------
static void test_NoPath()
{
    printTestHeader(2, "No path - goal is surrounded by obstacles");

    PathPlanning planner;
    planner.setGrid({
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 1, 1, 1},
        {0, 0, 1, 0, 1},
        {0, 0, 1, 1, 1}
        });
    planner.setStart(0, 0);
    planner.setGoal(3, 3);
    planner.AStar_Planner();
}

// ----------------------------------------------------------------
// Test 3 ? Start equals goal
// Start and goal are the same cell.
// Expected: early exit with "Start is already the goal" message.
// ----------------------------------------------------------------
static void test_StartEqualsGoal()
{
    printTestHeader(3, "Start equals goal - early exit");

    PathPlanning planner;
    planner.setGrid({
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0}
        });
    planner.setStart(1, 1);
    planner.setGoal(1, 1);
    planner.AStar_Planner();
}

// ----------------------------------------------------------------
// Test 4 ? Minimal 2x2 grid
// Smallest possible grid with a valid start and goal.
// Expected: one diagonal step from (0,0) to (1,1).
// ----------------------------------------------------------------
static void test_MinimalGrid()
{
    printTestHeader(4, "Minimal 2x2 grid - single diagonal step");

    PathPlanning planner;
    planner.setGrid({
        {0, 0},
        {0, 0}
        });
    planner.setStart(0, 0);
    planner.setGoal(1, 1);
    planner.AStar_Planner();
}

// ----------------------------------------------------------------
// Test 5 ? Start on an obstacle
// Validation should catch this before any search begins.
// Expected: "Invalid start or goal position" message, no search.
// ----------------------------------------------------------------
static void test_StartOnObstacle()
{
    printTestHeader(5, "Invalid input - start cell is an obstacle");

    PathPlanning planner;
    planner.setGrid({
        {1, 0, 0},
        {0, 0, 0},
        {0, 0, 0}
        });
    planner.setStart(0, 0);  // (0,0) is blocked
    planner.setGoal(2, 2);
    planner.AStar_Planner();
}

// ----------------------------------------------------------------
// Test 6 ? Goal on an obstacle
// Same as above but the goal cell is the obstacle.
// Expected: "Invalid start or goal position" message, no search.
// ----------------------------------------------------------------
static void test_GoalOnObstacle()
{
    printTestHeader(6, "Invalid input - goal cell is an obstacle");

    PathPlanning planner;
    planner.setGrid({
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 1}
        });
    planner.setStart(0, 0);
    planner.setGoal(2, 2);  // (2,2) is blocked
    planner.AStar_Planner();
}

// ----------------------------------------------------------------
// Test 7 ? Single winding corridor
// Only one valid route exists through the obstacles.
// Expected: path found, follows the only open corridor.
// ----------------------------------------------------------------
static void test_Corridor()
{
    printTestHeader(7, "Corridor - single winding path");

    PathPlanning planner;
    planner.setGrid({
        {0, 1, 0, 0, 0},
        {0, 1, 0, 1, 0},
        {0, 1, 0, 1, 0},
        {0, 0, 0, 1, 0},
        {1, 1, 1, 1, 0}
        });
    planner.setStart(0, 0);
    planner.setGoal(0, 4);
    planner.AStar_Planner();
}

// ============================================================
//  Public entry point ? called from main.cpp
// ============================================================
void runAllTests()
{
    std::cout << "\n################################################\n";
    std::cout << "#        A* PATHFINDING - TEST SUITE          #\n";
    std::cout << "#         " << 7 << " test cases, full trace output      #\n";
    std::cout << "################################################\n";

    test_NormalCase();
    test_NoPath();
    test_StartEqualsGoal();
    test_MinimalGrid();
    test_StartOnObstacle();
    test_GoalOnObstacle();
    test_Corridor();

    std::cout << "\n################################################\n";
    std::cout << "#           ALL TESTS COMPLETED               #\n";
    std::cout << "################################################\n";
}