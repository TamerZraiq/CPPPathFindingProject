/*
* Tamer Zraiq (G00425053)
* C++ Programming Path Finding Project - 4th Year 2026
* pathFinding.h
*/

#pragma once
#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include <vector>
#include <iostream>
#include <cmath>    // std::abs, std::sqrt

class PathPlanning
{
public:

    // ------------------------------------------------------------------
    // Heuristic options — passed into setHeuristic() or the comparison runner
    // ------------------------------------------------------------------
    enum class Heuristic {
        MANHATTAN,   // |dr| + |dc|         — admissible, fast
        EUCLIDEAN,   // sqrt(dr^2 + dc^2)   — true straight-line distance
        CHEBYSHEV    // max(|dr|, |dc|)      — tightest fit for 8-dir movement
    };

    // ------------------------------------------------------------------
    // PlannerResult — returned by AStar_Planner so callers and tests
    // can inspect the outcome without parsing terminal output.
    // ------------------------------------------------------------------
    struct PlannerResult {
        bool   pathFound = false;
        int    iterations = 0;
        int    nodesExpanded = 0;
        int    pathLength = 0;
        double timeMs = 0.0;   // wall-clock time in milliseconds
        std::vector<std::pair<int, int>> path;  // cells in start->goal order
    };

    // Run A* and return the result. All trace output goes to terminal.
    PlannerResult AStar_Planner();

    // ------------------------------------------------------------------
    // Setters — configure the planner; defaults come from config.h
    // ------------------------------------------------------------------
    void setGrid(const std::vector<std::vector<int>>& grid) { v = grid; }
    void setStart(int r, int c) { startR = r; startC = c; }
    void setGoal(int r, int c) { goalR = r; goalC = c; }
    void setHeuristic(Heuristic h) { heuristic = h; }

    // Verbose = true  ? print every iteration detail (open list, overlays, etc.)
    // Verbose = false ? only print startup grids and the final result
    void setVerbose(bool v) { verbose = v; }

private:

    // A 2D grid position (row, col)
    struct Position { int row, col; };

    // A single node in the search tree.
    // pr / pc are the parent cell coordinates (-1,-1 for the start node).
    struct SimpleNode {
        int r, c;       // this cell
        int g;          // actual cost from start
        double h, f;    // heuristic and total estimate (double for Euclidean)
        int pr, pc;     // parent cell (-1,-1 for start)
    };

    // Default grid (used when RANDOMISE_GRID = false in config.h)
    //
    // Cell coordinates (row, col):
    //   (0,0)  (0,1)  (0,2)  (0,3)  (0,4)
    //   (1,0)  (1,1)  (1,2)  (1,3)  (1,4)
    //   (2,0)  (2,1)  (2,2)  (2,3)  (2,4)
    //   (3,0)  (3,1)  (3,2)  (3,3)  (3,4)
    //   (4,0)  (4,1)  (4,2)  (4,3)  (4,4)
    //
    std::vector<std::vector<int>> v = {
        {0, 0, 0, 0, 0},
        {0, 1, 1, 0, 0},
        {1, 1, 0, 0, 1},
        {1, 1, 0, 1, 1},
        {1, 0, 0, 0, 0}
    };

    int       startR = 0;
    int       startC = 0;
    int       goalR = 4;
    int       goalC = 4;
    Heuristic heuristic = Heuristic::MANHATTAN;
    bool      verbose = true;

    std::vector<std::vector<int>> visited;
    std::vector<SimpleNode>       openList;
    std::vector<SimpleNode>       closedList;

    // ------------------------------------------------------------------
    // Search pipeline
    // ------------------------------------------------------------------
    void   printStartupInfo(int sr, int sc, int gr, int gc) const;
    bool   validateInputs(int sr, int sc, int gr, int gc) const;
    size_t selectBestNode() const;
    bool   expandSuccessors(const SimpleNode& q, int gr, int gc);
    void   updateVisitedOverlay(int rows, int cols);
    std::vector<std::pair<int, int>> reconstructPath(int gr, int gc) const;
    void   printResult(const std::vector<std::pair<int, int>>& path,
        int sr, int sc, int gr, int gc,
        int iteration, double timeMs) const;

    // ------------------------------------------------------------------
    // Heuristic / cost helpers
    // ------------------------------------------------------------------

    // g cost: uniform grid, all moves cost 1
    int g_cost(int, int, int, int) const { return 1; }

    // h cost: dispatches to the active heuristic
    double h_cost(int row, int col, int gr, int gc) const
    {
        double dr = std::abs(row - gr);
        double dc = std::abs(col - gc);
        switch (heuristic) {
        case Heuristic::EUCLIDEAN:  return std::sqrt(dr * dr + dc * dc);
        case Heuristic::CHEBYSHEV:  return std::max(dr, dc);
        case Heuristic::MANHATTAN:
        default:                    return dr + dc;
        }
    }

    double f_cost(int g, double h) const { return g + h; }

    // Human-readable name for the active heuristic
    std::string heuristicName() const
    {
        switch (heuristic) {
        case Heuristic::EUCLIDEAN: return "Euclidean";
        case Heuristic::CHEBYSHEV: return "Chebyshev";
        default:                   return "Manhattan";
        }
    }
};

#endif // PATH_PLANNING_H