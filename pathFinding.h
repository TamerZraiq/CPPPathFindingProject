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
#include <cmath>     // std::abs

class PathPlanning
{
public:
    void AStar_Planner();

private:
    // A 2D grid position (row, col)
    struct Position {
        int row;
        int col;
    };

    // A single node in the search tree.
    // pr / pc are the parent cell coordinates (-1 if this is the start node).
    struct SimpleNode {
        int r, c;           // this cell
        int g, h, f;        // path cost, heuristic, total estimate
        int pr, pc;         // parent cell (-1,-1 for start)
    };

    // 5x5 grid: 0 = free, 1 = obstacle
    std::vector<std::vector<int>> v = {
        {0, 0, 0, 0, 0},
        {0, 1, 1, 0, 0},
        {1, 0, 0, 0, 1},
        {1, 1, 0, 1, 1},
        {1, 0, 0, 0, 0}
    };

    std::vector<std::vector<int>> visited;  // exploration overlay for display
    std::vector<SimpleNode> openList;       // discovered, not yet expanded
    std::vector<SimpleNode> closedList;     // already expanded

    // Manhattan distance between two positions.
    // Used as the heuristic h.  Admissible for 4-directional grids;
    // it under-estimates on 8-directional grids (still admissible, but
    // Chebyshev distance would be a tighter fit for 8-direction movement).
    int manhattanDistance(const Position& a, const Position& b) const {
        return std::abs(a.row - b.row) + std::abs(a.col - b.col);
    }

    // Step cost from one cell to an adjacent cell.
    // All moves cost 1 (uniform grid).
    int g_cost(int /*from_r*/, int /*from_c*/, int /*to_r*/, int /*to_c*/) const {
        return 1;
    }

    // Heuristic cost estimate from (row,col) to the goal (gr,gc).
    int h_cost(int row, int col, int gr, int gc) const {
        return manhattanDistance({ row, col }, { gr, gc });
    }

    // Total estimated cost f = g + h.
    int f_cost(int g, int h) const {
        return g + h;
    }
};

#endif // PATH_PLANNING_H