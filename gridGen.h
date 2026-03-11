/*
* Tamer Zraiq (G00425053)
* C++ Programming Path Finding Project - 4th Year 2026
* gridGen.h
*
* Random grid generation with solvability validation.
* Reads settings from config.h and returns a ready-to-use grid.
*/

#pragma once
#ifndef GRID_GEN_H
#define GRID_GEN_H

#include <vector>
#include <queue>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include "config.h"

// BFS flood-fill from (sr,sc) — returns true if (gr,gc) is reachable.
// Used after generating obstacles to confirm the grid is solvable.
static bool isSolvable(const std::vector<std::vector<int>>& grid,
    int sr, int sc, int gr, int gc)
{
    int rows = static_cast<int>(grid.size());
    int cols = static_cast<int>(grid[0].size());

    std::vector<std::vector<bool>> seen(rows, std::vector<bool>(cols, false));
    std::queue<std::pair<int, int>> q;
    q.push({ sr, sc });
    seen[sr][sc] = true;

    const int dr[8] = { -1,-1,-1, 0, 0, 1, 1, 1 };
    const int dc[8] = { -1, 0, 1,-1, 1,-1, 0, 1 };

    while (!q.empty()) {
        auto [r, c] = q.front(); q.pop();
        if (r == gr && c == gc) return true;
        for (int k = 0; k < 8; ++k) {
            int nr = r + dr[k], nc = c + dc[k];
            if (nr >= 0 && nr < rows && nc >= 0 && nc < cols
                && !seen[nr][nc] && grid[nr][nc] == 0)
            {
                seen[nr][nc] = true;
                q.push({ nr, nc });
            }
        }
    }
    return false;
}

// Generate a random grid using settings from config.h.
// Keeps regenerating obstacle placements until the grid is solvable.
// Returns the generated grid, and sets outSeed to the seed that was used.
static std::vector<std::vector<int>> generateGrid(unsigned int& outSeed)
{
    // Pick seed
    unsigned int seed = RANDOM_SEED_AUTO
        ? static_cast<unsigned int>(std::time(nullptr))
        : RANDOM_SEED;

    outSeed = seed;
    std::srand(seed);

    int rows = GRID_ROWS;
    int cols = GRID_COLS;
    int sr = START_ROW, sc = START_COL;
    int gr = GOAL_ROW, gc = GOAL_COL;

    std::vector<std::vector<int>> grid;
    int attempts = 0;

    do {
        ++attempts;
        grid.assign(rows, std::vector<int>(cols, 0));

        // Place obstacles randomly, but never on start or goal
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                if ((r == sr && c == sc) || (r == gr && c == gc)) continue;
                double roll = static_cast<double>(std::rand()) / RAND_MAX;
                if (roll < OBSTACLE_DENSITY) grid[r][c] = 1;
            }
        }

        // Reseed slightly differently each retry so we don't loop forever
        if (!isSolvable(grid, sr, sc, gr, gc))
            std::srand(seed + attempts * 7);

    } while (!isSolvable(grid, sr, sc, gr, gc));

    std::cout << "Grid generated (seed=" << seed
        << ", attempts=" << attempts << ")\n";
    return grid;
}

// Returns the fixed default grid defined in pathFinding.h,
// used when RANDOMISE_GRID = false in config.h.
static std::vector<std::vector<int>> getDefaultGrid()
{
    return {
        {0, 0, 0, 0, 0},
        {0, 1, 1, 0, 0},
        {1, 1, 0, 0, 1},
        {1, 1, 0, 1, 1},
        {1, 0, 0, 0, 0}
    };
}

#endif // GRID_GEN_H