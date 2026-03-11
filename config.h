/*
* Tamer Zraiq (G00425053)
* C++ Programming Path Finding Project - 4th Year 2026
* config.h
*
* Central configuration file — change values here to affect the whole program.
* Nothing else needs to be touched to adjust grid size, start/goal, or behaviour.
*/

#pragma once
#ifndef CONFIG_H
#define CONFIG_H

// ------------------------------------------------------------------
// Grid dimensions
// ------------------------------------------------------------------
constexpr int GRID_ROWS = 8;    // number of rows  (min 3)
constexpr int GRID_COLS = 8;    // number of columns (min 3)

// ------------------------------------------------------------------
// Start and goal positions (row, col)
// Only used when RANDOMISE_GRID is false.
// Must be within bounds and not on an obstacle.
// ------------------------------------------------------------------
constexpr int START_ROW = 0;
constexpr int START_COL = 0;
constexpr int GOAL_ROW = 7;
constexpr int GOAL_COL = 7;

// ------------------------------------------------------------------
// Randomisation
// true  = generate a random grid every run (ignores the fixed grid below)
// false = use the fixed grid defined in pathFinding.h
// ------------------------------------------------------------------
constexpr bool RANDOMISE_GRID = true;

// Obstacle density when randomising: 0.0 = no obstacles, 1.0 = all blocked.
// 0.25 means roughly 25% of free cells become obstacles.
// Keep below 0.45 to avoid unsolvable grids.
constexpr double OBSTACLE_DENSITY = 0.28;

// Random seed — set to any number for reproducible runs.
// Change it to get a different layout each time, or set RANDOM_SEED_AUTO
// to true to use a different seed automatically on every run.
constexpr bool RANDOM_SEED_AUTO = true;   // true = new layout every run
constexpr unsigned int RANDOM_SEED = 42;  // used only when RANDOM_SEED_AUTO is false

// ------------------------------------------------------------------
// Heuristic comparison mode
// true  = run all 3 heuristics on the same grid and print a comparison table
// false = run with ACTIVE_HEURISTIC only (normal single-run mode)
// ------------------------------------------------------------------
constexpr bool COMPARE_HEURISTICS = true;

// Active heuristic for single-run mode (ignored when COMPARE_HEURISTICS = true)
// Options: "MANHATTAN"  "EUCLIDEAN"  "CHEBYSHEV"
#define ACTIVE_HEURISTIC "MANHATTAN"

// ------------------------------------------------------------------
// Display options
// ------------------------------------------------------------------
constexpr bool SHOW_ITERATION_TRACE = false; // true = print every iteration detail
// false = only print final result (cleaner for comparison mode)

#endif // CONFIG_H