/*
* Tamer Zraiq (G00425053)
* C++ Programming Path Finding Project - 4th Year 2026
* pathFinding.cpp
*/

#include <iostream>
#include <chrono>
#include "pathFinding.h"

// ============================================================
//  File-scope display helpers (pure output, no class state)
// ============================================================

static void printRawGrid(const std::vector<std::vector<int>>& grid)
{
    std::cout << "Raw Grid:\n";
    for (const auto& row : grid) {
        for (int cell : row) std::cout << cell << " ";
        std::cout << "\n";
    }
}

static void printSemanticGrid(const std::vector<std::vector<int>>& grid,
    int sr, int sc, int gr, int gc)
{
    int rows = static_cast<int>(grid.size());
    for (int r = 0; r < rows; ++r) {
        int cols = static_cast<int>(grid[r].size());
        for (int c = 0; c < cols; ++c) {
            if (r == sr && c == sc) std::cout << "S ";
            else if (r == gr && c == gc) std::cout << "G ";
            else if (grid[r][c] == 1)    std::cout << "# ";
            else                         std::cout << ". ";
        }
        std::cout << "\n";
    }
}

static void printCoordinateGrid(int rows, int cols)
{
    std::cout << "Coordinate Grid:\n";
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c)
            std::cout << "(" << r << "," << c << ") ";
        std::cout << "\n";
    }
}

static void printVisitedGrid(const std::vector<std::vector<int>>& grid,
    const std::vector<std::vector<int>>& visited,
    int sr, int sc, int gr, int gc)
{
    int rows = static_cast<int>(grid.size());
    for (int r = 0; r < rows; ++r) {
        int cols = static_cast<int>(grid[r].size());
        for (int c = 0; c < cols; ++c) {
            if (r == sr && c == sc) std::cout << "S ";
            else if (r == gr && c == gc) std::cout << "G ";
            else if (grid[r][c] == 1)    std::cout << "# ";
            else if (visited[r][c] == 1) std::cout << "* ";
            else                         std::cout << ". ";
        }
        std::cout << "\n";
    }
}

static void printPathGrid(const std::vector<std::vector<int>>& grid,
    const std::vector<std::pair<int, int>>& path,
    int sr, int sc, int gr, int gc)
{
    int rows = static_cast<int>(grid.size());
    int cols = static_cast<int>(grid[0].size());

    std::vector<std::vector<bool>> onPath(rows, std::vector<bool>(cols, false));
    for (const auto& p : path) onPath[p.first][p.second] = true;

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (r == sr && c == sc) std::cout << "S ";
            else if (r == gr && c == gc) std::cout << "G ";
            else if (grid[r][c] == 1)    std::cout << "# ";
            else if (onPath[r][c])       std::cout << "@ ";
            else                         std::cout << ". ";
        }
        std::cout << "\n";
    }
}


// ============================================================
//  Search pipeline — private methods
// ============================================================

void PathPlanning::printStartupInfo(int sr, int sc, int gr, int gc) const
{
    int rows = static_cast<int>(v.size());
    int cols = static_cast<int>(v[0].size());

    printRawGrid(v);
    std::cout << "\nGrid dimensions: " << rows << "x" << cols << "\n";
    std::cout << "Heuristic : " << heuristicName()
        << (heuristic == Heuristic::MANHATTAN ? "  (4-directional)" : "  (8-directional)")
        << "\n";
    std::cout << "Start: (" << sr << "," << sc << ")  "
        << "Goal:  (" << gr << "," << gc << ")\n";
    std::cout << "\nSemantic Grid:\n";
    printSemanticGrid(v, sr, sc, gr, gc);
    std::cout << "\n";
    printCoordinateGrid(rows, cols);
}

bool PathPlanning::validateInputs(int sr, int sc, int gr, int gc) const
{
    int rows = static_cast<int>(v.size());
    int cols = static_cast<int>(v[0].size());

    auto inBounds = [&](int r, int c) {
        return r >= 0 && r < rows && c >= 0 && c < cols;
        };
    auto isFree = [&](int r, int c) { return v[r][c] == 0; };

    if (!inBounds(sr, sc) || !inBounds(gr, gc) ||
        !isFree(sr, sc) || !isFree(gr, gc))
    {
        std::cout << "Invalid start or goal position.\n";
        return false;
    }
    if (sr == gr && sc == gc) {
        std::cout << "Start is already the goal. No pathfinding needed.\n";
        return false;
    }
    return true;
}

size_t PathPlanning::selectBestNode() const
{
    size_t min_idx = 0;
    for (size_t i = 1; i < openList.size(); ++i) {
        if (openList[i].f < openList[min_idx].f ||
            (openList[i].f == openList[min_idx].f &&
                openList[i].g < openList[min_idx].g))
        {
            min_idx = i;
        }
    }
    return min_idx;
}

bool PathPlanning::expandSuccessors(const SimpleNode& q, int gr, int gc)
{
    int rows = static_cast<int>(v.size());
    int cols = static_cast<int>(v[0].size());

    // Manhattan is a 4-directional heuristic (up/down/left/right only).
    // Euclidean and Chebyshev are designed for 8-directional movement.
    const int dr4[4] = { -1,  1,  0,  0 };
    const int dc4[4] = { 0,  0, -1,  1 };

    const int dr8[8] = { -1, -1, -1,  0, 0,  1, 1, 1 };
    const int dc8[8] = { -1,  0,  1, -1, 1, -1, 0, 1 };

    const int* dr = (heuristic == Heuristic::MANHATTAN) ? dr4 : dr8;
    const int* dc = (heuristic == Heuristic::MANHATTAN) ? dc4 : dc8;
    const int  nDirs = (heuristic == Heuristic::MANHATTAN) ? 4 : 8;

    auto inBounds = [&](int r, int c) {
        return r >= 0 && r < rows && c >= 0 && c < cols;
        };
    auto isFree = [&](int r, int c) { return v[r][c] == 0; };
    auto inClosedList = [&](int r, int c) {
        for (const auto& cn : closedList)
            if (cn.r == r && cn.c == c) return true;
        return false;
        };

    if (verbose) std::cout << "Generating successors of q:\n";

    for (int k = 0; k < nDirs; ++k)
    {
        int nr = q.r + dr[k];
        int nc = q.c + dc[k];

        if (!inBounds(nr, nc) || !isFree(nr, nc)) continue;
        if (inClosedList(nr, nc))                  continue;

        double succ_g = q.g + g_cost(q.r, q.c, nr, nc);
        double succ_h = h_cost(nr, nc, gr, gc);
        double succ_f = f_cost(succ_g, succ_h);

        // Goal check
        if (nr == gr && nc == gc) {
            if (verbose)
                std::cout << "  Goal found at (" << nr << "," << nc << ")"
                << "  parent=(" << q.r << "," << q.c << ")"
                << "  g=" << succ_g << "  h=" << succ_h
                << "  f=" << succ_f << "\n";
            closedList.push_back(SimpleNode{ nr, nc, succ_g, succ_h, succ_f, q.r, q.c });
            return true;
        }

        // Update OPEN if this path is better
        bool handledByOpen = false;
        for (auto& on : openList) {
            if (on.r == nr && on.c == nc) {
                if (succ_f < on.f) {
                    on.g = succ_g; on.h = succ_h; on.f = succ_f;
                    on.pr = q.r;   on.pc = q.c;
                    if (verbose)
                        std::cout << "  updated  (" << nr << "," << nc << ")"
                        << "  parent=(" << q.r << "," << q.c << ")"
                        << "  g=" << succ_g << "  h=" << succ_h
                        << "  f=" << succ_f << "  (improved)\n";
                }
                handledByOpen = true;
                break;
            }
        }
        if (handledByOpen) continue;

        // New cell — add to OPEN
        openList.push_back(SimpleNode{ nr, nc, succ_g, succ_h, succ_f, q.r, q.c });
        if (verbose)
            std::cout << "  successor (" << nr << "," << nc << ")"
            << "  parent=(" << q.r << "," << q.c << ")"
            << "  g=" << succ_g << "  h=" << succ_h
            << "  f=" << succ_f << "\n";
    }
    return false;
}

void PathPlanning::updateVisitedOverlay(int rows, int cols)
{
    visited.assign(rows, std::vector<int>(cols, 0));
    for (const auto& cn : closedList) visited[cn.r][cn.c] = 1;
}

std::vector<std::pair<int, int>>
PathPlanning::reconstructPath(int gr, int gc) const
{
    std::vector<std::pair<int, int>> path;
    int cr = gr, cc = gc;

    while (true) {
        path.push_back({ cr, cc });
        for (const auto& cn : closedList) {
            if (cn.r == cr && cn.c == cc) {
                if (cn.pr == -1 && cn.pc == -1) return path;  // reached start
                cr = cn.pr;
                cc = cn.pc;
                break;
            }
        }
    }
}

void PathPlanning::printResult(const std::vector<std::pair<int, int>>& path,
    int sr, int sc, int gr, int gc,
    int iteration, double timeMs) const
{
    std::cout << "\n===============================================\n";
    std::cout << "Heuristic : " << heuristicName() << "\n";
    std::cout << "Path found in " << iteration << " iteration(s)!\n";
    std::cout << "Nodes expanded : " << closedList.size() << "\n";
    std::cout << "Time           : " << timeMs << " ms\n";

    std::cout << "\nPath (goal -> start): ";
    for (const auto& p : path)
        std::cout << "(" << p.first << "," << p.second << ") ";
    std::cout << "\n";

    std::cout << "Path (start -> goal): ";
    for (int i = static_cast<int>(path.size()) - 1; i >= 0; --i)
        std::cout << "(" << path[i].first << "," << path[i].second << ") ";
    std::cout << "\n";

    std::cout << "Path length : " << path.size() << " cells\n";

    std::cout << "\nPath Grid (@ = path):\n";
    printPathGrid(v, path, sr, sc, gr, gc);
}


// ============================================================
//  AStar_Planner — orchestrates the pipeline, returns result
// ============================================================
PathPlanning::PlannerResult PathPlanning::AStar_Planner()
{
    PlannerResult result;

    const int sr = startR, sc = startC;
    const int gr = goalR, gc = goalC;

    int rows = static_cast<int>(v.size());
    int cols = static_cast<int>(v[0].size());

    // Step 1 — show grids (always shown regardless of verbose)
    printStartupInfo(sr, sc, gr, gc);

    // Step 2 — validate
    if (!validateInputs(sr, sc, gr, gc))
        return result;

    // Step 3 — seed OPEN
    openList.clear();
    closedList.clear();
    {
        double start_h = h_cost(sr, sc, gr, gc);
        openList.push_back(SimpleNode{ sr, sc, 0.0, start_h, f_cost(0.0, start_h), -1, -1 });
    }

    if (verbose) {
        std::cout << "\nA* Search Started\n";
        std::cout << "Open list initialised with start node ("
            << sr << "," << sc << ")\n";
    }

    bool goalFound = false;
    int  iteration = 0;

    // Start timer
    auto startTime = std::chrono::high_resolution_clock::now();

    // Step 4 — main search loop
    while (!openList.empty())
    {
        ++iteration;

        if (verbose) {
            std::cout << "\n--- Iteration " << iteration << " ---\n";
            std::cout << "Open list (" << openList.size() << " entries):\n";
            for (size_t i = 0; i < openList.size(); ++i) {
                const auto& n = openList[i];
                std::cout << "  " << i << ": (" << n.r << "," << n.c << ")"
                    << "  g=" << n.g << "  h=" << n.h << "  f=" << n.f << "\n";
            }
        }

        size_t min_idx = selectBestNode();
        SimpleNode q = openList[min_idx];
        openList.erase(openList.begin() + min_idx);

        if (verbose)
            std::cout << "\nSelected q: (" << q.r << "," << q.c << ")"
            << "  g=" << q.g << "  h=" << q.h << "  f=" << q.f << "\n";

        // Skip stale duplicates
        bool alreadyClosed = false;
        for (const auto& cn : closedList)
            if (cn.r == q.r && cn.c == q.c) { alreadyClosed = true; break; }
        if (alreadyClosed) {
            if (verbose) std::cout << "  (stale entry - already expanded, skipping)\n";
            continue;
        }

        closedList.push_back(q);
        if (verbose)
            std::cout << "Pushed q (" << q.r << "," << q.c << ") onto closed list\n";

        goalFound = expandSuccessors(q, gr, gc);
        if (goalFound) break;

        if (verbose) {
            updateVisitedOverlay(rows, cols);
            std::cout << "\nVisited overlay after iteration " << iteration << ":\n";
            printVisitedGrid(v, visited, sr, sc, gr, gc);
            std::cout << "-----------------------------------------------\n";
        }
    }

    // Stop timer
    auto endTime = std::chrono::high_resolution_clock::now();
    double timeMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();

    // Step 5 — final visited overlay
    updateVisitedOverlay(rows, cols);
    std::cout << "\nFinal visited overlay:\n";
    printVisitedGrid(v, visited, sr, sc, gr, gc);

    // Fill result
    result.iterations = iteration;
    result.nodesExpanded = static_cast<int>(closedList.size());
    result.pathFound = goalFound;
    result.timeMs = timeMs;

    // Step 6 — output
    if (goalFound) {
        auto path_goal_to_start = reconstructPath(gr, gc);
        result.path.assign(path_goal_to_start.rbegin(), path_goal_to_start.rend());
        result.pathLength = static_cast<int>(result.path.size());
        printResult(path_goal_to_start, sr, sc, gr, gc, iteration, timeMs);
    }
    else {
        std::cout << "\n===============================================\n";
        std::cout << "No path found - open list exhausted.\n";
    }

    return result;
}