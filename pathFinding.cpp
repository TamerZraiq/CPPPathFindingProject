/*
* Tamer Zraiq (G00425053)
* C++ Programming Path Finding Project - 4th Year 2026
* pathFinding.cpp
*/

#include <iostream>
#include "pathFinding.h"

// --------------------------------------------------------------------------
// Helper: print the raw numeric grid
// --------------------------------------------------------------------------
static void printRawGrid(const std::vector<std::vector<int>>& grid)
{
    std::cout << "Raw Grid:\n";
    for (const auto& row : grid) {
        for (int cell : row)
            std::cout << cell << " ";
        std::cout << "\n";
    }
}

// --------------------------------------------------------------------------
// Helper: print the semantic grid (S=start, G=goal, #=obstacle, .=free)
// --------------------------------------------------------------------------
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

// --------------------------------------------------------------------------
// Helper: print the coordinate reference grid showing (row,col) for each cell
// --------------------------------------------------------------------------
static void printCoordinateGrid(int rows, int cols)
{
    std::cout << "Coordinate Grid:\n";
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c)
            std::cout << "(" << r << "," << c << ") ";
        std::cout << "\n";
    }
}

// --------------------------------------------------------------------------
// Helper: print the visited overlay
//   S = start, G = goal, # = obstacle, * = expanded, . = not yet visited
// --------------------------------------------------------------------------
static void printVisitedGrid(const std::vector<std::vector<int>>& grid,
    const std::vector<std::vector<int>>& visited,
    int sr, int sc, int gr, int gc)
{
    int rows = static_cast<int>(grid.size());
    for (int r = 0; r < rows; ++r) {
        int cols = static_cast<int>(grid[r].size());
        for (int c = 0; c < cols; ++c) {
            if (r == sr && c == sc)  std::cout << "S ";
            else if (r == gr && c == gc)  std::cout << "G ";
            else if (grid[r][c] == 1)     std::cout << "# ";
            else if (visited[r][c] == 1)  std::cout << "* ";
            else                          std::cout << ". ";
        }
        std::cout << "\n";
    }
}

// --------------------------------------------------------------------------
// Helper: print the final path overlaid on the grid
//   S = start, G = goal, # = obstacle, @ = path cell, . = free
// --------------------------------------------------------------------------
static void printPathGrid(const std::vector<std::vector<int>>& grid,
    const std::vector<std::pair<int, int>>& path,
    int sr, int sc, int gr, int gc)
{
    int rows = static_cast<int>(grid.size());
    int cols = static_cast<int>(grid[0].size());

    std::vector<std::vector<bool>> onPath(rows, std::vector<bool>(cols, false));
    for (const auto& p : path)
        onPath[p.first][p.second] = true;

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

// Main A* planner
void PathPlanning::AStar_Planner()
{
    //grid dimensions
    int rows = static_cast<int>(v.size());
    int cols = static_cast<int>(v[0].size());

    //start / goal coordinates
    const int sr = 0, sc = 0;   // start (top-left)
    const int gr = 4, gc = 4;   // goal  (bottom-right)

    //bounds / free-cell lambdas
    auto inBounds = [&](int r, int c) {
        return r >= 0 && r < rows && c >= 0 && c < cols;
        };
    auto isFree = [&](int r, int c) {
        return v[r][c] == 0;
        };

    //print startup grids
    printRawGrid(v);
    std::cout << "\nGrid dimensions: " << rows << "x" << cols << "\n";
    std::cout << "Start: (" << sr << "," << sc << ")  "
        << "Goal:  (" << gr << "," << gc << ")\n";
    std::cout << "\nSemantic Grid:\n";
    printSemanticGrid(v, sr, sc, gr, gc);
    std::cout << "\n";
    printCoordinateGrid(rows, cols);

    //validate start / goal
    if (!inBounds(sr, sc) || !inBounds(gr, gc) ||
        !isFree(sr, sc) || !isFree(gr, gc))
    {
        std::cout << "Invalid start or goal position.\n";
        return;
    }
    if (sr == gr && sc == gc) {
        std::cout << "Start is already the goal. No pathfinding needed.\n";
        return;
    }

    //8-directional movement offsets
    const int dr8[8] = { -1, -1, -1,  0, 0,  1, 1, 1 };
    const int dc8[8] = { -1,  0,  1, -1, 1, -1, 0, 1 };

    //initialise open / closed lists
    openList.clear();
    closedList.clear();

    {
        int start_h = h_cost(sr, sc, gr, gc);
        openList.push_back(SimpleNode{ sr, sc, 0, start_h, f_cost(0, start_h), -1, -1 });
    }

    std::cout << "A* Search Started\n";
    std::cout << "Open list initialised with start node (" << sr << "," << sc << ")\n";

    bool goalFound = false;
    int  iteration = 0;

    //main A* loop
    while (!openList.empty())
    {
        ++iteration;
        std::cout << "\n--- Iteration " << iteration << " ---\n";

        //print current open list
        std::cout << "Open list (" << openList.size() << " entries):\n";
        for (size_t i = 0; i < openList.size(); ++i) {
            const auto& n = openList[i];
            std::cout << "  " << i << ": (" << n.r << "," << n.c << ")"
                << "  g=" << n.g << "  h=" << n.h << "  f=" << n.f << "\n";
        }

        //pick node q with lowest f (tie-break: prefer lower g)
        size_t min_idx = 0;
        for (size_t i = 1; i < openList.size(); ++i) {
            if (openList[i].f < openList[min_idx].f ||
                (openList[i].f == openList[min_idx].f &&
                    openList[i].g < openList[min_idx].g))
            {
                min_idx = i;
            }
        }

        SimpleNode q = openList[min_idx];
        openList.erase(openList.begin() + min_idx);

        std::cout << "\nSelected q: (" << q.r << "," << q.c << ")"
            << "  g=" << q.g << "  h=" << q.h << "  f=" << q.f << "\n";

        //skip q if already in the closed list
        //A node can be added to OPEN more than once before being popped. If a better version was already expanded, discard this stale copy.
        bool alreadyClosed = false;
        for (const auto& cn : closedList) {
            if (cn.r == q.r && cn.c == q.c) { alreadyClosed = true; break; }
        }
        if (alreadyClosed) {
            std::cout << "  (stale entry — already expanded, skipping)\n";
            continue;
        }

        //push q onto CLOSED before expanding
        closedList.push_back(q);
        std::cout << "Pushed q (" << q.r << "," << q.c << ") onto closed list\n";

        //generate 8 successors of q
        std::cout << "Generating successors of q:\n";
        for (int k = 0; k < 8; ++k)
        {
            int nr = q.r + dr8[k];
            int nc = q.c + dc8[k];

            if (!inBounds(nr, nc) || !isFree(nr, nc))
                continue;

            //never re-add a node that has already been expanded
            bool inClosed = false;
            for (const auto& cn : closedList) {
                if (cn.r == nr && cn.c == nc) { inClosed = true; break; }
            }
            if (inClosed) continue;

            int succ_g = q.g + g_cost(q.r, q.c, nr, nc);
            int succ_h = h_cost(nr, nc, gr, gc);
            int succ_f = f_cost(succ_g, succ_h);

            //goal check
            if (nr == gr && nc == gc) {
                std::cout << "  Goal found at (" << nr << "," << nc << ")"
                    << "  parent=(" << q.r << "," << q.c << ")"
                    << "  g=" << succ_g << "  h=" << succ_h
                    << "  f=" << succ_f << "\n";
                closedList.push_back(SimpleNode{ nr, nc, succ_g, succ_h,
                                                 succ_f, q.r, q.c });
                goalFound = true;
                break;
            }

            //update OPEN if a better path to this cell exists -
            //Instead of only skipping, we replace the entry when our new g is lower (better path through q discovered).
            bool handledByOpen = false;
            for (auto& on : openList) {
                if (on.r == nr && on.c == nc) {
                    if (succ_f < on.f) {
                        //better path found — update in place
                        on.g = succ_g;
                        on.h = succ_h;
                        on.f = succ_f;
                        on.pr = q.r;
                        on.pc = q.c;
                        std::cout << "  updated  (" << nr << "," << nc << ")"
                            << "  parent=(" << q.r << "," << q.c << ")"
                            << "  g=" << succ_g << "  h=" << succ_h
                            << "  f=" << succ_f << "  (improved)\n";
                    }
                    //already in OPEN regardless of whether we updated it
                    handledByOpen = true;
                    break;
                }
            }
            if (handledByOpen) continue;

            //not in OPEN or CLOSED: add as a new entry
            openList.push_back(SimpleNode{ nr, nc, succ_g, succ_h,
                                           succ_f, q.r, q.c });
            std::cout << "  successor (" << nr << "," << nc << ")"
                << "  parent=(" << q.r << "," << q.c << ")"
                << "  g=" << succ_g << "  h=" << succ_h
                << "  f=" << succ_f << "\n";
        }

        if (goalFound) break;

        //update and print visited overlay every iteration
        visited = std::vector<std::vector<int>>(rows, std::vector<int>(cols, 0));
        for (const auto& cn : closedList)
            visited[cn.r][cn.c] = 1;

        std::cout << "\nVisited overlay after iteration " << iteration << ":\n";
        printVisitedGrid(v, visited, sr, sc, gr, gc);
        std::cout << "-----------------------------------------------\n";
    }

    //final visited overlay
    visited = std::vector<std::vector<int>>(rows, std::vector<int>(cols, 0));
    for (const auto& cn : closedList)
        visited[cn.r][cn.c] = 1;

    std::cout << "\nFinal visited overlay:\n";
    printVisitedGrid(v, visited, sr, sc, gr, gc);

    //final result
    if (goalFound)
    {
        std::cout << "\n===============================================\n";
        std::cout << "Path found in " << iteration << " iteration(s)!\n";

        //clean path reconstruction
        //Walk back via parent pointers. Stop explicitly when we reach the start node (pr == -1, pc == -1) instead of relying on a failed lookup to terminate the loop.
        std::vector<std::pair<int, int>> path;
        int cr = gr, cc = gc;

        while (true) {
            path.push_back({ cr, cc });

            bool found = false;
            for (const auto& cn : closedList) {
                if (cn.r == cr && cn.c == cc) {
                    // FIX 4: reached the start node — stop cleanly
                    if (cn.pr == -1 && cn.pc == -1)
                        goto path_done;
                    cr = cn.pr;
                    cc = cn.pc;
                    found = true;
                    break;
                }
            }
            if (!found) break;  //guard against malformed closed list
        }
    path_done:

        std::cout << "\nPath (goal -> start): ";
        for (const auto& p : path)
            std::cout << "(" << p.first << "," << p.second << ") ";
        std::cout << "\n";

        std::cout << "Path (start -> goal): ";
        for (int i = static_cast<int>(path.size()) - 1; i >= 0; --i)
            std::cout << "(" << path[i].first << "," << path[i].second << ") ";
        std::cout << "\n";

        std::cout << "Path length: " << path.size() << " cells\n";

        std::cout << "\nPath Grid (@ = path):\n";
        printPathGrid(v, path, sr, sc, gr, gc);
    }
    else
    {
        std::cout << "\n===============================================\n";
        std::cout << "No path found — open list exhausted.\n";
    }
}