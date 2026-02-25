/*
* Tamer Zraiq (G00425053)
* C++ Programming Path Finding Project - 4th Year 2026
* pathFinding.cpp
*/

#include <iostream>
#include "pathFinding.h"

// Helper: print the raw numeric grid
static void printRawGrid(const std::vector<std::vector<int>>& grid)
{
    std::cout << "Raw Grid:\n";
    for (const auto& row : grid) {
        for (int cell : row)
            std::cout << cell << " ";
        std::cout << "\n";
    }
}

// Helper: print the coordinate reference grid showing (row,col) for each cell
static void printCoordinateGrid(int rows, int cols)
{
    std::cout << "Coordinate Grid:\n";
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c)
            std::cout << "(" << r << "," << c << ") ";
        std::cout << "\n";
    }
}

// Helper: print the semantic grid (S=start, G=goal, #=obstacle, .=free)
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

// Helper: print the visited overlay (* = visited free cell)
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

// Main A* planner
void PathPlanning::AStar_Planner()
{
    //grid dimensions
    int rows = static_cast<int>(v.size());
    int cols = static_cast<int>(v[0].size());

    //start / goal coordinates
    const int sr = 0, sc = 0;   // start  (top-left)
    const int gr = 4, gc = 4;   // goal   (bottom-right)

    //bounds / free-cell lambdas
    auto inBounds = [&](int r, int c) {
        return r >= 0 && r < rows && c >= 0 && c < cols;
        };
    auto isFree = [&](int r, int c) {
        return v[r][c] == 0;
        };

    //print grids
    printRawGrid(v);

    std::cout << "\nGrid dimensions: " << rows << "x" << cols << "\n";
    std::cout << "Start: (" << sr << "," << sc << ")  "
        << "Goal: (" << gr << "," << gc << ")\n";

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

    //Start node: g=0, h=h_cost, f=g+h
    {
        int start_h = h_cost(sr, sc, gr, gc);
        openList.push_back(SimpleNode{ sr, sc, 0, start_h, f_cost(0, start_h), -1, -1 });
    }

    std::cout << "\n-----------------------------------------------\n";
    std::cout << "Open list initialised with start node (" << sr << "," << sc << ")\n";

    bool goalFound = false;

    //main A* loop
    while (!openList.empty())
    {
        //print current open list
        std::cout << "\nOpen list (" << openList.size() << " entries):\n";
        for (size_t i = 0; i < openList.size(); ++i) {
            const auto& n = openList[i];
            std::cout << "  " << i << ": (" << n.r << "," << n.c << ")"
                << "  g=" << n.g << "  h=" << n.h << "  f=" << n.f << "\n";
        }

        //pick node q with lowest f (tie-break: lower g)
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

        //generate 8 successors of q
        std::cout << "Generating successors of q:\n";
        for (int k = 0; k < 8; ++k)
        {
            int nr = q.r + dr8[k];
            int nc = q.c + dc8[k];

            if (!inBounds(nr, nc) || !isFree(nr, nc))
                continue;

            int succ_g = q.g + g_cost(q.r, q.c, nr, nc);
            int succ_h = h_cost(nr, nc, gr, gc);
            int succ_f = f_cost(succ_g, succ_h);

            //goal check
            if (nr == gr && nc == gc) {
                std::cout << "  Goal found at (" << nr << "," << nc << ")"
                    << "  parent=(" << q.r << "," << q.c << ")"
                    << "  g=" << succ_g << "  h=" << succ_h << "  f=" << succ_f << "\n";
                closedList.push_back(SimpleNode{ nr, nc, succ_g, succ_h, succ_f, q.r, q.c });
                goalFound = true;
                break;   //stop expanding successors for this node
            }

            //skip if a better-or-equal node is already in OPEN
            bool skip = false;
            for (const auto& on : openList) {
                if (on.r == nr && on.c == nc && on.f <= succ_f) { skip = true; break; }
            }
            if (skip) continue;

            //skip if a better-or-equal node is already in CLOSED
            for (const auto& cn : closedList) {
                if (cn.r == nr && cn.c == nc && cn.f <= succ_f) { skip = true; break; }
            }
            if (skip) continue;

            //add successor to OPEN
            openList.push_back(SimpleNode{ nr, nc, succ_g, succ_h, succ_f, q.r, q.c });
            std::cout << "  successor (" << nr << "," << nc << ")"
                << "  parent=(" << q.r << "," << q.c << ")"
                << "  g=" << succ_g << "  h=" << succ_h << "  f=" << succ_f << "\n";
        }

        //push q onto CLOSED
        closedList.push_back(q);
        std::cout << "Pushed q (" << q.r << "," << q.c << ") onto closed list\n";

        if (goalFound) break;   //exit the main while-loop

        //update visited overlay (all cells expanded so far)
        visited = std::vector<std::vector<int>>(rows, std::vector<int>(cols, 0));
        for (const auto& cn : closedList) {
            visited[cn.r][cn.c] = 1;
        }

        std::cout << "\nVisited overlay after this expansion:\n";
        printVisitedGrid(v, visited, sr, sc, gr, gc);
        std::cout << "-----------------------------------------------\n";
    }

    //final result
    if (goalFound) {
        std::cout << "\n=== Path found! ===\n";
        // The goal is the last entry pushed into closedList.
        // Walk back through parent pointers to reconstruct the path.
        std::vector<std::pair<int, int>> path;
        int cr = gr, cc = gc;
        while (cr != -1 && cc != -1) {
            path.push_back({ cr, cc });
            // find the node in closedList that matches (cr,cc)
            bool found = false;
            for (const auto& cn : closedList) {
                if (cn.r == cr && cn.c == cc) {
                    cr = cn.pr;
                    cc = cn.pc;
                    found = true;
                    break;
                }
            }
            if (!found) break;
        }

        std::cout << "Path (goal -> start): ";
        for (const auto& p : path)
            std::cout << "(" << p.first << "," << p.second << ") ";
        std::cout << "\n";

        std::cout << "Path (start -> goal): ";
        for (int i = static_cast<int>(path.size()) - 1; i >= 0; --i)
            std::cout << "(" << path[i].first << "," << path[i].second << ") ";
        std::cout << "\n";
    }
    else {
        std::cout << "\n=== No path found. ===\n";
    }
}