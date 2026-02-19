/*
* Tamer Zraiq(G00425053)
* C++ Programming - 4th Year 2026
* pathFinding.cpp
*/

#include <iostream>
#include <limits>
#include "pathFinding.h"

void PathPlanning::AStar_Planner()
{
    //printing the grid as is
    for (const auto& row : v) {
        for (int cell : row) {
            std::cout << cell << " ";
        }
        std::cout << "\n";
    }
    std::cout << "\n";
    //printing grid as semantic grid 
    int sr = 0, sc = 0;   // start row, start column
    int gr = 4, gc = 4;   // goal row, goal column


    //for validating the heigh and width of the grid (row & col)
    int rows = v.size();
    int cols = v[0].size();

    //output the dimensions of the grid
    std::cout << "\nGrid Dimensions: " << rows << "x" << cols << "\n";

    //making sure the starting/goal point coordinate is real
    auto inBounds = [&](int r, int c) {return r >= 0 && r < rows && c >= 0 && c < cols; };

    //making sure the cell is free (not an obstacle)
    auto isFree = [&](int r, int c) {return v[r][c] == 0; };

    //validating that the start and goal positions are within bounds and free
    if (!inBounds(sr, sc) || !inBounds(gr, gc) || !isFree(sr, sc) || !isFree(gr, gc)) {
        std::cout << "Invalid start or goal position.\n";
        return;
    }

    std::cout << "Start: (" << sr << "," << sc << ")\n";
    std::cout << "Goal : (" << gr << "," << gc << ")\n\n";

    for (int r = 0; r < v.size(); r++) {//nested for loop to traverse the 2D vector
        for (int c = 0; c < v[r].size(); c++) {
            if (r == sr && c == sc)
                std::cout << "S "; //marking the start position when the row and column match the sr and sc (0,0)
            else if (r == gr && c == gc)
                std::cout << "G ";//marking the goal/finish position when the row and column match the gr and gc (4,4)
            else if (v[r][c] == 1)
                std::cout << "# ";//if its a 1 which means obstacle, print it as #
            else
                std::cout << ". ";//anything other than 1 is free space, print it as .
        }
        std::cout << "\n";
    }

    if (sr == gr && sc == gc) { //more validation
        std::cout << "Start is the goal. No pathfinding needed.\n";
        return;
    }
    
    //discovering what valid moves can be made, discovered cells 
    int dr[4] = { -1, 1, 0, 0 };//rows go down when increased 
    int dc[4] = { 0, 0, -1, 1 };//columns move right when increased , so any move is just an offset to (r,c)

    std::cout << "\nValid neighbors of Start:\n";
    for (int k = 0; k < 4; k++) { //when k=0, dr[0] = -1, and dc[0] = 0, so (-1, 0) so move up by 1 since its decreased by 1, same goes for k 0-3
        int nr = sr + dr[k];//neighbor cell is the start cell + the offset/move, so since k=0 is up, its off the grid
        int nc = sc + dc[k];//however k=3 is move to the right by 1 (0,1) so its valid

        if (inBounds(nr, nc) && isFree(nr, nc)) {//making sure its valid and printing the valid neighbor/possible move
            int g = g_cost(sr, sc, nr, nc);
            int h = h_cost(nr, nc, gr, gc);
            int f = f_cost(g, h);
            // collect neighbor into open list
            openList.push_back(SimpleNode{ nr, nc, g, h, f });
            std::cout << "(" << nr << "," << nc << ")  g=" << g << "  h=" << h << "  f=" << f << "\n";


        }
    }

    // show open list contents (simple view)
    std::cout << "\nOpen list contents (" << openList.size() << " entries):\n";
    for (size_t i = 0; i < openList.size(); ++i) {
        const auto& n = openList[i];
        std::cout << i << ": (" << n.r << "," << n.c << ") g=" << n.g << " h=" << n.h << " f=" << n.f << "\n";
    }

    //visisted here means that it has been discovered, not physicially there
    visited = std::vector<std::vector<int>>(rows, std::vector<int>(cols, 0)); //creating a new grid to show whats visited and whats not
    visited[sr][sc] = 1; //1 means visited, 0 means not, since it starts from start coordinate themn thats visited 
    for (int k = 0; k < 4; k++) {
        int nr = sr + dr[k];
        int nc = sc + dc[k];//as before it generates whats a valid neighbor cell
        if (inBounds(nr, nc) && isFree(nr, nc)) {
            visited[nr][nc] = 1;//marking the neighbor as visited if its a valid point
        }
    }
    std::cout << "\nVisited overlay (with neighbors):\n";
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            if (r == sr && c == sc) std::cout << "S ";
            else if (r == gr && c == gc) std::cout << "G ";
            else if (v[r][c] == 1) std::cout << "# ";
            else if (visited[r][c] == 1) std::cout << "* ";//free visited cells are shown as *
            else std::cout << ". ";
        }//so this visited section is just to make sure that the the already discoverd cells no need to discover them again as they are considered and reachable
        std::cout << "\n";
    }


}
