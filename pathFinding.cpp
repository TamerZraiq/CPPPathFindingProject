/*
* Tamer Zraiq(G00425053)
* C++ Programming - 4th Year 2026
* pathFinding.cpp
*/

#include <iostream>
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
}

/* geeksforgeeks:
A* Search Algorithm
1.  Initialize the open list
2.  Initialize the closed list
    put the starting node on the open
    list (you can leave its f at zero)
3.  while the open list is not empty
    a) find the node with the least f on
       the open list, call it "q"
    b) pop q off the open list

    c) generate q's 8 successors and set their
       parents to q

    d) for each successor
        i) if successor is the goal, stop search

        ii) else, compute both g and h for successor
          successor.g = q.g + distance between
                              successor and q
          successor.h = distance from goal to
          successor (This can be done using many
          ways, we will discuss three heuristics-
          Manhattan, Diagonal and Euclidean
          Heuristics)

          successor.f = successor.g + successor.h
        iii) if a node with the same position as
            successor is in the OPEN list which has a
           lower f than successor, skip this successor
        iV) if a node with the same position as
            successor  is in the CLOSED list which has
            a lower f than successor, skip this successor
            otherwise, add  the node to the open list
     end (for loop)

    e) push q on the closed list
    end (while loop)

*/