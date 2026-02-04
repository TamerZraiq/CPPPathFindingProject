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
    for (const auto& row : v){
        for (int cell : row){
            std::cout << cell << " ";
        }
        std::cout << "\n";
    }
    std::cout << "\n";
	//printing grid as semantic grid 
    int sr = 0, sc = 0;   // start row, start column
    int gr = 4, gc = 4;   // goal row, goal column
    
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