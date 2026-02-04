/*
* Tamer Zraiq(G00425053)
* C++ Programming - 4th Year 2026
* pathFinding.cpp
*/

#include <iostream>
#include "pathFinding.h"

void PathPlanning::AStar_Planner()
{
    for (const auto& row : v){
        for (int cell : row){
            std::cout << cell << " ";
        }
        std::cout << "\n";
    }
}