/*
* Tamer Zraiq(G00425053)
* C++ Programming Path Finding Project - 4th Year 2026
* pathFinding.h
*/

#pragma once
#ifndef PATH_PLANNING_H //if not defined
#define PATH_PLANNING_H //define it
#include <vector>
#include <iostream>


class PathPlanning
{
	public:
		void AStar_Planner();
		

	private:
		bool inBounds(int r, int c, int rows, int cols);
		bool isFree(int r, int c);
		std::vector<std::vector<int>>v = {
			{0, 0, 0, 0, 0},
			{0, 1, 1, 0, 0},
			{1, 0, 0, 0, 1},
			{1, 1, 0, 1, 1},
			{1, 0, 0, 0, 0} 
		};
		
};

#endif // !PATH_PLANNING_H