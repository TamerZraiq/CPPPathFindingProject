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
		struct Position {
			int row;
			int col;
		};
		struct Node { int f, g, r, c; };
		struct SimpleNode { int r; int c; int g; int h; int f; int pr; int pc; };

		std::vector<std::vector<int>>v = {
			{0, 0, 0, 0, 0},
			{0, 1, 1, 0, 0},
			{1, 0, 0, 0, 1},
			{1, 1, 0, 1, 1},
			{1, 0, 0, 0, 0} 
		};
		std::vector<std::vector<int>>visited;
		std::vector<SimpleNode> openList;   // nodes discovered and waiting to be expanded
		std::vector<SimpleNode> closedList; // nodes already expanded (not used yet)

		// Manhattan distance (h) heuristic function
		int manhattanDistance(const Position &a, const Position &b) {
			return std::abs(a.row - b.row) + std::abs(a.col - b.col);
		}

		// Cost from one cell to an adjacent cell (g). For 4-neighborhood, cost = 1
		int g_cost(int from_r, int from_c, int to_r, int to_c) {
			(void)from_r;
			(void)from_c;
			(void)to_r;
			(void)to_c;
			return 1;
		}

		// Heuristic cost to goal (h) using Manhattan distance
		int h_cost(int row, int col, int gr, int gc) {
			Position a{row, col};
			Position b{ gr, gc };
			return manhattanDistance(a, b);
		}

		// Total estimated cost (f = g + h)
		int f_cost(int g, int h) {
			return g + h;
		}
		
};

#endif // !PATH_PLANNING_H