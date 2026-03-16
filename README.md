# A* Pathfinding in C++
 
**Tamer Zraiq | G00425053 | 4th Year 2026**
 
| | |
|---|---|
| **Language** | C++17 |
| **Files** | `config.h` · `gridGen.h` · `pathFinding.h/cpp` · `runner.h/cpp` · `tests.h/cpp` · `main.cpp` |
 
---
 
## Table of Contents
 
## 1. Project Overview
This project implements the A* search algorithm in C++ for 2D grid-based pathfinding in Visual Studio 2022 IDE. The program finds the shortest path between a start cell and a goal cell on a grid that may contain obstacles. It supports three different heuristic functions, random grid generation with a solvability check, and a comparison mode that runs all three heuristics on the same grid and prints a results table side by side.
 
All modifiable parameters live in a single `config.h` file. Grid size, start and goal positions, obstacle density, which heuristic to use, and whether to compare all three are all set there. Nothing in the algorithm files needs to be touched between runs.

---
 
## 2. The A* Algorithm
A* is a graph traversal and pathfinding algorithm \[1\] \[4\]. It works by combining the actual cost of reaching a node with an estimated cost to the goal, using that combined value to decide which node to explore next. This makes it significantly more directed than Dijkstra's algorithm, which has no awareness of where the goal is and expands nodes in all directions equally.
 
### f(n) = g(n) + h(n)
 
Every node in the search is scored using this formula \[4\]:
 
- **g(n)** is the exact cost of the path from the start node to node n, accumulated as the search progresses
- **h(n)** is the heuristic estimate of the cost from n to the goal. This is never exact, but it must never over-estimate the true cost — a property called **admissibility** \[1\] \[2\]. If h over-estimates, A* may discard the actual shortest path and return a suboptimal result
- **f(n)** is the sum of the two. The algorithm always expands the node with the lowest f next, which is what directs the search toward the goal rather than outward in all directions
 
### The open and closed lists
 
A* maintains two lists throughout the search \[4\]:
 
- **Open list**: nodes that have been discovered and scored but not yet expanded
- **Closed list**: nodes that have already been expanded and do not need to be revisited
 
### How the search runs
 
This is the structure the implementation follows, based on the GeeksforGeeks breakdown \[4\]:
 
```
1. Initialise the open list
2. Initialise the closed list
   Put the start node on the open list (f can be left as zero)
 
3. While the open list is not empty:
 
   a) Find the node with the lowest f on the open list — call it q
   b) Pop q off the open list
 
   c) Generate q's successors and set their parent to q
 
   d) For each successor:
      i)  If the successor is the goal — stop, path found
 
      ii) Otherwise compute g, h, and f:
            successor.g = q.g + distance from q to successor
            successor.h = heuristic distance from successor to goal
            successor.f = successor.g + successor.h
 
      iii) If a node with the same position exists in the OPEN list
           with a lower f — skip this successor
 
      iv)  If a node with the same position exists in the CLOSED list
           with a lower f — skip this successor
           Otherwise add the successor to the open list
 
   e) Push q onto the closed list
 
If the open list empties without finding the goal — no path exists
```
 
### Parent pointers and path recovery
 
Each node stores the coordinates of the node it was reached from \[4\]. When the goal is found, the path is recovered by following these parent references backwards from the goal to the start, then reversing the sequence. In this implementation that chain is stored in the `pr` and `pc` fields of `SimpleNode`, with `-1, -1` used as the sentinel value to mark the start node.

---
## 3. AI Tools Declaration
 
AI tools were used throughout this project for several supporting tasks. Claude was used to help structure code across multiple files, add comments to functions, suggest fixes for bugs that were found throughout implementation, and clear up the feasibility of certain options. The algorithm logic, design decisions, heuristic matching, and the critical analysis in Section 9 were developed through my own understanding and review of the generated code.
 
AI was not used to simply generate and submit code. Each piece of generated code was reviewed, tested, broken, and fixed. A number of structural bugs in the initial output had to be identified and corrected before the algorithm produced correct results, and several C-style patterns were identified and documented as examples of what the AI did not handle well from a C++17 standpoint.
 
---
 
## 4. Development History
---
 
## 5. Code Walkthrough
 
---
## 6. Heuristic Functions
## 7. Architecture and Design Choices
## 9. C-Style Code: What the AI Got Wrong
## 10. Limitations and Improvements
## 11. Project Planning
## 12. Problems Encountered
## 13. Reflection
## 14. References
---
 
*Tamer Zraiq | G00425053 | C++ Pathfinding Project | 4th Year 2026*
