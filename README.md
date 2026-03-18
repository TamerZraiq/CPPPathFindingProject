# A* Pathfinding in C++
 
**Tamer Zraiq | G00425053 | 4th Year 2026**
 
| | |
|---|---|
| **Language** | C++17 |
| **Files** | `config.h` · `gridGen.h` · `pathFinding.h/cpp` · `runner.h/cpp` · `tests.h/cpp` · `main.cpp` |
 
---
 
## Table of Contents
1. [Project Overview]
2. [The A* Algorithm]
3. [AI Tools Declaration]
4. [Development History]
5. [Code Walkthrough]
   - [config.h]
   - [gridGen.h]
6. [Heuristic Functions]
7. [Architecture and Design Choices]
8. [Testing]
9. [C-Style Code: What the AI Got Wrong]
10. [Limitations and Improvements]
11. [Project Planning]
12. [Problems Encountered]
13. [Reflection]
14. [References]
    
## 1. Project Overview
 
This project implements the A* search algorithm in C++ for 2D grid-based pathfinding in Visual Studio 2022 IDE. The program finds the shortest path between a start cell and a goal cell on a grid that may contain obstacles. It supports three heuristic functions, random grid generation with a built-in solvability check, and a comparison mode that runs all three heuristics on the same grid and prints a results table.

The implementation follows the A* structure from GeeksforGeeks [4] and builds on it with a full C++ class-based design. The PathPlanning class handles the entire search pipeline internally: open and closed list management, successor generation, cost computation, and path reconstruction. Outside the class, config.h controls all runtime parameters so the grid size, start and goal positions, heuristic choice, and output verbosity can all be changed in one place without touching the algorithm code. The project was developed in four stages: getting a correct single-file implementation, splitting it into a modular multi-file structure, adding the config system and the three heuristics, and finally a code review pass. Each stage is covered in Section 4.

### How to Run
 
Once setting up the structure and heirarchy of the project files, compile/build the files and run using the Start Without Debugging button. 

On startup the program presents a menu:
 
```
================================
   A* Pathfinding - Tamer Z
================================
  1. Run A* planner
  2. Run test suite
================================
Choice:
``` 
**Option 1** runs the planner. What happens next depends on `config.h`:
- If `RANDOMISE_GRID = true`, a random grid is generated and the seed is printed so the layout can be reproduced
- If `COMPARE_HEURISTICS = true`, all three heuristics run on the same grid and a comparison table is printed at the end
- If `COMPARE_HEURISTICS = false`, only the heuristic set in `ACTIVE_HEURISTIC` runs
- If `SHOW_ITERATION_TRACE = true`, the open list, selected node, and visited overlay are printed after every iteration
 
**Option 2** runs the full test suite of seven cases.
 
### Configuring a Run
 
All parameters that are modifiable are in `config.h`.
 
| Parameter | What it controls |
|---|---|
| `GRID_ROWS` / `GRID_COLS` | Grid dimensions |
| `START_ROW` / `START_COL` | Start position |
| `GOAL_ROW` / `GOAL_COL` | Goal position |
| `RANDOMISE_GRID` | Random grid or fixed default |
| `OBSTACLE_DENSITY` | Fraction of cells that become obstacles (0.0 to ~0.45) |
| `RANDOM_SEED_AUTO` | New layout every run, or fixed seed |
| `RANDOM_SEED` | Specific seed when auto is off |
| `COMPARE_HEURISTICS` | Run all three or just one |
| `ACTIVE_HEURISTIC` | Which heuristic when not comparing (`"MANHATTAN"`, `"EUCLIDEAN"`, `"CHEBYSHEV"`) |
| `SHOW_ITERATION_TRACE` | Full per-iteration output or final result only |
 
### Expected Output
 
On a successful run with `COMPARE_HEURISTICS = true` and `SHOW_ITERATION_TRACE = false`, you will see the startup grids printed once, then each heuristic's final result, then the comparison table. Screenshots of the expected terminal output are shown below.
 
(add screenshots later for startup grids (raw grid, semantic grid, coordinate grid), path grid for one heuristic, and the result table)

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
The project went through four clear stages. Each one built on a working version of the previous, which made it possible to test and verify changes without breaking everything.

 
### Stage 1: Getting a Working Algorithm
 
The first version was a single file built by following the GeeksforGeeks A* structure \[4\] directly. The core loop, open and closed lists, successor generation, and path reconstruction were all implemented from that reference. During testing several bugs showed up that needed to be found and fixed.
 
The most significant was an outer `for (k < 4)` loop wrapped around the entire search. This caused A* to run four times from scratch on every execution. The output looked plausible at first because the path was found eventually, but the algorithm was restarting completely on each outer iteration.
 
```cpp
// Bug: outer loop that should not exist
for (int k = 0; k < 4; ++k) {
    while (!openList.empty()) {
        // entire search ran inside here
    }
}
 
// Fix: single while loop
while (!openList.empty()) {
    // ...
}


```
 
Another issue was that when the goal was found during successor expansion, nothing signalled the main loop to stop. The search kept going after the goal was already reached. A `goalFound` flag and a break fixed this.
 
The start node also had its `f` value hardcoded to 0, meaning the heuristic had no effect on the first step. This was corrected to compute `h` properly and set `f = g + h`.
 
Path reconstruction used a `goto` label, which was cleaned up to a proper return statement.

### Stage 2: Splitting into Files and Adding Structure
 
Once the algorithm was correct, the single file was split into eight files each with one clear job. This made every later change faster and safer.
 
The key change here was making `AStar_Planner()` return a `PlannerResult` struct instead of being void. Before this, the only way to inspect the result was to read the terminal output. With the struct, test cases and the comparison runner could check outcomes directly without parsing printed text.
 
A `verbose` flag was added so the per-iteration trace could be turned off. Without it, the comparison mode would produce hundreds of lines per run, burying the comparison table.
 
Timing was added using `std::chrono::high_resolution_clock` so `PlannerResult.timeMs` reflects actual measured time.
 
Seven test cases were written at this stage, covering normal paths, no path situations, start equal to goal, start or goal on an obstacle, a minimal 2x2 grid, and a narrow corridor.


### Stage 3: Config File, Random Grids, and Three Heuristics
 
At this point all hardcoded values were moved into `config.h` as `constexpr` values. Grid size, start position, goal position, obstacle density and all behaviour flags live there. Nothing in the algorithm files has magic numbers anymore.
 
Random grid generation was added in `gridGen.h`. After placing obstacles randomly it runs a BFS flood-fill from start to goal to confirm the grid is solvable before returning it. If not, it adjusts the seed and tries again.
 
Euclidean and Chebyshev heuristics were added alongside Manhattan. The important discovery here was that Manhattan must be restricted to 4-directional movement. Using Manhattan with 8-directional movement makes it inadmissible because it over-estimates diagonal moves \[2\] \[3\]. Chebyshev is the correct heuristic for 8-directional movement, and Euclidean also works well with it.
 
At the same time `h` and `f` in `SimpleNode` were changed from `int` to `double`. When Euclidean was first added with `int` storage, the `sqrt` result was being silently truncated, which made Euclidean behave almost the same as Manhattan and made the comparison pointless.

### Stage 4: Stage 4: Full Code Review and Final State
 
With all three heuristics working and the config system in place, the focus shifted to reviewing the entire codebase for correctness and code quality.

The C++ audit identified four patterns that were substandard: raw int* pointer arrays used for direction offsets instead of std::array, std::rand/std::srand instead of std::mt19937, const char* arrays instead of std::string_view, and linear O(n) scans through the closed list on every neighbour check. These are documented in Section 9 with their modern alternatives.

Beyond the C++ quality issues, the review also confirmed the algorithm behaviour was correct end to end. The stale duplicate handling in the main loop was verified — a node can appear in the open list more than once if its cost is updated after first insertion, and the closed list check before expansion correctly skips those stale copies. The h and f fields being double rather than int was confirmed necessary for Euclidean to produce meaningful fractional values. The movement model tie between heuristic and direction array was checked against all three heuristics to confirm Manhattan never uses diagonals.

The final project state at this point: eight files, config.h controlling all parameters, random grids with BFS pre-validation, three heuristics each with the correct movement model, a comparison table with iteration count, nodes expanded, path length and timing, seven test cases, and a 30-line main.cpp. The PlannerResult return type meant every run could be inspected programmatically rather than by parsing terminal output.
 
---
 
## 5. Code Walkthrough
### 5.1 `config.h`
 
This file's main purpose is to hold every modifiable parameter in one place.
 
```cpp
#pragma once
#ifndef CONFIG_H
#define CONFIG_H
```

`#pragma once` prevents the file from being included more than once per translation unit. The `#ifndef` guard does the same thing in a more traditional way. Both are kept for compatibility.
 
```cpp
constexpr int GRID_ROWS = 8;
constexpr int GRID_COLS = 8;
```
 
`constexpr` means the value is resolved at compile time and substituted wherever it appears. There is no runtime variable. This is preferred over `#define` for numeric values because it is type-safe and respects scope \[6\] \[7\].

```cpp
constexpr int START_ROW = 0;
constexpr int START_COL = 0;
constexpr int GOAL_ROW  = 7;
constexpr int GOAL_COL  = 7;
```

Used in `gridGen.h` to keep those cells free during obstacle placement, and in `runner.cpp` to configure the planner before each run.

```cpp
constexpr bool   RANDOMISE_GRID   = true;
constexpr double OBSTACLE_DENSITY = 0.28;
```

`OBSTACLE_DENSITY` of 0.28 means roughly 28% of cells become obstacles. Going above about 0.45 risks generating grids where no path exists at all, which would cause the generator to loop many times retrying.

```cpp
constexpr bool         RANDOM_SEED_AUTO = true;
constexpr unsigned int RANDOM_SEED      = 42;
```

When `RANDOM_SEED_AUTO` is true, the seed comes from the current Unix timestamp so the grid changes every run. Setting it false and fixing `RANDOM_SEED` makes a specific layout reproducible, which is useful for debugging.

```cpp
constexpr bool COMPARE_HEURISTICS  = true;
#define        ACTIVE_HEURISTIC "MANHATTAN"
constexpr bool SHOW_ITERATION_TRACE = false;
```

`COMPARE_HEURISTICS` runs all three heuristics and prints a table. `ACTIVE_HEURISTIC` was originally a `#define` string, which is inconsistent with every other declaration in this file and unsafe to compare directly, `#define` has no type and no scope and comparing `const char*` with `==` compares pointer addresses not content. It was changed to `constexpr std::string_view` \[6\] so it is type-safe, consistent with the rest of `config.h`, and can be compared with `==` directly in `runner.cpp` without copying into a `std::string` first. `SHOW_ITERATION_TRACE` when true prints the open list, selected node, successors, and visited overlay after every single iteration. 

---

### 5.2 `gridGen.h`

Provides two functions: a Breadth-First Search (BFS) solvability checker and a random grid generator.

#### `isSolvable`
 
```cpp
static bool isSolvable(const std::vector<std::vector<int>>& grid,
                        int sr, int sc, int gr, int gc)
```

`static` gives this function file-scope linkage, meaning it is not visible outside `gridGen.h`. It takes the grid by `const` reference so no copy is made.

```cpp
    std::vector<std::vector<bool>> seen(rows, std::vector<bool>(cols, false));
    std::queue<std::pair<int,int>> q;
    q.push({sr, sc});
    seen[sr][sc] = true;
```

A 2D `seen` array stops the BFS from visiting the same cell twice. The queue holds row-column pairs. The start cell is pushed first and immediately marked so it is not re-added.

```cpp
    while (!q.empty()) {
        auto [r, c] = q.front(); q.pop();
        if (r == gr && c == gc) return true;
```

`auto [r, c]` is C++17 structured binding \[6\]. It unpacks the pair into named variables without needing `.first` and `.second`. The goal check happens immediately after dequeuing.
 
```cpp
        for (int k = 0; k < 8; ++k) {
            int nr = r + dr[k], nc = c + dc[k];
            if (nr >= 0 && nr < rows && nc >= 0 && nc < cols
                && !seen[nr][nc] && grid[nr][nc] == 0)
            {
                seen[nr][nc] = true;
                q.push({nr, nc});
            }
        }
    }
    return false;
```

All eight neighbours are checked. Bounds are checked before the cell is accessed to prevent out-of-range reads. If the queue empties without finding the goal, the function returns false.
 
#### `generateGrid`

```cpp
static std::vector<std::vector<int>> generateGrid(unsigned int& outSeed)
```

The seed is passed by reference so the caller can record which one was actually used.

```cpp
    do {
        ++attempts;
        grid.assign(rows, std::vector<int>(cols, 0));
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                if ((r == sr && c == sc) || (r == gr && c == gc)) continue;
                double roll = static_cast<double>(std::rand()) / RAND_MAX;
                if (roll < OBSTACLE_DENSITY) grid[r][c] = 1;
            }
        }
        if (!isSolvable(grid, sr, sc, gr, gc))
            std::srand(seed + attempts * 7);
    } while (!isSolvable(grid, sr, sc, gr, gc));
```

The `do-while` guarantees at least one generation attempt. Start and goal cells are skipped with `continue` so they are always free. Each cell gets a random roll between 0 and 1. If the result is unsolvable, the seed shifts by `attempts * 7` to escape the same bad layout and the loop retries.
 
---
## 6. Heuristic Functions
## 7. Architecture and Design Choices
## 8. Testing
## 9. C-Style Code: What the AI Got Wrong
## 10. Limitations and Improvements
## 11. Project Planning
## 12. Problems Encountered
## 13. Reflection
## 14. References
---
 
*Tamer Zraiq | G00425053 | C++ Pathfinding Project | 4th Year 2026*
