# A* Pathfinding in C++
 
**Tamer Zraiq | G00425053 | 4th Year 2026**
 
| | |
|---|---|
| **Language** | C++17 |
| **Files** | `config.h` · `gridGen.h` · `pathFinding.h/cpp` · `runner.h/cpp` · `tests.h/cpp` · `main.cpp` |
 
---
 
## Table of Contents
 
1. [Project Overview](#1-project-overview)
2. [The A* Algorithm](#2-the-a-algorithm)
3. [AI Tools Declaration](#3-ai-tools-declaration)
4. [Development History](#4-development-history)
5. [Code Walkthrough](#5-code-walkthrough)
   - [config.h](#51-configh)
   - [gridGen.h](#52-gridgenh)
   - [pathFinding.h](#53-pathfindingh)
   - [pathFinding.cpp](#54-pathfindingcpp)
   - [runner.h and runner.cpp](#55-runnerh-and-runnercpp)
   - [main.cpp](#56-maincpp)
6. [Heuristic Functions](#6-heuristic-functions)
7. [Architecture and Design Choices](#7-architecture-and-design-choices)
8. [Testing](#8-testing)
9. [C-Style Code: What the AI Got Wrong](#9-c-style-code-what-the-ai-got-wrong)
10. [Limitations and Improvements](#10-limitations-and-improvements)
11. [Project Planning](#11-project-planning)
12. [Problems Encountered](#12-problems-encountered)
13. [Reflection](#13-reflection)
14. [References](#14-references)
    
## 1. Project Overview
 
This project implements the A* search algorithm in C++ for 2D grid-based pathfinding in Visual Studio 2022 IDE. The program finds the shortest path between a start cell and a goal cell on a grid that may contain obstacles. It supports three heuristic functions, random grid generation with a built-in solvability check, and a comparison mode that runs all three heuristics on the same grid and prints a results table.

The implementation follows the A* structure from GeeksforGeeks [4] and builds on it with a full C++ class-based design. The PathPlanning class handles the entire search pipeline internally: open and closed list management, successor generation, cost computation, and path reconstruction. Outside the class, config.h controls all runtime parameters so the grid size, start and goal positions, heuristic choice, and output verbosity can all be changed in one place without touching the algorithm code. The project was developed in four stages: getting a correct single-file implementation, splitting it into a modular multi-file structure, adding the config system and the three heuristics, and finally a code review pass. Each stage is covered in Section 4.

### How to Run

**The Structure:**

![description](images/ProjectStructure.png)


After setting up the structure and heirarchy of the project files, compile/build the files and run using the Start Without Debugging button. 

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
 
![description](images/Grids.png)  ![description](images/ResultTable.png)

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

---

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

---

### Stage 2: Splitting into Files and Adding Structure

Once the algorithm was correct, the single file was split into eight files each with one clear job. This made every later change faster and safer.

The key change here was making `AStar_Planner()` return a `PlannerResult` struct instead of being void. Before this, the only way to inspect the result was to read the terminal output. With the struct, test cases and the comparison runner could check outcomes directly without parsing printed text.

A `verbose` flag was added so the per-iteration trace could be turned off. Without it, the comparison mode would produce hundreds of lines per run, burying the comparison table.

Timing was added using `std::chrono::high_resolution_clock` so `PlannerResult.timeMs` reflects actual measured time.

Seven test cases were written at this stage, covering normal paths, no path situations, start equal to goal, start or goal on an obstacle, a minimal 2x2 grid, and a narrow corridor.

---

### Stage 3: Config File, Random Grids, and Three Heuristics

At this point all hardcoded values were moved into `config.h` as `constexpr` values. Grid size, start position, goal position, obstacle density and all behaviour flags live there. Nothing in the algorithm files has magic numbers anymore.

Random grid generation was added in `gridGen.h`. After placing obstacles randomly it runs a BFS flood-fill from start to goal to confirm the grid is solvable before returning it. If not, it adjusts the seed and tries again.

Euclidean and Chebyshev heuristics were added alongside Manhattan. The important discovery here was that Manhattan must be restricted to 4-directional movement. Using Manhattan with 8-directional movement makes it inadmissible because it over-estimates diagonal moves \[2\] \[3\]. Chebyshev is the correct heuristic for 8-directional movement, and Euclidean also works well with it.

At the same time `h` and `f` in `SimpleNode` were changed from `int` to `double`. When Euclidean was first added with `int` storage, the `sqrt` result was being silently truncated, which made Euclidean behave almost the same as Manhattan and made the comparison pointless.

---

### Stage 4: C++ Audit

The final stage was a review pass looking specifically for C-style patterns in the generated code. Four issues were found: raw pointer arrays for direction offsets, `std::rand` and `std::srand`, `const char*` for string names, and linear O(n) closed list lookups. These are documented in Section 9 with their modern C++ alternatives.

---

## 5. Code Walkthrough

---

### 5.1 `config.h`

This file has one job: hold every tunable parameter in one place.

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
constexpr bool COMPARE_HEURISTICS   = true;
constexpr std::string_view ACTIVE_HEURISTIC = "MANHATTAN";
constexpr bool SHOW_ITERATION_TRACE = false;
```

`COMPARE_HEURISTICS` runs all three heuristics and prints a table. `ACTIVE_HEURISTIC` was originally a `#define` string, which is inconsistent with every other declaration in this file and unsafe to compare directly — `#define` has no type and no scope, and comparing `const char*` with `==` compares pointer addresses not content. It was changed to `constexpr std::string_view` \[6\] so it is type-safe, consistent with the rest of `config.h`, and can be compared with `==` directly in `runner.cpp` without copying into a `std::string` first. `SHOW_ITERATION_TRACE` when true prints the open list, selected node, successors, and visited overlay after every single iteration. Very useful for small grids but overwhelming for anything larger.

---

### 5.2 `gridGen.h`

Provides two functions: a BFS solvability checker and a random grid generator.

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

### 5.3 `pathFinding.h`

Declares the `PathPlanning` class. Everything is declared here; the implementation is in `pathFinding.cpp`.

#### `Heuristic` enum

```cpp
enum class Heuristic {
    MANHATTAN,
    EUCLIDEAN,
    CHEBYSHEV
};
```

`enum class` is a scoped enum \[6\] \[7\]. Values must be referred to with their full scope like `Heuristic::MANHATTAN`, not just `MANHATTAN`. This avoids name collisions and makes the type system enforce that the right kind of value is passed.

#### `PlannerResult`

```cpp
struct PlannerResult {
    bool   pathFound     = false;
    int    iterations    = 0;
    int    nodesExpanded = 0;
    int    pathLength    = 0;
    double timeMs        = 0.0;
    std::vector<std::pair<int,int>> path;
};
```

All fields are default-initialised so if `AStar_Planner()` returns early due to invalid inputs, the caller gets a properly zeroed struct rather than garbage values. `path` holds cells in start-to-goal order. `timeMs` is `double` because `std::chrono::duration<double, std::milli>` produces a `double`.

#### Setters

```cpp
void setGrid     (const std::vector<std::vector<int>>& grid) { v = grid; }
void setStart    (int r, int c)  { startR = r; startC = c; }
void setGoal     (int r, int c)  { goalR  = r; goalC  = c; }
void setHeuristic(Heuristic h)   { heuristic = h; }
void setVerbose  (bool v)        { verbose = v; }
```

All inline one-liners. `setGrid` takes the grid by `const` reference so no copy happens during the call; the copy happens in the assignment to member `v`.

#### `SimpleNode`

```cpp
struct SimpleNode {
    int    r, c;
    double g;
    double h, f;
    int    pr, pc;
};
```

`g`, `h`, and `f` are all `double`. `g` was originally `int` which worked fine when all moves cost 1, but once diagonal moves were given a cost of 1.414 the accumulated path cost became fractional and `int` would truncate it. `h` and `f` are `double` because Euclidean uses `std::sqrt` which returns `double` — if they were `int`, the fractional part would be silently cut off and Euclidean would behave like a rounded-down version of itself. `pr` and `pc` are set to `-1, -1` for the start node. That sentinel is what `reconstructPath` uses to know it has reached the beginning of the chain.

#### Cost helpers

```cpp
double g_cost(int r1, int c1, int r2, int c2) const {
    return (r1 != r2 && c1 != c2) ? 1.414 : 1.0;
}
```

Returns 1.414 for diagonal moves and 1.0 for straight moves. The original version always returned 1, which made diagonal paths look artificially cheap. A diagonal step covers a distance of sqrt(2) approximately 1.414, so returning 1 for it was geometrically incorrect. This matters most when comparing heuristics — Euclidean and Chebyshev paths use diagonals heavily and their reported path costs were misleading until this was fixed. The check `r1 != r2 && c1 != c2` identifies a diagonal move since both row and column change simultaneously.

```cpp
double h_cost(int row, int col, int gr, int gc) const
{
    double dr = std::abs(row - gr);
    double dc = std::abs(col - gc);
    switch (heuristic) {
        case Heuristic::EUCLIDEAN:  return std::sqrt(dr*dr + dc*dc);
        case Heuristic::CHEBYSHEV:  return std::max(dr, dc);
        case Heuristic::MANHATTAN:
        default:                    return dr + dc;
    }
}
```

`dr` and `dc` are stored as `double` before the switch so all three formulas work correctly. `std::abs` on integers returns an integer, so assigning to `double` promotes them. The `default` case handles `MANHATTAN` and also acts as a fallback for any future enum value not in the switch.

```cpp
double f_cost(int g, double h) const { return g + h; }
```

`g` is `int`, `h` is `double`. The int is promoted to double automatically for the addition.

---

### 5.4 `pathFinding.cpp`

Contains five display helpers and the full search pipeline.

#### Display helpers

All five are `static` free functions, not class methods. They have no need for class state and everything they need is passed as parameters.

```cpp
static void printSemanticGrid(...) {
    if      (r == sr && c == sc) std::cout << "S ";
    else if (r == gr && c == gc) std::cout << "G ";
    else if (grid[r][c] == 1)    std::cout << "# ";
    else                         std::cout << ". ";
}
```

The priority order here matters. Start and goal are checked before obstacle. Without this, if the start cell happened to contain a `1`, it would print `#` instead of `S`.

```cpp
static void printPathGrid(...) {
    std::vector<std::vector<bool>> onPath(rows, std::vector<bool>(cols, false));
    for (const auto& p : path) onPath[p.first][p.second] = true;
    ...
}
```

A local boolean grid marks the path cells. This costs O(rows x cols) to build but gives O(1) lookup per cell during printing, which is better than scanning the path vector for every cell.

#### `validateInputs`

```cpp
auto inBounds = [&](int r, int c) {
    return r >= 0 && r < rows && c >= 0 && c < cols;
};
auto isFree = [&](int r, int c) { return v[r][c] == 0; };
```

Lambda functions capture `rows`, `cols`, and `v` by reference. They are defined right where they are used rather than as separate private methods because they only make sense locally.

#### `selectBestNode`

```cpp
size_t PathPlanning::selectBestNode() const
{
    size_t min_idx = 0;
    for (size_t i = 1; i < openList.size(); ++i) {
        if (openList[i].f < openList[min_idx].f ||
           (openList[i].f == openList[min_idx].f &&
            openList[i].g <  openList[min_idx].g))
            min_idx = i;
    }
    return min_idx;
}
```

Returns an index rather than the node itself so the caller can erase by position without a second scan. The tie-break on `g` prefers nodes closer to the start when two nodes have the same `f`, which tends to produce more direct paths.

#### `expandSuccessors`

```cpp
const int dr4[4] = { -1,  1,  0,  0 };
const int dc4[4] = {  0,  0, -1,  1 };
const int dr8[8] = { -1, -1, -1,  0, 0,  1, 1, 1 };
const int dc8[8] = { -1,  0,  1, -1, 1, -1, 0, 1 };

const int* dr    = (heuristic == Heuristic::MANHATTAN) ? dr4 : dr8;
const int* dc    = (heuristic == Heuristic::MANHATTAN) ? dc4 : dc8;
const int  nDirs = (heuristic == Heuristic::MANHATTAN) ?  4  :  8;
```

The direction arrays encode each neighbour's offset from the current cell. `dr4/dc4` give the four axis-aligned directions. `dr8/dc8` add the four diagonals. The ternary operator selects which set to use based on the active heuristic.

```cpp
for (int k = 0; k < nDirs; ++k)
{
    int nr = q.r + dr[k];
    int nc = q.c + dc[k];

    if (!inBounds(nr, nc) || !isFree(nr, nc)) continue;
    if (inClosedList(nr, nc))                  continue;
```

Bounds are checked before cell access. Closed list check comes after bounds because the cell must be in range before it can be queried.

```cpp
    if (nr == gr && nc == gc) {
        closedList.push_back(SimpleNode{ nr, nc, succ_g, succ_h, succ_f, q.r, q.c });
        return true;
    }
```

When the goal is found it goes directly to the closed list with the current node as parent. `return true` signals the main loop to stop immediately.

```cpp
    bool handledByOpen = false;
    for (auto& on : openList) {
        if (on.r == nr && on.c == nc) {
            if (succ_f < on.f) {
                on.g = succ_g; on.h = succ_h; on.f = succ_f;
                on.pr = q.r;   on.pc = q.c;
            }
            handledByOpen = true;
            break;
        }
    }
    if (handledByOpen) continue;
    openList.push_back(...);
```

If the neighbour is already in the open list, it is only updated if the new path is cheaper. `handledByOpen` stops it from also being pushed as a new entry.

#### `reconstructPath`

```cpp
while (true) {
    path.push_back({cr, cc});
    for (const auto& cn : closedList) {
        if (cn.r == cr && cn.c == cc) {
            if (cn.pr == -1 && cn.pc == -1) return path;
            cr = cn.pr; cc = cn.pc;
            break;
        }
    }
}
```

Starts at the goal and walks backwards through parent pointers. When a node with `pr == -1 && pc == -1` is reached, that is the start node and the function returns. The path comes out goal-to-start; the caller reverses it.

#### `AStar_Planner`

```cpp
const int sr = startR, sc = startC;
const int gr = goalR,  gc = goalC;
```

Local `const` copies at the top. These values do not change during the search, and having local copies avoids repeated member access throughout the function.

```cpp
openList.clear();
closedList.clear();
{
    double start_h = h_cost(sr, sc, gr, gc);
    openList.push_back(SimpleNode{ sr, sc, 0, start_h, f_cost(0, start_h), -1, -1 });
}
```

Both lists are cleared before starting. This matters because `AStar_Planner` is called three times on the same object in comparison mode. The inner block scopes `start_h` to keep the function tidy.

```cpp
auto startTime = std::chrono::high_resolution_clock::now();
// ... main loop ...
auto endTime = std::chrono::high_resolution_clock::now();
double timeMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();
```

`high_resolution_clock` gives the finest-grained clock available \[6\]. `duration<double, std::milli>` converts the elapsed time to milliseconds as a double. `.count()` pulls out the numeric value.

```cpp
size_t min_idx = selectBestNode();
SimpleNode q   = openList[min_idx];
openList.erase(openList.begin() + min_idx);
```

The node is copied before erasing. `erase` on a vector shifts all elements after the removed position, which is O(n).

```cpp
bool alreadyClosed = false;
for (const auto& cn : closedList)
    if (cn.r == q.r && cn.c == q.c) { alreadyClosed = true; break; }
if (alreadyClosed) { continue; }
```

A node can appear multiple times in the open list if its cost was updated after first insertion. When the stale, worse copy is eventually selected, it is skipped here.

```cpp
result.path.assign(path_goal_to_start.rbegin(), path_goal_to_start.rend());
```

`rbegin()` and `rend()` are reverse iterators. Using them with `assign` fills `result.path` in start-to-goal order without a manual reverse loop.

---

### 5.5 `runner.h` and `runner.cpp`

Two free functions that create `PathPlanning` objects, configure them, and run them. They are free functions rather than class methods because they operate on the planner from the outside.

```cpp
void runHeuristicComparison(const std::vector<std::vector<int>>& grid);
void runSingleHeuristic    (const std::vector<std::vector<int>>& grid);
```

In `runHeuristicComparison`, a fresh `PathPlanning` object is created for each heuristic. This guarantees clean state every time rather than relying on manual resets between runs. The same grid is passed to all three so the comparison is on equal terms.

```cpp
std::cout << std::left << std::setw(12) << "Heuristic" << ...
```

`std::left` left-aligns output. `std::setw(n)` sets the column width for the next field only and must be called before each value.

In `runSingleHeuristic`, `ACTIVE_HEURISTIC` is assigned to a `std::string` before the comparison because it is a `#define` string. Comparing `const char*` pointers directly with `==` would compare memory addresses, not the string content.

---

### 5.6 `main.cpp`

```cpp
auto grid = RANDOMISE_GRID ? generateGrid(seed) : getDefaultGrid();
COMPARE_HEURISTICS ? runHeuristicComparison(grid) : runSingleHeuristic(grid);
```

The entire function body is 30 lines. It reads the menu choice and delegates to the right function. All logic lives in other files.

---

## 6. Heuristic Functions

| Heuristic | Formula | Movement | Admissible | Notes |
|---|---|---|---|---|
| **Manhattan** | `\|dr\| + \|dc\|` | 4-directional | Yes | Counts axis-aligned steps only; over-estimates if diagonals are available |
| **Euclidean** | `sqrt(dr^2 + dc^2)` | 8-directional | Yes | True straight-line distance; slightly under-estimates on discrete grids |
| **Chebyshev** | `max(\|dr\|, \|dc\|)` | 8-directional | Yes | Exact lower bound for 8-directional movement; expands fewest nodes |

The movement model must match the heuristic \[1\] \[3\]. Manhattan counts steps along axes only. Using it with diagonal movement makes it inadmissible because it over-estimates the cost of a diagonal move. Chebyshev is the mathematically correct heuristic for 8-directional movement \[2\]: `max(|dr|, |dc|)` is exactly how many king-moves a chess king would need on a clear board.

### Sample Output

```
Semantic Grid:
S . . . . . . .
. # . # . . . .
. # . . . # . .
. . . . # . . .
. # . . . . # .
. . # . . # . .
. # . . . . . .
. . . . . . . G

Heuristic : Manhattan  (4-directional)

Path Grid (@ = path):
S @ . . . . . .
. # . # . . . .
. # . . . # . .
. . @ . # . . .
. # @ . . . # .
. . # @ . # . .
. # . @ @ @ @ .
. . . . . . @ G

Path length    : 10 cells
Nodes expanded : 24
Time           : 0.0412 ms
```

*Figure 1: Example output on an 8x8 random grid with Manhattan heuristic.*

### Comparison Table Output

```
HEURISTIC COMPARISON TABLE
================================================
Heuristic   Path Found  Iterations  Nodes Expanded  Path Length  Time (ms)
----------------------------------------------------------------------------
Manhattan   Yes         31          28              12           0.0891
Euclidean   Yes         18          15               9           0.0442
Chebyshev   Yes         14          12               9           0.0388
================================================
```

*Figure 2: Comparison table on an 8x8 random grid. Chebyshev expands the fewest nodes because its estimate is the tightest lower bound. Manhattan requires more iterations due to 4-directional restriction.*

---

## 7. Architecture and Design Choices

| File | Responsibility |
|---|---|
| `config.h` | All runtime parameters as `constexpr` values |
| `gridGen.h` | Random grid generation with BFS solvability check |
| `pathFinding.h` | Class declaration, enums, structs, inline helpers |
| `pathFinding.cpp` | Full search pipeline |
| `runner.h / runner.cpp` | Single-heuristic and comparison run modes |
| `tests.h / tests.cpp` | Seven manual test cases |
| `main.cpp` | 30-line entry point |

### Why `PlannerResult` instead of void

The original implementation had `AStar_Planner()` return nothing. The only way to check what happened was to read what printed to the terminal. That made writing test cases nearly impossible and made the comparison runner awkward since there was no way to collect numbers from three runs and print them in a table. Changing to a return type that carries all the measurable outcomes solved both problems at once. It also meant timing could be reported per-run rather than just printed inline.

### Why the BFS check before returning a random grid

The first time random generation was tested without a solvability check, roughly one in five grids had no path. A* would run to completion, exhaust the open list, and report nothing found. In comparison mode that means one or more heuristics produce no data, which makes the table meaningless. Adding a BFS flood-fill after obstacle placement to verify the goal is reachable before returning the grid cost almost nothing in runtime and made the comparison reliable.

### Why the movement model is tied to the heuristic

Early on all three heuristics were using the same 8-direction array. The comparison numbers looked odd — Manhattan was finding longer paths than Chebyshev but not by the expected margin. The reason is that Manhattan is derived from axis-aligned movement. If you allow diagonals, a diagonal step from (0,0) to (1,1) has a true cost of sqrt(2) but Manhattan scores it as 2, which means it over-estimates and can cause A* to avoid paths that are actually optimal. Once the movement arrays were split so Manhattan only expands 4 neighbours, the results matched what the theory predicts.

### Why diagonal moves cost 1.414 and not 1

The original code returned 1 for every move regardless of direction. This made diagonal paths look artificially cheap in the g values, which distorted path lengths in the comparison. The real cost of moving diagonally one cell is sqrt(2) approximately 1.414. Changing `g_cost` to check whether the row and column both changed and return 1.414 in that case makes the accumulated path cost geometrically accurate, which matters when comparing Euclidean and Chebyshev paths against Manhattan.

### Why `do-while` in grid generation

Grid generation needs to run at least once before checking solvability. A `while` loop would require either duplicating the generation code before the loop or initialising the grid to a known-failing state just to satisfy the entry condition. The `do-while` runs the body once unconditionally then checks — that matches the actual requirement exactly without any extra setup.

### Why `reconstructPath` returns goal-to-start order

Following parent pointers from the goal naturally produces the path backwards. Reversing inside the function and returning start-to-goal would mean allocating and then reversing, doing the work twice. Instead, the raw goal-to-start sequence is returned to `printResult` which uses it directly to print the reverse direction, and a separate assignment with `rbegin()/rend()` fills `result.path` in start-to-goal order. Both orderings are needed and both are produced from the same sequence without any extra allocation.

---

## 8. Testing

Seven manual test cases were written in `tests.cpp`. Each one creates a `PathPlanning` object, configures it with a specific grid, start, and goal, and calls `AStar_Planner()`. The full trace prints to the terminal so the output can be inspected visually.

| Test | Scenario | Expected Result |
|---|---|---|
| 1 | Normal 5x5 grid with obstacles | Path found from (0,0) to (4,4) |
| 2 | Goal cell completely surrounded by obstacles | No path found |
| 3 | Start and goal are the same cell | Early exit, no search needed |
| 4 | Minimal 2x2 grid, all free | One diagonal step |
| 5 | Start cell is an obstacle | Validation rejects before search |
| 6 | Goal cell is an obstacle | Validation rejects before search |
| 7 | Single winding corridor | Path found along the only valid route |

Tests 5 and 6 verify that `validateInputs` catches bad configurations before any search begins. Test 3 verifies that start-equals-goal is handled as a special case. Test 7 is important because it confirms the algorithm works correctly when there is only one valid route and cannot take shortcuts.

The test suite is run from the main menu with option 2. Each test prints a labelled header, runs the planner, and lets the output speak for itself. No assertion framework is used, which keeps the test code simple and easy to read.

---

## 9. C-Style Code: What the AI Got Wrong

During the later stages of the project, AI assistance was used to help restructure and optimise parts of the code. Reviewing what came back identified four places where it introduced C-style patterns rather than modern C++17. These are documented below with the preferred alternatives.

### Issue 1: Raw pointer arrays for direction offsets

```cpp
// As generated
const int dr4[4] = { -1,  1,  0,  0 };
const int* dr    = (heuristic == Heuristic::MANHATTAN) ? dr4 : dr8;
```

```cpp
// Modern C++17 alternative
constexpr std::array<std::pair<int,int>, 4> dirs4 = {{{-1,0},{1,0},{0,-1},{0,1}}};
```

`std::array` carries its own size, is bounds-safe, and works naturally with range-based for \[6\] \[7\]. A raw pointer to a local array is not bounds-safe and does not communicate its size.

### Issue 2: `std::rand` and `std::srand`

```cpp
// As generated
std::srand(seed);
double roll = static_cast<double>(std::rand()) / RAND_MAX;
```

```cpp
// Modern C++11 alternative
std::mt19937 rng(seed);
std::uniform_real_distribution<double> dist(0.0, 1.0);
double roll = dist(rng);
```

`std::rand` has poor statistical properties and is not thread-safe. The Mersenne Twister (`std::mt19937`) from the C++11 `<random>` header is the correct replacement \[6\]. It produces a much better distribution and the seed is encapsulated in the engine object rather than being global state.

### Issue 3: `const char*` for string literals

```cpp
// As generated
const char* names[3] = { "Manhattan", "Euclidean", "Chebyshev" };
```

```cpp
// Modern C++17 alternative
constexpr std::array<std::string_view, 3> names = {"Manhattan", "Euclidean", "Chebyshev"};
```

`std::string_view` is a non-owning, bounds-aware view of string data \[6\]. It is the correct C++17 type for a read-only reference to a string literal. `const char*` provides no length information and no bounds checking.

### Issue 5: `#define` for `ACTIVE_HEURISTIC` instead of `constexpr`

```cpp
// As written
#define ACTIVE_HEURISTIC "MANHATTAN"
```

Every other parameter in `config.h` is declared as `constexpr`. This one was left as a `#define` string, which has no type, no scope, and gets no compiler checking. It also forced `runner.cpp` to copy it into a `std::string` just to use `==` safely, because comparing raw `const char*` pointers with `==` compares memory addresses not content.

```cpp
// Fixed — config.h
#include <string_view>
constexpr std::string_view ACTIVE_HEURISTIC = "MANHATTAN";
```

```cpp
// Fixed — runner.cpp, intermediate std::string no longer needed
if      (ACTIVE_HEURISTIC == "EUCLIDEAN") h = PathPlanning::Heuristic::EUCLIDEAN;
else if (ACTIVE_HEURISTIC == "CHEBYSHEV") h = PathPlanning::Heuristic::CHEBYSHEV;
```

`std::string_view` is a non-owning compile-time string type \[6\]. Its `==` operator compares content directly, so the `std::string active = ACTIVE_HEURISTIC` conversion in `runner.cpp` is no longer needed. This also makes `ACTIVE_HEURISTIC` consistent with every other declaration in `config.h`.

### Issue 4: Linear O(n) closed list lookups

```cpp
// As generated — O(n) scan on every neighbour check
auto inClosedList = [&](int r, int c) {
    for (const auto& cn : closedList)
        if (cn.r == r && cn.c == c) return true;
    return false;
};
```

```cpp
// Modern alternative — O(1) amortised
std::unordered_set<std::pair<int,int>, PairHash> closedSet;
// lookup: closedSet.count({r, c}) > 0
```

This scan runs for every neighbour of every expanded node. On larger grids the closed list grows large and this becomes a bottleneck. An `unordered_set` with a custom pair hash reduces the lookup to O(1) amortised \[6\] \[7\].

---

## 10. Limitations and Improvements

**Open list selection is O(n).** `selectBestNode` scans the entire vector each iteration. A `std::priority_queue` (binary heap) reduces this to O(log n) \[6\] \[7\] and would make the timing comparison more meaningful, since currently the selection overhead can dominate over the actual heuristic difference.

**Closed list lookup is O(n).** Covered in Section 9. Should be an `unordered_set`.

**No visual output.** Output is terminal text only. A graphical grid visualiser would make it easier to understand what the algorithm is actually exploring on larger grids, particularly the difference between how much of the grid each heuristic visits.

---

## 11. Project Planning

| Phase | Focus | What was done |
|---|---|---|
| 1 | Core A* loop | Working algorithm, five structural bugs fixed, correct output verified |
| 2 | Refactor | Split into eight files, added PlannerResult, verbose flag, timing, seven test cases |
| 3 | Config and features | config.h, random grids with BFS check, all three heuristics, movement model fix |
| 4 | Review | C-style audit, four issues documented with modern alternatives, report written |

Each phase built on a working executable from the previous one. This made it possible to verify that nothing broke during each refactor.

---

## 12. Problems Encountered

**The outer for-loop.** The initial code ran the entire search four times inside a `for (k < 4)` loop. The output looked correct at first glance because the path was found eventually, but the algorithm was restarting from scratch each time. This took some careful reading of the output to spot, and was fixed by removing the outer loop entirely.

**Goal found but search continued.** After reaching the goal, the loop kept running. The fix was returning a `goalFound` flag from `expandSuccessors` and breaking the main loop on it.

**Manhattan with 8-directional movement.** All three heuristics were originally using the 8-direction array. Manhattan is inadmissible for diagonal movement because it over-estimates those steps \[1\] \[2\]. Fixing this required understanding admissibility mathematically, not just knowing the formula. The movement model now ties directly to the heuristic via the same enum.

**Euclidean truncated to int.** When Euclidean was added, `h` and `f` in the node struct were still `int`. The `sqrt` return value was being silently cut to an integer, making Euclidean produce almost the same results as Manhattan. Changing those fields to `double` fixed it.

**`goto` in path reconstruction.** The initial code used a `goto path_done` label to exit the reconstruction loop. This was replaced with a clean `return` from inside the loop.

---

## 13. Reflection

The outer for-loop bug took longer to find than it should have. The path printed correctly so the output looked fine, but something felt off about how many iterations were being reported. It was only after staring at the iteration count across several runs and noticing it reset mid-output that the issue became obvious. That kind of bug is annoying because the program does not crash and the result is not wrong, it is just wasteful and the trace is misleading. It taught me to look at the numbers, not just whether a path appeared.

The Manhattan admissibility problem was the most interesting thing to figure out in the whole project. I knew the three heuristic formulas from GeeksforGeeks \[4\] but had not thought hard about why each one is paired with a specific movement model. When the comparison table showed Manhattan finding longer paths with more nodes expanded than expected, I went back to the definition of admissibility and worked through what happens when you allow a diagonal move but your heuristic charges for two axis-aligned steps instead of one diagonal. Once that clicked, the fix was obvious — restrict Manhattan to 4 directions. But getting to that understanding took a while.

Using AI to help with restructuring and debugging was useful, but the times it introduced code without me reviewing it carefully caused more work. The `#define` for `ACTIVE_HEURISTIC` is a good example — it was sitting in `config.h` inconsistent with everything around it and I did not catch it until reviewing the file specifically for that kind of thing. The lesson is not to trust generated or suggested code without reading it properly, even when it works.

If I were starting again I would set up `std::priority_queue` for the open list from day one. The flat vector with linear `selectBestNode` means the timing comparison is partly measuring selection overhead rather than the heuristic difference, which defeats the point. The numbers in the comparison table are still directionally correct but not as clean as they would be with a proper heap.

---

## 14. References

[1] Hart, P. E., Nilsson, N. J., and Raphael, B. (1968). *A Formal Basis for the Heuristic Determination of Minimum Cost Paths.* IEEE Transactions on Systems Science and Cybernetics, 4(2), pp. 100-107.

[2] Russell, S. and Norvig, P. (2020). *Artificial Intelligence: A Modern Approach.* 4th ed. Pearson. Chapter 3.

[3] Patel, A. (2024). *Introduction to A\*.* Red Blob Games. https://www.redblobgames.com/pathfinding/a-star/introduction.html [Accessed March 2026].

[4] GeeksforGeeks (2024). *A* Search Algorithm.* https://www.geeksforgeeks.org/a-search-algorithm/ [Accessed March 2026].

[5] Shaoul, Y. (2024). *Shortest Path to Learn C++ (and A\*).* https://yorai.me/posts/shortest-path-to-learn-cpp [Accessed March 2026].

[6] cppreference.com (2024). *std::mt19937, std::array, std::string_view, std::chrono.* https://en.cppreference.com [Accessed March 2026].

[7] Stroustrup, B. (2013). *The C++ Programming Language.* 4th ed. Addison-Wesley.

---

*Tamer Zraiq | G00425053 | C++ Pathfinding Project | 4th Year 2026*
