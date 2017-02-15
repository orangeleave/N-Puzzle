# N-Puzzle
Create a search agent to solve the n-puzzle game (including BFS, DFS, AST, IDA).

An instance of the n-puzzle game consists of a board holding n^2 − 1 distinct movable tiles, plus an empty space.
The tiles are numbers from the set {1, ..., n^2 − 1}.

For any such board, the empty space may be legally swapped with any tile horizontally or vertically adjacent to it. Here the blank space is represented as the number 0.

The method argument will be one of the following:
bfs (Breadth-First Search)
dfs (Depth-First Search)
ast (A-Star Search)
ida (IDA-Star Search)

At last, driver.py will generate a file called output.txt, containing the following statistics:
path_to_goal: the sequence of moves taken to reach the goal
cost_of_path: the number of moves taken to reach the goal
nodes_expanded: the number of nodes that have been expanded
fringe_size: the size of the frontier set when the goal node is found
max_fringe_size: the maximum size of the frontier set in the lifetime of the algorithm
search_depth: the depth within the search tree when the goal node is found
max_search_depth: the maximum depth of the search tree in the lifetime of the algorithm
running_time: the total running time of the search instance, reported in seconds
max_ram_usage: the maximum RAM usage in the lifetime of the process as measured by the ru_maxrss attribute in the resource module, reported in megabytes
