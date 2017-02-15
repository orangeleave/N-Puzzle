import numpy as np
from copy import deepcopy
import time
import resource
from collections import deque
from queue import PriorityQueue
from sys import argv

max_depth = 0
max_frontier = 0
count = 0

class Node(object):
    def __init__(self, state = None):
        self.state = state
        self.depth = 0
        self.parent = None
        self.opt = None
        self.blank = []

"""
Define a function to compute the corresponding position of a given tile.
"""
def getPos(state, val):
    for i in range(0, len(state)):
        for j in range(0, len(state)):
            if (state[i][j] == val):
                return [i, j]

"""
Define the four directions of the blank space: 'Up', 'Down', 'Left', 'Right'.
Given the current state, one move at a time, and each movement would result in a new state.
Notice here we also need to check if the new state is valid.

currNode: indicates the current situation of board, represented as a Node;
x: the movement of '0' in the x-axis;
y: the movemetn of '0' in the y-axis.
"""
def move(currNode, x, y):
    n = len(currNode.state)
    pos_old = currNode.blank
    x_new = pos_old[0] + x
    y_new = pos_old[1] + y
    if (x_new < 0 or x_new >= n or y_new < 0 or y_new >= n):
        pass
    else:
        newNode = Node(deepcopy(currNode.state))
        newNode.depth = currNode.depth + 1
        newNode.blank = [x_new, y_new]
        newNode.state[pos_old[0]][pos_old[1]], newNode.state[x_new][y_new] = newNode.state[x_new][y_new], newNode.state[pos_old[0]][pos_old[1]]
        return newNode

def up(currNode):
    return move(currNode, -1, 0)

def down(currNode):
    return move(currNode, 1, 0)

def left(currNode):
    return move(currNode, 0, -1)

def right(currNode):
    return move(currNode, 0, 1)

"""
Define a heuristic function that estimates how close a state is to the goal.
Here we use the Manhattan priority function, the sum of the distances of the tiles from their goal positions.
The manhattan distance of each tile is the sum of the horizontal and vertical distances.
"""
def manhattan(currState, goalState):
    sum = 0
    for i in range(0, len(currState)):
        for j in range(0, len(currState)):
            if (currState[i][j] != 0):
                x_goal, y_goal = getPos(goalState, currState[i][j])
                sum += abs(x_goal - i) + abs(y_goal - j)
    return sum


"""
Define the basic search method.
Keep track of the parent and movement of every node, so that we can rebuild the path.
In order to complete four methods, we need to check the method and make different steps among them.
"""
def search(initNode, goalNode, frontier, method, f_limit = None, valueRecord = None):
    global max_depth   # to record the max depth
    global max_frontier    # to record the max size of frontier
    global count   # to count the number of expanded nodes
    path = []   # to record the path
    if (method == 'ASTAR'):
        priority = manhattan(initNode.state, goalNode.state)
        frontier.put((priority, 0, time.clock(), initNode))
    elif (method == 'IDASTAR'):
        f_initNode = initNode.depth + manhattan(initNode.state, goalNode.state)
        if (f_initNode <= f_limit):
            frontier.append(initNode)
        else:
            return None
    else:
        frontier.append(initNode)
    explored = set()
    explored.add(''.join(str(n) for n in initNode.state))
    # print (explored)
    while (frontier):
        if (method == 'ASTAR'):
            max_frontier = max(max_frontier, frontier.qsize())
        else:
            max_frontier = max(max_frontier, len(frontier))
            # print(max_frontier)
        node = Node()
        if (method == 'BFS'):
            node = frontier.popleft()
        if (method == 'DFS' or method == 'IDASTAR'):
            node = frontier.pop()
            # print(node.state)
        if (method == 'ASTAR'):
            # print(frontier.get())
            node = frontier.get()[3]
        # print (node.state)
        flag = True
        for i in range(0, len(node.state)):
            for j in range(0, len(node.state)):
                if node.state[i][j] != goalNode.state[i][j]:
                    flag = False
        if flag:
            while (node):
                neighbor = ''.join(str(n) for n in node.state)
                # path.append(parent[neighbor][1])
                # state = parent[neighbor][0]
                path.append(node.opt)
                node = node.parent
            if (method == 'ASTAR'):
                len_frontier = frontier.qsize()
            else:
                len_frontier = len(frontier)
            return [path[::-1][1:], len(path) - 1, count, len_frontier, max_frontier, len(path) - 1, max_depth]
        count += 1
        upNode = up(node)
        downNode = down(node)
        leftNode = left(node)
        rightNode = right(node)

        def appendNode(node, neighborNode, s, order):
            global max_depth
            if (neighborNode):
                neighbor = ''.join(str(n) for n in neighborNode.state)
                # print (explored)
                if (method == 'IDASTAR'):   # IDASTAR uses map instead of set
                    f = neighborNode.depth + manhattan(neighborNode.state, goalNode.state)
                    if (f <= f_limit):
                        if (neighbor not in valueRecord or valueRecord[neighbor] >= f):
                            max_depth = max(node.depth + 1, max_depth)
                            frontier.append(neighborNode)
                            valueRecord[neighbor] = f
                else:   # BFS, DFS and ASTAR share the same recording method for visited nodes
                    if neighbor not in explored:
                        max_depth = max(node.depth + 1, max_depth)
                        if (method == 'ASTAR'):
                            p1 = neighborNode.depth + manhattan(neighborNode.state, goalNode.state)
                            p2 = order
                            # print(p1, p2)
                            frontier.put((p1, p2, time.clock(), neighborNode))
                        else:
                            frontier.append(neighborNode)
                        explored.add(neighbor)
                neighborNode.parent = node
                neighborNode.opt = s
        if (method == 'BFS' or method == 'ASTAR'):
            appendNode(node, upNode, 'Up', 0)
            # print ('up')
            appendNode(node, downNode, 'Down', 1)
            # print ('down')
            appendNode(node, leftNode, 'Left', 2)
            # print ('left')
            appendNode(node, rightNode, 'Right', 3)
            # print ('right')
        if (method == 'DFS' or method == 'IDASTAR'):
            appendNode(node, rightNode, 'Right', 3)
            appendNode(node, leftNode, 'Left', 2)
            appendNode(node, downNode, 'Down', 1)
            appendNode(node, upNode, 'Up', 0)
        # print (node.state)
    return None

"""
Define the BFS method, using an explicit queue.
Keep track of the parent and movement of every node, so that we can rebuild the path.
"""
def BFS(initNode, goalNode):
    frontier = deque()
    return search(initNode, goalNode, frontier, 'BFS')

"""
Define the DFS method, using an explicit stack.
Keep track of the parent and movement of every node, so that we can rebuild the path.
"""
def DFS(initNode, goalNode):
    frontier = []
    return search(initNode, goalNode, frontier, 'DFS')

"""
Define the A-Star method, using a priority queue.
Keep track of the parent and movement of every node, so that we can rebuild the path.
"""
def ASTAR(initNode, goalNode):
    frontier = PriorityQueue()
    return search(initNode, goalNode, frontier, 'ASTAR')

"""
Define the IDA-Star method, using an explicit stack.
Note the differences with DFS:
1) IDA-Star needs f-limit;
2) IDA-Star uses valueRecord instead of explored to check the visited nodes.
"""
def IDASTAR(initNode, goalNode):
    f_limit = 0
    res = []
    while (not res):
        valueRecord = {}
        frontier = []
        res = search(initNode, goalNode, frontier, 'IDASTAR', f_limit, valueRecord)
        f_limit += 1
    return res

"""
Define the main function to check the answer
"""
if __name__ == '__main__':
    start_time = time.clock()

    """
    Read the input into method and input_list.
    """
    method, input_list = argv[1], argv[2]

    """
    Convert the input into the matrix format.
    And build the corresponding goalState.
    """
    init = []
    input_new = input_list.split(',')
    n = int(np.sqrt(len(input_new)))
    for i in range(0, n):
        init.append([])
        ele = input_new[n * i : (i + 1) * n]
        for j in ele:
            init[i].append(int(j))

    goal = [[(n * x + y) for y in range(0, n)] for x in range(0, n)]
    # print(goal)
    # print(init)

    """
    Solution starts here.
    """
    initNode = Node(init)
    goalNode = Node(goal)
    initNode.blank = getPos(initNode.state, 0)
    goalNode.blank = getPos(goalNode.state, 0)
    if (method == 'bfs'):
        res = BFS(initNode, goalNode)
    if (method == 'dfs'):
        res = DFS(initNode, goalNode)
    if (method == 'ast'):
        res = ASTAR(initNode, goalNode)
    if (method == 'ida'):
        res = IDASTAR(initNode, goalNode)
    max_RAM = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / (10 ** 6)
    # print (res, "--- %s seconds ---" %(time.clock() - start_time), max_RAM)

    """
    Generate the output file for result.
    """
    output = []
    output.append('path_to_goal: ' + str(res[0]))
    output.append('cost_of_path: ' + str(res[1]))
    output.append('nodes_expanded: ' + str(res[2]))
    output.append('fringe_size: ' + str(res[3]))
    output.append('max_fringe_size: ' + str(res[4]))
    output.append('search_depth: ' + str(res[5]))
    output.append('max_search_depth: ' + str(res[6]))
    output.append('running_time: ' + str(time.clock() - start_time))
    output.append('max_ram_usage: ' + str(max_RAM))
    file = open("output.txt","w+")
    for i in range(0, 9):
        file.write(output[i] + '\n')
    file.close
