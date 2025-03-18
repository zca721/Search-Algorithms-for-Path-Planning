# TCSS 435 A Fall
# Zachary Anderson

from collections import OrderedDict, deque

# Global Variables---------------------------------------------------------
depth = 0
numCreated = 0
numExpanded = 0
maxFringe = 0

# https://www.google.com/search?q=use+deque+as+priority+queue+python&oq=use+deque+as+priority+&gs_lcrp=EgZjaHJvbWUqBwgCECEYoAEyBggAEEUYOTIHCAEQIRigATIHCAIQIRigATIHCAMQIRigATIHCAQQIRigATIHCAUQIRirAjIHCAYQIRirAjIHCAcQIRirAjIHCAgQIRiPAjIHCAkQIRiPAtIBCjE0NzI3ajBqMTWoAgiwAgE&sourceid=chrome&ie=UTF-8
# AI response on goolge for resource, and added extra functions of my own to use with Priority Queue
# Class used for creating a priority queue from deque
class PriorityQueue:
    def __init__(self):
        self.queue = deque()

    def append(self, item, priority):
        # Find the correct position to insert the item based on priority
        i = 0
        while i < len(self.queue) and priority >= self.queue[i][0]:
            i += 1
        self.queue.insert(i, (priority, item))

    def appendleft(self, item):
        self.queue.appendleft(item)

    def pop(self):
        if len(self.queue) == 0:
            return None
        return self.queue.popleft()

    def popleft(self):
        if len(self.queue) == 0:
            return None
        return self.queue.popleft()[1]
    
    def len(self):
        return len(self.queue)
    
# Main queue for all search algorithms
queueMain = PriorityQueue()

# Helper Function(s)--------------------------------------------------------

# https://www.google.com/search?q=search+for+value+to+find+key+in+list+of+dictionaries+python&sca_esv=969c9be58d3ff7ea&ei=ytEaZ_TtGYKf0PEPuPSB4Qg&ved=0ahUKEwj0-pvdj6iJAxWCDzQIHTh6IIwQ4dUDCBA&uact=5&oq=search+for+value+to+find+key+in+list+of+dictionaries+python&gs_lp=Egxnd3Mtd2l6LXNlcnAiO3NlYXJjaCBmb3IgdmFsdWUgdG8gZmluZCBrZXkgaW4gbGlzdCBvZiBkaWN0aW9uYXJpZXMgcHl0aG9uMggQIRigARjDBEjGU1CYCFinRnABeAGQAQCYAWegAf8HqgEEMTIuMbgBA8gBAPgBAZgCDqACqgjCAgoQABiwAxjWBBhHwgIKECEYoAEYwwQYCsICBBAhGArCAggQABiABBiiBJgDAIgGAZAGCJIHBDEzLjGgB70x&sclient=gws-wiz-serp
# AI response on google for resource
# Used to find the key from a value in a dictionary
def find_key_for_value(list_of_dicts, value):
    for dict_item in list_of_dicts:
        for key, val in dict_item.items():
            if val == value:
                return key
    return None

# https://www.google.com/search?q=creating+heuristic+values+for+random+maze+python&oq=&gs_lcrp=EgZjaHJvbWUqCQgGEEUYOxjCAzIJCAAQRRg7GMIDMgkIARBFGDsYwgMyCQgCEEUYOxjCAzIJCAMQRRg7GMIDMgkIBBBFGDsYwgMyCQgFEEUYOxjCAzIJCAYQRRg7GMIDMgkIBxBFGDsYwgPSAQk1MDIzajBqMTWoAgiwAgE&sourceid=chrome&ie=UTF-8
# AI response on google for resource
# Used to calculate the manhattan distance for each position to goal
def manhattan_distance(current, goal):
    """Calculates the Manhattan distance between two points."""
    return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

# Calculates step cost for A*
def step_cost(current, start, list):
    current = list[-1]
    dictStep = OrderedDict(current)
    
    if len(list) == 1:
                returnDict = current
                
                return len(returnDict)

    # Uses dictionary to trace backwards through a list of dictionary steps to create the tree depth
    while current != start:
                child = next(iter(dictStep))

                current = find_key_for_value(list, child)
                if current == None:
                    dictStep.update({current: child})
                    dictStep.move_to_end(current, last=False)
                    returnDict = dict(dictStep)

                    return len(returnDict)

                dictStep.update({current: child})
                dictStep.move_to_end(current, last=False)

    returnDict = dict(dictStep)

    return len(returnDict)

# Search Algortihms---------------------------------------------------------

# Breadth-first Search (BFS) search algorithm and using Graph Search
def BFS(m, start, goal):
    queueMain.append(start, 1)
    listBFS = []
    dictBFS = {}
    queueBFS = PriorityQueue()
    finalDictBFS = {}
    finalListBFS = []
    parent = start

    numCreated = 1
    numExpanded = 0
    maxFringe = 0

    while queueMain:
        numExpanded = numExpanded + 1
        
        if queueMain.len() > maxFringe:
            maxFringe = queueMain.len()
            
        if queueBFS and numExpanded > 1:
            index_0 = queueBFS.popleft()
            finalListBFS.append(index_0)

        visited = queueMain.popleft()
        
        parent = visited

        # Checks for expanded nodes, and if node has been expanded it skips the step
        if parent in listBFS:
            continue
        
        # Checks for expanded nodes, and if node is not in expanded nodes, the node is added to fringe
        if parent not in listBFS:
            listBFS.append(visited)

        # Checks if expanded node is the goal state
        if visited == goal:

            parent = finalListBFS[-1]

            finalDictBFS = OrderedDict(parent)

            # Uses dictionary to trace backwards through a list of dictionary steps to create the solution path
            while parent != start and len(finalListBFS) > 3:
                child = next(iter(finalDictBFS))

                parent = find_key_for_value(finalListBFS, child)

                finalDictBFS.update({parent: child})
                finalDictBFS.move_to_end(parent, last=False)

            returnDictBFS = dict(finalDictBFS)

            # Writes to consol and Readme.txt the required information on search algorithm
            depth = len(returnDictBFS)
            print("depth: ", depth)
            print("numCreated: ", numCreated)
            print("numExpanded: ", numExpanded)
            print("maxFringe: ", maxFringe)
            text = open("Readme.txt", "a")
            text.write(" depth: " + str(depth) 
                        + " numCreated: " + str(numCreated) 
                        + " numExpanded: " + str(numExpanded) 
                        + " maxFringe: " + str(maxFringe)
                        + "\n")
            text.close()

            return returnDictBFS
        
        # Navigates the maze map dictionary of dictionaries and adds possible moves from expanded node
        for item in m.maze_map[visited]:
            if item == 'E' and m.maze_map[visited]['E'] == 1:
                x, y = visited
                east = x + 0, y + 1
                # Checks for expanded nodes, and if node is in expanded nodes, the node is not added to fringe
                if east not in listBFS:
                    numCreated = numCreated + 1
                    child = east
                    dictBFS = {parent: child}
                    queueBFS.append(dictBFS, 1)
                    queueMain.append(east, 1)
            if item == 'W' and m.maze_map[visited]['W'] == 1:
                x, y = visited
                west = x + 0, y - 1
                # Checks for expanded nodes, and if node is in expanded nodes, the node is not added to fringe
                if west not in listBFS:
                    numCreated = numCreated + 1
                    child = west
                    dictBFS = {parent: child}
                    queueBFS.append(dictBFS, 1)
                    queueMain.append(west, 1)
            if item == 'N' and m.maze_map[visited]['N'] == 1:
                x, y = visited
                north = x - 1, y + 0
                # Checks for expanded nodes, and if node is in expanded nodes, the node is not added to fringe
                if north not in listBFS:
                    numCreated = numCreated + 1
                    child = north
                    dictBFS = {parent: child}
                    queueBFS.append(dictBFS, 1)
                    queueMain.append(north, 1)
            if item == 'S' and m.maze_map[visited]['S'] == 1:
                x, y = visited
                south = x + 1, y + 0
                # Checks for expanded nodes, and if node is in expanded nodes, the node is not added to fringe
                if south not in listBFS:
                    numCreated = numCreated + 1
                    child = south
                    dictBFS = {parent: child}
                    queueBFS.append(dictBFS, 1)
                    queueMain.append(south, 1)

# Depth-first Search (DFS) search algorithm and using Graph Search
def DFS(m, start, goal):
    queueMain.appendleft(start)
    listDFS = []
    dictDFS = {}
    queueDFS = deque()
    finalDictDFS = {}
    finalListDFS = []
    parent = start

    numCreated = 1
    numExpanded = 0
    maxFringe = 0

    while queueMain:
        numExpanded = numExpanded + 1

        if queueMain.len() > maxFringe:
            maxFringe = queueMain.len()

        if queueDFS:
            index_0 = queueDFS.popleft()
            finalListDFS.append(index_0)

        visited = queueMain.pop()
        
        parent = visited

        # Checks for expanded nodes, and if node has been expanded it skips the step
        if parent in listDFS:
            continue
        
        # Checks for expanded nodes, and if node is not in expanded nodes, the node is added to fringe
        if parent not in listDFS:
            listDFS.append(visited)

        # Checks if expanded node is the goal state
        if visited == goal:

            parent = finalListDFS[-1]
            finalDictDFS = OrderedDict(parent)

            # Uses dictionary to trace backwards through a list of dictionary steps to create the solution path
            while parent != start and len(finalListDFS) > 3:
                child = next(iter(finalDictDFS))

                parent = find_key_for_value(finalListDFS, child)

                finalDictDFS.update({parent: child})
                finalDictDFS.move_to_end(parent, last=False)

            returnDictDFS = dict(finalDictDFS)

            # Writes to consol and Readme.txt the required information on search algorithm
            depth = len(returnDictDFS)
            print("depth: ", depth)
            print("numCreated: ", numCreated)
            print("numExpanded: ", numExpanded)
            print("maxFringe: ", maxFringe)
            text = open("Readme.txt", "a")
            text.write(" depth: " + str(depth) 
                        + " numCreated: " + str(numCreated) 
                        + " numExpanded: " + str(numExpanded) 
                        + " maxFringe: " + str(maxFringe)
                        + "\n")
            text.close()

            return returnDictDFS
        
        # Navigates the maze map dictionary of dictionaries and adds possible moves from expanded node
        for item in m.maze_map[visited]:
            if item == 'E' and m.maze_map[visited]['E'] == 1:
                x, y = visited
                east = x + 0, y + 1
                # Checks for expanded nodes, and if node is in expanded nodes, the node is not added to fringe
                if east not in listDFS:
                    numCreated = numCreated + 1
                    child = east
                    dictDFS = {parent: child}
                    queueDFS.appendleft(dictDFS)
                    queueMain.appendleft(east)
            if item == 'W' and m.maze_map[visited]['W'] == 1:
                x, y = visited
                west = x + 0, y - 1
                # Checks for expanded nodes, and if node is in expanded nodes, the node is not added to fringe
                if west not in listDFS:
                    numCreated = numCreated + 1
                    child = west
                    dictDFS = {parent: child}
                    queueDFS.appendleft(dictDFS)
                    queueMain.appendleft(west)
            if item == 'N' and m.maze_map[visited]['N'] == 1:
                x, y = visited
                north = x - 1, y + 0
                # Checks for expanded nodes, and if node is in expanded nodes, the node is not added to fringe
                if north not in listDFS:
                    numCreated = numCreated + 1
                    child = north
                    dictDFS = {parent: child}
                    queueDFS.appendleft(dictDFS)
                    queueMain.appendleft(north)
            if item == 'S' and m.maze_map[visited]['S'] == 1:
                x, y = visited
                south = x + 1, y + 0
                # Checks for expanded nodes, and if node is in expanded nodes, the node is not added to fringe
                if south not in listDFS:
                    numCreated = numCreated + 1
                    child = south
                    dictDFS = {parent: child}
                    queueDFS.appendleft(dictDFS)
                    queueMain.appendleft(south)

# Greedy Search (GS) search algorithm using Manhattan Distance and Graph Search
def GS(m, start, goal):
    manhattan = manhattan_distance(start, goal)
    queueMain.append(start, manhattan)
    listGS = []
    dictGS = {}
    queueGS = PriorityQueue()
    finalDictGS = {}
    finalListGS = []
    parent = start

    numCreated = 1
    numExpanded = 0
    maxFringe = 0

    while queueMain:
        numExpanded = numExpanded + 1

        if queueMain.len() > maxFringe:
            maxFringe = queueMain.len()
            
        if queueGS and numExpanded > 1:
            index_0 = queueGS.popleft()
            finalListGS.append(index_0)

        visited = queueMain.popleft()
        
        parent = visited

        # Checks for expanded nodes, and if node has been expanded it skips the step
        if parent in listGS:
            continue
        
        # Checks for expanded nodes, and if node is not in expanded nodes, the node is added to fringe
        if parent not in listGS:
            listGS.append(visited)

        # Checks if expanded node is the goal state
        if visited == goal:

            parent = finalListGS[-1]
            finalDictGS = OrderedDict(parent)

            # Uses dictionary to trace backwards through a list of dictionary steps to create the solution path
            if len(finalListGS) == 1:
                returnDictGS = parent

                # Writes to consol and Readme.txt the required information on search algorithm
                depth = len(returnDictGS)
                print("depth: ", depth)
                print("numCreated: ", numCreated)
                print("numExpanded: ", numExpanded)
                print("maxFringe: ", maxFringe)
                text = open("Readme.txt", "a")
                text.write(" depth: " + str(depth) 
                            + " numCreated: " + str(numCreated) 
                            + " numExpanded: " + str(numExpanded) 
                            + " maxFringe: " + str(maxFringe)
                            + "\n")
                text.close()

                return returnDictGS

            # Uses dictionary to trace backwards through a list of dictionary steps to create the solution path
            while parent != start:
                child = next(iter(finalDictGS))

                parent = find_key_for_value(finalListGS, child)

                finalDictGS.update({parent: child})
                finalDictGS.move_to_end(parent, last=False)

            returnDictGS = dict(finalDictGS)

            # Writes to consol and Readme.txt the required information on search algorithm
            depth = len(returnDictGS)
            print("depth: ", depth)
            print("numCreated: ", numCreated)
            print("numExpanded: ", numExpanded)
            print("maxFringe: ", maxFringe)
            text = open("Readme.txt", "a")
            text.write(" depth: " + str(depth) 
                        + " numCreated: " + str(numCreated) 
                        + " numExpanded: " + str(numExpanded) 
                        + " maxFringe: " + str(maxFringe)
                        + "\n")
            text.close()

            return returnDictGS
        
        # Navigates the maze map dictionary of dictionaries and adds possible moves from expanded node
        for item in m.maze_map[visited]:
            if item == 'E' and m.maze_map[visited]['E'] == 1:
                x, y = visited
                east = x + 0, y + 1
                # Checks for expanded nodes, and if node is in expanded nodes, the node is not added to fringe
                if east not in listGS:
                    numCreated = numCreated + 1
                    child = east
                    manhattan = manhattan_distance(child, goal)
                    dictGS = {parent: child}
                    queueGS.append(dictGS, manhattan)
                    queueMain.append(east, manhattan)
            if item == 'W' and m.maze_map[visited]['W'] == 1:
                x, y = visited
                west = x + 0, y - 1
                # Checks for expanded nodes, and if node is in expanded nodes, the node is not added to fringe
                if west not in listGS:
                    numCreated = numCreated + 1
                    child = west
                    manhattan = manhattan_distance(child, goal)
                    dictGS = {parent: child}
                    queueGS.append(dictGS, manhattan)
                    queueMain.append(west, manhattan)
            if item == 'N' and m.maze_map[visited]['N'] == 1:
                x, y = visited
                north = x - 1, y + 0
                # Checks for expanded nodes, and if node is in expanded nodes, the node is not added to fringe
                if north not in listGS:
                    numCreated = numCreated + 1
                    child = north
                    manhattan = manhattan_distance(child, goal)
                    dictGS = {parent: child}
                    queueGS.append(dictGS, manhattan)
                    queueMain.append(north, manhattan)
            if item == 'S' and m.maze_map[visited]['S'] == 1:
                x, y = visited
                south = x + 1, y + 0
                # Checks for expanded nodes, and if node is in expanded nodes, the node is not added to fringe
                if south not in listGS:
                    numCreated = numCreated + 1
                    child = south
                    manhattan = manhattan_distance(child, goal)
                    dictGS = {parent: child}
                    queueGS.append(dictGS, manhattan)
                    queueMain.append(south, manhattan)

# A* Search (AStar) search algorithm using Manhattan Distance and Graph Search
def AStar(m, start, goal):
    startManhattan = manhattan_distance(start, goal)
    queueMain.append(start, startManhattan)
    listAStar = []
    dictAStar = {}
    queueAStar = PriorityQueue()
    finalDictAStar = {}
    finalListAStar = []
    parent = start

    numCreated = 1
    numExpanded = 0
    maxFringe = 0

    while queueMain:
        numExpanded = numExpanded + 1

        if queueMain.len() > maxFringe:
            maxFringe = queueMain.len()
            
        if queueAStar and numExpanded > 1:
            index_0 = queueAStar.popleft()
            finalListAStar.append(index_0)

        visited = queueMain.popleft()
        parent = visited

        # Checks for expanded nodes, and if node has been expanded it skips the step
        if parent in listAStar:
            continue
        
        # Checks for expanded nodes, and if node is not in expanded nodes, the node is added to fringe
        if parent not in listAStar:
            listAStar.append(visited)

        # Checks if expanded node is the goal state
        if visited == goal:

            parent = finalListAStar[-1]
            finalDictAStar = OrderedDict(parent)

            # Uses dictionary to trace backwards through a list of dictionary steps to create the solution path
            if len(finalListAStar) == 1:
                returnDictAStar = parent

                # Writes to consol and Readme.txt the required information on search algorithm
                depth = len(returnDictAStar)
                print("depth: ", depth)
                print("numCreated: ", numCreated)
                print("numExpanded: ", numExpanded)
                print("maxFringe: ", maxFringe)
                text = open("Readme.txt", "a")
                text.write(" depth: " + str(depth) 
                            + " numCreated: " + str(numCreated) 
                            + " numExpanded: " + str(numExpanded) 
                            + " maxFringe: " + str(maxFringe)
                            + "\n")
                text.close()

                return returnDictAStar

            # Uses dictionary to trace backwards through a list of dictionary steps to create the solution path
            while parent != start:
                child = next(iter(finalDictAStar))

                parent = find_key_for_value(finalListAStar, child)

                finalDictAStar.update({parent: child})
                finalDictAStar.move_to_end(parent, last=False)

            returnDictAStar = dict(finalDictAStar)

            # Writes to consol and Readme.txt the required information on search algorithm
            depth = len(returnDictAStar)
            print("depth: ", depth)
            print("numCreated: ", numCreated)
            print("numExpanded: ", numExpanded)
            print("maxFringe: ", maxFringe)
            text = open("Readme.txt", "a")
            text.write(" depth: " + str(depth) 
                        + " numCreated: " + str(numCreated) 
                        + " numExpanded: " + str(numExpanded) 
                        + " maxFringe: " + str(maxFringe)
                        + "\n")
            text.close()

            return returnDictAStar
        
        # Navigates the maze map dictionary of dictionaries and adds possible moves from expanded node
        for item in m.maze_map[visited]:
            if item == 'E' and m.maze_map[visited]['E'] == 1:
                x, y = visited
                east = x + 0, y + 1
                # Checks for expanded nodes, and if node is in expanded nodes, the node is not added to fringe
                if east not in listAStar:
                    numCreated = numCreated + 1
                    child = east
                    manhattan = manhattan_distance(child, goal)

                    if len(finalListAStar) > 0:
                        stepCost = step_cost(child, start, finalListAStar)
                    else:
                        stepCost = 1
                        
                    stepCost = manhattan + stepCost
                    dictAStar = {parent: child}
                    queueAStar.append(dictAStar, stepCost)
                    queueMain.append(east, stepCost)
            if item == 'W' and m.maze_map[visited]['W'] == 1:
                x, y = visited
                west = x + 0, y - 1
                # Checks for expanded nodes, and if node is in expanded nodes, the node is not added to fringe
                if west not in listAStar:
                    numCreated = numCreated + 1
                    child = west
                    manhattan = manhattan_distance(child, goal)

                    if len(finalListAStar) > 0:
                        stepCost = step_cost(child, start, finalListAStar)
                    else:
                        stepCost = 1

                    stepCost = manhattan + stepCost
                    dictAStar = {parent: child}
                    queueAStar.append(dictAStar, stepCost)
                    queueMain.append(west, stepCost)
            if item == 'N' and m.maze_map[visited]['N'] == 1:
                x, y = visited
                north = x - 1, y + 0
                # Checks for expanded nodes, and if node is in expanded nodes, the node is not added to fringe
                if north not in listAStar:
                    numCreated = numCreated + 1
                    child = north
                    manhattan = manhattan_distance(child, goal)
                    if len(finalListAStar) > 0:
                        stepCost = step_cost(child, start, finalListAStar)
                    else:
                        stepCost = 1
                        
                    stepCost = manhattan + stepCost
                    dictAStar = {parent: child}
                    queueAStar.append(dictAStar, stepCost)
                    queueMain.append(north, stepCost)
            if item == 'S' and m.maze_map[visited]['S'] == 1:
                x, y = visited
                south = x + 1, y + 0
                # Checks for expanded nodes, and if node is in expanded nodes, the node is not added to fringe
                if south not in listAStar:
                    numCreated = numCreated + 1
                    child = south
                    manhattan = manhattan_distance(child, goal)

                    if len(finalListAStar) > 0:
                        stepCost = step_cost(child, start, finalListAStar)
                    else:
                        stepCost = 1
                        
                    stepCost = manhattan + stepCost
                    dictAStar = {parent: child}
                    queueAStar.append(dictAStar, stepCost)
                    queueMain.append(south, stepCost)