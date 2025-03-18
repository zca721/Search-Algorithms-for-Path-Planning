# TCSS 435 A Fall
# Zachary Anderson

import pymaze as maze
import random, sys
from SearchSolution import DFS, BFS, GS, AStar

text = open("Readme.txt", "a")

# Arguments passed for creating maze size and what algorithm to be ran
print("\nName of Python script:", sys.argv[0])
print("M:", int(sys.argv[1]))
print("N:", int(sys.argv[2]))
print("search method:", sys.argv[3])

M = int(sys.argv[1])
N = int(sys.argv[2])
algorithm = sys.argv[3]

text.write(str(M) + "x" + str(N) + " " + algorithm + ":")
text.close()

# All (x,y) cooridinates for Agents and maze goal
aX = random.randint(1,M)
aY = random.randint(1,N)
goalX = random.randint(1,M)
goalY = random.randint(1,N)
start = (aX,aY)
goal = (goalX,goalY)

# Checks if b Agent does spawn on the same spot as a Agent or maze goal,
# if so it reasigns b Agent (x,y) coordinants and rechecks if the new
# spawn point is clear of a Agent and maze goal, and does so till clear.
while aX == goalX and aY == goalY:
    print("Agent a has the same spot as Goal")
    aX = random.randint(1,M)
    aY = random.randint(1,N)
    start = (aX,aY)

# Creates maze and maze goal
m = maze.maze(M, N)
m.CreateMaze(goalX,goalY, loopPercent=100, theme = maze.COLOR.dark)

# Creates a Agent and its path
a = maze.agent(m,aX,aY, filled = True, shape = 'arrow', footprints = True, color = maze.COLOR.red)

# Based off input algorith, said algorithm will run and solution path is received for agent
if algorithm == "BFS":
    path = BFS(m, start, goal)
    print("BFS ran")
elif algorithm == "DFS":
    path = DFS(m, start, goal)
    print("DFS ran")
elif algorithm == "GS":
    path = GS(m, start, goal)
    print("GS ran")
elif algorithm == "AStar":
    path = AStar(m, start, goal)
    print("AStar ran")

m.tracePath({a:path})
m.run()
