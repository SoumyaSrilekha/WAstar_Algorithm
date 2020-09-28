"""Weighted A* Algorithm implimentation"""
import math

# Nodes for the Algorithm Implimentation
class Node():
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0
    def __eq__(self, other):
        return self.position == other.position

# WW=A* function Definition
def WAstar(binarymap, start, goal):
    
    # Create start and goal nodes
    node_start = Node(None, start)
    node_start.g = 0
    node_start.h = 0
    node_start.f = 0
    node_goal = Node(None, goal)
    node_goal.g = 0
    node_goal.h = 0
    node_goal.f = 0
    # Initialize both open and closed list
    open_list = []
    close_list = []
    path=[]
    #Initializing the Epsilon value to >1
    E=4.7
    #Add the start node Put node_start in the OPEN list
    open_list.append(node_start)

    #While the OPEN list is not empty
    while len(open_list)>0:
        #Take from the open list the node node_current with the lowest f 
        node_current = open_list[0]
        index_current = 0
        for index, item in enumerate(open_list):
            if item.f < node_current.f:
                node_current = item
                index_current = index
        # Pop current off open list, add to closed list
        open_list.pop(index_current)
        close_list.append(node_current)


        #Generate each state node_successor that come after node_current
        successors = []
        # Get the successor positions (4 connected successors)
        for new_position in [ (0, 1), (0, -1), (1, 0),(-1, 0)]: 

            # Get the relative node position
            node_position = (node_current.position[0] + new_position[0], node_current.position[1] + new_position[1])
            
            #To make sure that the range is in bounds
            if node_position[0] > (len(binarymap) - 1) or node_position[0] < 0 or node_position[1] > (len(binarymap[len(binarymap)-1]) -1) or node_position[1] < 0:
               continue
            #To make sure that the successors ane not obstacles 
            if binarymap[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(node_current, node_position)

            # Append
            successors.append(new_node)
        #for each node_successor of node_current
        # Loop through successors
        for successor in successors:
            #Set h(node_successor) to be the heuristic distance to node_goal
            #Considering the Euclidian Distance
            successor.h = abs(successor.position[0] - node_goal.position[0])  + abs(successor.position[1] - node_goal.position[1]) 
            #Set g(node_successor) = successor_current_cost
            successor.g = node_current.g + 1
            #Set successor_current_cost = g(node_current) + w(node_current, node_successor)
            successor.f = successor.g + E*(successor.h)
            #if node_successor is in the OPEN list {
            #if g(node_successor) ≤ successor_current_cost 
            for node in open_list:
                if successor == node and successor.f >= node.f:
                    break
            #else if node_successor is in the CLOSED list {
            #else 
            #Add node_successor to the OPEN list
            else:
              # Add the child to the open list
              open_list.append(successor)
        
        
        #if node_current is node_goal we have found the solution; break
        if node_current == node_goal:
            #Set the parent of node_successor to node_current
            while node_current:
                path.append(node_current.position)
                #Add node_current to the CLOSED list
                node_current= node_current.parent
            return path 
            break


#Creation of Map 
binarymap = []
binarymap.append([0, 0, 0, 0, 0, 1, 0, 0, 0, 0])    
binarymap.append([0, 0, 0, 0, 0, 1, 0, 0, 0, 0])
binarymap.append([0, 0, 0, 0, 0, 1, 1, 1, 0, 0])
binarymap.append([0, 0, 1, 0, 0, 0, 0, 1, 0, 0])
binarymap.append([1, 0, 1, 0, 0, 1, 0, 1, 1, 0])
binarymap.append([0, 1, 0, 0, 1, 0, 0, 0, 0, 0])
binarymap.append([0, 0, 1, 0, 1, 1, 0, 0, 1, 0])
binarymap.append([0, 0, 0, 1, 0, 1, 0, 0, 0, 0])
binarymap.append([0, 1, 1, 0, 0, 0, 1, 0, 1, 0])
binarymap.append([0, 0, 0, 0, 0, 0, 0, 0, 1, 0])
#Start and the goal points
start = (9, 0)
goal = (1, 8)
#Executing WA* 
path = WAstar(binarymap, start, goal)
#printing the path traversed
print(path)
