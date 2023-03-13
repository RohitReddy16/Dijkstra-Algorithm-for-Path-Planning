import cv2 as cv
import numpy as np
import time 
import heapq as hq
#creating the clearance and obstacle colour
clearance = 5 
obstracle = (0,0,255)

#defining Linear equation for obstacles
def linear_equation(i,j,x1,y1,x2,y2,clearance):
    if x2 - x1 == 0:
        # handle case where the denominator is zero
        if i == x1:
            return j - y1
        else:
            return float('inf')
    else:
        return ((y2 - y1) / (x2 - x1 + clearance)) * (i - x1 - clearance/2) + y1 - j

#Generating map
def Map_Generator(height, width):

    map = np.zeros((height, width, 3))
    for i in range(map.shape[1]):
        for j in range(map.shape[0]):
        #rectangle obstacle1
            if(i>=100 and i<=150 and j>=0 and j<=100):
                map[j][i] = obstracle
        #rectangle obstacle 2
            if(i>=100 and i<=150 and j>=150 and j<=250):
                map[j][i] = obstracle
        #hexagon obstacle
            if(linear_equation(i,j,*(300,50),*(364.95,87.5),clearance)<=0 and i<364.95 and linear_equation(i,j,*(364.95,162.5),*(300,200),clearance)>=0
           and linear_equation(i,j,*(300,200),*(235.05,162.5),clearance)>=0 and i>235.05 and linear_equation(i,j,*(235.05,87.5),*(300,50),clearance)<=0):
                map[j][i] = obstracle
        #triangle obstacle
            if (linear_equation(i,j,*(460,25),*(510,125),clearance) <= 0 and linear_equation(i,j,*(460,225),*(510,125),clearance) >= 0 and linear_equation(i,j,*(460,225),*(460,25),clearance) >= 0 and i>460):
                map[j][i] = obstracle
    return map

#defining the obstacles
def isObstacle(map, x, y):
 
    if (map[x][y][2] < obstracle[2]):
        return False
    else:
        return True
    
# Checking for applicable inputs 
def checkInputFeasibility(x_start, y_start, x_goal, y_goal, map):
 
    input_flag = True

    if isObstacle(map, y_start, x_start):
        print("!! Start Position is in an Obstacle/Wall, try again!")
        input_flag = False
    if isObstacle(map, y_goal, x_goal):
        print("!! Goal Position is in an Obstacle/Wall!, try again")
        input_flag = False
    
    return input_flag

#checking the goal node is reached
def isGoalNode(CurrentNode, goalNode):

    if list(CurrentNode) == goalNode:
        return True
    else:
        return False

# Function to Move Top 
def actionMoveTop(CurrentNode,map):

    NextNode = CurrentNode.copy()
    if(NextNode[1]-1 > 0) and (not isObstacle(map, NextNode[1]-1, NextNode[0])):
        Status = True
        NextNode[1] = NextNode[1] - 1 
    else:
        Status = False   

    return (Status, NextNode)

# Function to Move diagonally top right of cost 1.4
def actionMoveTopRight(CurrentNode,map):

    NextNode = CurrentNode.copy()
    
    if(NextNode[1]-1 > 0) and (NextNode[0]+1 <map.shape[1]) and (not isObstacle(map, NextNode[1]-1, NextNode[0])):
        Status = True
        NextNode[0] = NextNode[0] + 1 
        NextNode[1] = NextNode[1] - 1
    else:
        Status = False   

    return (Status, NextNode)

# Function to Move Right diagonally
def actionMoveRight(CurrentNode,map):
    
    NextNode = CurrentNode.copy()
    if(NextNode[0]+1 <map.shape[1]) and (not isObstacle(map, NextNode[1], NextNode[0]+1)):
        Status = True
        NextNode[0] = NextNode[0] + 1 
    else:
        Status = False   

    return (Status, NextNode)

# Function to Move Bottom Right
def actionMoveBottomRight(CurrentNode,map):
    
    NextNode = CurrentNode.copy()
    if(NextNode[1]+1 < map.shape[0]) and (NextNode[0]+1 <map.shape[1]) and (not isObstacle(map, NextNode[1]+1, NextNode[0]+1)):
        Status = True
        NextNode[0] = NextNode[0] + 1 
        NextNode[1] = NextNode[1] + 1
    else:
        Status = False   

    return (Status, NextNode)

# Function to Move Bottom
def actionMoveBottom(CurrentNode,map):
    
    NextNode = CurrentNode.copy()
    if(NextNode[1]+1 < map.shape[0]) and (not isObstacle(map, NextNode[1]+1, NextNode[0])):
        Status = True 
        NextNode[1] = NextNode[1] + 1
    else:
        Status = False   

    return (Status, NextNode)

# Function to Move Bottom Left(Diagonally)
def actionMoveBottomLeft(CurrentNode,map):
    
    NextNode = CurrentNode.copy()
    if(NextNode[1]+1 < map.shape[0]) and (NextNode[0]-1 >0) and (not isObstacle(map, NextNode[1]+1, NextNode[0]-1)):
        Status = True 
        NextNode[0] = NextNode[0] - 1
        NextNode[1] = NextNode[1] + 1
    else:
        Status = False   

    return (Status, NextNode)

# Function to Move Left
def actionMoveLeft(CurrentNode,map):
    
    NextNode = CurrentNode.copy()
    if(NextNode[0]-1 > 0) and (not isObstacle(map, NextNode[1], NextNode[0]-1)):  
        Status = True 
        NextNode[0] = NextNode[0] - 1
    else:
        Status = False   

    return (Status, NextNode)

# Function to Move Top Left (Diagonally)
def actionMoveTopLeft(CurrentNode,map):
    
    NextNode = CurrentNode.copy()
    if(NextNode[1]-1 > 0) and (NextNode[0]-1 > 0) and (not isObstacle(map, NextNode[1]-1, NextNode[0]-1)):
        Status = True 
        NextNode[0] = NextNode[0] - 1
        NextNode[1] = NextNode[1] - 1
    else:
        Status = False   

    return (Status, NextNode)

# Dijkstras Algorithm for finding the path
def Dijkstra_algo(startNode, goalNode, map):
    
    closed_list = {}    
    opened_list = []    
    

    hq.heapify(opened_list)
    hq.heappush(opened_list, [0, startNode, startNode])
    
    start_time = time.time()
    while True:
        
        if (len(opened_list) > 0):
            
            explored_node = hq.heappop(opened_list)
            cost_to_come, present_node, parent_node = explored_node[0], explored_node[1], explored_node[2]
           
            closed_list[(present_node[0],present_node[1])] = parent_node
            
            if isGoalNode(present_node, goalNode):
                print("\n Goal reached!")
                end_time = time.time()
                print("\nTime: "+str(round((end_time-start_time),4)) + " [secs]")
                back_Tracking_Algo(goalNode,startNode,closed_list,map)
                return True

            else:

                # Top 
                flag, child_node = actionMoveTop(present_node,map)    
                child_node = tuple(child_node)

                if(flag is True and child_node not in closed_list):
                    if not isGoalNode(child_node, goalNode):     
                        cost = cost_to_come + 1
                        child_node = list(child_node)
                        closelist_flag = False    

                        for node in opened_list:
                            if(node[1] == child_node):
                                closelist_flag = True        
                                if(node[0] > cost):    
                                    node[0] = cost
                                    node[2] = present_node
                                break
                        
                        if(not closelist_flag):    
                            hq.heappush(opened_list,[cost, child_node, present_node])
                    else:
                        print("\nGoal Reached!")
                        end_time = time.time()
                        print("\nTime: "+str(round((end_time-start_time),4)) + " [secs]")
                        closed_list[child_node] = present_node
                        back_Tracking_Algo(goalNode,startNode,closed_list,map)
                        return True

                
                # Moving to Top Right according to the current node
                flag, child_node = actionMoveTopRight(present_node,map)    
                child_node = tuple(child_node)

                if(flag is True and child_node not in closed_list):
                    if not isGoalNode(child_node, goalNode):
                        closelist_flag = False   
                        cost = cost_to_come + 1.4
                        child_node = list(child_node)

                        for node in opened_list:
                            if(node[1] == child_node):
                                closelist_flag = True
                                if(node[0] > cost):   
                                    node[0] = cost
                                    node[2] = present_node
                                break

                        if(not closelist_flag):    
                            hq.heappush(opened_list,[cost, child_node, present_node])
                    else:
                        print("\nGoal reached!")
                        end_time = time.time()
                        print("\nTime: "+str(round((end_time-start_time),4)) + " [secs]")
                        closed_list[child_node] = present_node
                        back_Tracking_Algo(goalNode,startNode,closed_list,map)
                        return True

                # Moving to Right according to the present_node      
                flag, child_node = actionMoveRight(present_node,map)    
                child_node = tuple(child_node)

                if(flag is True and child_node not in closed_list):
                    if not isGoalNode(child_node, goalNode):
                        closelist_flag = False    
                        cost = cost_to_come+1
                        child_node = list(child_node)
                        
                        for node in opened_list:
                            if(node[1] == child_node):
                                closelist_flag = True
                                if(node[0] > cost):    
                                    node[0] = cost
                                    node[2] = present_node
                                break

                        if(not closelist_flag):  
                            hq.heappush(opened_list,[cost, child_node, present_node])
                    else:
                        print("\nGoal reached!")
                        end_time = time.time()
                        print("\nTime: "+str(round((end_time-start_time),4)) + " [secs]")
                        closed_list[child_node] = present_node
                        back_Tracking_Algo(goalNode,startNode,closed_list,map)
                        return True

                
                # Moving to Bottom Right diagonally wrt to present node
                flag, child_node = actionMoveBottomRight(present_node,map) 
                child_node = tuple(child_node)

                if(flag is True and child_node not in closed_list):
                    if not isGoalNode(child_node, goalNode):
                        closelist_flag = False    
                        cost = cost_to_come + 1.4
                        child_node = list(child_node)

                        for node in opened_list:
                            if(node[1] == child_node):
                                closelist_flag = True
                                if(node[0] > cost):  
                                    node[0] = cost
                                    node[2] = present_node
                                break

                        if(not closelist_flag):   
                            hq.heappush(opened_list,[cost, child_node, present_node])
                    else:
                        print("\nGoal reached!")
                        end_time = time.time()
                        print("\nTime: "+str(round((end_time-start_time),4)) + " [secs]")
                        closed_list[child_node] = present_node
                        back_Tracking_Algo(goalNode,startNode,closed_list,map)
                        return True
    

                # Moving down wrt present node
                flag,child_node = actionMoveBottom(present_node,map)   
                child_node = tuple(child_node)

                if(flag is True and child_node not in closed_list):
                    if not isGoalNode(child_node, goalNode):
                        closelist_flag = False 
                        cost = cost_to_come + 1
                        child_node = list(child_node)

                        for node in opened_list:
                            if(node[1] == child_node):
                                closelist_flag = True
                                if(node[0] > cost):    
                                    node[0] = cost
                                    node[2] = present_node
                                break
                        if(not closelist_flag):  
                            hq.heappush(opened_list,[cost, child_node, present_node])
                    else:
                        print("\nGoal reached!")
                        end_time = time.time()
                        print("\nTime: "+str(round((end_time-start_time),4)) + " [secs]")
                        closed_list[child_node] = present_node
                        back_Tracking_Algo(goalNode,startNode,closed_list,map)
                        return True
                

                # Moving to Bottom Left wrt to the present node
                flag, child_node = actionMoveBottomLeft(present_node,map)  
                child_node = tuple(child_node)

                if(flag is True and child_node not in closed_list):
                    if not isGoalNode(child_node, goalNode):
                        closelist_flag = False  
                        cost = cost_to_come + 1.4
                        child_node = list(child_node)

                        for node in opened_list:
                            if(node[1] == child_node):
                                closelist_flag = True        
                                if(node[0] > cost):   
                                    node[0] = cost
                                    node[2] = present_node
                                break
                        if(not closelist_flag):    
                            hq.heappush(opened_list,[cost, child_node, present_node])
                    else:
                        print("\nGoal reached!")
                        end_time = time.time()
                        print("\nTime: "+str(round((end_time-start_time),4)) + " [secs]")
                        closed_list[child_node] = present_node
                        back_Tracking_Algo(goalNode,startNode,closed_list,map)
                        return True
                

                # Moving to Left wrt to the present node
                flag,child_node = actionMoveLeft(present_node,map)  
                child_node = tuple(child_node)

                if(flag is True and child_node not in closed_list):
                    if not isGoalNode(child_node, goalNode):
                        closelist_flag = False 
                        cost = cost_to_come + 1
                        child_node = list(child_node)

                        for node in opened_list:
                            if(node[1] == child_node):
                                closelist_flag = True
                                if(node[0] > cost):    
                                    node[0] = cost
                                    node[2] = present_node
                                break
                        if(not closelist_flag):    
                            hq.heappush(opened_list,[cost, child_node, present_node])
                    else:
                        print("\nGoal reached!")
                        end_time = time.time()
                        print("\nTime: "+str(round((end_time-start_time),4)) + " [secs]")
                        back_Tracking_Algo(goalNode,startNode,closed_list,map)
                        return True                


                # Moving to Top Left wrt present node
                flag,child_node = actionMoveTopLeft(present_node,map)   
                child_node = tuple(child_node)

                if(flag is True and child_node not in closed_list):
                    if not isGoalNode(child_node, goalNode):
                        closelist_flag = False    
                        cost = cost_to_come + 1.4
                        child_node = list(child_node)

                        for node in opened_list:
                            if(node[1] == child_node):
                                closelist_flag = True
                                cost = cost_to_come + 1.4
                                if(node[0] > cost):  
                                    node[0] = cost
                                    node[2] = present_node
                                break

                        if(not closelist_flag):    
                            hq.heappush(opened_list,[cost, child_node, present_node])
                    else:
                        print("\nGoal reached!")
                        end_time = time.time()
                        print("\nTime: "+str(round((end_time-start_time),4)) + " [secs]")
                        back_Tracking_Algo(goalNode,startNode,closed_list,map)
                        return True    
                        
        else:
            print("\n No path found between the start and goal explored_nodes") 
            return False

# Implementing back_Tracking_Algo algorithm to trace the shortest path 
def back_Tracking_Algo(goalNode, startNode, closed_list, map):
    video_writer = cv.VideoWriter_fourcc(*'mp4v')
    out = cv.VideoWriter('project2.mp4',video_writer,1000,(600,250)) # Saving the recorded video

    final_parent = closed_list.get(tuple(goalNode))   
    cv.line(map, tuple(goalNode), tuple(final_parent), (255,0,0), 1)

    parent_node_keys = closed_list.keys()
    for key in parent_node_keys:
        if key is not tuple(startNode):   
            map[key[1]][key[0]] = [255,255,255]
            cv.circle(map,tuple(startNode),5,(0,255,0),-1)
            out.write(map)
        cv.circle(map,tuple(goalNode),5,(0,255,0),-1)
        
        cv.imshow("Path Generation",map)
        cv.waitKey(1)

    while True:
        key = closed_list.get(tuple(final_parent))    
        
        cv.line(map, tuple(key), tuple(final_parent), (255,0,0), 1)
        out.write(map)

        final_parent = key
        
        if key is startNode:
            break

    cv.imshow("Path Generation", map)
    cv.waitKey(0)

# Calling the map generating functions 
if __name__ == '__main__':
# display map with original obstracles  
    map = Map_Generator(250, 600)
    # cv.imshow('map',map)
    # cv.waitKey(0)                 
    print('Enter the start position:')
    x_start = int(input("Enter your value of X-Axis: "))
    y_start = int(input("Enter your value of Y-Axis: "))
    print('Enter the goal position:')
    x_goal = int(input("Enter your value of X-Axis: "))
    y_goal = int(input("Enter your value of Y-Axis: "))
    if (checkInputFeasibility(x_start, y_start, x_goal, y_goal, map)):
        startNode = [x_start, y_start]
        goalNode = [x_goal, y_goal]
            
        res = Dijkstra_algo(startNode, goalNode, map)
        cv.destroyAllWindows()