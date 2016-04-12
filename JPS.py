
import heapq
import math
from collections import deque

# Customised heapq for faster retrieving of elements
class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements,(priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]
    def length(self):
        return len(self.elements)


class Agent(object):
    def __init__(self,**kwargs):
        self.path = []

    # method to reconstruct the path from the parents dict
    def reconstruct(self,mapref,came_from,goal,current):
        #path = []
        node = goal
        next = came_from[node]
        while (next != None) and (node != current):
            dx = next[0] - node[0]
            if (dx != 0):
                dx /= int(round(math.fabs(dx)))
            dy = next[1] - node[1]
            if (dy != 0):
                dy /= int(round(math.fabs(dy)))
            
            while (node != next):
                self.path.append(node)
                cX = node[0]
                cY = node[1]
                nX = cX + dx
                nY = cY + dy
                node = (nX,nY)
            node = next
            next = came_from[next]
        #self.path.append(current)
        return self.path
        #self.path = [(309, 203), (308, 203), (307, 203), (306, 203), (305, 203), (304, 203), (303, 203), (302, 203), (301, 203), (300, 203), (299, 203), (298, 203), (297, 203), (296, 203), (295, 203), (294, 202), (293, 201), (292, 200), (291, 199), (290, 198), (289, 197), (288, 196), (287, 195), (286, 194), (285, 193), (284, 192), (283, 191), (282, 190), (281, 189), (280, 188), (279, 187), (278, 186), (277, 185), (276, 184), (275, 183), (274, 182), (273, 181), (272, 180), (271, 179), (270, 178), (269, 177), (268, 176), (267, 175), (266, 174), (265, 173), (264, 172), (263, 171), (262, 170), (261, 169), (260, 168), (259, 167), (258, 166), (257, 166), (256, 165), (255, 164), (254, 163), (253, 162), (252, 162), (251, 162), (250, 162), (249, 162), (248, 162), (247, 162), (246, 162), (245, 162), (244, 162), (243, 162), (242, 162), (241, 162), (240, 162), (239, 162), (238, 162), (237, 162), (236, 162), (235, 162), (234, 162), (233, 162), (232, 162), (231, 162), (230, 162), (229, 162), (228, 162), (227, 162), (226, 162), (225, 162), (224, 162), (223, 162), (222, 162), (221, 162), (220, 162), (219, 162), (218, 162), (217, 162), (216, 162), (215, 162), (214, 162), (213, 162), (212, 162), (211, 162), (210, 162), (209, 162), (208, 162), (207, 162), (206, 162), (205, 162), (204, 162), (203, 162), (202, 162), (201, 162), (200, 162), (199, 162), (198, 162), (197, 162), (196, 162), (195, 162), (194, 162), (193, 161), (192, 160), (191, 159), (190, 158), (189, 157), (188, 156), (187, 155), (186, 154), (185, 153), (184, 152), (183, 151), (182, 150), (181, 149), (180, 148), (179, 147), (178, 146), (177, 145), (176, 144), (175, 143), (174, 142), (173, 141), (172, 140), (171, 139), (170, 138), (169, 137), (168, 136), (167, 135), (166, 134), (165, 133), (164, 132), (163, 131), (162, 130), (161, 129), (160, 128), (159, 127), (158, 126), (157, 125), (156, 124),(155, 123)]
        #return self.path

    # returns a step towards the goal using JPS algorithm
    def getNext(self, mapref, current, goal, timeremaining):
        frontier = []
        heapq.heapify(frontier)
        costSoFar={}
        came_from = {}
        came_from[current] = None
        heapq.heappush(frontier,(0,current))
        costSoFar[current] = 0
        while len(frontier) != 0:
            position = heapq.heappop(frontier)[1]
            
            if position == goal:
                self.reconstruct(mapref,came_from,goal,current)
                break
            adj = self.identifySuccessors(mapref,position,current,goal,came_from)
            for move in adj:
                check = bool(mapref.isPassable(move,position))
                new_cost = costSoFar[position] + mapref.getCost(position,move)
                if check == False or move in costSoFar:
                    continue
                
                if move not in costSoFar or new_cost < costSoFar[move]:
                    came_from[move] = position
                    costSoFar[move] = new_cost
                    priority = new_cost + mapref.getH(move,goal)
                    heapq.heappush(frontier,(priority,move))
        print "No of nodes in OPEN: %d" % (len(frontier))
        return self.path.pop()

    # Method to prune the neighbors 
    def getNeighborsPruned(self,mapref,node,dx,dy):
            neighbors = []
            nextX = node[0]+dx
            nextY = node[1]+dy
            temp = (0,0)

            # checking for diagonal case
            if (dx != 0 and dy != 0):
                # Adding natural neighbors to the list
                temp = (node[0],node[1]+dy)
                checkDA = mapref.isPassable(temp,node)
                if((temp != None) and (checkDA==True)):
                    neighbors.append(temp)
                temp = (node[0]+dx,node[1])
                checkDB = mapref.isPassable(temp,node)
                if ((temp != None) and (checkDB==True)):
                    neighbors.append(temp)
                temp = (node[0]+dx,node[1]+dy)
                checkDC = mapref.isPassable(temp,node)
                if ((temp != None) and (checkDC==True)):
                    neighbors.append(temp)

                # checking for forced neighbors 
                xCheck = (mapref.isPassable((node[0],node[1]-dy))) and (mapref.isPassable((nextX,node[1]-dy)))
                yCheck = (mapref.isPassable((node[0]-dx,node[1]+dy))) and (mapref.isPassable((node[0]-dx,nextY)))
                # add forced neighbors to the list
                if(xCheck):
                    neighbors.append((node[0]+dx,node[1]-dy))
                if(yCheck):
                    neighbors.append((node[0]-dx,node[1]+dy))

            # checking for horizontal/vertical case
            elif(dx != 0):
                nextY = node[1]
                temp = (nextX,node[1])
                forwardCheck = bool((temp != None) and (mapref.isPassable(temp,node)==True))
                # add natural neighbors to the list
                if(forwardCheck):
                    neighbors.append(temp)
                    # checking for forced neighbors
                upCheck = bool((mapref.isPassable((node[0],nextY-1))==False) and (mapref.isPassable((nextX,nextY-1))==True))
                downCheck = bool((mapref.isPassable((node[0],nextY+1))==False) and (mapref.isPassable((nextX,nextY+1))==True))
                    # adding forced neighbors
                if(upCheck):
                    neighbors.append((nextX,nextY-1))
                if(downCheck):
                    neighbors.append((nextX,nextY+1))
                
            # checking for vertical case
            elif(dy != 0):
                nextX = node[0]
                temp = (node[0],nextY)
                check = mapref.isPassable(temp,node)
                forwardCheck = bool((temp != None) and (mapref.isPassable(temp,node)))
                if(forwardCheck):
                    neighbors.append(temp)
                leftCheck = bool((mapref.isPassable((nextX-1,node[1]))==False) and (mapref.isPassable((nextX-1,nextY))==True))
                rightCheck = bool((mapref.isPassable((nextX+1,node[1]))==False) and (mapref.isPassable((nextX+1,nextY))==True))
                if(leftCheck):
                    neighbors.append((nextX-1,nextY))
                if(rightCheck):
                    neighbors.append((nextX+1,nextY))
              
            return neighbors

    # returns successors of a node after pruning and jumping
    def identifySuccessors(self,mapref,nodes,current,goal,came_from):
        successors = []
        dx = 0
        dy = 0
        neighbor = []
        parent = came_from[nodes]
        if(parent == None):
            for cell in mapref.getAdjacents(nodes):
                check = mapref.isPassable(cell,nodes)
                if(check==True):
                    neighbor.append(cell)
        else:
            dx = nodes[0] - parent[0]
            if(dx != 0):
                dx /= int(round(math.fabs(dx)))
            dy = nodes[1] - parent[1]
            if(dy != 0):
                dy /= int(round(math.fabs(dy)))
            neighbor.extend(self.getNeighborsPruned(mapref,nodes,dx,dy))
        for jumpNeighbor in neighbor:
            #if mapref.isPassable(jumpNeighbor):
            dx = (jumpNeighbor[0] - nodes[0])
            dy = (jumpNeighbor[1] - nodes[1])

            jumpPoint = self.jump(mapref,dx,dy,nodes,current,goal)
            if(jumpPoint != None):
                successors.append(jumpPoint)
        return successors

    # returns jump point towards goal
    def jump(self,mapref,dx,dy,current_node,current,goal):
        nextX = current_node[0]+dx
        nextY = current_node[1]+dy

        if(mapref.isPassable((nextX,nextY))==False):
            return None
        next = (nextX,nextY)
        if(next == goal):
            return next
        if(dx != 0 and dy != 0):
            
            xCheck = bool((mapref.isPassable((current_node[0],current_node[1]-dy))==False) and (mapref.isPassable((nextX,current_node[1]-dy),current_node)==True))
            
            yCheck = bool ((mapref.isPassable((current_node[0]-dx,current_node[1]))==False) and (mapref.isPassable((current_node[0]-dx,nextY),current_node)==True))

            if (xCheck or yCheck):
                return next
            if((self.jump(mapref,dx,0,next,current,goal) != None) or (self.jump(mapref,0,dy,next,current,goal))!=None):
                return next
            
        else:
            if(dx != 0):
                upCheck = bool((mapref.isPassable((nextX,nextY-1))==False) and (mapref.isPassable((nextX+dx,nextY-1))==True))
                downCheck = bool((mapref.isPassable((nextX,nextY+1))==False) and (mapref.isPassable((nextX+dx,nextY+1))==True))

                if(upCheck or downCheck):
                        return next
            else:
                leftCheck = bool((mapref.isPassable((nextX-1,nextY))==False) and (mapref.isPassable((nextX-1,nextY+dy))==True))
                rightCheck = bool((mapref.isPassable((nextX+1,nextY))==False) and (mapref.isPassable((nextX+1,nextY+dy))==True))

                if (leftCheck or rightCheck):
                    return next


        return self.jump(mapref,dx,dy,next,current,goal)


    def reset(self, **kwargs):
        pass
