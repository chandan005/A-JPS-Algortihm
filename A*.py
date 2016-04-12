import heapq
import math

class Agent(object):
    def __init__(self,**kwargs):
        self.openList = []
        self.closedSet = set()
        self.came_from = {}
        self.path = []
        self.call = True
        

    # method to reconstruct the path from the parents dict
    def reconstruct(self,came_from,goal):
        node = goal
        self.path = [node]
        while node in came_from:
            node = came_from[node]
            self.path.append(node)
        return self.path


    # returns a step towards the goal using A Star algorithm
    def getNext(self, mapref, current, goal, timeremaining):
        c = (64,215)
        for move in mapref.getAllAdjacents(c):
            if mapref.isPassable(move,(64,214)):
                print move
        if self.call == True:
            self.call = False
            heapq.heappush(self.openList,(mapref.getH(current,goal),0,current))
            while goal not in self.closedSet:
                _,cost,position = heapq.heappop(self.openList)
            
                if position == goal:
                    print "Goal Found"
                    self.reconstruct(self.came_from,goal)
                    break
            
                if position in self.closedSet:
                    continue
                self.closedSet.add(position)
                for move in mapref.getAllAdjacents(position):
                    check = mapref.isPassable(move,position)

                    if check == True:
                        

                        if move in self.closedSet:
                            continue
                        newCost = mapref.getCost(position,move)
                
                        if (move not in self.closedSet or newCost < cost):
                            self.came_from[move] = position
                            cost = newCost
                            p = cost + mapref.getH(goal,move)
                            heapq.heappush(self.openList,(p,newCost,move))
                               
            print "No of nodes in OPEN: %d" % (len(self.openList))
            return self.path.pop()
        else:
            return self.path.pop()
        return None

    
    def reset(self, **kwargs):
        pass
