from heapq import heappush, heappop, heapify
from collections import deque

class Agent(object):
    def __init__(self,**kwargs):
        self.clockface = ((1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1))

    

    def getNext(self, mapref, current, goal, timeremaining): 
        queue = [(0, current, 0, None)]
        enqueued = {}
        explored = {}
        while queue:
            _, current_node, dist, parent = heappop(queue)
            if current_node == goal:
                cs = self.reconstruct(current_node,parent,explored)
                next(cs)
                break
            if current_node in explored:
                continue
            explored[current_node] = parent
            for move in mapref.getAllAdjacents(current_node):
                if move in explored:
                    continue
                ncost = dist + mapref.getCost(current_node,move)
                if move in enqueued:
                    qcost, h = enqueued[move]
                    if qcost <= ncost:
                        continue
                else:
                    h = mapref.getH(goal,move)
                enqueued[move] = ncost, h
                heappush(queue,(ncost+h, move, ncost, current_node))
        return next(cs)

    def reconstruct(self,curnode,parent,explored):
        path = [curnode]
        node = parent
        while node is not None:
            path.append(node)
            node = explored[node]
        path.reverse()
        for a in path:
            yield a

    def reset(self, **kwargs):
        pass
