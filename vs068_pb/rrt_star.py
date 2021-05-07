from vs068_pb.rrt import TreeNode, configs

class OptimalNode(TreeNode):
    def __init__(self):
        self.config = config
        self.parent = parent
        self.children = set()
        self.d = d
        self.path = path
        if parent is not None:
            self.cost = parent.cost + d
            self.parent.children.add(self)
        else:
            self.cost = d
        self.solution = False
        self.creation = iteration
        self.last_rewire = iteration

    def retrace(self):
        if self.parent is None:
            return self.path + [self.config]
        return self.parent.retrace() + self.path + [self.config]



# RRT* Pseudo Code - https://theclassytim.medium.com/robotic-path-planning-rrt-and-rrt-212319121378
# Rad = r
# G(V,E) //Graph containing edges and vertices
# For itr in range(0…n)
#     Xnew = RandomPosition()
#     If Obstacle(Xnew) == True, try again
#     Xnearest = Nearest(G(V,E),Xnew)
#     Cost(Xnew) = Distance(Xnew,Xnearest)
#     Xbest,Xneighbors = findNeighbors(G(V,E),Xnew,Rad)
#     Link = Chain(Xnew,Xbest)
#     For x’ in Xneighbors
#         If Cost(Xnew) + Distance(Xnew,x’) < Cost(x’)
#             Cost(x’) = Cost(Xnew)+Distance(Xnew,x’)
#             Parent(x’) = Xnew
#             G += {Xnew,x’}
#     G += Link 
# Return G