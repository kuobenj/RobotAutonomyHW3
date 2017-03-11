import numpy
from DiscreteEnvironment import DiscreteEnvironment

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        self.table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        
        self.robot.GetEnv().Add(self.table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        self.table.SetTransform(table_pose)
        
        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
    
    def GetSuccessors(self, node_id):

        successors = []
       
        grid=self.discrete_env.NodeIdToGridCoord(node_id)
        for i in range(0,self.discrete_env.dimension):
            #newCoord=grid[:]
            print "newCoord"
            print grid

            # Surbtaction neighbor
            grid[i] = grid[i]+1
            print "g1"
            print grid
            newNode = self.discrete_env.GridCoordToNodeId(grid)
            print "node"
            print newNode
            if (self.IsCollision(newNode )==False  and self.IsBoundary(newNode)==True):
                successors.append(newNode )

            # Addition Neighbor
            grid[i] = grid[i]-2
            print "g2"
            print grid
            print "node"
            print newNode
            newNode  = self.discrete_env.GridCoordToNodeId(grid)
            if (not self.IsCollision(newNode)  and self.IsBoundary(newNode)):
                successors.append(newNode )
            grid[i]=grid[i]+1

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        
        return successors

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids
        line = NodeIdToConfiguration(end_id) - NodeIdToConfiguration(start_id)
        dist = numpy.linalg.norm(line)
        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0
        #start_config=self.discrete_env.NodeIdToConfigration(start_config)
        #goal_config=self.discrete_env.NodeIdToConfigration(goal_config)


        start_conf=self.discrete_env.NodeIdToGridCoord(start_id)
        goal_conf=self.discrete_env.NodeIdToGridCoord(goal_id)

        for i in range(len(start_conf)):
            cost=cost+abs(start_conf[i]-goal_conf[i])

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        
        return cost

    def IsBoundary(self,coord):
        conf=self.discrete_env.NodeIdToConfiguration(coord)
        for i in range(len(conf)):
            if conf[i]<self.lower_limits[i] or conf[i]>self.upper_limits[i] :
                return False
        return True



    def IsCollision(self,coord):
        conf=self.discrete_env.NodeIdToConfiguration(coord)
        self.robot.SetActiveDOFValues(conf)

        if self.robot.GetEnv().CheckCollision(self.robot,self.table) or self.robot.GetEnv().CheckCollision(self.robot,self.robot):
            return True
        else:
            return False
          







