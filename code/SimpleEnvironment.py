import numpy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.5], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        
        coord = self.discrete_env.NodeIdToGridCoord(node_id)

        #since we're doing 4 connectivity
        n_coord = [coord[0], coord[1]+1]
        s_coord = [coord[0], coord[1]-1]
        e_coord = [coord[0]+1, coord[1]]
        w_coord = [coord[0]-1, coord[1]]
        potential_successors = []
        potential_successors.append(n_coord)
        potential_successors.append(s_coord)
        potential_successors.append(e_coord)
        potential_successors.append(w_coord)
        for idx in range(len(potential_successors)):
            config = self.discrete_env.GridCoordToConfiguration(potential_successors[idx])
            if (config[0] > self.lower_limits[0]) and (config[0] < self.upper_limits[0]) and (config[1] > self.lower_limits[1]) and (config[1] < self.upper_limits[1]):
                successors.append(self.discrete_env.ConfigurationToNodeId(config))

        return successors

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids

        start_coord = self.discrete_env.NodeIdToGridCoord(start_id)
        end_coord = self.discrete_env.NodeIdToGridCoord(end_id)

        dist = numpy.abs(start_coord[0]-end_coord[0])+numpy.abs(start_coord[1]-end_coord[1])

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids

        cost_factor = 1.0

        distance = self.ComputeDistance(start_id,goal_id)

        cost = distance*cost_factor

        return cost

    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()

        
    # Check if new locn has collisions
    def HasCollisions(self, locn):
        collisions = []
        with self.robot.GetEnv():
            # Apply new_locn to current robot transform
            orig_transform = self.robot.GetTransform()
            new_transform = self.ApplyMotion(locn)
            self.robot.SetTransform(new_transform)

            # Check for collision
            bodies = self.robot.GetEnv().GetBodies()[1:]
            collisions = map(lambda body: self.robot.GetEnv().CheckCollision(self.robot, body), bodies)                

            # Set robot back to original transform
            self.robot.SetTransform(orig_transform)
        return reduce(lambda x1, x2: x1 or x2, collisions)