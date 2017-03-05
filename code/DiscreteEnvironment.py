import numpy

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for idx in range(self.dimension):
            self.num_cells[idx] = numpy.ceil((upper_limits[idx] - lower_limits[idx])/resolution)


    def ConfigurationToNodeId(self, config):
        
        # TODO:
        # This function maps a node configuration in full configuration
        # space to a node in discrete space
        #
        node_id = 0

        node_id = self.GridCoordToNodeId(self.ConfigurationToGridCoord(config))

        return node_id

    def NodeIdToConfiguration(self, nid):
        
        # TODO:
        # This function maps a node in discrete space to a configuraiton
        # in the full configuration space
        #
        config = [0] * self.dimension

        config = self.GridCoordToConfiguration(self.NodeIdToGridCoord(nid))

        return config
        
    def ConfigurationToGridCoord(self, config):
        
        # TODO:
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #
        coord = [0] * self.dimension

        for idx in range(self.dimension):
            coord[idx] = numpy.floor((config[idx] - self.lower_limits[idx])/self.resolution)

        return coord

    def GridCoordToConfiguration(self, coord):
        
        # TODO:
        # This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        #
        config = [0] * self.dimension

        for idx in range(self.dimension):
            #need to check if logic needs to be in place if the lower limit lines up improperly with the resolution
            config[idx] = (coord[idx]*self.resolution)+(self.resolution/2.0)+self.lower_limits[idx]

        return config

    def GridCoordToNodeId(self,coord):
        
        # TODO:
        # This function maps a grid coordinate to the associated
        # node id 
        node_id = 0
        shift_accum = 0
        for idx in range(self.dimension):
            shifted = coord[idx] << shift_accum
            node_id += shifted
            shift_accum += numpy.ceil(numpy.log2(self.num_cells[idx]))
        return node_id

    def NodeIdToGridCoord(self, node_id):
        
        # TODO:
        # This function maps a node id to the associated
        # grid coordinate
        coord = [0] * self.dimension

        shift_accum = 0
        for idx in range(self.dimension):
            shifted = node_id >> shift_accum
            coord[idx] = shifted & (pow(2,numpy.ceil(numpy.log2(self.num_cells[idx])))-1
            shift_accum += numpy.ceil(numpy.log2(self.num_cells[idx]))
        
        return coord
        
        
        
