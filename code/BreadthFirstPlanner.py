import numpy

class BreadthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        
    def Plan(self, start_config, goal_config):
        
        plan = []

        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        # inspired by https://en.wikipedia.org/wiki/Breadth-first_search

        # Breadth-First-Search(Graph, root):
            
        #     create empty set S
        #     create empty queue Q      

        #     root.parent = NIL
        #     add root to S
        #     Q.enqueue(root)                      

        #     while Q is not empty:
        #         current = Q.dequeue()
        #         if current is the goal:
        #             return current
        #         for each node n that is adjacent to current:
        #             if n is not in S:
        #                 add n to S
        #                 n.parent = current
        #                 Q.enqueue(n)

        start_config_discrete = self.planning_env.discrete_env.NodeIdToConfiguration(self.planning_env.discrete_env.ConfigurationToNodeId(start_config))
        goal_config_discrete = self.planning_env.discrete_env.NodeIdToConfiguration(self.planning_env.discrete_env.ConfigurationToNodeId(goal_config))

        S = []
        Q = []
        #keep track of parents
        parents = {}
        S.append(start_config_discrete)
        Q.append(start_config_discrete)

        while len(Q) > 0:
            current = Q.pop(0)
            if current == goal_config_discrete:
                break
            adjacent_ids = self.planning_env.GetSuccessors(self.planning_env.discrete_env.ConfigurationToNodeId(current))
            adjacent = []
            for id_num in adjacent_ids:
                adjacent.append(self.planning_env.discrete_env.NodeIdToConfiguration( id_num ))
            for n in adjacent:
                if n not in S:
                    S.append(n)
                    parents[tuple(n)] = tuple(current)
                    Q.append(n)

        plan.append(goal_config)
        parent = parents[tuple(goal_config_discrete)]
        while parents[parent] != tuple(start_config_discrete):
            plan.append(list(parent))
            parent = parents[parent]
        plan.append(start_config)
   
        plan.reverse()

        numpy.array(plan)

        return plan
