import math
import heapq

def shortest_path(M,start,goal):

# Helper Class, represents an edge between 2 nodes, stores the distance
    class GraphEdge(object):
        def __init__(self, destinationNode, distance):
            self.node = destinationNode
            self.distance = distance

# Helper Classes
    class GraphNode(object):
        def __init__(self, val):
            self.value = val   # node index
            parent = None
            self.coordx = 0
            self.coordy = 0
            self.gvalue = 0
            self.hvalue = 0
            self.fvalue = math.inf
            self.edges = []     #  list of GraphEdge objects
        def __lt__(self, other):          
            	return self.fvalue < other.fvalue   # sort nodes by min fvalue

        # children = Edges. adds GraphEdge object to another node
        # Note: children a twice in the list because edges are added in both directions
        def add_child(self, node, distance):		
            self.edges.append(GraphEdge(node, distance))

        def remove_child(self, del_node):          # removes GraphEdge object to another node (del_node)
            if del_node in self.edges:
                self.edges.remove(del_node)

        # calculate Euklidian distance to another node
        def getdistance(self, othernode):
            distance = math.sqrt( math.pow((self.coordx - othernode.coordx), 2) + math.pow((self.coordy - othernode.coordy), 2) )
            return distance

    # Graph object   
    class Graph(object):
        def __init__(self, node_list):
            self.nodes = node_list

    # adds an edge between node1 and node2 in both directions
        def add_edge(self, node1, node2, distance):
            if node1 in self.nodes and node2 in self.nodes:
                node1.add_child(node2, distance)
                node2.add_child(node1, distance)

        def remove_edge(self, node1, node2):
            if node1 in self.nodes and node2 in self.nodes:
                node1.remove_child(node2)
                node2.remove_child(node1)


    # create list of nodes from the map of intersections; store x, y coordinates in each node
    nodelist = []
    for key in M.intersections:
        x = M.intersections[key][0]
        y = M.intersections[key][1]
        node = GraphNode(key)
        node.coordx = x
        node.coordy = y
        nodelist.append(node)

    # create graph object by using the road map. Add edge objects to nodes and store the distance in each edge
    graph = Graph(nodelist)
    
    #for key in M.roads:					     
        #for x in range(len(M.roads[key])):                 
            #value = M.roads[key][x]      
            #distance_ = graph.nodes[key].getdistance(graph.nodes[value])
            #graph.add_edge(graph.nodes[key], graph.nodes[value], distance_)

    for index, list in enumerate(M.roads):
        for x in range(len(list)):
            value = list[x]
            distance_ = graph.nodes[index].getdistance(graph.nodes[value])
            graph.add_edge(graph.nodes[index], graph.nodes[value], distance_)

    # create a priority queue with heapq; push all nodes from graph.nodes into the queue
    queue = []	
    for node in graph.nodes:
        heapq.heappush(queue, node)

    # The A star search algorithm:
    path = []
    current_node = queue[start]    # start node 
    current_node.fvalue = 0
    current_node.gvalue = 0
    end_node = queue[goal]         # end node 

    # if start node = end node, put only that node in the path
    if current_node == end_node:    
        found_path = [end_node.value]
        return found_path
        return

    while current_node != end_node :         
        # find the node with the smallest f-value in the queue, remove it from queue and place it in path list
        heapq.heapify(queue)
        current_node = heapq.heappop(queue)
        path.append(current_node)

        # for each neighbour of current node, update the g-value, h-value , f-value
        for edge in current_node.edges : 
            if edge.node in queue :      
                # update g-value (total travelled distance from start node)   
                edge.node.gvalue = current_node.gvalue + edge.distance               
                # update h-value (euklidian distance to end node)
                edge.node.hvalue = edge.node.getdistance(end_node) 
                # update f-value = g-value + h-value
                edge.node.fvalue = edge.node.gvalue + edge.node.hvalue   
                edge.node.parent = current_node
    
    found_path = [node.value for node in path]
    return found_path
