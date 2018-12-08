import unittest
import datetime
import math
from collections import deque

class Graph(object):
    """Class to store a graph as a dictionary of dictionaries"""

    def __init__(self, value): #O(1)
        self.graph = {}
        self.graph[value] = {}

    def addNode(self, node): #O(1)
        """Adds a node to the graph"""
        self.graph[node] = {}                                       #Adds a node to the graph with no connections to it

    def addEdge(self, node, target, weight): #O(1)
        """Creates a link between two nodes"""
        if type(weight) != (int or float):                          #Raise error when try to enter weight that isnt a number
            raise TypeError("Weight must be an number")
        elif (node or target) not in self.graph.keys():             #Raise an error when the node or the target arent in the graph
            raise KeyError("Either Node or Target not found")
        else:
            self.graph[node][target] = weight                       #Adds the connected node to its connections with the weight supplied
            self.graph[target][node] = weight                       #Adds the start node to the target node's connections with the weight supplied

    def printGraph(self): #O(n)
        """Prints the graph with list of nodes and their connected nodes"""
        for i in self.graph:
            print(str(i) +": " + str(self.graph[i]))
        return True

    def depthFirst(self, node): #O(n^2)
        """Takes a starting node and returns a list from a depth first search of the graph"""
        if node not in self.graph.keys():                                           #Raise ValueError it start node isn't in the graph
            raise ValueError("The start node was not found in the graph")
        toVisit = []                                                                #Stack of nodes to visit
        visited = []                                                                #List of nodes visited
        toVisit.append(node)
        while len(toVisit) != 0:                                                    
            workingNode = toVisit.pop()
            if workingNode not in visited:                                          #If node hasn't been visited add it to visited
                visited.append(workingNode)
                for edge in self.graph[workingNode]:                                #Add every edge connected to the working node
                    toVisit.append(edge)
                print(toVisit)
        return visited

    def breadthFirst(self, node): #O(n^2)
        """Takes a starting node and returns a list from a breath first search of the graph"""
        if node not in self.graph.keys():                                           #Raise ValueError it start node isn't in the graph
            raise ValueError("The start node was not found in the graph")
        toVisit = deque([])                                                         #Queue of items to visit
        visited = []                                                                #List of nodes visited  
        toVisit.append(node)
        while len(toVisit) != 0:
            workingNode = toVisit.popleft()                                         #Dequeues node
            if workingNode not in visited:                                          #If node hasn't been visited add it to visited
                visited.append(workingNode)
                for edge in self.graph[workingNode]:                                #Add every edge connected to the working node
                    toVisit.append(edge)
        return visited

    def isPath(self, v, w): #O(n^2)
        """Takes a graph and two nodes and checks if there is a path between them.
        If yes returns true and writes path to file. If false returs False"""
        if  (v not in self.graph.keys()) or (w not in  self.graph.keys()):          #Raise ValueError it start node or target node isn't in the graph
            raise ValueError("One of the nodes was not found in the graph")
        toVisit = deque([])
        visited = []
        toVisit.append(v)
        while len(toVisit) != 0:
            workingNode = toVisit.popleft()
            if w in self.graph[workingNode]:                                        #If the target node is in the list of nodes the working node is connected to
                visited.append(workingNode)                                         #Add working node to visited
                visited.append(w)                                                   #Add target node to visited
                break  
            elif workingNode not in visited:
                visited.append(workingNode)
                for edge in self.graph[workingNode]:
                    toVisit.append(edge)
        if w in visited:
            fileName = str(datetime.datetime.today()) + "Shortest_Path.txt"
            f = open(fileName, "w+")
            for i in visited:
                f.write(str(i) + " ")
            f.close()
            return True
        else:
            return False

    def isConnected(self): #O(n^2)
        """Checks is a graph is connected"""           
        firstItem = (list(self.graph.keys())[0])           #Chooses start node as the first node in the dictionary for the graph
        search = self.depthFirst(firstItem)                #Do a depth first search from this start node
        return "yes" if len(search) == len((self.graph).keys()) else "no"

    def dijkstra(self, start, target): #O(n^2)
        """Takes a graph and a start node and target node and uses dijkstra's algorithm to find the shortest path to the target"""
        if (start not in self.graph.keys()) or (target not in self.graph.keys()):   #Checks that the starting node and target node are in the graph
            raise ValueError("One of the nodes was not found in the graph")
        shortestPath = {}                                                           #Node and the weight of the shortest path to that node from the start node
        fromNode = {}                                                               #Node and the node before that the shortest path comes from
        toVisit = self.graph.copy()                                                 #Copy of the graph to keep a list of nodes that need visiting
        for i in toVisit:
            shortestPath[i] = math.inf                                              #Initialise every nodes shortest path to infinity
        shortestPath[start] = 0                                                     #Set the start node's shortest path to 0
        while toVisit:
            minNode = None
            for node in toVisit:                                                    #Iterates over all nodes in toVisit and selects one with the shortest weight in shortest path
                if minNode == None:                                                 #First case where node == None
                    minNode = node
                elif shortestPath[node] < shortestPath[minNode]:
                    minNode = node
            for toNode, weight in toVisit[minNode].items():                         #Looks at all the nodes that the current node can get to 
                if (weight + shortestPath[minNode]) < shortestPath[toNode]:         #If the weight to the node + the weight to get to the current node is less than its current path
                    shortestPath[toNode] = (weight + shortestPath[minNode])         #Update the shortest path to the weight to the node + the weight to get to the current node
                    fromNode[toNode] = minNode                                      #Update which node you came from to get to that node
            toVisit.pop(minNode)                                                    #Remove the current node from the list to visit
        path = []
        workingNode = target
        while workingNode != start:                                                 #Iterate over every node's path node to get path
            path.insert(0, workingNode)                                             #Inserts current node at the start of the list
            workingNode = fromNode[workingNode]                                     #Set the next node to the one before
        path.insert(0, start)
        cost = shortestPath[target]
        return path, cost

class GraphTest(unittest.TestCase):

    def setUp(self):
        self.g1 = Graph(1)
        self.g2 = Graph(0)
        self.g3 = Graph("Cat")
        self.g4 = Graph(-1)
        self.g5 = Graph(True)
        #This is a graph called fledged used in testing is not connected
        self.fledged = Graph(1)
        self.fledged.addNode(2)
        self.fledged.addNode(3)
        self.fledged.addNode(4)
        self.fledged.addNode(5)
        self.fledged.addNode(6)
        self.fledged.addNode(7)
        self.fledged.addEdge(1, 2, 1)
        self.fledged.addEdge(2, 3, 2)
        self.fledged.addEdge(3, 5, 3)
        self.fledged.addEdge(2, 6, 4)
        self.fledged.addEdge(3, 4, 5)
        self.fledged.addEdge(5, 6, 8)


    def test_makeTree(self):
        #Test 1 to test that graph was produced with first node 1
        self.assertEqual(self.g1.graph, {1: {}})
        #Test 2 to test that graph was produced with first node 0
        self.assertEqual(self.g2.graph, {0: {}})
        #Test 3 to test that graph was produced with first node "Cat"
        self.assertEqual(self.g3.graph, {"Cat": {}})
        #Test 4 to test that graph was produced with first node -1
        self.assertEqual(self.g4.graph, {-1: {}})
        #Test 5 to test that graph was produced with first node True
        self.assertEqual(self.g5.graph, {True: {}})

    def test_addNode(self):
        #Test 1
        self.g1.addNode(0)
        self.assertEqual(self.g1.graph, {1: {}, 0: {}})
        #Test 2
        self.g1.addNode(-1)
        self.assertEqual(self.g1.graph, {1: {}, 0: {}, -1: {}})
        #Test 3
        self.g2.addNode("Cat")
        self.assertEqual(self.g2.graph, {0: {}, "Cat": {}})
        #Test 4
        self.g3.addNode(False)
        self.assertEqual(self.g3.graph, {"Cat": {}, False: {}})
            
    def test_addEdge(self):
        #Creating graph to test on
        self.g1.addNode(2)
        self.g1.addNode("Dog")
        self.g1.addNode(-1)
        #Create edge between 1 and 2
        self.g1.addEdge(1, 2, 1)
        self.assertEqual(self.g1.graph, {1:{2:1}, 2: {1: 1}, "Dog": {}, -1: {}})
        #Create edge between "Dog" and 2
        self.g1.addEdge("Dog", 2, -1)
        self.assertEqual(self.g1.graph, {1: {2:1}, 2: {1:1,"Dog":-1}, "Dog": {2:-1}, -1: {}})
        #Create edge with something that isn't in tree
        with self.assertRaises(KeyError):
            self.g1.addEdge("Dog", "Pony", 0)
        #When variable is not a graph object
        c = 0
        with self.assertRaises(AttributeError):
            c.addEdge(1, 2, 3)

    def test_printGraph(self):
        #Testing on graphs made in setUp
        self.assertEqual(self.g1.printGraph(), True)
        self.assertEqual(self.g2.printGraph(), True)
        self.assertEqual(self.g3.printGraph(), True)
        self.assertEqual(self.g4.printGraph(), True)
        self.assertEqual(self.g5.printGraph(), True)
        self.assertEqual(self.fledged.printGraph(), True)
        a = Graph(None)
        self.assertEqual(a.printGraph(), True)
        #When there is no graph object woth that variable
        with self.assertRaises(NameError):
            b.printGraph()
        #When variable is not a graph object
        c = 0
        with self.assertRaises(AttributeError):
            c.printGraph()

    def test_depthFirst(self):
        #Connected part of graph
        self.assertEqual(self.fledged.depthFirst(1), [1, 2, 6, 5, 3, 4])
        #Disconnected node
        self.assertEqual(self.fledged.depthFirst(7), [7])
        #Start node not in graph
        with self.assertRaises(ValueError):
            self.fledged.depthFirst(0)
        #When variable is not a graph object
        c = 0
        with self.assertRaises(AttributeError):
            c.depthFirst()

    def test_breadthFirst(self):
        #Connected part of graph
        self.assertEqual(self.fledged.breadthFirst(1), [1, 2, 3, 6, 5, 4])
        #Disconnected node
        self.assertEqual(self.fledged.breadthFirst(7), [7])
        #Start node not in graph
        with self.assertRaises(ValueError):
            self.fledged.breadthFirst(0)
        #When variable is not a graph object
        c = 0
        with self.assertRaises(AttributeError):
            c.breadthFirst()

    def test_isPath(self):
        self.assertEqual(self.fledged.isPath(1, 2), True)
        self.assertEqual(self.fledged.isPath(1, 3), True)
        self.assertEqual(self.fledged.isPath(1, 4), True)
        self.assertEqual(self.fledged.isPath(1, 5), True)
        self.assertEqual(self.fledged.isPath(1, 6), True)
        self.assertEqual(self.fledged.isPath(1, 7), False)
        self.assertEqual(self.fledged.isPath(7, 7), True)
        #When a searched value isn't in the graph
        with self.assertRaises(ValueError):
            self.fledged.isPath(-1, "Cat")

    def test_isConnected(self):
        self.assertEqual(self.fledged.isConnected(),"no")
        self.assertEqual(self.g1.isConnected(), "yes")
        #When variable is not a graph object
        c = 0
        with self.assertRaises(AttributeError):
            c.isConnected()

    def test_dijkstra(self):
        self.assertEqual(self.fledged.dijkstra(1, 6), ([1, 2, 6], 5))
        self.assertEqual(self.fledged.dijkstra(1, 5), ([1, 2, 3, 5], 6))
        #When a searched value isn't in the graph
        with self.assertRaises(ValueError):
            self.fledged.dijkstra(1, 9)

if __name__ == '__main__':
    unittest.main(buffer=True)              #Buffer mutes print statements
    
