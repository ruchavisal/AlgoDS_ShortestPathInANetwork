"""
Name: Rucha Pramod Visal
Student ID: 801257824
"""

import os
import numpy 

#Edge class is used to represent edges of the graph
class EdgeWeight:
    def __init__(self,name,weight):
        self.name = name
        self.weight = weight

    def EdgeWeight(self,name,weight):
        self.name = name
        self.weight = weight

#Vertex class is used to represent vertex of the graph
class Vertex:
    def __init__(self,name):
        self.name = name
        self.adjacentNodes = []
        self.status ='Up'
        self.resetVertex()

    def Vertex(self,name):
        self.name = name
        self.adjacentNodes = []
        self.resetVertex()

    def resetVertex(self):
        self.previous = None
        self.distanceFromSource = numpy.inf        

class PriorityQueue:
    def __init__(self):
        self.queue = []
    
    def insert(self,value):
        self.queue.append(value)

    def deleteMin(self):
        minIndex = 0
        for i in range(len(self.queue)):
            #get minimum index of max element
            if self.queue[i].distanceFromSource<= self.queue[minIndex].distanceFromSource:
                minIndex=i

        item = self.queue[minIndex]
        del self.queue[minIndex]
        return item

    def length(self):
        return len(self.queue)

    def findElement(self, vertex):
        if vertex in self.queue:
            i = 0
            while i< self.length():
                if self.queue[i].name == vertex.name:
                    return self.queue[i]
                i+=1
        else:
            return None
    
    def replaceElementWeight(self,vertex):
        i = 0
        while i< self.length():
                if self.queue[i].name == vertex.name:
                    self.queue[i] = vertex
                    return
                i+=1

#Build the network Graph and perform operations on the network
class Graph:
    def __init__(self):
        self.vertexMap = dict()
        self.edgeWeights = dict()
        self.edgeStatus = dict()

    #construct inital graph
    def addDoubleEdges(self,source, destination,weight):

        #set edges and their weights in the vertexWeights
        self.edgeWeights[source+"_"+destination]= weight
        self.edgeWeights[destination+"_"+source]= weight

        #set up/down status - initially all are up
        self.edgeStatus[source+"_"+destination]= 'Up'
        self.edgeStatus[destination+"_"+source]= 'Up'
        u = self.getVertexData(source)
        v= self.getVertexData(destination)
        u.adjacentNodes.append(v)
        v.adjacentNodes.append(u)
       
    #make edge offline
    def alterEdgeStatus(self,source, destination, status):
        self.edgeStatus[source+"_"+destination] = status

    #bring vertex up
    def alterVertexStatus(self, vertex, status):
        self.vertexMap[vertex].status = status
        print("Vertex Status Changed!")

    #add single directed edge from source to vertex as specifid by user
    def addSingleEdge(self, source, destination, weight):
        self.edgeWeights[source+"_"+destination] = weight
        u = self.getVertexData(source)
        v = self.getVertexData(destination)
        u.adjacentNodes.append(v)

    #delete single edge from the graph
    def deleteSingleEdge(self, source, destination):
        del self.edgeWeights[source+"_"+destination]

    #adding vertex to the list of available vertices
    def getVertexData(self,vertex):
        if vertex not in self.vertexMap:
            v = Vertex(vertex)
            self.vertexMap[vertex] = v
        v = self.vertexMap[vertex]
        return v

    #clear all the vertices datamap 
    def resetVertices(self):
        for vertex in self.vertexMap.values():
            vertex.resetVertex()

    #calculate the shortest path
    def getshortestPath(self, sourceVertex):
        self.resetVertices()
        source = self.getVertexData(sourceVertex)
        if source is None :
            print("Source Vertex not found!")
        else :
            #implement queue here
            knownQueue = []
            q = PriorityQueue()
            source.distanceFromSource = 0.0
            source.previous = None
            q.insert(source)

            while q.length()>0:
                vertex = q.deleteMin()
                for adjacentNode in vertex.adjacentNodes:
                        if adjacentNode not in knownQueue:
                            if self.edgeWeights[vertex.name+"_"+adjacentNode.name]:
                                nodeDistance = float(self.edgeWeights[vertex.name+"_"+adjacentNode.name])
                            else: nodeDistance = 0.0
                            distanceFromSource = vertex.distanceFromSource + nodeDistance
                            
                            #replace or add edge if and only if the edge is UP and adjacent vertex is up
                            if self.edgeStatus[vertex.name+"_"+adjacentNode.name] == 'Up' and adjacentNode.status == 'Up':
                                node = q.findElement(adjacentNode)
                                if node == None or adjacentNode.distanceFromSource is numpy.inf:
                                    adjacentNode.previous = vertex
                                    adjacentNode.distanceFromSource = distanceFromSource
                                    q.insert(adjacentNode) 
                                elif distanceFromSource< adjacentNode.distanceFromSource:
                                    adjacentNode.previous = vertex
                                    adjacentNode.distanceFromSource = distanceFromSource
                                    q.replaceElementWeight(adjacentNode)

                #whatever nodes are processed, added to is_known queue
                knownQueue.append(vertex)

    #print graph
    def printGraphPath(self, destinationVertex):
        destination = self.getVertexData(destinationVertex)
        if destination is None:
           print("Destination Node not found!")
        elif destination.distanceFromSource is numpy.inf:
            print("Destination not reched!")
        else:
            s = []
            tempVertex = destination
            s.append(str(round(tempVertex.distanceFromSource,2)))
            
            #adding vertices to stack
            while tempVertex!= None:
                s.append(tempVertex.name)
                tempVertex = tempVertex.previous

            #printing the sequence
            i = len(s)-1
            while i>-1:
                print(s[i], end= "  ")
                i -=1

            #adding new line at end
            print()

    #get vertices names in sorted order
    def sortVertexMap(self):
        keys=[]
        for key in sorted (self.vertexMap.keys()):
            keys.append(key)
        return keys

    #get all outgoing edges from a vertex
    def getOutwardEdges(self,key):
        allEdges = self.edgeWeights.keys()
        keyOutwardEdges = []
        for edge in sorted(allEdges):
            edgearray = edge.split("_")
            if(edgearray[0] == key):
                keyOutwardEdges.append(EdgeWeight(edgearray[1], self.edgeWeights[edge]))
        return keyOutwardEdges

    #print all the outgoing edges of the vertex
    def printGraphOutwardEdges(self):
        keys = self.sortVertexMap()
        for key in keys:
            print(key+ ":")
            edges= self.getOutwardEdges(key)
            if len(edges) != 0:
                for edge in edges:
                    print(edge.name + "  "+str(edge.weight))
            else:
                print("No outward going edges!")
            print() 
            print()

    #get all outgoing edges from a vertex
    def getReachableEdges(self,key):
        allEdges = self.edgeWeights.keys()
        reachableEdges = []
        for edge in sorted(allEdges):
            edgearray = edge.split("_")
            if(edgearray[0] == key and self.edgeStatus[edgearray[0]+"_"+edgearray[1]] == 'Up' and self.vertexMap[edgearray[1]].status=='Up'):
                reachableEdges.append(edgearray[1])
        return reachableEdges

    #print all the reachable edges
    def printReachableEdges(self):
        keys = self.sortVertexMap()
        for key in keys:
            if self.vertexMap[key].status=='Up':
                print(key+ ":")
                edges= self.getReachableEdges(key)
                if len(edges) != 0:
                    for edge in edges:
                        print(edge)
                    print()
                else:
                    print("No reachable edges!")
            

#read file network.txt and read edges to build a graph
def initGraphData(fileName):
    #initialize graph
    graph = Graph()
    #define path
    path = os.getcwd()+"/"+fileName
    #read network file
    with open(path) as f:
        lines = f.readlines()
        for line in lines:
            newArray = line.strip().split(" ")
            #set source and destination
            source = newArray[0]
            destination = newArray[1]
            weight = float(newArray[2])
            #construct graph
            graph.addDoubleEdges(source,destination,weight)
    print("Creating Graph!")
    return graph


# A main routine that:
# 1. Reads a file containing edges (supplied as a command-line parameter);
# 2. Builds the graph;
# 3. Repeatedly prompts for queries (stops at "quit" query)
def main():
    graph = initGraphData("network.txt")
    print("Number of vertices:",len(graph.vertexMap))
    try:
        while True:
            inputCommand = input("Input Command:")
            commandsArray = inputCommand.split(" ")
            command = commandsArray[0]
           
            if command == "path":
                source = commandsArray[1]
                destination = commandsArray[2]
                print("Finding Shortest path from "+source+" to "+destination+"!")
                graph.getshortestPath(source)
                graph.printGraphPath(destination)

            elif command == "print":
                print("Direct outward edge!")
                graph.printGraphOutwardEdges()

            elif command == "addedge":
                tailVertex = commandsArray[1]
                headVertex = commandsArray[2]
                transmissionTime = commandsArray[3]
                graph.addSingleEdge(tailVertex,headVertex,float(transmissionTime))
                graph.printGraphOutwardEdges()


            elif command == "deleteedge":
                tailVertex = commandsArray[1]
                headVertex = commandsArray[2]
                graph.deleteSingleEdge(tailVertex,headVertex)
                graph.printGraphOutwardEdges()

            elif command == "edgedown" or command == "edgeup":
                tailVertex = commandsArray[1]
                headVertex = commandsArray[2]
                status=""
                if command == "edgedown":
                    status = 'Down'
                else:
                    status = 'Up'
                graph.alterEdgeStatus(tailVertex,headVertex,status)

            elif command == "vertexdown" or command == "vertexup":
                vertex = commandsArray[1]
                status =""
                if command == "vertexdown":
                    status = 'Down'
                else:
                    status = 'Up'
                graph.alterVertexStatus(vertex, status)

            elif command == "reachable":
                print("current Reachables!")
                graph.printReachableEdges()
                graph

            if command == "quit":
                exit()

    except Exception as exception:
        print("Exception occurred!",exception)

if __name__=="__main__":
    main()
