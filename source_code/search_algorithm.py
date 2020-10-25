import pygame
import graphUI
from node_color import white, yellow, black, red, blue, purple, orange, green
import time
import math


"""
Feel free print graph, edges to console to get more understand input.
Do not change input parameters
Create new function/file if necessary
"""


class Node:
    def __init__(self, cost=0, h_heuristic=0, preNode=None):
        self.cost = cost
        self.h_heuristic = h_heuristic
        self.preNode = preNode


def BFS(graph, edges, edge_id, start, goal):
    """
    BFS search
    """
    # TODO: your code
    print("Implement BFS algorithm.")

    graph[start][3] = red
    graphUI.updateUI()

    openList = [start]
    closeList = []
    pathArr = {}

    while True:
        newOpenList = []
        for i in openList:
            if i == goal:
                graph[i][3] = purple
                graphUI.updateUI()
                des = goal

                while des != start:
                    pre = pathArr[des]
                    edges[edge_id(pre, des)][1] = green
                    des = pre
                    graphUI.updateUI()

                graph[start][3] = orange
                graphUI.updateUI()
                return

            else:
                graph[i][3] = yellow
                graphUI.updateUI()
            closeList.append(i)

            for j in graph[i][1]:
                if j not in closeList + openList + newOpenList:
                    newOpenList.append(j)
                    pathArr[j] = i
                    graph[j][3] = red
                    edges[edge_id(i, j)][1] = white

            graph[i][3] = blue
            graphUI.updateUI()
        time.sleep(1)
        openList = newOpenList

    pass


def DFS(graph, edges, edge_id, start, goal):
    """    DFS search
    """
    # TODO: your code
    print("Implement DFS algorithm.")

    graph[start][3] = red
    graphUI.updateUI()

    if start == goal:
        graph[goal][3] = purple
        graphUI.updateUI()
        return True

    graph[start][3] = blue
    graphUI.updateUI()
    openList = graph[start][1]

    for i in openList:
        if graph[i][3] == blue:
            continue
        graph[i][3] = red
    time.sleep(0.3)
    graphUI.updateUI()

    for i in openList:
        if graph[i][3] == blue:
            continue
        graph[i][3] = yellow
        time.sleep(0.3)
        graphUI.updateUI()
        des = DFS_recursion(graph, edges, edge_id, i, goal)
        if des:
            edges[edge_id(start, i)][1] = green
            graph[start][3] = orange
            graphUI.updateUI()
            return True

    pass


def DFS_recursion(graph, edges, edge_id, start, goal):
    if start == goal:
        graph[goal][3] = purple
        graphUI.updateUI()
        return True

    graph[start][3] = yellow
    graphUI.updateUI()
    time.sleep(0.3)

    openList = graph[start][1]
    for i in openList:
        if graph[i][3] == blue:
            continue
        graph[i][3] = red
    graphUI.updateUI()
    time.sleep(0.3)

    graph[start][3] = blue
    graphUI.updateUI()
    time.sleep(0.6)

    for i in openList:
        if graph[i][3] == blue:
            continue
        des = DFS_recursion(graph, edges, edge_id, i, goal)
        if des:
            edges[edge_id(start, i)][1] = green
            time.sleep(0.2)
            graphUI.updateUI()
            return True

    pass


def UCS(graph, edges, edge_id, start, goal):
    """
    Uniform Cost Search search
    """
    # TODO: your code
    print("Implement Uniform Cost Search algorithm.")

    graph[start][3] = red
    graphUI.updateUI()
    time.sleep(0.3)

    node = Node(cost=0)
    openList = {start: node}
    closeList = {}

    graph[start][3] = yellow
    graphUI.updateUI()

    while True:

        i = FindMinimizeCost(openList)

        if i == goal:
            graph[i][3] = purple
            graphUI.updateUI()
            des = goal
            closeList[des] = openList[des]
            while des != start:
                pre = closeList[des].preNode
                edges[edge_id(pre, des)][1] = green
                des = pre
                graphUI.updateUI()
            graph[start][3] = orange
            graphUI.updateUI()
            return

        graph[i][3] = yellow
        graphUI.updateUI()
        time.sleep(0.3)

        for j in graph[i][1]:
            if j not in closeList.keys():
                cost = math.sqrt(
                    (graph[i][0][0]-graph[j][0][0])**2+(graph[i][0][1]-graph[j][0][1])**2) + openList[i].cost
                if j not in openList.keys():
                    openList[j] = Node(cost=cost, preNode=i)
                    graph[j][3] = red
                    edges[edge_id(i, j)][1] = white
                else:
                    if cost < openList[j].cost:
                        edges[edge_id(openList[j].preNode, j)][1] = black
                        openList[j] = Node(cost=cost, preNode=i)
                        graph[j][3] = red
                        edges[edge_id(i, j)][1] = white


        graph[i][3] = blue
        closeList[i] = openList[i]
        openList.pop(i)
        graphUI.updateUI()
        time.sleep(1)

    pass


def FindMinimizeCost(openList, heuristicArr = None):
    minNodeID = list(openList.keys())[0]
    for nodeID, node in openList.items():
        if heuristicArr:
            if node.cost + heuristicArr[nodeID] < openList[minNodeID].cost + heuristicArr[minNodeID]:
                minNodeID = nodeID
        else:
            if node.cost < openList[minNodeID].cost:
                minNodeID = nodeID
    return minNodeID
    pass

def HeuristicCalculate(graph, goal):
    heuristicArr = {}
    for i, node in enumerate(graph):
        heuristicArr[i] = math.sqrt((graph[i][0][0]-graph[goal][0][0])**2+(graph[i][0][1]-graph[goal][0][1])**2)
    return heuristicArr
    pass


def AStar(graph, edges, edge_id, start, goal):
    """
    A star search
    """
    # TODO: your code
    print("Implement A* algorithm.")


    graph[start][3] = red
    graphUI.updateUI()
    time.sleep(0.3)

    node = Node(cost=0)
    openList = {start: node}
    closeList = {}
    heuristicArr = HeuristicCalculate(graph,goal)

    graph[start][3] = yellow
    graphUI.updateUI()

    while True:

        i = FindMinimizeCost(openList, heuristicArr)

        if i == goal:
            graph[i][3] = purple
            graphUI.updateUI()
            des = goal
            closeList[des] = openList[des]
            while des != start:
                pre = closeList[des].preNode
                edges[edge_id(pre, des)][1] = green
                des = pre
                graphUI.updateUI()
            graph[start][3] = orange
            graphUI.updateUI()
            return

        graph[i][3] = yellow
        graphUI.updateUI()
        time.sleep(0.3)

        for j in graph[i][1]:
            if j not in closeList.keys():
                cost = math.sqrt(
                    (graph[i][0][0]-graph[j][0][0])**2+(graph[i][0][1]-graph[j][0][1])**2) + openList[i].cost
                if j not in openList.keys():
                    openList[j] = Node(cost=cost, preNode=i)
                    graph[j][3] = red
                    edges[edge_id(i, j)][1] = white
                else:
                    if cost < openList[j].cost:
                        edges[edge_id(openList[j].preNode, j)][1] = black
                        openList[j] = Node(cost=cost, preNode=i)
                        graph[j][3] = red
                        edges[edge_id(i, j)][1] = white


        graph[i][3] = blue
        closeList[i] = openList[i]
        openList.pop(i)
        graphUI.updateUI()
        time.sleep(1)


    pass


def example_func(graph, edges, edge_id, start, goal):
    """
    This function is just show some basic feature that you can use your project.
    @param graph: list - contain information of graph (same value as global_graph)
                    list of object:
                     [0] : (x,y) coordinate in UI
                     [1] : adjacent node indexes
                     [2] : node edge color
                     [3] : node fill color
                Ex: graph = [
                                [
                                    (139, 140),             # position of node when draw on UI
                                    [1, 2],                 # list of adjacent node
                                    (100, 100, 100),        # grey - node edged color
                                    (0, 0, 0)               # black - node fill color
                                ],
                                [(312, 224), [0, 4, 2, 3], (100, 100, 100), (0, 0, 0)],
                                ...
                            ]
                It means this graph has Node 0 links to Node 1 and Node 2.
                Node 1 links to Node 0,2,3 and 4.
    @param edges: dict - dictionary of edge_id: [(n1,n2), color]. Ex: edges[edge_id(0,1)] = [(0,1), (0,0,0)] : set color
                    of edge from Node 0 to Node 1 is black.
    @param edge_id: id of each edge between two nodes. Ex: edge_id(0, 1) : id edge of two Node 0 and Node 1
    @param start: int - start vertices/node
    @param goal: int - vertices/node to search
    @return:
    """

    # Ex1: Set all edge from Node 1 to Adjacency node of Node 1 is green edges.
    node_1 = graph[1]
    for adjacency_node in node_1[1]:
        edges[edge_id(1, adjacency_node)][1] = green
    graphUI.updateUI()

    # Ex2: Set color of Node 2 is Red
    graph[2][3] = red
    graphUI.updateUI()

    # Ex3: Set all edge between node in a array.
    path = [4, 7, 9]  # -> set edge from 4-7, 7-9 is blue
    for i in range(len(path) - 1):
        edges[edge_id(path[i], path[i + 1])][1] = blue
    graphUI.updateUI()
