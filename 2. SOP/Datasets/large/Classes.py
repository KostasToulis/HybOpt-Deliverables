class Node:
    def __init__(self, id, x, y):
        self.ID = id
        self.x = x
        self.y = y
        self.set_id = -1


class Set:
    def __init__(self, id, profit, nodes):
        self.ID = id
        self.profit = profit
        self.nodes = nodes


class Model:
    def __init__(self, dataset, node_crowd, set_crowd, allNodes, allSets, path):
        self.dataset = dataset
        self.node_crowd = node_crowd
        self.set_crowd = set_crowd
        self.allNodes = allNodes
        self.allSets = allSets
        self.path = path

class SOPModel:
    def __init__(self, filename, name, m, node_crowd, setNum, allNodes, allSets, Tmax, category, profit):
        self.filename = filename
        self.name = name
        self.m = m
        self.node_crowd = node_crowd
        self.setNum = setNum
        self.allNodes = allNodes
        self.allSets = allSets
        self.Tmax = Tmax
        self.category = category
        self.profit = profit


class TSPModel:
    def __init__(self, dataset, node_crowd, allNodes):
        self.dataset = dataset
        self.node_crowd = node_crowd
        self.allNodes = allNodes
