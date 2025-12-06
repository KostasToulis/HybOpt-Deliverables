from Classes import Node, Set, Model, TSPModel, SOPModel
from sklearn.cluster import KMeans
import numpy as np
import random
import re


def importModel(filename):
    # open solution report file
    project_path = "../"  # where project is located
    datasets_place = "Datasets/sop/"  # where datasets are located
    sol_report_place = ""  # where sol_report files are located

    # read the reported solution
    with open(project_path + sol_report_place + filename, 'r') as file:
        for line in file:
            elements = line.strip().split(":")
            if elements[0] == "Dataset_name":
                dataset = elements[1]
            elif elements[0] == "Sol_path":
                nodes = elements[1].strip().split(",")
                path = [int(i) for i in nodes]
            elif elements[0] == "Sol_profit":
                profit = elements[1].strip()
    file.close()

    # according to the dataset_name reported, find the corresponding model dataset and construct the problem
    allNodes = []
    allSets = []
    working_on_nodes = False
    working_on_sets = False
    with open(project_path + datasets_place + dataset + ".sop", 'r') as f:
        for line in f:
            line = line.replace("\n", "")
            elements = line.split(":")
            if elements[0] == "DIMENSION":
                node_crowd = int(elements[1].strip())
            elif elements[0] == "SETS":
                set_crowd = int(elements[1].strip())
            elif elements[0] == "NODE_COORD_SECTION":
                working_on_nodes = True
            elif elements[0] == "GTSP_SET_SECTION":
                working_on_nodes = False
                working_on_sets = True
            elif working_on_nodes:
                node_obj = elements[0]
                node_elements = node_obj.strip().split()
                allNodes.append(Node(int(node_elements[0]), float(node_elements[1]), float(node_elements[2])))
            elif working_on_sets:
                set_obj = elements[0]
                set_elements = set_obj.strip().split()
                nodes_in_set = [allNodes[int(ID) - 1] for ID in set_elements[2:]]
                for n in nodes_in_set:
                    n.set_id = int(set_elements[0])
                allSets.append(Set(int(set_elements[0]), int(set_elements[1]), nodes_in_set))
        for n in allNodes:
            n.ID -= 1
    return Model(dataset, node_crowd, set_crowd, allNodes, allSets, path)

def importTSPModel(filename):
    # open solution report file
    datasets_place = "ALL_tsp/"  # where datasets are located
    sol_report_place = ""  # where sol_report files are located

    # according to the dataset_name reported, find the corresponding model dataset and construct the problem
    allNodes = []
    allSets = []
    working_on_nodes = False
    working_on_sets = False
    with open(datasets_place + filename, 'r') as f:
        for line in f:
            line = line.replace("\n", "")
            #line = line.replace("\t", " ")
            #' '.join(line.split())
            line = re.sub('\s+', ' ', line)
            line = line.strip()
            #line = line.replace('', )

            elements = line.split(" ")
            if elements[0] == "DIMENSION":
                node_crowd = int(elements[2].strip())
            elif elements[0] == "SETS":
                set_crowd = int(elements[2].strip())
            elif elements[0] == "NODE_COORD_SECTION":
                working_on_nodes = True
            elif elements[0] == "GTSP_SET_SECTION":
                working_on_nodes = False
                working_on_sets = True
            elif working_on_nodes:
                node_obj = elements[0]
                if node_obj != "EOF":
                    allNodes.append(Node(int(elements[0]), float(elements[1]), float(elements[2])))
            elif working_on_sets:
                set_obj = elements[0]
                set_elements = set_obj.strip().split()
                nodes_in_set = [allNodes[int(ID) - 1] for ID in set_elements[2:]]
                for n in nodes_in_set:
                    n.set_id = int(set_elements[0])
                allSets.append(Set(int(set_elements[0]), int(set_elements[1]), nodes_in_set))
        #for n in allNodes:
        #    n.ID -= 1
    return TSPModel(filename, node_crowd, allNodes)

def generateSOPModel(filename, m, T, category, profitType, t_percentage):

    setNum = round(len(m.allNodes)/5) #calculate sets number
    name = ""
    plainFilename =  filename[0:-4]
    allSets = []
    Tmax = 0

    if category == "":
        name = str(setNum) + plainFilename + "_" + t_percentage + "_" + profitType + ".sop"
    elif category == "RND":
        name = str(setNum) + plainFilename + "_" + category + "_" + t_percentage + "_" + profitType + ".sop"

    print(name)

    if category == "":
        coords = []
        for node in m.allNodes:
            if node.ID == 1:
                pass
            else:
                coords.append([node.x, node.y])
        X = np.array(coords)

        kmeans = KMeans(init="random", n_clusters=setNum, n_init = 20, random_state = 0)
        kmeans.fit(X)
        clusters = kmeans.labels_
        #print(clusters)

        for node in m.allNodes:
            if node.ID == 1:
                node.set_id = 0
            else:
                node.set_id = clusters[node.ID-2] + 1
    elif category == "RND":
        # ensure no empty clusters
        for node in m.allNodes:
            node.set_id = -1

        for i in range(0, setNum+1):
            cont = True
            while cont:
                idx = random.randint(1,len(m.allNodes)-1)
                nodeCur = m.allNodes[idx]
                if nodeCur.set_id == -1:
                    nodeCur.set_id = i
                    cont = False

        for node in m.allNodes:
            if (node.set_id == -1):
                if node.ID == 1:
                    node.set_id = 0
                else:
                    node.set_id = random.randint(1, setNum)

    for n in range(0, setNum + 1):
        profit = 0
        nodes_in_set = []
        for node in m.allNodes:
            if node.set_id == n:
                profit += 1
                nodes_in_set.append(node)
        if n == 0:
            profit = 0
        allSets.append(Set(n, profit, nodes_in_set))

    # overwrite profit
    if profitType == "p2":
        for set in allSets:
            if set.ID != 0:
                set.profit = 0
                for node in set.nodes:
                    nodeprofit = 1 + (7141*node.ID + 73) % 100
                    set.profit += nodeprofit


    if t_percentage == "T40":
        Tmax = round(0.4*T)
    elif t_percentage == "T60":
        Tmax = round(0.6*T)
    elif t_percentage == "T80":
        Tmax = round(0.8*T)

    return SOPModel(filename, name, m, m.node_crowd, setNum, m. allNodes, allSets, Tmax, category, profit)


def writeSOPinTxt(m, contents):
    file = open("sop/"+m.name, "w")

    file.write("NAME: " + m.name[0:-4] + "\n")

    line = contents[1].replace("\n", "")
    elements = line.split(":")
    file.write("COMMENT: " + elements[-1] +  " SOP::[From Dontas, Sideris, Manousakis, Zachariadis (2021)]" + "\n")

    file.write("TYPE: SOP\n")

    file.write("DIMENSION: " + str(m.node_crowd) + "\n")

    file.write("TMAX: " + str(m.Tmax) + "\n")

    file.write("START_SET: 1\n")

    file.write("END_SET: 1\n")

    file.write("SETS: " + str(m.setNum) + "\n")

    file.write("EDGE_WEIGHT_TYPE: CEIL_2D\n")

    file.write("NODE_COORD_SECTION\n")

    for node in m.allNodes:
        file.write(str(node.ID) + " " + str(node.x) + " " + str(node.y) + "\n")

    file.write("SOP_SET_SECTION: set_id set_profit id-vertex-list\n")
    for set in m.allSets:
        nodesList = ""
        for node in set.nodes:
            nodesList += str(node.ID) + " "
        nodesList = nodesList[0:-1]
        file.write(str(set.ID) + " " + str(set.profit) + " " + nodesList + "\n")

    file.close()