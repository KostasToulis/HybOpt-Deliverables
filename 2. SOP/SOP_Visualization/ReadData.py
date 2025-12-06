from Classes import Node, Set, Model


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
