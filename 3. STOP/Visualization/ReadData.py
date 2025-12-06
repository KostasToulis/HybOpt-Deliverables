from Classes import Node, Set, Model, Solution

def read_solution(sol_file_path):
    # read the reported solution
    with open(sol_file_path, 'r') as file:
        lines = file.readlines()
    file.close()
    dataset = take_value_of_line(lines[0])
    vehicles = int(take_value_of_line(lines[1]))
    routes = []
    for i in range(3, 3 + vehicles):
        route = take_value_of_line(lines[i]).split(",")

        # capture distance
        last_element = route[-1]
        distance = last_element.split(' ')[-1].strip('()')
        # Remove the last element from the list
        route[-1] = '0'
        routes.append([int(node_id) for node_id in route])
    profit = int(take_value_of_line(lines[i + 1]))
    return Solution(routes, profit)

def read_model(dataset_name):
    datasets_place = "../MSOP/MSOP_datasets/all/"  # where datasets are located
    # according to the dataset_name reported, find the corresponding model dataset and construct the problem
    allNodes = []
    allSets = []
    working_on_nodes = False
    working_on_sets = False
    dataset_path = datasets_place + dataset_name + '.msop'
    with open(dataset_path, 'r') as f:
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
    return Model(dataset_name, node_crowd, set_crowd, allNodes, allSets, dataset_path)


def take_value_of_line(line):
    return line.strip().split(":")[1].strip()