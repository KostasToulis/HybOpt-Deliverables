import random
import ReadData
from Classes import Node, Set, Model
from matplotlib import pyplot as plt

# Folder to read from
# extracted_solutions_folder = "../MSOP/extracted_solutions/mls/"
# extracted_solutions_folder = "Experiment mls_unit_clusters/normal/"
extracted_solutions_folder = "Experiment mls_unit_clusters/"

# Set parameters
random.seed(1)
#dataset_name = "30ch150_T100_p2_v4"
#dataset_name = "80rd400_T60_p2_v4"
#dataset_name = "39rat195_T100_p2_v2"
dataset_name = "132d657_T60_p1_v4"


for type in ["unit", "normal"]:
    extracted_solutions_folder_cur = extracted_solutions_folder + type + "/"

    sol_type = "final"
    solution_file = dataset_name + "_" + sol_type + ".txt"
    solution_file_path = extracted_solutions_folder_cur + solution_file



    sol = ReadData.read_solution(solution_file_path)
    m = ReadData.read_model(dataset_name)
    included_sets = [m.allSets[m.allNodes[i].set_id] for i in sol.included_nodes]
    print(m.node_crowd, m.set_crowd, len(included_sets))

    plt.scatter(m.allNodes[0].x, m.allNodes[0].y, color="black", marker="s")
    for set in m.allSets[1:]:
        if type == "unit":
            for n in set.nodes:
                color = (random.random(), random.random(), random.random())
                x_points = n.x
                y_points = n.y
                if n.ID in sol.included_nodes:
                    plt.scatter(x_points, y_points, color=color, s=set.profit/(6*len(set.nodes)))
                else:
                    plt.scatter(x_points, y_points, marker='x', color=color, s=set.profit/(6*len(set.nodes)))
        elif type == "normal":
            color = (random.random(), random.random(), random.random())
            x_points = [n.x for n in set.nodes]
            y_points = [n.y for n in set.nodes]
            if set in included_sets:
                plt.scatter(x_points, y_points, color=color, s=set.profit/6)
            else:
                plt.scatter(x_points, y_points, marker='x', color=color, s=set.profit/6)
    for route in sol.routes:
        color = (random.random(), random.random(), random.random())
        path_x = [m.allNodes[node].x for node in route]
        path_y = [m.allNodes[node].y for node in route]
        plt.plot(path_x, path_y, linewidth=2.5, color=color)

    plt.savefig(dataset_name + "_" + type + "_solution.png", dpi=300)
    #plt.savefig(dataset_name + "_" + sol_type + "_solution.svg")

    plt.close()