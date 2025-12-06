import random
import ReadData
from Classes import Node, Set, Model
from matplotlib import pyplot as plt
import os

# Folder to read from
# extracted_solutions_folder = "../MSOP/extracted_solutions/mls/"
# extracted_solutions_folder = "Experiment mls_unit_clusters/normal/"
extracted_solutions_folder = "../MSOP/extracted_solutions/Experiment mls_unit_clusters/"

# Set parameters
random.seed(58008) #1 # 58008 #5318008

dataset_name = ""
#dataset_name = "30ch150_T100_p2_v4"
#dataset_name = "80rd400_T60_p2_v4"
#dataset_name = "39rat195_T100_p2_v2"
#dataset_name = "11berlin52_T100_p2_v2"
#dataset_name = "80rd400_T60_p2_v4"
#dataset_name = "132d657_T60_p1_v4"


def plot_solutions(dataset_name):
    for type in ["normal", "unit", "vrp"]:
        if type == "unit" or type == "vrp":
            continue

        # paths & vars
        extracted_solutions_folder_cur = extracted_solutions_folder + type + "/"
        sol_type = ""
        factor_scale_profits = 6

        if type == "unit" or type == "normal":
            sol_type = "final"
        elif type == "vrp":
            sol_type = "vrp"

        solution_file = dataset_name + "_" + sol_type + ".txt"
        solution_file_path = extracted_solutions_folder_cur + solution_file

        # load the solution
        sol = ReadData.read_solution(solution_file_path)

        # load the instance
        m = ReadData.read_model(dataset_name)
        included_sets = [m.allSets[m.allNodes[i].set_id] for i in sol.included_nodes]
        print("#Nodes: ", m.node_crowd, "#Sets: ", m.set_crowd, "#Visited sets: ", len(included_sets))
        max_x = max(node.x for node in m.allNodes)
        max_y = max(node.y for node in m.allNodes)

        # plot the depot
        plt.scatter(m.allNodes[0].x, m.allNodes[0].y, color="black", marker="s")
        margin = 0.05 * max(max_x, max_y)
        plt.xlim(0 - margin, max_x + margin)
        plt.ylim(0 - margin, max_y + margin)

        # plot the nodes (different color for each set, X (unvisited) or O (visited), marker size depending on the profit)
        for set in m.allSets[1:]:
            if type == "unit":
                for n in set.nodes:
                    color = (random.random(), random.random(), random.random())
                    x_points = n.x
                    y_points = n.y
                    if n.ID in sol.included_nodes:
                        plt.scatter(x_points, y_points, color=color, s=set.profit/(factor_scale_profits *len(set.nodes)))
                    else:
                        plt.scatter(x_points, y_points, marker='x', color=color, s=set.profit/(factor_scale_profits *len(set.nodes)))
            elif type == "normal":
                color = (random.random(), random.random(), random.random())
                x_points = [n.x for n in set.nodes]
                y_points = [n.y for n in set.nodes]
                if set in included_sets:
                    plt.scatter(x_points, y_points, color=color, s=set.profit/factor_scale_profits )
                else:
                    plt.scatter(x_points, y_points, marker='x', color=color, s=set.profit/factor_scale_profits )
            elif type == "vrp":
                if set in included_sets:
                    for n in set.nodes:
                        color = (random.random(), random.random(), random.random())
                        x_points = n.x
                        y_points = n.y
                        if n.ID in sol.included_nodes:
                            plt.scatter(x_points, y_points, color=color, s=15)
                        else:
                            plt.scatter(x_points, y_points, marker='x', color=color, s=15)


        # plot the routes
        for route in sol.routes:
            color = (random.random(), random.random(), random.random())
            path_x = [m.allNodes[node].x for node in route]
            path_y = [m.allNodes[node].y for node in route]
            plt.plot(path_x, path_y, linewidth=1.5, color=color)


        plt.savefig(dataset_name + "_" + type + "_solution.pdf", format="pdf", dpi=300)
        #plt.savefig(dataset_name + "_" + sol_type + "_solution.svg")

        plt.close()

# execution
for filename in os.listdir(extracted_solutions_folder + "normal/"):
    filename = filename[:-10]
    plot_solutions(filename)
