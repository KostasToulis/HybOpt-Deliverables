import random
import ReadData
from Classes import Node, Set, Model
from matplotlib import pyplot as plt

dataset_name = "11berlin52_T100_p2_v2"

m = ReadData.read_model(dataset_name)

print(m.node_crowd, m.set_crowd)

random.seed(1)
plt.scatter(m.allNodes[0].x, m.allNodes[0].y, color="black", marker="s")
for set in m.allSets[1:]:
    color = (random.random(), random.random(), random.random())
    x_points = [n.x for n in set.nodes]
    y_points = [n.y for n in set.nodes]
    plt.scatter(x_points, y_points, color='white')
    #plt.scatter(x_points, y_points, color=color, s=set.profit/6)
plt.savefig(dataset_name + "_" + "dataset" + "_vis.png", dpi=300)
plt.close()