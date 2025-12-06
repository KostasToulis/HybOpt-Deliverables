from matplotlib import pyplot as plt

profits = []

# read the reported solutions
with open("../Sol_Evolution.txt", 'r') as file:
    for line in file:
        profits.append(int(line.strip().split("\n")[0]))
file.close()

plt.plot([i for i in range(len(profits))], profits, '.-', linewidth=1)
plt.savefig("Sol_evolution_217vm1084_T40_p1.png")
plt.close()
