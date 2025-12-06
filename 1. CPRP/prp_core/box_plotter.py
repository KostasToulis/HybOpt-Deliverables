import matplotlib.pyplot as plt

#Parse Archetti Data

f = open("archetti_gap_data.csv","r")
archetti_lines = f.readlines()
f.close()

f = open("boudia_gap_data.csv","r")
boudia_lines = f.readlines()
f.close()


total_algorithms = 9
archetti_algorithm_labels = []
boudia_algorithm_labels = []

archetti_15_data = {}
archetti_50_data = {}
archetti_100_data = {}

boudia_50_data = {}
boudia_100_data = {}
boudia_200_data = {}

data = []

data.append(archetti_15_data)
data.append(archetti_50_data)
data.append(archetti_100_data)
data.append(boudia_50_data)
data.append(boudia_100_data)
data.append(boudia_200_data)

for d in data:
    d["data"] = []
    for i in range(total_algorithms):
        d["data"].append([])
    d["labels"] = []


for line_index in range(len(archetti_lines)):
    line = archetti_lines[line_index]
    sp = line.strip().split(',')
    
    #Parse Labels
    if (line_index == 0):
        for i in range(total_algorithms):
            name = sp[7 + i]
            archetti_algorithm_labels.append(name)
        #Set Our name
        archetti_algorithm_labels[total_algorithms - 1] = "Our"
        continue

    best_from_lit = float(sp[5])
    best_without_us = float(sp[6])

    best = best_from_lit

    print(sp[0], sp[1])
    for i in range(total_algorithms):
        val = sp[7 + i]
        if val == '':
            continue
        
        val = val.strip("%")
        val = float(val)

        gap = 100*(val - best) / best

        if (sp[0] == "A"):
            if (sp[1] == "100"):
                archetti_100_data["data"][i].append(gap)
            elif (sp[1] == "50"):
                archetti_50_data["data"][i].append(gap)
            elif (sp[1] == "15"):
                archetti_15_data["data"][i].append(gap)
    #print(line.strip())


for line_index in range(len(boudia_lines)):
    line = boudia_lines[line_index]
    sp = line.strip().split(',')
    print(sp)
    #Parse Labels
    if (line_index == 0):
        for i in range(total_algorithms):
            name = sp[4 + i]
            boudia_algorithm_labels.append(name)
        #Set Our name
        boudia_algorithm_labels[total_algorithms - 1] = "Our"
        continue

    if (sp[0] == ''):
        continue
    
    best_from_lit = float(sp[2])
    best_without_us = float(sp[3])

    best = best_from_lit
    
    for i in range(total_algorithms):
        val = sp[4 + i]
        if val == '':
            continue
        
        val = val.strip("%")
        val = float(val)

        gap = 100*(val - best) / best

        if (sp[0] == "50"):
            boudia_50_data["data"][i].append(gap)
        elif (sp[0] == "100"):
            boudia_100_data["data"][i].append(gap)
        elif (sp[0] == "200"):
            boudia_200_data["data"][i].append(gap)
        
    #print(line.strip())


fig, axes = plt.subplots(1,3,figsize =(15,6))
#ax.set_title("Box Plot")
print(boudia_algorithm_labels)

#Prepare archetti 100 data


fdata = []
flabels = []
notch = False
showfliers = False

for i in range(total_algorithms):
    if (archetti_100_data["data"][i]):
        fdata.append(archetti_100_data["data"][i])
        flabels.append(archetti_algorithm_labels[i])
print(fdata)

axes[2].boxplot(fdata, whis = 3.5, notch=notch, showfliers=showfliers)
axes[2].set_xticklabels(flabels, rotation = 45)
axes[2].set_title("A3")

fdata = []
flabels = []

for i in range(total_algorithms):
    if (archetti_50_data["data"][i]):
        fdata.append(archetti_50_data["data"][i])
        flabels.append(archetti_algorithm_labels[i])
print(fdata)

axes[1].boxplot(fdata, whis = 3.5, notch=notch, showfliers=showfliers)
axes[1].set_xticklabels(flabels, rotation = 45)
axes[1].set_title("A2")

fdata = []
flabels = []

for i in range(total_algorithms):
    if (archetti_15_data["data"][i]):
        fdata.append(archetti_15_data["data"][i])
        flabels.append(archetti_algorithm_labels[i])
print(fdata)

axes[0].boxplot(fdata, whis = 3.5, notch=notch, showfliers=showfliers)
axes[0].set_xticklabels(flabels, rotation = 45)
axes[0].set_title("A1")



#Labels

#axes[2].set_xlabel("Benchmarks")
axes[0].set_ylabel("OG (%)")

plt.savefig("archetti_box_plot.pdf", format="pdf", dpi=600, bbox_inches="tight")



#boudia Plot
fig, axes = plt.subplots(1,3,figsize =(15,6))
#ax.set_title("Box Plot")

fdata = []
flabels = []

for i in range(total_algorithms):
    if (boudia_200_data["data"][i]):
        fdata.append(boudia_200_data["data"][i])
        flabels.append(boudia_algorithm_labels[i])
print(fdata)

axes[2].boxplot(fdata, whis = 3.5, notch=notch, showfliers=showfliers)
axes[2].set_xticklabels(flabels, rotation = 45)
axes[2].set_title("B3")

fdata = []
flabels = []

for i in range(total_algorithms):
    if (boudia_100_data["data"][i]):
        fdata.append(boudia_100_data["data"][i])
        flabels.append(boudia_algorithm_labels[i])
print(fdata)

axes[1].boxplot(fdata, whis = 3.5, notch=notch, showfliers=showfliers)
axes[1].set_xticklabels(flabels, rotation = 45)
axes[1].set_title("B2")

fdata = []
flabels = []

for i in range(total_algorithms):
    if (boudia_50_data["data"][i]):
        fdata.append(boudia_50_data["data"][i])
        flabels.append(boudia_algorithm_labels[i])
print(fdata)

axes[0].boxplot(fdata, whis = 3.5, notch=notch, showfliers=showfliers)
axes[0].set_xticklabels(flabels, rotation = 45)
axes[0].set_title("B1")


axes[0].set_ylabel("OG (%)")
plt.savefig("boudia_box_plot.pdf", format="pdf", dpi=600, bbox_inches="tight")


plt.show()
