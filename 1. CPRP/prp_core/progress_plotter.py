import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib as mpl
import math
import numpy as np
import os

path = "raw/Bou"
path = "raw/Bou (50 runs)"
#path = "raw/Arc"
mpl.rc('text', usetex=True)
#mpl.rc('font', family='sans-serif')

iterUntilNewBestAfterMIP50 = []
iterUntilNewBestAfterMIP100 = []
iterUntilNewBestAfterMIP200 = []
which = 50

def plotAndCount(which, iterUntilNewBestAfterMIP):

    for filename in os.listdir(path):
        #if filename.endswith("TestOutput_RelSol_Bou_100_instance20_0_1_0_0_0_1_0_0_0_1_0_0_0_1_0_0_0_1_0_0.txt"):
        if "_" + str(which) + "_" in filename:
            # TestOutput_RelSol_Bou_100_instance20_0_1_0_0_0_1_0_0_0_1_0_0_0_1_0_0_0_1_0_0

            #name = "TestOutput_RelSol_Bou_200_instance9_0_1_0_0_0_1_0_0_1_0_0_0_1_0_0_0_1_0_0_0"
            # name = "TestOutput_RelSol_Bou_50_instance9_0_1_0_0_0_1_0_0_0_0_1_0_0_0_0_1_0_0_0_0"
            # name = "TestOutput_RelSol_Bou_50_instance30_0_1_0_0_0_1_0_0_0_0_1_0_0_0_0_1_0_0_0_0"
            #TestOutput_RelSol_Bou_200_instance30_0_1_0_0_0_1_0_0_1_0_0_0_1_0_0_0_1_0_0_0

            f = open(path + "/" + filename, "r")

            x = []
            y_inf = []
            y = []
            c = []
            vlines = []
            vlinesMax = []

            counter = 0
            min_ob = math.inf
            min_it = 0
            start = 0
            line_segments = []

            line_segment = None
            line_segment_state = None

            for l in f.readlines():
                # print(l.strip())
                sp = list(map(int, l.strip().split()))

                x.append(sp[0])
                y.append(sp[1])
                if (sp[1] < min_ob) and (sp[2] == 1):
                    min_ob = sp[1]
                    min_it = sp[0]
                    if (counter != 0):
                        #print(counter)
                        iterUntilNewBestAfterMIP.append(counter)
                    counter = 0
                else:
                #elif sp[2] == 1:
                    counter += 1

                if (sp[2] == 0):
                    c.append('r')
                    counter = 0 #SOS see that
                else:
                    c.append('g')

                #Init Segment
                if (line_segment is None):
                    line_segment = {"x" : [], "y" : []}
                    line_segment_state = sp[2]
                    if (line_segment_state == 0):
                        line_segment["c"] = 'r'
                    else:
                        line_segment["c"] = 'g'

                #Change Segment
                if (sp[2] != line_segment_state):
                    line_segments.append(line_segment)
                    line_segment = {"x" : [line_segment["x"][-1]], "y" : [line_segment["y"][-1]]}
                    line_segment_state = sp[2]
                    if (line_segment_state == 0):
                        line_segment["c"] = 'r'
                    else:
                        line_segment["c"] = 'g'

                #Save point to segment
                line_segment["x"].append(sp[0])
                line_segment["y"].append(sp[1])

                if (sp[3] == 1):
                    vlinesMax.append(sp[1])
                    vlines.append(sp[0])
                    counter = 0

                if (sp[3] == 1 and start == 0):
                    start = sp[0]

                # print(l.strip())
            f.close()
            x = x[start:-1]
            y = y[start:-1]
            c = c[start:-1]


            """
            #vlines = vlines[start:-1]
            #vlinesMax = vlinesMax[start:-1]
            #line_segments = line_segments[-1]
            plt.figure(figsize=(plt.figaspect(.45)))
    
            plt.xlim((start, min(x[-1],25000)))
            plt.ylim((min(y)-750, max(y)+750))
    
            for v, maxV in zip(vlines, vlinesMax):
                plt.axvline(x=v, ymax=1, linestyle='solid', color="gray",
                            linewidth=0.3)  # 'solid', 'dashed', 'dashdot', 'dotted'
    
            #plt.scatter(x, c=c, marker='.')
            #plt.scatter(y, c=c, marker='x')
            # plt.plot(x, y, color="gray", linewidth=0.8)
            
            #PLot Line Segments
            for seg in line_segments:
                plt.plot(seg["x"], seg["y"], color=seg["c"], linewidth =0.8)
    
    
            #plt.scatter(x, y, c=c, marker='.', s = 10)
            plt.plot(min_it, min_ob, c="black", marker='*', markersize=12)
            plt.axhline(min_ob, c="black", linestyle='dashed', linewidth=0.8)  # 'solid', 'dashed', 'dashdot', 'dotted'
            plt.xlabel("Iteration $i$")
            plt.ylabel("$Z(S)$")
    
            
            red_patch = mpatches.Patch(color='r', label='Infeasible')
            green_patch = mpatches.Patch(color='g', label='Feasible')
            plt.legend(handles=[green_patch,red_patch])
    
            plt.tight_layout()
            plt.savefig(filename + ".eps", format="eps")
            plt.show()
            """
            #break
        else:
            continue
    return iterUntilNewBestAfterMIP
#quit()


plotAndCount(50, iterUntilNewBestAfterMIP50)
plotAndCount(100, iterUntilNewBestAfterMIP100)
plotAndCount(200, iterUntilNewBestAfterMIP200)


# frequency histogram
npiterUntilNewBestAfterMIP50 = np.array(iterUntilNewBestAfterMIP50)
npiterUntilNewBestAfterMIP100 = np.array(iterUntilNewBestAfterMIP100)
npiterUntilNewBestAfterMIP200 = np.array(iterUntilNewBestAfterMIP200)

fig, (ax1, ax2, ax3) = plt.subplots(1, 3, sharey=False, figsize=(plt.figaspect(0.35)))
#plt.yticks(np.arange(0, 1.001, step=0.1))
#plt.xlabel("Iterations until new best")
#plt.ylabel("Cumulative frequency")
#plt.ylim(0,1)
n, bins, patches = ax1.hist(npiterUntilNewBestAfterMIP50, max(npiterUntilNewBestAfterMIP50)+1, density=True, histtype='step', cumulative=True, color="g", linewidth=1.7) #histtype='stepfilled',
n, bins, patches = ax2.hist(npiterUntilNewBestAfterMIP100, max(npiterUntilNewBestAfterMIP100)+1, density=True, histtype='step', cumulative=True, color="g", linewidth=1.7) #histtype='stepfilled',
n, bins, patches = ax3.hist(npiterUntilNewBestAfterMIP200, max(npiterUntilNewBestAfterMIP200)+1, density=True, histtype='step', cumulative=True, color="g", linewidth=1.7) #histtype='stepfilled',

ax1.set(xlabel='Iterations until new best solution', ylabel="Cumulative frequency", yticks=np.arange(0, 1.001, step=0.1), ylim=(0,1)) # title='Derivative Function of f'
ax2.set(xlabel='Iterations until new best solution', yticks=np.arange(0, 1.001, step=0.1), ylim=(0,1))
ax3.set(xlabel='Iterations until new best solution', yticks=np.arange(0, 1.001, step=0.1), ylim=(0,1))
ax1.set_title('$|V_c|=50$')
ax2.set_title('$|V_c|=100$')
ax3.set_title('$|V_c|=200$')

ax1.set_xlim(0, max(npiterUntilNewBestAfterMIP50))
ax2.set_xlim(0, max(npiterUntilNewBestAfterMIP100))
ax3.set_xlim(0, max(npiterUntilNewBestAfterMIP200))


ax1.grid(b=None, which='major', axis='y')
ax2.grid(b=None, which='major', axis='y')
ax3.grid(b=None, which='major', axis='y')

#axes[0].xlabel("$|V|=50$")
#axes[0].xlim(0, max(npiterUntilNewBestAfterMIP50))

"""
fig, axes = plt.subplots(nrows=1, ncols=3, figsize=(7, 3),sharey=True)
plt.yticks(np.arange(0, 1.001, step=0.1))
plt.xlabel("Iterations until new best")
plt.ylabel("Cumulative frequency")
plt.ylim(0,1)

n, bins, patches = axes[0].hist(npiterUntilNewBestAfterMIP50, max(npiterUntilNewBestAfterMIP50)+1, density=True, histtype='step', cumulative=True, color="g") #histtype='stepfilled',
axes[0].grid(b=None, which='major', axis='y')
#axes[0].xlabel("$|V|=50$")
#axes[0].xlim(0, max(npiterUntilNewBestAfterMIP50))

n, bins, patches = axes[1].hist(npiterUntilNewBestAfterMIP100, max(npiterUntilNewBestAfterMIP100)+1, density=True, histtype='step', cumulative=True, color="g") #histtype='stepfilled',
axes[1].grid(b=None, which='major', axis='y')
#axes[1].xlabel("$|V|=100$")
#axes[1].xlim(0, max(npiterUntilNewBestAfterMIP100))

n, bins, patches = plt.hist(npiterUntilNewBestAfterMIP200, max(npiterUntilNewBestAfterMIP200)+1, density=True, histtype='step', cumulative=True, color="g") #histtype='stepfilled',
axes[2].grid(b=None, which='major', axis='y')

"""
#axes[2].xlabel("$|V|=200$")
#axes[2].xlim(0, max(npiterUntilNewBestAfterMIP200))

#fig, (ax1, ax2, ax3) = plt.subplots(1,3, sharey=True)
#fig.suptitle('Aligning x-axis using sharex')
#plt.hist(npiterUntilNewBestAfterMIP, bins=100)


#plt.hist(bins[:-1], bins, weights=counts)
#plt.hist(npiterUntilNewBestAfterMIP[npiterUntilNewBestAfterMIP != 0], bins=np.arange(npiterUntilNewBestAfterMIP.min(), npiterUntilNewBestAfterMIP.max()+1))

#plt.hist(npiterUntilNewBestAfterMIP, bins=75)
#n, bins, patches = plt.hist(npiterUntilNewBestAfterMIP, max(npiterUntilNewBestAfterMIP)+1, density=True, histtype='step', cumulative=True, color="g") #histtype='stepfilled',

#ax1.yticks(np.arange(0, 1.001, step=0.1))

#fig.ylim(0,1)

#ax1.xlabel("$|V|=50$")
#ax2.xlabel("$|V|=100$")
#ax3.xlabel("$|V|=200$")
fig.tight_layout()
fig.savefig("histbar_triple.pdf", format="pdf",  bbox_inches='tight')
#fig.savefig("cumulative_triple.eps", format="eps", bbox_inches='tight')
fig.show()



"""
plt.yticks(np.arange(0, 1.001, step=0.1))
plt.grid(b=None, which='major', axis='y')
plt.xlim(0, max(npiterUntilNewBestAfterMIP))
plt.ylim(0,1)

plt.xlabel("Iterations before new best")
plt.ylabel("Cumulative frequency")
plt.savefig("histbar_" + str(which) + ".eps", format="eps", bbox_inches='tight')
plt.show()
"""




"""
BAR_WIDTH = 0.25
bins_number = len(set(iterUntilNewBestAfterMIP))
counts, bins = np.histogram(iterUntilNewBestAfterMIP, bins=np.arange(npiterUntilNewBestAfterMIP.min(), npiterUntilNewBestAfterMIP.max()+1))
non_zero = np.nonzero(counts)
hist = counts[non_zero]
bin_edges = bins[non_zero]
x_ticks = [str(int(edge)) for edge in bin_edges]
indices = np.arange(len(bin_edges))
plt.bar(indices, hist, BAR_WIDTH,  align='center', )
plt.xticks(indices, x_ticks)
"""

#ax.set_ylabel('Number of Motions (Total: '+ str(len(trajectoryIds)) + ')')
#ax.set_xlabel('Planning Solution (%)')
#ax.set_title('Planning Success Rate (Avg: ' + str(round(avgSuccess,2)) + '%)')

#b  = 5
#bins = (np.linspace(np.min(npiterUntilNewBestAfterMIP)**b, np.max(npiterUntilNewBestAfterMIP)**b))**(1/b)

#plt.hist(npiterUntilNewBestAfterMIP, bins=bins, edgecolor="k")

#plt.xticks(np.arange(0, max(npiterUntilNewBestAfterMIP))) #step=50