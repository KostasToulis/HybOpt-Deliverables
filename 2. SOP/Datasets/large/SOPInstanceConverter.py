import random, ReadData
import Classes
from matplotlib import pyplot as plt
import os
from VisualizeSolution import visualizeTSP, visualizeSOP
from ReadData import writeSOPinTxt

def readDatasetT(filename):
   T = 1000000
   return T


path = os.getcwd()
path = "ALL_tsp"
for filename in os.listdir("ALL_tsp"):
   print(filename)
   if filename.endswith('.tsp'): # vm1748
      with open(os.path.join(path, filename), 'r') as f:

         contents = f.readlines()
         print(contents)

         m = ReadData.importTSPModel(filename)
         #visualizeTSP(m)

         if  m.node_crowd < 10000:
            for category in ["","RND"]:
               for profit in ["p1", "p2"]:
                  for t_percentage in ["T40", "T60", "T80"]:

                     T = readDatasetT(filename)

                     sop = ReadData.generateSOPModel(filename, m, T, category, profit, t_percentage)

                     #visualizeSOP(sop)

                     writeSOPinTxt(sop, contents)
