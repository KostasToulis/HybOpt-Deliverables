import os
import warnings
import openpyxl
from openpyxl import load_workbook
from datetime import datetime

def column_string(n):
    string = ""
    while n > 0:
        n, remainder = divmod(n - 1, 26)
        string = chr(65 + remainder) + string
    return string

def retrieveInstanceInfo(dataset, customers, i, j=0):
    instanceDict = dict()
    instanceDict["totalTime"] = 0
    instanceDict["bestSchedule"] = ""
    instanceDict["bestObj"] = 0
    instanceDict["totalSchedules"] = 0
    instanceDict["bestScheduleIdx"] = 0
    instanceDict["bestRestartIdx"] = 0
    instanceDict["bestRestartTime"] = 0
    instanceDict["bestRestartTotalTime"] = 0
    instanceDict["bestRestartFeasIters"] = 0
    instanceDict["bestRestartInfeasIters"] = 0
    instanceDict["bestRestartRepairs"] = 0
    instanceDict["bestRestartTimeRepairing"] = 0
    instanceDict["numOfRestarts"] = 0
    instanceDict["relObj"] = 0
    instanceDict["bestScheduleTime"] = 0
    instanceDict["invObj"] = 0
    instanceDict["prodObj"] = 0
    instanceDict["setupObj"] = 0
    instanceDict["routingObj"] = 0
    instanceDict["schedules"] = list()

    # read summary
    try:
        file = ""
        if dataset == "Bou":
            file = open("{}/RelSol_{}_{}_{}.DAT_results_summary.txt".format(resultpath, dataset.capitalize(), customers, i), "r")
        elif dataset == "Arc":
            file = open("{}/RelSol_{}_ABS{}_{}_{}.DAT_results_summary.txt".format(resultpath, dataset, i , customers, j), "r")

        first = True
        for line in file:
            lineSplit = line.strip().split(' ')
            if first:
                instanceDict["totalTime"] = lineSplit[0]
                instanceDict["bestSchedule"] = lineSplit[1]
                instanceDict["bestObj"] = lineSplit[2]
                instanceDict["totalSchedules"] = lineSplit[3]
                first = False
            else:
                if lineSplit[-1] == "True":
                    scheduleDict = dict()
                    scheduleDict["schedule"] = lineSplit[0]
                    if scheduleDict["schedule"] == instanceDict["bestSchedule"]:
                        instanceDict["bestScheduleIdx"] = len(instanceDict["schedules"])
                        instanceDict["bestScheduleTime"] = lineSplit[3]
                        instanceDict["relObj"] = lineSplit[1]
                    scheduleDict["relObj"] = lineSplit[1]
                    scheduleDict["obj"] = lineSplit[2]
                    scheduleDict["time"] = lineSplit[3]
                    scheduleDict["feasible"] = lineSplit[4]
                    instanceDict["schedules"].append(scheduleDict)

        # read best schedule log
        if dataset == "Bou":
            file = open("{}/Log_RelSol_{}_{}_instance{}_{}.txt".format(resultpath, dataset, customers, i, instanceDict["bestSchedule"]), "r")
        elif dataset == "Arc":
            file = open("{}/Log_RelSol_{}_ABS{}_{}_{}.DAT_{}.txt".format(resultpath, dataset, i, customers, j, instanceDict["bestSchedule"]), "r")


        first = True
        for line in reversed(list(file)):
            lineSplit = line.strip().split(' ')
            if first:
                instanceDict["numOfRestarts"] = int(float(lineSplit[0])) + 1
                first = False

            if lineSplit[1] == instanceDict["bestObj"]:
                instanceDict["bestRestartIdx"] = lineSplit[0]
                instanceDict["bestRestartTime"] = lineSplit[2]
                if len(lineSplit) > 3: # this is the new logging
                    instanceDict["bestRestartTotalTime"] = lineSplit[3]
                    instanceDict["bestRestartFeasIters"] = lineSplit[4]
                    instanceDict["bestRestartInfeasIters"] = lineSplit[5]
                    instanceDict["bestRestartRepairs"] =  lineSplit[6]
                    instanceDict["bestRestartTimeRepairing"] = lineSplit[7]

            if lineSplit == "0":
                break

        # read best solution file
        if dataset == "Bou":
            file = open("{}/Sol_RelSol_{}_{}_instance{}_{}.txt".format(resultpath, dataset, customers, i, instanceDict["bestSchedule"]), "r")
        elif dataset == "Arc":
            file = open("{}/Sol_RelSol_{}_ABS{}_{}_{}.DAT_{}.txt".format(resultpath, dataset, i, customers, j, instanceDict["bestSchedule"]), "r")

        lineCnt = 0
        for line in file:
            lineSplit = line.strip().split(' ')

            if lineCnt > 4:
                break
            elif lineCnt == 1:
                instanceDict["routingObj"] = int(float(lineSplit[1]))
            elif lineCnt == 2:
                instanceDict["invObj"] = int(float(lineSplit[1]))
            elif lineCnt == 3:
                instanceDict["prodObj"] = int(float(lineSplit[1]))
            elif lineCnt == 4:
                instanceDict["setupObj"] = int(float(lineSplit[1]))

            lineCnt += 1
    except:
        print("File not found")

    return instanceDict

def writeToExcel(instanceDict, dataset, customers, i, j=0):
    row = -1
    if dataset == "Bou":
        row = i + 3
    elif dataset == "Arc":
        row = i + ((j-1)*96) + 3

    # 3. write results to excels
    wb = load_workbook(filename='{}/Results/{}_PRP_benchmarks_matheuristic.xlsx'.format(os.path.abspath(os.getcwd()), dataset),
                       data_only=True)

    # access the specific sheet
    if dataset == "Bou":
        if customers == 50:
            sheet = wb['B1Cost']
        elif customers == 100:
            sheet = wb['B2Cost']
        elif customers == 200:
            sheet = wb['B3Cost']
        else:
            print("Wrong customer value")
            raise
    elif dataset == "Arc":
        if customers == 15:
            sheet = wb['A1Cost']
        elif customers == 50:
            sheet = wb['A2Cost']
        elif customers == 100:
            sheet = wb['A3Cost']
        else:
            print("Wrong customer value")
            raise

    column = 16
    sheet['{}{}'.format(column_string(column),row)] = instanceDict["totalTime"]
    column +=1

    sheet['{}{}'.format(column_string(column),row)] = instanceDict["totalSchedules"]
    column +=1

    sheet['{}{}'.format(column_string(column),row)] = instanceDict["bestScheduleIdx"]
    column +=1

    sheet['{}{}'.format(column_string(column),row)] = instanceDict["bestSchedule"]
    column +=1

    sheet['{}{}'.format(column_string(column),row)] = instanceDict["relObj"]
    column +=1

    sheet['{}{}'.format(column_string(column),row)] = instanceDict["bestScheduleTime"]
    column +=1

    sheet['{}{}'.format(column_string(column),row)] = instanceDict["numOfRestarts"]
    column +=1

    sheet['{}{}'.format(column_string(column),row)] = instanceDict["bestRestartIdx"]
    column +=1

    sheet['{}{}'.format(column_string(column),row)] = instanceDict["bestRestartTime"]
    column +=1

    sheet['{}{}'.format(column_string(column),row)] = instanceDict["bestRestartTotalTime"]
    column +=1

    sheet['{}{}'.format(column_string(column),row)] = instanceDict["bestRestartFeasIters"]
    column +=1

    sheet['{}{}'.format(column_string(column),row)] = instanceDict["bestRestartInfeasIters"]
    column +=1

    sheet['{}{}'.format(column_string(column),row)] = instanceDict["bestRestartRepairs"]
    column +=1

    sheet['{}{}'.format(column_string(column),row)] = instanceDict["bestRestartTimeRepairing"]
    column +=1

    sheet['{}{}'.format(column_string(column),row)] = instanceDict["bestObj"]
    column +=1

    sheet['{}{}'.format(column_string(column),row)] = instanceDict["setupObj"]
    column +=1

    sheet['{}{}'.format(column_string(column),row)] = instanceDict["prodObj"]
    column +=1

    sheet['{}{}'.format(column_string(column),row)] = instanceDict["invObj"]
    column +=1

    sheet['{}{}'.format(column_string(column),row)] = instanceDict["routingObj"]
    column +=1

    # datetime object containing current date and time
    sheet['{}{}'.format(column_string(column),row)] = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
    column +=1

    # calculate bests, improvements, worse etc
    #sheet['M{}'.format(row)] = "=IF(AE{}>0,(AD{}-N{})/N{},' ')".format(row, row, row, row)

    #sheet['M1'] = "=COUNTIF(M4:M483,'=0')"
    #sheet['M2'] = "=COUNTIF(M4:M483,'<0')"
    #sheet['M3'] = "=COUNTIF(M4:M483,'>0')"

    # wb.save("{}_PRP_benchmarks.xlsx".format(dataset))
    wb.save(filename='{}/Results/{}_PRP_benchmarks_matheuristic.xlsx'.format(os.path.abspath(os.getcwd()), dataset))

#=========================================== Main =====================================================================#
resultpath = "bin/release/netcoreapp3.1"
resultpath = "bin/release/Archetti full runs v3.1 (2_4) 100"
resultpath = "bin/release/Boudia full runs v3 (2_1, 100 res)"

print("Parsing experiments' results and writing to excel")

#Boudia

dataset = "Bou" # Bou Arc
customers = 200
rangeStart = 1
rangeEnd = 30
"""

#Arc
dataset = "Arc" # Bou Arc
customers = 15
rangeStart = 1
rangeEnd = 96
instanceStart = 1
instanceEnd = 5
"""





# 1. fetch results file
print("Fetching results")
#print(os.path.abspath(os.getcwd()))

# 2. read results for the specific problems
if dataset == "Bou":
    for customers in [50,100,200]:
        for i in range(rangeStart,rangeEnd+1):
            instanceDict = retrieveInstanceInfo(dataset, customers, i)
            if instanceDict["bestObj"] == 0:
                pass
            else:
                writeToExcel(instanceDict, dataset, customers, i)
                print(instanceDict)
elif dataset == "Arc":
    for i in range(rangeStart,rangeEnd+1):
        for j in range(instanceStart,instanceEnd+1):
            instanceDict = retrieveInstanceInfo(dataset, customers, i, j)
            if instanceDict["bestObj"] == 0:
                pass
            else:
                writeToExcel(instanceDict, dataset, customers, i, j)
                print(instanceDict)


