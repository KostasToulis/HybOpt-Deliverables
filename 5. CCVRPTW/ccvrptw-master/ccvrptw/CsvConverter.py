from os import listdir

path = "your_path"
names = listdir(path)
if "combined.csv" in names:
    names.remove("combined.csv")
instances = []
for name in names:
    with open(path + "/" + name) as file:
        lines = file.readlines()[1:]
        lines = list(map(str.strip, lines))
        instances += lines
instances = sorted(instances)
with open(path + "/combined.csv", "w") as file:
    file.write("Instance,Vehicles,Not Visited,Cost,Kyriakakis Cost," +
                "LS Last Improving Iter,Refresh Promises Iter,Execution Time (s), Best Restart, Biggest Gap\n")
    for line in instances:
        file.write(line + "\n")
