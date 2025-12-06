
class route:
	def __init__(self):
		self.nodes = []
		self.quantities = {}
	
	def report(self):
		for i in range(len(self.nodes)):
			print("Node: " + str(self.nodes[i]) + " Delivery: " + str(self.quantities[self.nodes[i]]))

	def getLoad(self):
		d = 0
		for i in range(len(self.nodes)):
			d += self.quantities[self.nodes[i]]
		return d;
			

class period:
	def __init__(self):
		self.routes = []
	
	def report(self):
		for rt in self.routes:
			rt.report()

class problem:
	def __init__(self):
		self.periods = []
	
	def report_depot(self):
		for i in range(len(self.periods)):
			per = self.periods[i]
			d = 0
			for rt in per.routes:
				d += rt.getLoad()

			print("Period ", i , "Total delivery", d)


	def report(self):
		for per in self.periods:
			per.report()


#Init problem
num_periods = 20
p = problem()

for i in range(num_periods):
	p.periods.append(period())


#Parse routing
f = open("D200-1out", 'r')
lines = f.readlines()
f.close()

route_l = list(map(int, lines[1].strip().split()))
print(route_l)


#Init routes
# for i in range(num_periods):
# 	per = p.periods[i]
# 	for j in range(route_l[i]):
# 		per.routes.append(route())

l_index = 2
active_period_id = 0;
imported_routes = 0;
while(l_index < len(lines)):
	info_l = list(map(int, lines[l_index].strip().split()))
	print(info_l)
	num_nodes = info_l[0]

	#Calculate period_index
	if (len(p.periods[active_period_id].routes) == route_l[active_period_id]):
		#proceed to next period
		for i in range(active_period_id + 1, num_periods):
			if (route_l[i] > 0):
				active_period_id = i
				break
	
	period_index = active_period_id
	route_index = info_l[2] - 1
	l_index += 1

	print(active_period_id, route_index)
	period = p.periods[period_index]
	rt = route()

	for i in range(num_nodes):
		rt.nodes.append(int(lines[l_index].strip()))
		l_index += 1 
	
	print(rt.nodes)
	period.routes.append(rt)
	




#Parse deliveries
f = open("IRP201.dat.output", 'r')
lines = f.readlines()
f.close()


l_index = 0
for period in p.periods:
	for rt in period.routes:
		#Just need the count here
		for i in range(len(rt.nodes)):
			sp = lines[l_index].strip().split()
			node_id = int(sp[2])
			rt.quantities[node_id] = int(sp[3])
			l_index += 1


p.report_depot()
#p.report()


print("$$$$$$$$$$$$$$$$")

#Write solution output
f = open("output.txt", "w");

#Routes
for i in range(len(p.periods)):
	per = p.periods[i]
	f.writelines("Period_" + str(i + 1) + "_Routes:\n")
	
	if (len(per.routes) == 0):
		f.writelines("-Route_0: 1 1\n")
	else:
		for j in range(len(per.routes)):
			rt = per.routes[j]
			s = "-Route_" + str(j) + " : 1 "
			for node_id in rt.nodes:
				s += str(node_id) + " "
				l_index += 1
			s += "1"
			f.writelines(s + "\n")
	f.writelines("\n")


f.writelines("\n")
f.writelines("\n")

#Production Quantities
for i in range(len(p.periods)):
	f.writelines("Period_" + str(i + 1) + "_production_quantity:\n")
	f.writelines("0\n")

f.writelines("\n")
f.writelines("\n")

#Delivered Quantities
for i in range(len(p.periods)):
	per = p.periods[i]
	f.writelines("Period_" + str(i + 1) + "_delivered_quantity:\n")
	s_n = ""
	s_q = ""
	for j in range(len(per.routes)):
		rt = per.routes[j]
		for node_id in rt.nodes:
			s_n += str(node_id) + " "
			s_q += str(rt.quantities[node_id]) + " "
	
	if (s_n == ""):
		f.writelines("-1\n")
		f.writelines("0.0\n")
	else:
		f.writelines(s_n + "\n")
		f.writelines(s_q + "\n")
	f.writelines("\n")

f.close()
 