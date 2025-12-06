
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
		self.production = 0
	
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


def find_neighbor_node(arc_list, node_id):
	for arc in arc_list:
		if node_id in arc:
			arc_list.remove(arc)
			if node_id == arc[0]:
				return arc[1]
			else:
				return arc[0]
	return None


#Init problem
num_periods = 20
p = problem()

for i in range(num_periods):
	p.periods.append(period())

#Parse file
f = open("RESULTS_B1_2EPRP/Detailed Results/50Clients_2.log", 'r')
lines = f.readlines()
f.close()


lines = lines[1:] # SKip first comment line




problem_arc_list = {}

for l in lines:
	l = l.strip()
	print(l)

	#End inventory data
	if (l.startswith("Iit")):
		pass
	elif (l.startswith("Qikt")): #Delivery quantities
		sp = l.lstrip("Qikt").split('=')
		sp_i = sp[0].split('_')
		cust = int(sp_i[0])
		route_index = int(sp_i[1]) - 1
		period_index = int(sp_i[2]) - 1

		q = float(sp[1])

		period = p.periods[period_index]

		#Add routes if they doesn't exist
		while(route_index >= len(period.routes)):
			period.routes.append(route())

		rt = period.routes[route_index]
		
		#Add delivery to customers
		rt.quantities[cust + 1] = q
	elif (l.startswith("Xijkt")): #Save arcs
		sp = l.lstrip("Xijkt").split("=")
		sp_i = sp[0].split('_')
		cust_i = int(sp_i[0])
		cust_j = int(sp_i[1])
		route_index = int(sp_i[2]) - 1
		period_index = int(sp_i[3]) - 1

		val = int(sp[1])

		if (period_index, route_index) not in problem_arc_list:
			problem_arc_list[(period_index, route_index)] = set()

		route_arc_list = problem_arc_list[(period_index, route_index)]
		route_arc_list.add((cust_i, cust_j))

		if (val == 2):
			route_arc_list.add((cust_j, cust_i))

	elif (l.startswith("pt")):
		sp = l.lstrip("pt").split("=")
		period_index = int(sp[0])
		q = int(sp[1])
		p.periods[period_index].production = q


for l in problem_arc_list:
	print("route", l)
	print(problem_arc_list[l])


#Assemble routes from arc lists

for l in problem_arc_list:
	print("route", l)
	route_arc_list = problem_arc_list[l]

	rt = p.periods[l[0]].routes[l[1]]
	
	#Find the first arc that includes 0
	node_id = 0
	while(True):
		node_id = find_neighbor_node(route_arc_list, node_id)
		if (node_id is None) or (node_id == 0):
			break
		rt.nodes.append(node_id + 1)

	print(rt.nodes)


p.report_depot()
#p.report()


print("$$$$$$$$$$$$$$$$")

#Write solution output

#Routes
for i in range(len(p.periods)):
	per = p.periods[i]
	print("Period_" + str(i + 1) + "_Routes:")
	
	if (len(per.routes) == 0):
		print("-Route_0: 1 1")
	else:
		for j in range(len(per.routes)):
			rt = per.routes[j]
			s = "-Route_" + str(j) + " : 1 "
			for node_id in rt.nodes:
				s += str(node_id) + " "
			s += "1"
			print(s)
	print()


print()
print()

#Production Quantities
for i in range(len(p.periods)):
	print("Period_" + str(i + 1) + "_production_quantity:")
	print(p.periods[i].production)

print()
print()

#Delivered Quantities
for i in range(len(p.periods)):
	per = p.periods[i]
	print("Period_" + str(i + 1) + "_delivered_quantity:")
	s_n = ""
	s_q = ""
	for j in range(len(per.routes)):
		rt = per.routes[j]
		for node_id in rt.nodes:
			s_n += str(node_id) + " "
			s_q += str(rt.quantities[node_id]) + " "
	
	if (s_n == ""):
		print("-1")
		print("0.0")
	else:
		print(s_n)
		print(s_q)
	print()
 