import matplotlib.pyplot as plt
import math

import json

# Specify the path to your JSON file
json_file_path1 = 'vrpdo_data.json'
json_file_path2 = 'solution_data.json'

# Read the JSON file
with open(json_file_path1, 'r') as file:
    data = json.load(file)
    
with open(json_file_path2, 'r') as file:
    solution_data = json.load(file)
    

class Location:
    def __init__(self, Id, Xx, Yy, MaxCap, Ready, Due, Type, ServiceTime, Cap):
        self.Id = Id
        self.Xx = Xx
        self.Yy = Yy
        self.MaxCap = MaxCap
        self.Ready = Ready
        self.Due = Due
        self.Type = Type
        self.ServiceTime = ServiceTime
        self.Cap = Cap

class Customer:
    def __init__(self, Id, Dem, IsRouted):
        self.Id = Id
        self.Dem = Dem
        self.IsRouted = IsRouted

class Option:
    def __init__(self, Id, Location, Cust, Prio, ServiceTime, Cost, IsServed, Ready, Due):
        self.Id = Id
        self.Location = Location
        self.Cust = Cust
        self.Prio = Prio
        self.ServiceTime = ServiceTime
        self.Cost = Cost
        self.IsServed = IsServed
        self.Ready = Ready
        self.Due = Due

class Route:
    def __init__(self, Id, Load, Capacity, Duration, Cost, FixedCost, SequenceOfStartingTime, SequenceOfEndingTime, SequenceOfEct, SequenceOfLat, SequenceOfOptions):
        self.Id = Id
        self.Load = Load
        self.Capacity = Capacity
        self.Duration = Duration
        self.Cost = Cost
        self.FixedCost = FixedCost
        self.SequenceOfStartingTime = SequenceOfStartingTime
        self.SequenceOfEndingTime = SequenceOfEndingTime
        self.SequenceOfEct = SequenceOfEct
        self.SequenceOfLat = SequenceOfLat
        self.SequenceOfOptions = SequenceOfOptions
        
options = []
for option_data in data['Options']:
    location_data = option_data['Location']
    location = Location(
        location_data['Id'],
        location_data['Xx'],
        location_data['Yy'],
        location_data['MaxCap'],
        location_data['Ready'],
        location_data['Due'],
        location_data['Type'],
        location_data['ServiceTime'],
        location_data['Cap']
    )

    customer_data = option_data['Cust']
    customer = Customer(
        customer_data['Id'],
        customer_data['Dem'],
        customer_data['IsRouted']
    )

    option = Option(
        option_data['Id'],
        location,
        customer,
        option_data['Prio'],
        option_data['ServiceTime'],
        option_data['Cost'],
        option_data['IsServed'],
        option_data['Ready'],
        option_data['Due']
    )

    options.append(option)
    
routes = []
for route_data in solution_data['Routes']:
    sequence_of_options = []
    for opt in route_data['SequenceOfOptions']:
        location = Location(
            opt['Location']['Id'],
            opt['Location']['Xx'],
            opt['Location']['Yy'],
            opt['Location']['MaxCap'],
            opt['Location']['Ready'],
            opt['Location']['Due'],
            opt['Location']['Type'],
            opt['Location']['ServiceTime'],
            opt['Location']['Cap']
        )
        customer = Customer(
            opt['Cust']['Id'],
            opt['Cust']['Dem'],
            opt['Cust']['IsRouted']
        )
        option = Option(
            opt['Id'],
            location,
            customer,
            opt['Prio'],
            opt['ServiceTime'],
            opt['Cost'],
            opt['IsServed'],
            opt['Ready'],
            opt['Due']
        )
        sequence_of_options.append(option)
    
    route = Route(
        route_data['Id'],
        route_data['Load'],
        route_data['Capacity'],
        route_data['Duration'],
        route_data['Cost'],
        route_data['FixedCost'],
        route_data['SequenceOfStartingTime'],
        route_data['SequenceOfEndingTime'],
        route_data['SequenceOfEct'],
        route_data['SequenceOfLat'],
        sequence_of_options
    )
    routes.append(route)
    
customer_ids = set()
for option in options:
    customer_ids.add(option.Cust.Id)
for route in routes:
    for opt in route.SequenceOfOptions:
        customer_ids.add(opt.Cust.Id)

# Assign colors to each customer
customer_colors = {}
colors = plt.cm.tab20

for i, customer_id in enumerate(customer_ids):
    customer_colors[customer_id] = colors(i / len(customer_ids))

plt.figure(figsize=(30, 20))
# Plot the Depot 
depot_x = routes[0].SequenceOfOptions[0].Location.Xx
depot_y = routes[0].SequenceOfOptions[0].Location.Yy
plt.scatter(depot_x, depot_y, marker='s', color='black', s=1000)
plt.text(depot_x, depot_y + 2, 'Depot', fontsize=32, color='black', ha='left', va='center')

colors = plt.cm.tab20.colors  # Get colors from tab20 colormap
num_routes = len(routes)
routes_to_draw = []
for i, route in enumerate(routes):
    route_color = customer_colors[route.SequenceOfOptions[i].Cust.Id]  # Use customer color for the route
    
    # Initialize lists to store route coordinates
    xs = []
    ys = []
    
    # Add the coordinates of each location in the route
    for opt in route.SequenceOfOptions:
        xs.append(opt.Location.Xx)
        ys.append(opt.Location.Yy)
    
    routes_to_draw.append((route_color, xs, ys))
        
location_options = {}
for option in options:
    if option.Location.Id not in location_options:
        location_options[option.Location.Id] = []
    location_options[option.Location.Id].append(option)
   
display_option = 1 # or 2
#Option displays the plot with the routes
if display_option == 1: 
    # Plot the locations with colors corresponding to customers
    #""""
    for option in options:
        if option.Location.Type == 1:
            plt.scatter(option.Location.Xx, option.Location.Yy, color='red' ,s=600, marker='^')
        else:
            plt.scatter(option.Location.Xx, option.Location.Yy, color='green', s=500, marker='o')
        plt.text(option.Location.Xx - 1, option.Location.Yy - 1, f"{option.Location.Id}", fontsize=32, ha='left', va='center')
        for i, (route_color, xs, ys) in enumerate(routes_to_draw):
            plt.plot(xs, ys, color=route_color, label=f"Route {i + 1}", alpha=0.7, linewidth=3)
    #"""
#Option 2 displays the plot with the option connections
else:
    # Plot the locations with colors corresponding to customers
    #"""
    for loc_id, loc_options in location_options.items():
        if loc_options[0].Location.Type == 1:
            # Find all options at the same location
            n = len(loc_options)
            angle_step = 2 * math.pi / n
            radius = 0.5  # Distance from the central point

            for i, loc_option in enumerate(loc_options):
                angle = i * angle_step
                offset_x = radius * math.cos(angle)
                offset_y = radius * math.sin(angle)
                plt.scatter(loc_option.Location.Xx + offset_x, loc_option.Location.Yy + offset_y, 
                            color=customer_colors[loc_option.Cust.Id], s=300, marker='^')
                if i == 0:
                    plt.text(loc_option.Location.Xx + offset_x - 1, loc_option.Location.Yy + offset_y - 2, 
                        f"{loc_option.Location.Id}", fontsize=32, ha='left', va='center')
        else:
            for option in loc_options:
                plt.scatter(option.Location.Xx, option.Location.Yy, color=customer_colors[option.Cust.Id], s=500, marker='o')
                # Optionally, add text annotations near each point
                plt.text(option.Location.Xx - 1, option.Location.Yy - 1, f"{option.Location.Id}", fontsize=32, ha='left', va='center')

            # Plot dashed lines connecting options of the same customer
            customer_options = {}
            for option in options:
                if option.Cust.Id not in customer_options:
                    customer_options[option.Cust.Id] = []
                customer_options[option.Cust.Id].append(option)

            for customer_id, customer_opts in customer_options.items():
                if len(customer_opts) > 1:
                    for i in range(len(customer_opts) - 1):
                        x_values = [customer_opts[i].Location.Xx, customer_opts[i + 1].Location.Xx]
                        y_values = [customer_opts[i].Location.Yy, customer_opts[i + 1].Location.Yy]
                        plt.plot(x_values, y_values, color=customer_colors[customer_id], linestyle='--')
    #"""


plt.xlabel('X Coordinate', fontsize=24)
plt.ylabel('Y Coordinate', fontsize=24)
plt.title('VRPDO Solution Plot', fontsize=32)
plt.grid(True)
plt.savefig('vrpdo_solution_plot.jpg', format='jpeg')