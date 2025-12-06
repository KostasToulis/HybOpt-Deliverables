using System.Globalization;
using System.Text.RegularExpressions;
using ScottPlot;

namespace CHRVRP
{
    public class Solution
    {
        public List<Route> routes; // lista me route objects pou anaparistoun dromologia kai tis idiothtes tous
        public double objective1;
        public double objective2;
        public double objective3;
        public double objective4;
        public double max;  // for the 4rth kpi 
        public double min; // for the 4rth kpi
        public bool feasible;
        public Model model; //Hold a local reference to the model
        public string name;
        public Solution(string filename, Model model, Boolean isResults = true)
        {
            //Init
            this.model = model;
            this.objective1 = 0;
            this.objective2 = 0;
            this.objective3 = 0;
            this.objective4 = 0;
            this.max = 0;
            this.min = 100_000;
            this.routes = new List<Route>();
            this.feasible = true;
            this.name = model.name;

            if (isResults)
            {
                this.ReadResults(filename);
            }
            
        }

        public Solution(Solution s)
        {
            model = s.model;
            routes = new List<Route>();
            feasible = s.feasible;
            name = s.name;
            
            for (int i = 0; i < s.routes.Count; i++)
            {
                routes.Add(new Route(new Node(s.routes[i].sequence[0]), i));
                routes[i].load = s.routes[i].load;
                routes[i].totalDistance = s.routes[i].totalDistance;
                routes[i].min = s.routes[i].min;
                routes[i].max = s.routes[i].max;
                routes[i].cumDistance = s.routes[i].cumDistance;

                for (int j = 1; j < s.routes[i].sequence.Count; j++)
                {
                    routes[i].sequence.Add(new Node(s.routes[i].sequence[j]));
                }
            }
            this.objective1 = this.CalculateObjective1();
            this.objective2 = this.CalculateObjective2();
            this.objective3 = this.CalculateObjective3();
            this.objective4 = this.CalculateObjective4();
        }
        
        public void FeasibilityCheck()
        {
            var capacity = this.model.capacity;
            foreach (var route in this.routes)
            {
                if (route.load > capacity)
                {
                    this.feasible = false;
                    break;
                }
            }
        }

        public void ReadResults(string textFile)
        {
            Regex bracket = new Regex("^\\[");
            Regex id = new Regex("\\w\\d+");
            Regex posFloat = new Regex("\\d+\\.\\d+");
            CultureInfo usCulture = new CultureInfo("en-US");
            int counter = 0;

            var fileLines = File.ReadAllLines(textFile);

            foreach (var line in fileLines)
            {
                if (bracket.IsMatch(line))
                {
                    if (posFloat.IsMatch(line))
                    {
                        this.routes[counter].totalDistance = double.Parse(posFloat.Match(line).ToString(), usCulture);

                        if (this.routes[counter].totalDistance > this.objective1)
                        {
                            this.objective1 = this.routes[counter].totalDistance;
                        }
                        counter++;
                    }
                    else
                    {
                        Route route = new Route();
                        foreach (var item in id.Matches(line).ToArray())
                        {
                            Node node = GetNodeById(item.ToString(), model.nodes);
                            route.sequence.Add(node);
                        }
                        this.routes.Add(route);
                    }
                }
            }
        }
        private static Node GetNodeById(string id, List<Node> nodes)
        {
            foreach (var node in nodes)
            {
                if (node.id.Equals(id))
                {
                    return node;
                }
            }
            return new Node();
        }

        private Dictionary<Node, List<Node>> CreateDepotDict(List<Node> centers, List<Node> depots, double[,] matrix)
        {
            Dictionary<Node, List<Node>> depotDict = new Dictionary<Node, List<Node>>();

            foreach (var center in centers)
            {
                depotDict.Add(center, new List<Node>());
            }

            foreach (var depot in depots)
            {
                double minDistance = 100_000_000;
                int bestCenter = 0;

                for (int centerPos = 0; centerPos < centers.Count; centerPos++)
                {
                    var center = centers[centerPos];
                    double dist = matrix[center.serialNumber, depot.serialNumber];
                    if (dist < minDistance)
                    {
                        minDistance = dist;
                        bestCenter = centerPos;
                    }
                }

                depotDict[centers[bestCenter]].Add(depot);
                //centers[bestCenter].vehiclesArrivalTimes.Add(depot, minDistance);
            }

            return depotDict;
        }

        private Dictionary<Node, List<Node>> CreateCustomerDict(List<Node> centers, List<Node> customers, double[,] matrix, Dictionary<Node, List<Node>> vehicles, int capacity)
        {
            Dictionary<Node, List<Node>> customerDict = new Dictionary<Node, List<Node>>();
            Dictionary<Node, int> maxCapacity = new Dictionary<Node, int>();
            Dictionary<Node, int> currentCapacity = new Dictionary<Node, int>();
            List<Node> unserved = new List<Node>();

            foreach (var customer in customers)
            {
                unserved.Add(customer);
            }

            foreach (var center in centers)
            {
                customerDict.Add(center, new List<Node>());
                maxCapacity.Add(center, vehicles[center].Count * capacity);
                currentCapacity.Add(center, 0);
            }

            foreach (var customer in unserved)
            {
                double minDistance = 100_000_000;
                Node bestCenter = new Node();

                foreach (var center in centers)
                {
                    if (currentCapacity[center] == maxCapacity[center])
                    {
                        continue;
                    }

                    double dist = matrix[center.serialNumber, customer.serialNumber];
                    if (dist < minDistance)
                    {
                        bestCenter = center;
                        minDistance = dist;
                    }
                }

                customerDict[bestCenter].Add(customer);
                currentCapacity[bestCenter]++;
            }
            
            
            return customerDict;
        }

        private Dictionary<Node, List<Route>> InstantiateRoutes(List<Node> centers, Dictionary<Node, List<Node>> depots, double[,] matrix)
        {
            Dictionary<Node, List<Route>> routesDict = new Dictionary<Node, List<Route>>();
            int count = 0;
            foreach (var center in centers)
            {
                routesDict.Add(center, new List<Route>());
                foreach (var depot in depots[center])
                {
                    this.routes.Add(new Route(depot, count));
                    depot.routeIndex = count;
                    this.routes[count].sequence.Add(center);
                    center.indexInRoute = 1;
                    center.routeIndex = count;
                    this.routes[count].totalDistance = matrix[depot.serialNumber, center.serialNumber];
                    this.routes[count].cumDistance = 0;
                    UpdateArrivalTimes(this.routes[count]);
                    routesDict[center].Add(this.routes[count]);
                    count++;
                }
            }

            if (this.model.mainKPI == Model.KPI.kpi2)
            {
                this.objective2 = CalculateObjective2();
            }

            return routesDict;
        }

        private InsertionCandidate ChooseRandomCandidate(List<InsertionCandidate> rcl, int n)
        {
            rcl = rcl.OrderBy(x => x.objectiveCriterion).ToList();
            n = (rcl.Count < n) ? rcl.Count : n; 
            int choice = this.model.random.Next(n);

            return rcl[choice];
        }

        private double GetInsertedDistance(Node ins, Node prev, Node next)
        {
            return this.model.distances[prev.serialNumber, ins.serialNumber] +
                       this.model.distances[ins.serialNumber, next.serialNumber] -
                       this.model.distances[prev.serialNumber, next.serialNumber];
        }

        private double GetDistance(Node n1, Node n2)
        {
            return this.model.distances[n1.serialNumber, n2.serialNumber];
        }

        public void Construct(int n)
        {
            var centers = this.model.supply;
            double[,] matrix = this.model.distances;
            var depots = CreateDepotDict(centers, this.model.depots, matrix);
            var customers = CreateCustomerDict(centers, this.model.customers, matrix, depots, this.model.capacity);
            var routesDict =  InstantiateRoutes(centers, depots, matrix);

            foreach (var center in centers)
            {
                int served = 0;
                
                var halfCapacity = model.capacity / 2;
                int rclCapacity = routesDict[center].Count * customers[center].Count * halfCapacity;

                List<InsertionCandidate> rcl = new List<InsertionCandidate>(rclCapacity);

                while (served < customers[center].Count)
                {

                    for (int routeInd = 0; routeInd < routesDict[center].Count; routeInd++)
                    {
                        Route route = routesDict[center][routeInd];
                        
                        if (route.load == this.model.capacity)
                        {
                            continue;
                        }

                        for (int customerInd = 0; customerInd < customers[center].Count; customerInd++)
                        {
                            Node customer = customers[center][customerInd];
                            if (customer.isRouted)
                            {
                                continue;
                            }
                            
                            for (int inRouteInd = 1; inRouteInd < route.sequence.Count; inRouteInd++)
                            {
                                double distanceChange = CalculateDistanceChange(customer, route, inRouteInd);
                                (double tempMin, double tempMax) = FindMinMax(customer, route, inRouteInd, distanceChange);
                                double tempCumDistChange = CalculateTempCumDistChange(customer, route, inRouteInd, distanceChange);
                                double objectiveCriterion = CalculateObjectiveCriterion(distanceChange, this.model.mainKPI, route, tempCumDistChange,tempMax, tempMin);
                                rcl.Add(new InsertionCandidate(routeInd, customerInd, inRouteInd + 1, distanceChange, tempCumDistChange,
                                                               tempMin, tempMax, objectiveCriterion));
                            }   
                        }
                    }
                    var candidate = ChooseRandomCandidate(rcl, n);
                    Route candRoute = routesDict[center][candidate.routeInd];
                    Node candCustomer = customers[center][candidate.nodeInd];
                    candCustomer.routeIndex = candidate.routeInd;
                    int inRoutePos = candidate.inRouteInd;
                    candRoute.Add(candCustomer, inRoutePos, candidate.distanceChange);
                    UpdateArrivalTimes(candRoute);
                    UpdateCumDistance(candRoute);
                    UpdateMinMax(candidate);
                    UpdateObjective(candidate.objectiveCriterion);
                    rcl.Clear();
                    served++;
                }
            }

            IndexRoutes();
        }

        private void IndexRoutes()
        {
            foreach (var route in this.routes)
            {
                route.IndexInRoute();
            }
        }

        private void UpdateMinMax(InsertionCandidate candidate)
        {
            this.max = candidate.max;
            this.min = candidate.min;
            UpdateRouteMinMax();
        }

        public void UpdateRouteMinMax()
        {
            foreach (var route in routes)
            {
                if (route.sequence.Count > 2)
                {
                    route.min = route.sequence[2].arrivalTime;
                    route.max = route.sequence.Last().arrivalTime;
                }
            }
        }

        public void UpdateCumDistance(Route candRoute)
        {
            double cumDistance = 0;
            for (int j = 2; j < candRoute.sequence.Count; j++)
            {
                Node node = candRoute.sequence[j]; 
                cumDistance += node.arrivalTime;
            }

            candRoute.cumDistance = cumDistance;
        }

        private double CalculateTempCumDistChange(Node customer, Route route, int inRouteInd, double distanceChange)
        {
            double previousArrivalTime;

            if (inRouteInd == 1)
            {
                previousArrivalTime = this.model.distances[route.sequence[0].serialNumber, route.sequence[1].serialNumber];
            }
            else
            {
                previousArrivalTime = route.sequence[inRouteInd].arrivalTime;
            }

            if (inRouteInd < route.sequence.Count - 1)
            {
                return previousArrivalTime + 
                    this.model.distances[route.sequence[inRouteInd].serialNumber, customer.serialNumber] +
                    distanceChange * (route.sequence.Count - inRouteInd - 1);
            }
            else
            {
                return previousArrivalTime + distanceChange;
            }
        }

        private (double, double) FindMinMax(Node customer, Route route, int inRouteInd, double distanceChange)
        {
            double tempMax = GetMax();
            double tempMin = GetMin();

            var sp = route.sequence[1];
            var vehicle = route.sequence[0];

            var vehArrTime = this.model.distances[vehicle.serialNumber, sp.serialNumber];

            if (inRouteInd == 1)
            {
                double distanceAdded = GetDistance(sp, customer);

                if (distanceAdded + vehArrTime < tempMin)
                {
                    tempMin = distanceAdded + vehArrTime;
                }
            }

            int last = route.sequence.Count - 1;

            if (last == 1) 
            {
                tempMax = distanceChange + vehArrTime;
            }
            if (distanceChange + route.sequence[last].arrivalTime > tempMax)
            {
                tempMax = distanceChange + route.sequence.Last().arrivalTime;
            }

            return (tempMin, tempMax);
        }

        private double GetMin()
        {
            double min = 10_000_000;
            for (var i = 0; i < routes.Count; i++)
            {
                if (routes[i].sequence.Count > 2)
                {
                    min = (routes[i].min < min) ? routes[i].min : min;
                }
            }
            return min;
        }

        private double GetMax()
        {
            double max = 0;
            for (var i = 0; i < routes.Count; i++)
            {
                max = (routes[i].max > max) ? routes[i].max : max;
            }
            return max;
        }

        private double CalculateDistanceChange(Node customer, Route route,int inRouteInd)
        {
            Node prev = route.sequence[inRouteInd];
            double distanceChange;

            if (inRouteInd < route.sequence.Count - 1)
            {
                Node next = route.sequence[inRouteInd + 1];
                distanceChange = GetInsertedDistance(customer, prev, next);
            }
            else
            {
                distanceChange = GetDistance(prev, customer);
            }
            return distanceChange;
        }
        public void UpdateArrivalTimes(Route route)
        {
            Node vehicle = route.sequence[0];
            Node sp = route.sequence[1];
            
            var vehArrTime = this.model.distances[vehicle.serialNumber, sp.serialNumber];
            
            if (route.sequence.Count > 2)
            {
                Node n = route.sequence[2];
                route.sequence[2].arrivalTime = this.model.distances[sp.serialNumber, n.serialNumber] + vehArrTime;
            }

            for (int i = 3; i < route.sequence.Count; i++)
            {
                Node n1 = route.sequence[i - 1];
                Node n2 = route.sequence[i];
                route.sequence[i].arrivalTime = this.model.distances[n1.serialNumber, n2.serialNumber] + route.sequence[i - 1].arrivalTime;
            }
        }

        public void UpdateObjective(double obj)
        {
            switch (this.model.mainKPI)
            {
                case Model.KPI.kpi1:
                    if (this.objective1 < obj)
                    {
                        this.objective1 = obj;
                    }
                    break;
                case Model.KPI.kpi2:
                    this.objective2 += obj;
                    break;
                case Model.KPI.kpi3:
                    this.objective3 = obj / this.model.customers.Count;
                    break;
                case Model.KPI.kpi4:
                    this.objective4 = obj;
                    break;
                default:
                    break;
            }
        }

        private double CalculateObjectiveCriterion(double distanceChange, Model.KPI kpi, Route route, double cumDistChange, double max, double min)
        {
            switch (kpi)
            {
                case Model.KPI.kpi1:
                    return route.totalDistance + distanceChange;
                case Model.KPI.kpi2:
                    return distanceChange;
                case Model.KPI.kpi3:
                    double sum = 0;
                    foreach (var r in routes) 
                    {
                        sum += r.cumDistance;
                    }
                    return sum + cumDistChange;
                case Model.KPI.kpi4:
                    return max - min;
                default:
                    return 0;
            }
        }

        public double CalculateObjective1()
        {
            double obj1 = this.routes[0].totalDistance;
            for (int i = 1; i < this.routes.Count; i++)
            {
                if (this.routes[i].totalDistance > obj1)
                {
                    obj1 = this.routes[i].totalDistance;
                }
            }
            return obj1;
        }

        public double CalculateObjective2()
        {
            var obj2 = 0.0;
            var capacityViolated = 0;
            foreach (var route in routes)
            {
                obj2 += route.totalDistance;
                capacityViolated += (route.load > model.capacity ? route.load - model.capacity : 0);
            }
            return obj2 + 10000 * capacityViolated;
        }

        public double CalculateObjective3()
        {
            double obj3 = 0;
            foreach (var route in this.routes)
            {
                obj3 += route.cumDistance;
            }
            obj3 = obj3 / this.model.customers.Count;
            return obj3;
        }
        public double CalculateObjective4()
        {
            double min = 10_000_000;
            double max = 0;

            UpdateRouteMinMax();
            
            var capacityViolated = 0;
            
            foreach (var route in routes)
            {
                capacityViolated += (route.load > model.capacity ? route.load - model.capacity : 0);
                max = (route.max > max ? route.max : max);
                min = (route.min < min ? route.min : min);
            }
            return (max - min) + 10000 * capacityViolated;
        }


        /********************************************************************************************
         * a episis min jexaseis na sviseis ta sxolia otan ftiajeis tis methodous                   *
         ********************************************************************************************/

        public void CalculateObjectives()
        {
            if (this.model.mainKPI != Model.KPI.kpi1)
            {
                this.objective1 = CalculateObjective1();
            }

            if (this.model.mainKPI != Model.KPI.kpi2)
            {
                this.objective2 = CalculateObjective2();
            }

            if (this.model.mainKPI != Model.KPI.kpi3)
            {
                this.objective3 = CalculateObjective3();
            }

            if (this.model.mainKPI != Model.KPI.kpi4)
            {
                this.objective4 = CalculateObjective4();
            }
        }

        public void Print()
        {
            foreach (var route in routes)
            { 
                route.printRoute();
                Console.WriteLine(route.cumDistance);
            }
            Console.WriteLine($"KPI: {this.model.mainKPI}");
            Console.WriteLine($"Objective 1: {this.objective1}");
            Console.WriteLine($"Objective 2: {this.objective2}");
            Console.WriteLine($"Objective 3: {this.objective3}");
            Console.WriteLine($"Objective 4: {this.objective4}");
        }

        public override string ToString()
        {
            return String.Format("Solution(kpi1: {0} | kpi2: {1} | kpi3: {2} | kpi4: {3} | mainKPI: {4})",
                this.objective1, this.objective2, this.objective3, this.objective4, this.model.mainKPI);
        }

        public void PlotRoutes(int iter)
         {
             var plt = new Plot(800, 600);

             double[] nodesX = new double[this.model.supply.Count];
             double[] nodesY = new double[this.model.supply.Count];
             for (int i = 0; i < this.model.supply.Count; i++)
             {
                 nodesX[i] = this.model.supply[i].x;
                 nodesY[i] = this.model.supply[i].y;
             }
             plt.AddScatter(nodesX, nodesY, lineWidth: 0, markerSize:18, markerShape: MarkerShape.filledTriangleUp);

            nodesX = new double[this.model.customers.Count];
            nodesY = new double[this.model.customers.Count];
            for (int i = 0; i < this.model.customers.Count; i++)
            {
                nodesX[i] = this.model.customers[i].x;
                nodesY[i] = this.model.customers[i].y;
            }
            plt.AddScatter(nodesX, nodesY, lineWidth: 0, markerSize: 10);

            nodesX = new double[this.model.depots.Count];
            nodesY = new double[this.model.depots.Count];
            for (int i = 0; i < this.model.depots.Count; i++)
            {
                nodesX[i] = this.model.depots[i].x;
                nodesY[i] = this.model.depots[i].y;
            }
            plt.AddScatter(nodesX, nodesY, lineWidth: 0, markerSize: 10, markerShape: MarkerShape.filledSquare);

            foreach (Route rt in routes)
             {
                 double[] rtX = new double[rt.sequence.Count];
                 double[] rtY = new double[rt.sequence.Count];
                 for (int i = 0; i < rt.sequence.Count; i++)
                 {
                     rtX[i] = rt.sequence[i].x;
                     rtY[i] = rt.sequence[i].y;
                 }
                 plt.AddScatter(rtX, rtY, markerSize: 0, lineWidth: 2);
             }
             plt.SaveFig($"../../../../plot_{iter}.png");
         }

         /*public void PlotCostProgression(List<double> progression, double best, int iter, int restart)
         {
             var plt = new Plot(1000, 700);
             double[] dataX = new double[progression.Count];
             double[] dataY = new double[progression.Count];
             for (int i = 0; i < progression.Count; i ++)
             {
                 dataX[i] = i;
                 dataY[i] = progression[i];
             }
             var hline = plt.AddHorizontalLine(best);
             hline.LineWidth = 1;
             hline.PositionLabel = true;
             var vline = plt.AddVerticalLine(iter);
             vline.LineWidth = 1;
             vline.PositionLabel = true;
             plt.AddScatterLines(dataX, dataY);
             plt.SaveFig($"../../../../CostPlot{restart}.png");
         }*/
         
         public double GetMainKPIObjective()
         {
             switch (model.mainKPI)
             {
                 case Model.KPI.kpi1:
                     return this.objective1;
                 case Model.KPI.kpi2:
                     return this.objective2;
                 case Model.KPI.kpi3:
                     return this.objective3;
                 case Model.KPI.kpi4:
                     return this.objective4;
                 default:
                     return -1;
             }
         }
        
        public bool CheckEverything()
        {
            List<string> errorMessages = new List<string>();
            const int customerStart = 2;

            for (int routeInd = 0; routeInd < routes.Count; routeInd++)
            {
                Route route = routes[routeInd];
                if (route.sequence[0].category != Model.Category.depot)
                {
                    errorMessages.Add(string.Format("Route {0}'s does not start from a depot", routeInd));
                }

                if (route.sequence[1].category != Model.Category.supply)
                {
                    errorMessages.Add(string.Format("Route {0}'s second node is not a supply point", routeInd));
                }

                int customerCount = 0;

                for (int nodeInd = customerStart; nodeInd < route.sequence.Count; nodeInd++)
                {
                    customerCount++;
                    if (route.sequence[nodeInd].category != Model.Category.customer)
                    {
                        errorMessages.Add(string.Format("Route {0}'s Node {1} is not a customer", routeInd, nodeInd));
                    }
                }

                if (customerCount != route.load)
                {
                    errorMessages.Add(string.Format("Route {0} load and customer count don't match: load={1}, customer count={2}",
                        routeInd, route.load, customerCount));
                }

                /*if (customerCount > this.model.capacity)
                {
                    errorMessages.Add(string.Format("Route {0} has been breached", routeInd));
                }*/
            }

            const double epsilon = 0.001;

            double obj1 = this.CalculateObjective1();
            if (Math.Abs(this.objective1 - obj1) > epsilon && this.feasible)
            {
                errorMessages.Add(string.Format("Error in Objective 1: Original Value={0}, Check Value={1}", this.objective1, obj1));
            }
            
            // tsekare oti i objective 1 einai isi me to max ton max ton routes
            var maxOfRoutes = 0.0;
            foreach (var route in routes)
            {
                if (route.max > maxOfRoutes)
                {
                    maxOfRoutes = route.max;
                }
            }

            if (Math.Abs(this.objective1 - maxOfRoutes) > epsilon && this.feasible)
            {
                Console.WriteLine(this.feasible);
                errorMessages.Add(string.Format("Error in Objective 1: Doesn't match the max of all routes {0}, {1}", this.objective1, maxOfRoutes));
            }

            double obj2 = this.CalculateObjective2();
            if (Math.Abs(this.objective2 - obj2) > epsilon)
            {
                errorMessages.Add(string.Format("Error in Objective 2: Original Value={0}, Check Value={1}", this.objective2, obj2));
            }

            /* EVGENIA OTAN EXEIS ETOIMES TIS OBJECTIVES VGALE TO SXOLIO
            */
            double obj3 = this.CalculateObjective3();
            if (Math.Abs(this.objective3 - obj3) > epsilon)
            {
                errorMessages.Add(string.Format("Error in Objective 3: Original Value={0}, Check Value={1}", this.objective3, obj3));
            }

            double obj4 = this.CalculateObjective4();
            if (Math.Abs(this.objective4 - obj4) > epsilon)
            {
                errorMessages.Add(string.Format("Error in Objective 4: Original Value={0}, Check Value={1}", this.objective4, obj4));
            }


            if (errorMessages.Count > 0)
            {
                foreach (var message in errorMessages)
                {
                    Console.WriteLine(message);
                }
                return false;
            }

            return true;
        }
    }
}