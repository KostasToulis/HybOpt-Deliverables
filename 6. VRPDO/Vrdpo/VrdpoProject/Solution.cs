using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using System.Xml;
using OxyPlot;
//using Newtonsoft.Json;

namespace VrdpoProject
{

    public class Solution
    {
        private double duration;
        private double cost;
        private int repetition;
        private int restart;
        private List<Route> routes;
        private double[,] timeMatrix;
        private double[,] distanceMatrix;
        private int cap;
        private Location depot;
        private List<Option> options = new();
        private double[,] promises;
        private List<Customer> customers = new();
        private Dictionary<int, List<Option>> optionsPerCustomer;
        private Dictionary<int, List<int>> optionsPrioritiesPerCustomer;
        private string lastMove;
        private double solutionUtilizationMetric;
        private double ratioCombinedMoveCost;
        private int lowerBoundRoutes;

        public Solution()
        {
            InstanceReader model = new();
            model.BuildModel();
            //model.ExportToJson("./vrpdo_data.json");

            this.duration = 0;
            this.cost = 0;
            this.routes = new List<Route>();
            this.DistanceMatrix = model.DistanceMatrix;
            this.TimeMatrix = model.TimeMatrix;
            this.Cap = model.Cap;
            this.Depot = model.Depot;
            this.Options = model.Options;
            this.Customers = model.AllCustomers;
            this.Promises = new double[Options.Count + 1, Options.Count + 1];
            this.optionsPerCustomer = model.OptionsPerCustomer;
            this.optionsPrioritiesPerCustomer = model.OptionsPrioritiesPerCustomer;
            this.Repetition = repetition;
            this.SolutionUtilizationMetric = 0;
            this.ratioCombinedMoveCost = 0;
            this.lowerBoundRoutes = (int)Math.Ceiling((double)model.AllCustomers.Sum(customer => customer.Dem) / model.Cap);
            for (int i = 0; i < Math.Pow(Options.Count + 1, 2); i++) promises[i % (Options.Count + 1), i / (Options.Count + 1)] = double.MaxValue;
        }

        public Solution(double duration, double cost, List<Route> routes, double[,] distanceMatrix,
            double[,] timeMatrix, int cap, Location depot, List<Option> options, double[,] promises, List<Customer> customers, Dictionary<int, List<Option>> optionsPerCustomer,
            Dictionary<int, List<int>> optionsPrioritiesPerCustomer, int repetition, double solutionUtilizationMetric, double ratioCombinedMoveCost, int lowerBoundRoutes)
        {
            this.Duration = duration;
            this.Cost = cost;
            this.RatioCombinedMoveCost = ratioCombinedMoveCost;
            this.SolutionUtilizationMetric = solutionUtilizationMetric;
            this.LowerBoundRoutes = lowerBoundRoutes;
            List<Location> clonedLocations = new List<Location>();
            List<Option> clonedOptions = new List<Option>();
            List<Customer> clonedCustomers = new List<Customer>();
            List<Route> clonedRoutes = new List<Route>();
            foreach (Option option in options)
            {
                if (!(clonedLocations.Select(x => x.Id).ToList()).Contains(option.Location.Id))
                {
                    clonedLocations.Add((Location)option.Location.Clone());
                }
                var clonedloc = clonedLocations.SingleOrDefault(x => x.Id == option.Location.Id);
                clonedOptions.Add((Option)option.Clone(clonedloc));
            }
            foreach (Customer customer in customers)
            {
                List<Option> customersOptions = clonedOptions.Where(x => x.Cust.Id == customer.Id).ToList();
                clonedCustomers.Add((Customer)customer.Clone(customersOptions));
            }
            foreach (Option option in clonedOptions)
            {
                option.Cust = clonedCustomers.SingleOrDefault(x => x.Id == option.Cust.Id);
            }
            
            
            foreach (Route rt in routes)
            {
                Route clonedRoute = new Route(rt);
                for (int i = 1; i < rt.SequenceOfCustomers.Count-1; i++)
                {
                    if (i == 0 || i == rt.SequenceOfCustomers.Count - 1)
                    {
                        clonedRoute.SequenceOfLocations[i] = (Location)depot.Clone();
                        clonedRoute.SequenceOfCustomers[i] = new Customer(1000, 0, true);
                        clonedRoute.SequenceOfOptions[i] = (Option)rt.SequenceOfOptions[0].Clone(depot);
                    } else
                    {
                        clonedRoute.SequenceOfLocations[i] = (Location)clonedLocations.Where(x => x.Id == rt.SequenceOfLocations[i].Id).ToList()[0];
                        clonedRoute.SequenceOfCustomers[i] = (Customer)clonedCustomers.Where(x => x.Id == rt.SequenceOfCustomers[i].Id).ToList()[0];
                        clonedRoute.SequenceOfOptions[i] = (Option)clonedOptions.Where(x => x.Id == rt.SequenceOfOptions[i].Id).ToList()[0];
                    }
                }

                clonedRoutes.Add(clonedRoute);
            }

            this.DistanceMatrix = distanceMatrix;
            this.TimeMatrix = timeMatrix;
            this.Cap = cap;
            this.Depot = depot;
            this.Options = clonedOptions;
            this.Customers = clonedCustomers;
            this.Routes = clonedRoutes;
            this.Promises = promises;
            this.optionsPerCustomer = optionsPerCustomer;
            this.optionsPrioritiesPerCustomer = optionsPrioritiesPerCustomer;
            this.Repetition = repetition;
        }


        public Solution DeepCopy(Solution sol)
        {
            Solution deepCopySol = new Solution(sol.Duration, sol.Cost, sol.Routes,
                sol.DistanceMatrix, sol.TimeMatrix, sol.Cap, sol.Depot, sol.Options,
                sol.Promises, sol.Customers, sol.optionsPerCustomer,
                sol.optionsPrioritiesPerCustomer, sol.Repetition, sol.SolutionUtilizationMetric, sol.RatioCombinedMoveCost, sol.LowerBoundRoutes);
            return deepCopySol;
        }

        public double Duration { get => duration; set => duration = value; }
        public double Cost { get => cost; set => cost = value; }
        internal List<Route> Routes { get => routes; set => routes = value; }
        public double[,] DistanceMatrix { get => distanceMatrix; set => distanceMatrix = value; }
        public double[,] TimeMatrix { get => timeMatrix; set => timeMatrix = value; }
        public List<Option> Options { get => options; set => options = value; }
        public double[,] Promises { get => promises; set => promises = value; }
        public List<Customer> Customers { get => customers; set => customers = value; }
        public Location Depot { get => depot; set => depot = value; }
        public int Cap { get => cap; set => cap = value; }
        public Dictionary<int, List<Option>> OptionsPerCustomer { get => optionsPerCustomer; set => optionsPerCustomer = value; }
        public Dictionary<int, List<int>> OptionsPrioritiesPerCustomer { get => optionsPrioritiesPerCustomer; set => optionsPrioritiesPerCustomer = value; }
        public int Repetition { get => repetition; set => repetition = value; }
        public int Restart { get => restart; set => restart = value; }
        public string LastMove { get => lastMove; set => lastMove = value; }
        public double SolutionUtilizationMetric { get => solutionUtilizationMetric; set => solutionUtilizationMetric = value; }
        public double RatioCombinedMoveCost { get => ratioCombinedMoveCost; set => ratioCombinedMoveCost = value; }
        public int LowerBoundRoutes { get => lowerBoundRoutes; set => lowerBoundRoutes = value; }

        public double CalculateDistance(Location n1, Location n2)
        {
            if (n1.Id > n2.Id)
            {
                return DistanceMatrix[n2.Id, n1.Id - n2.Id];
            }
            else
            {
                return DistanceMatrix[n1.Id, n2.Id - n1.Id];
            }
        }

        public double CalculateTime(Location n1, Location n2)
        {
            if (n1.Id > n2.Id)
            {
                return TimeMatrix[n2.Id, n1.Id - n2.Id];
            }
            else
            {
                return TimeMatrix[n1.Id, n2.Id - n1.Id];
            }
        }

        public void InitPromises()
        {
            for (int i = 0; i < Math.Pow(Options.Count + 1, 2); i++) Promises[i % (Options.Count + 1), i / (Options.Count + 1)] = double.MaxValue;
        }

        
        public double[] RespectsTimeWindow(Route rt, int loc, Location l)
        {
            /// loc: the position to be placed after
            double lat = Math.Min(rt.SequenceOfLat[loc + 1] - CalculateTime(l, rt.SequenceOfLocations[loc + 1]) - l.ServiceTime, l.Due - l.ServiceTime);
            if (l.Id == rt.SequenceOfLocations[loc].Id)
            {
                lat += l.ServiceTime;
            }

            if (l.Id == rt.SequenceOfLocations[loc + 1].Id)
            {
                lat = Math.Min(rt.SequenceOfLat[loc + 1] - CalculateTime(l, rt.SequenceOfLocations[loc + 1]), l.Due - l.ServiceTime); //??

               if (l.Id == rt.SequenceOfLocations[loc].Id)
                {
                    lat = Math.Min(rt.SequenceOfLat[loc + 1] - CalculateTime(l, rt.SequenceOfLocations[loc + 1]), l.Due);
                }
            }

            double ect = Math.Max(rt.SequenceOfEct[loc] + CalculateTime(rt.SequenceOfLocations[loc], l) + l.ServiceTime, l.Ready + l.ServiceTime);
            if (l.Id == rt.SequenceOfLocations[loc].Id)
            {
                ect -= l.ServiceTime;
            }
            double lat2 = 0;
            for (int j = loc; j > 0; j--)
            {
                if (loc == 1) { continue; };
                if (j == loc)
                {
                    lat2 = Math.Min(rt.SequenceOfLocations[j].Due - rt.SequenceOfLocations[j].ServiceTime,
                                               lat - CalculateTime(rt.SequenceOfLocations[j], l)
                                               - rt.SequenceOfLocations[j].ServiceTime);

                    if (j > 1 && rt.SequenceOfLocations[j-1].Id == rt.SequenceOfLocations[j].Id)
                    {
                        lat2 += rt.SequenceOfLocations[j].ServiceTime;
                    }

                } else
                {
                    lat2 = Math.Min(rt.SequenceOfLocations[j].Due - rt.SequenceOfLocations[j].ServiceTime,
                                               lat2 - CalculateTime(rt.SequenceOfLocations[j], rt.SequenceOfLocations[j + 1])
                                               - rt.SequenceOfLocations[j].ServiceTime);


                    if (j > 1 && rt.SequenceOfLocations[j - 1].Id == rt.SequenceOfLocations[j].Id)
                    {
                        lat2 += rt.SequenceOfLocations[j].ServiceTime;

                    }
                }

                if (lat2 < rt.SequenceOfEct[j]) { 
                    return new double[] { 1, 0 }; 
                };
            }

            double[] tw = new double[] { ect, lat };
            return tw;
        }

        public Tuple<bool, double[], double[]> RespectsTimeWindow2(Route rt, int loc, Location location)
        {
           
            double[] ects = new double[rt.SequenceOfLocations.Count + 1];
            double[] lats = Enumerable.Repeat((double)7200, rt.SequenceOfLocations.Count + 1).ToArray();
            int k = 1;

            for (int i = 1; i < rt.SequenceOfLocations.Count + 1; i++)
            {
                if (i == loc + 1)
                {
                    k--;
                    ects[i] = Math.Max(location.Ready + location.ServiceTime,
                                               ects[i - 1] + CalculateTime(location, rt.SequenceOfLocations[k])
                                               + location.ServiceTime);

                    if (i != 1 && (location.Id == rt.SequenceOfLocations[k].Id))
                    {
                        ects[i] -= (location.ServiceTime);
                    }
                }
                else if (i == loc + 2)
                {
                    ects[i] = Math.Max(rt.SequenceOfLocations[k].Ready + rt.SequenceOfLocations[k].ServiceTime,
                                               ects[i - 1] + CalculateTime(rt.SequenceOfLocations[k], location)
                                               + rt.SequenceOfLocations[k].ServiceTime);

                    if ((location.Id == rt.SequenceOfLocations[k].Id))
                    {
                        ects[i] -= (rt.SequenceOfLocations[k].ServiceTime);
                    }
                }
                else
                {
                    ects[i] = Math.Max(rt.SequenceOfLocations[k].Ready + rt.SequenceOfLocations[k].ServiceTime,
                                                   ects[i - 1] + CalculateTime(rt.SequenceOfLocations[k], rt.SequenceOfLocations[k - 1])
                                                   + rt.SequenceOfLocations[k].ServiceTime);

                    if (rt.SequenceOfLocations[k - 1] == rt.SequenceOfLocations[k])
                    {
                        ects[i] -= (rt.SequenceOfLocations[k].ServiceTime);
                    }
                }
                k++;
            }
            k = rt.SequenceOfLocations.Count - 2;
            for (int j = rt.SequenceOfLocations.Count - 1; j > -1; j--)
            {
                if (j == loc + 1)
                {
                    k++;
                    lats[j] = Math.Min(location.Due - location.ServiceTime,
                                               lats[j + 1] - CalculateTime(location, rt.SequenceOfLocations[k])
                                               - location.ServiceTime);

                    if (rt.SequenceOfLocations[k].Id == location.Id)
                    {
                        lats[j + 1] += (rt.SequenceOfLocations[k].ServiceTime);//- 20);

                        lats[j] = Math.Min(location.Due - location.ServiceTime,
                                               lats[j + 1] - CalculateTime(location, rt.SequenceOfLocations[k + 1])
                                               - location.ServiceTime);
                    }
                }
                else if (j == loc)
                {
                    lats[j] = Math.Min(rt.SequenceOfLocations[k].Due - rt.SequenceOfLocations[k].ServiceTime,
                                               lats[j + 1] - CalculateTime(rt.SequenceOfLocations[k], location)
                                               - rt.SequenceOfLocations[k].ServiceTime);

                    if (location.Id == rt.SequenceOfLocations[k].Id)
                    {
                        lats[j + 1] += (location.ServiceTime);

                        lats[j] = Math.Min(rt.SequenceOfLocations[k].Due - rt.SequenceOfLocations[k].ServiceTime,
                                               lats[j + 1] - CalculateTime(rt.SequenceOfLocations[k], location)
                                               - rt.SequenceOfLocations[k].ServiceTime);
                    }
                }
                else
                {
                    lats[j] = Math.Min(rt.SequenceOfLocations[k].Due - rt.SequenceOfLocations[k].ServiceTime,
                                                   lats[j + 1] - CalculateTime(rt.SequenceOfLocations[k], rt.SequenceOfLocations[k + 1])
                                                   - rt.SequenceOfLocations[k].ServiceTime);

                    if (rt.SequenceOfLocations[k + 1].Id == rt.SequenceOfLocations[k].Id)
                    {
                        lats[j + 1] += (rt.SequenceOfLocations[k + 1].ServiceTime);

                        lats[j] = Math.Min(rt.SequenceOfLocations[k].Due - rt.SequenceOfLocations[k].ServiceTime,
                                                   lats[j + 1] - CalculateTime(rt.SequenceOfLocations[k], rt.SequenceOfLocations[k + 1])
                                                   - rt.SequenceOfLocations[k].ServiceTime);
                    }
                }
                k--;
            }

            bool feasible = ects.Zip(lats, (a, b) => a < b).All(x => x);

            return new Tuple<bool, double[], double[]>(feasible, ects, lats);
            
        }

        /// <summary>
        /// Calculates the time windows of the route <paramref>rt</paramref> for
        /// all the <paramref>locations</paramref> to be visited after the specified
        /// index <paramref>loc</paramref>
        /// </summary>
        double[] tw;
        public Tuple<bool, double[], double[]> RespectsTimeWindow(Route rt, int loc, List<Location> locations)
        {
            tw = RespectsTimeWindow(rt, loc, locations.First());
            if (tw[0] > tw[1])
            {
                return new Tuple<bool, double[], double[]>(false, new double[1], new double[1]);
            }
            List<double> ects = new();
            List<double> lats = new();
            Route tempRoute = new(44, 150, depot);
            tempRoute.SequenceOfLocations = rt.SequenceOfLocations.Take(loc + 1).ToList();
            tempRoute.SequenceOfLocations.AddRange(locations);
            tempRoute.SequenceOfLat.AddRange(Enumerable.Repeat((double)7200, tempRoute.SequenceOfLocations.Count - 2).ToList());
            for (int i = 1; i < tempRoute.SequenceOfLocations.Count; i++)
            {
                double ect = Math.Max(tempRoute.SequenceOfLocations[i].Ready + tempRoute.SequenceOfLocations[i].ServiceTime,
                                               tempRoute.SequenceOfEct[i - 1] + CalculateTime(tempRoute.SequenceOfLocations[i], tempRoute.SequenceOfLocations[i - 1])
                                               + tempRoute.SequenceOfLocations[i].ServiceTime);
                if (tempRoute.SequenceOfLocations[i - 1] == tempRoute.SequenceOfLocations[i])
                {
                    tempRoute.SequenceOfEct[i] -= (tempRoute.SequenceOfLocations[i].ServiceTime);// - 20);
                }
                tempRoute.SequenceOfEct.Insert(tempRoute.SequenceOfEct.Count - 1, ect);
            }
            tempRoute.SequenceOfEct.RemoveAt(tempRoute.SequenceOfEct.Count - 1);
            for (int j = tempRoute.SequenceOfLocations.Count - 2; j > -1; j--)
            {
                double lat = Math.Min(tempRoute.SequenceOfLocations[j].Due - tempRoute.SequenceOfLocations[j].ServiceTime,
                                               tempRoute.SequenceOfLat[j + 1] - CalculateTime(tempRoute.SequenceOfLocations[j], tempRoute.SequenceOfLocations[j + 1])
                                               - tempRoute.SequenceOfLocations[j].ServiceTime);
                if (tempRoute.SequenceOfLocations[j + 1] == tempRoute.SequenceOfLocations[j])
                {
                    tempRoute.SequenceOfLat[j + 1] += (tempRoute.SequenceOfLocations[j + 1].ServiceTime);//- 20);

                    lat = Math.Min(tempRoute.SequenceOfLocations[j].Due - tempRoute.SequenceOfLocations[j].ServiceTime,
                                               tempRoute.SequenceOfLat[j + 1] - CalculateTime(tempRoute.SequenceOfLocations[j], tempRoute.SequenceOfLocations[j + 1])
                                               - tempRoute.SequenceOfLocations[j].ServiceTime);
                }
                tempRoute.SequenceOfLat.RemoveAt(0);
                tempRoute.SequenceOfLat.Insert(j, lat);
            }
            //tempRoute.SequenceOfLat.RemoveAt(0);
            ects = tempRoute.SequenceOfEct.ToList();
            lats = tempRoute.SequenceOfLat.ToList();
            bool xs = !lats.SequenceEqual(lats.OrderBy(x => x));
            if (!ects.SequenceEqual(ects.OrderBy(x => x)) || !lats.SequenceEqual(lats.OrderBy(x => x))
                || ects.Last() > 7200)
            {
                return new Tuple<bool, double[], double[]>(false, tempRoute.SequenceOfEct.ToArray(), tempRoute.SequenceOfLat.ToArray());
            }
            else
            {
                bool feasible = true;
                for (int i = 0; i < tempRoute.SequenceOfEct.Count; i++)
                {
                    if (tempRoute.SequenceOfEct[i] > tempRoute.SequenceOfLat[i])
                    {
                        feasible = false;
                    }
                }
                return new Tuple<bool, double[], double[]>(feasible, tempRoute.SequenceOfEct.ToArray(), tempRoute.SequenceOfLat.ToArray());
            }
        }

        public void UpdateTimes(Route rt)
        {
            for (int i = 1; i < rt.SequenceOfLocations.Count; i++)
            {
                rt.SequenceOfEct[i] = Math.Max(rt.SequenceOfLocations[i].Ready + rt.SequenceOfLocations[i].ServiceTime,
                                               rt.SequenceOfEct[i - 1] + CalculateTime(rt.SequenceOfLocations[i], rt.SequenceOfLocations[i - 1])
                                               + rt.SequenceOfLocations[i].ServiceTime);
                if (rt.SequenceOfLocations[i - 1] == rt.SequenceOfLocations[i])
                {
                    rt.SequenceOfEct[i] -= (rt.SequenceOfLocations[i].ServiceTime);
                }
            }

            for (int j = rt.SequenceOfLocations.Count - 2; j > -1; j--)
            {
                rt.SequenceOfLat[j] = Math.Min(rt.SequenceOfLocations[j].Due - rt.SequenceOfLocations[j].ServiceTime,
                                               rt.SequenceOfLat[j + 1] - CalculateTime(rt.SequenceOfLocations[j], rt.SequenceOfLocations[j + 1])
                                               - rt.SequenceOfLocations[j].ServiceTime);
                if (rt.SequenceOfLocations[j + 1] == rt.SequenceOfLocations[j])
                {
                    rt.SequenceOfLat[j + 1] += (rt.SequenceOfLocations[j + 1].ServiceTime);

                    rt.SequenceOfLat[j] = Math.Min(rt.SequenceOfLocations[j].Due - rt.SequenceOfLocations[j].ServiceTime,
                                               rt.SequenceOfLat[j + 1] - CalculateTime(rt.SequenceOfLocations[j], rt.SequenceOfLocations[j + 1])
                                               - rt.SequenceOfLocations[j].ServiceTime);
                }
            }
        }

        public bool CalculateTimes(Route rt)
        {
            double totalTime = 0;
            for (int i = 0; i < rt.SequenceOfLocations.Count - 1; i++)
            {
                totalTime += CalculateTime(rt.SequenceOfLocations[i], rt.SequenceOfLocations[i + 1]);

                if (rt.SequenceOfLocations[i + 1].Ready > totalTime)
                {
                    totalTime = rt.SequenceOfLocations[i + 1].Ready;
                }

                if (rt.SequenceOfLocations[i].Id != rt.SequenceOfLocations[i+1].Id)
                {
                    totalTime += rt.SequenceOfLocations[i + 1].ServiceTime;
                }

                if (!(totalTime <= rt.SequenceOfLocations[i + 1].Due))
                {
                    return false;
                }
            }
            return true;
        }

        public double CalculateUtilizationMetric()
        {
            double utilizationMetric = 0;
            foreach (Route rt in Routes)
            {
                //Console.WriteLine(rt.RouteUtilizationMetric);
                utilizationMetric += rt.RouteUtilizationMetric;
            }
            return utilizationMetric;
        }

        public bool CheckEverything(Solution sol)
        {
            bool feasible;
            Dictionary<int, int> timesVisited = new Dictionary<int, int>();
            foreach (Route route in sol.Routes)
            {
                feasible = CheckRouteFeasibility(route);
                if (!feasible)
                {
                    return false;
                }
                foreach (Location location in route.SequenceOfLocations)
                {
                    if (!timesVisited.ContainsKey(location.Id))
                    {
                        timesVisited.Add(location.Id, 0);
                    }
                    timesVisited[location.Id] += 1;
                    if (location.Type == 1 && location.MaxCap < timesVisited[location.Id])
                    {
                        Console.WriteLine("Shared location exceeds max capacity!");
                        return false;
                    }
                }
                if (sol.Customers.Where(x => x.IsRouted).ToList().Count != sol.Customers.Count)
                {
                    return false;
                }

            }
            foreach (Route rt in sol.routes)
            {
                foreach (Location location in rt.SequenceOfLocations)
                {
                    if (location.Type == 1 && location.Cap != timesVisited[location.Id])
                    {
                        Console.WriteLine("Location {0} capacity is wrong (solution may be feasible)!", location.Id);
                        
                        return false;
                    }
                    else if (location.Type == 2 && timesVisited[location.Id] > 1)
                    {
                        Console.WriteLine("Private location is used two times!");
                        return false;
                    }
                }
            }
            return true;
        }

        public bool CheckRouteFeasibility(Route rt)
        {
            int totalCapacity = 0;
            bool timeWindowFeasibility = true;
            bool depotFeasibility = true;
            bool costFeasibility = true;
            double cost = 0;
            for (int i = 0; i < rt.SequenceOfOptions.Count - 1; i++)
            {
                Option currentOpt = rt.SequenceOfOptions[i];
                Option nextOpt = rt.SequenceOfOptions[i + 1];
                bool tw = CalculateTimes(rt);
                if (rt.SequenceOfEct[i + 1] > rt.SequenceOfLat[i + 1])
                {
                    Console.WriteLine("Time Window Feasibility Error");
                    timeWindowFeasibility = false;
                }
                if (!tw)
                {
                    Console.WriteLine("Time Window Feasibility Error");
                    timeWindowFeasibility = false;
                }
                if (currentOpt.Location.Type == 0)
                {
                    if (i + 1 != rt.SequenceOfLocations.Count - 1 && i != 0)
                    {
                        Console.WriteLine("Depot Feasibility Error");
                        depotFeasibility = false;
                    }
                }
                cost += CalculateDistance(rt.SequenceOfOptions[i].Location, nextOpt.Location);
            }
            if (Math.Abs(cost - rt.Cost) > 0.001)
            {
                Console.WriteLine("Cost Feasibility Error");
                costFeasibility = false;
            }
            for (int i = 0; i < rt.SequenceOfCustomers.Count; i++)
            {
                totalCapacity += rt.SequenceOfCustomers[i].Dem;
            }
            bool capacityFeasibility = (totalCapacity > rt.Capacity) ? false : true;
            if (!capacityFeasibility)
            {
                Console.WriteLine("Route Capacity Feasibility Error");
            }
            return (timeWindowFeasibility && capacityFeasibility && depotFeasibility && costFeasibility);
        }

        public void RemoveEmptyRoutes()
        {
            this.Routes.RemoveAll(rt => rt.Load == 0);
        }

        //public void ExportToJson(string filePath)
        //{
        //    List<object> routeDataList = new List<object>();
        //    foreach (Route rt in Routes)
        //    {
        //        // Get JSON string for each route and parse it to an object
        //        string routeJson = rt.ExportToJson(null);
        //        var routeData = JsonConvert.DeserializeObject<object>(routeJson);
        //        routeDataList.Add(routeData);
        //    }

        //    var solutionData = new
        //    {
        //        Cost = this.Cost,
        //        Routes = routeDataList
        //    };

        //    // Serialize the dto object to JSON
        //    string json = JsonConvert.SerializeObject(solutionData, Newtonsoft.Json.Formatting.Indented);

        //    // Write JSON to file
        //    File.WriteAllText(filePath, json);
        //}
    }
}

