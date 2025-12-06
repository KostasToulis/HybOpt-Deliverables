using ScottPlot;
using System.Globalization;

namespace CCVRPTW
{
    public class Solution
    {
        public List<Route> routes; // lista me route objects pou anaparistoun dromologia kai tis idiothtes tous
        public List<Node> unvisited; //list with non visited routes
        public double cost; // to cost mias Solution einai iso me to sum twn cost twn routes ths
        public int lastImprovedIteration; // to teleftaio iteration to promises poy ginetai beltiosh
        public int biggestIterationsGap; // to megalytero diasthma metaxi dyo kalyteron lyseon

        // constructor 1
        // arxikopoiei ena Solution object me vash to montelo.
        public Solution()
        {
            unvisited = ProblemConfiguration.model.customers.GetRange(0, ProblemConfiguration.model.customers.Count);
            cost = ProblemConfiguration.model.extraNodePenalty * ProblemConfiguration.model.customers.Count;
            routes = new List<Route>();
            lastImprovedIteration = 0;
            biggestIterationsGap = 0;

            Node dStart, dEnd;
            // gia kathe dromogolio apo ta R, arxikopoihse to sth morfh [depot, depot]
            // kathe dromologio exei to diko tou zeugos apo depots.
            for (int i = 0; i < ProblemConfiguration.model.vehicles; i++)
            {
                dStart = ProblemConfiguration.model.depots[i, 0];
                dEnd = ProblemConfiguration.model.depots[i, 1];
                routes.Add(new Route(dStart, dEnd, false));
            }
            
        }

        // constructor 2
        // to xrhsimopoioume gia na kanoume deep copy enos Solution object.
        public Solution(Solution s)
        {
            routes = new List<Route>();
            for (int i = 0; i < s.routes.Count; i++)
            {
                routes.Add(new Route(new Node(s.routes[i].sequence[0]), new Node(s.routes[i].sequence[s.routes[i].sequence.Count - 1]), false));
                routes[i].load = s.routes[i].load;
                routes[i].cost = s.routes[i].cost;
                routes[i].timeWindowPenalty = s.routes[i].timeWindowPenalty;
                routes[i].totalSlack = s.routes[i].totalSlack;
                
                for (int j = 1; j < s.routes[i].sequence.Count - 1; j++)
                {
                    routes[i].sequence.Insert(j, new Node(s.routes[i].sequence[j]));
                }
            }

            cost = s.cost;
            lastImprovedIteration = s.lastImprovedIteration;
            biggestIterationsGap = s.biggestIterationsGap;

            unvisited = new List<Node>();
            foreach (Node n in s.unvisited)
            {
                unvisited.Add(new Node(n));
            }

        }

        // upologizoume th sunolikh apostash (total distance) pou dianuetai sta dromologia enos Solution object.
        // gia kathe dromologio poso distance dianuetai apo depot se depot.
        public double ComputeDistances(Route rt, bool saveToRoute = false)
        {
            double route_distance = 0.0;
            for (int i = 0; i < rt.sequence.Count - 2; i++)
            {
                Node u = rt.sequence[i];
                Node v = rt.sequence[i + 1];
                route_distance += ProblemConfiguration.model.distances[u.id, v.id];
            }

            if (saveToRoute)
                rt.cost = route_distance;

            return route_distance;
        }

        public double ComputeDistances(bool saveToRoute = false)
        {
            double totalDistances = 0;
            for (int i = 0; i < routes.Count; i++)
            {
                totalDistances += ComputeDistances(routes[i], saveToRoute);
            }
            
            totalDistances += ProblemConfiguration.model.extraNodePenalty * (unvisited.Count);
            return totalDistances;
        }

        // upologizoume to objective Cumulative Distances gia ena dromologio.
        // ousiastika athroizoume thn sunolikh apostash pou dianuetai gia thn afiksh se kathe
        // pelath pou periexei to dromologio (dhladh eksw to depot_end).
        public double ComputeRouteCumulativeDistances(Route route, bool saveToRoute=false)
        {
            double cumulativeDistances = 0;
            double routeTravelTimes = 0;
            double routePenalties = 0;
            // den metrame thn apostash apo ton teleutaio customer sto depotEnd
            for (int i = 0; i < route.sequence.Count - 2; i++)
            {
                Node u = route.sequence[i];
                Node v = route.sequence[i + 1];
                routePenalties += (v.waitingTime < 0) ? Math.Abs(v.waitingTime) : 0;
                routeTravelTimes += ProblemConfiguration.model.distances[u.id, v.id];
                cumulativeDistances += routeTravelTimes;
            }

            if (saveToRoute)
            {
                route.cost = cumulativeDistances + FeasibleSegment.timeWindowPenaltyWeight * routePenalties;
                route.timeWindowPenalty = FeasibleSegment.timeWindowPenaltyWeight * routePenalties;
            }
                
            return cumulativeDistances + FeasibleSegment.timeWindowPenaltyWeight * routePenalties;
        }

        // upologizoume to objective Cumulative Distances gia ena Solution object.
        public double ComputeCumulativeDistances(bool saveToRoute = false)
        {
            double cumulativeDistances = 0;
            for (int i = 0; i < routes.Count; i++)
            {
                Route rt = routes[i];
                cumulativeDistances += ComputeRouteCumulativeDistances(rt, saveToRoute);
            }
            
            cumulativeDistances += ProblemConfiguration.model.extraNodePenalty * (unvisited.Count);
            
            return cumulativeDistances;
        }

        // upologizoume to objective Cumulative Service Times (tou montelou) gia ena dromologio.
        // ousiastika athroizoume ton xrono ekkinishs tou service gia kathe pelath tou dromologiou.
        public double ComputeRouteCumulativeServiceTimes(Route route)
        {
            double cumulativeServiceTimes = 0;
            double routePenalties = 0;
            for (int i = 1; i < route.sequence.Count - 1; i++)
            {
                Node v = route.sequence[i];
                routePenalties += (v.waitingTime < 0) ? Math.Abs(v.waitingTime) : 0;
                cumulativeServiceTimes += (v.arrivalTime + v.waitingTime);
            }

            return cumulativeServiceTimes + FeasibleSegment.timeWindowPenaltyWeight * routePenalties;
        }

        // upologizoume to objective Cumulative Service Times gia ena Solution object.
        public double ComputeCumulativeServiceTimes()
        {
            double cumulativeServiceTimes = 0;
            foreach (Route route in routes)
            {
                cumulativeServiceTimes += ComputeRouteCumulativeServiceTimes(route);
            }

            cumulativeServiceTimes += ProblemConfiguration.model.extraNodePenalty * (unvisited.Count);

            return cumulativeServiceTimes;
        }

        // DEN KSERW AN TO XREIAZOMASTE TELIKA.
        public double ComputeSumOfLastServiceTimes()
        {
            double sumOfLastServiceTimes = 0;
            foreach (Route route in routes)
            {
                int routeLength = route.sequence.Count;
                Node lastCustomerForRoute = route.sequence[routeLength - 2];
                sumOfLastServiceTimes += lastCustomerForRoute.arrivalTime + lastCustomerForRoute.waitingTime;
            }
            return sumOfLastServiceTimes;
        }
        
        public void Export(string filename)
        {
            File.WriteAllText(filename, ToString());
        }

        public void ExportSimple(string filename)
        {
            StringWriter sw = new StringWriter();
            sw.WriteLine(cost);
            sw.WriteLine(routes.Count);
            foreach (Route route in routes)
                sw.WriteLine(route.ToString(true));
            sw.Write("Unvisited Nodes: ");
            foreach (Node n in unvisited)
                sw.Write(n.id + " ");
            sw.Write('\n');
            File.WriteAllText(filename, sw.ToString());
        }

        public override string ToString()
        {
            StringWriter sw = new StringWriter();
            sw.WriteLine(cost);
            sw.WriteLine(routes.Count);
            foreach (Route route in routes)
                sw.Write(route);
            return sw.ToString();
        }

        public void PlotRoutes(int iter)
        {
            var plt = new Plot(800, 600);

            double[] nodesX = new double[ProblemConfiguration.model.nodes.Count];
            double[] nodesY = new double[ProblemConfiguration.model.nodes.Count];
            for (int i = 0; i < ProblemConfiguration.model.nodes.Count; i++)
            {
                nodesX[i] = ProblemConfiguration.model.nodes[i].x;
                nodesY[i] = ProblemConfiguration.model.nodes[i].y;
            }
            plt.AddScatter(nodesX, nodesY, lineWidth: 0);

            foreach (Route rt in routes)
            {
                double[] rtX = new double[rt.sequence.Count];
                double[] rtY = new double[rt.sequence.Count];
                for (int i = 0; i < rt.sequence.Count; i++)
                {
                    rtX[i] = rt.sequence[i].x;
                    rtY[i] = rt.sequence[i].y;
                }
                plt.AddScatter(rtX, rtY, markerSize: 0);
            }
            plt.SaveFig($"../../../../plot_{iter}.png");
        }

        public void PlotCostProgression(List<double> progression, double best, int iter, int restart)
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
        }

        public bool IsFeasible()
        {
            if (unvisited.Count > 0)
                return false;
            
            foreach (Route rt in routes)
            {
                if (rt.timeWindowPenalty > 0)
                    return false;
            }
            return true;
        }

        // elegxoume me vash ena sugkekrimeno objetive
        // an mia lush einai egkurh. An dhladh h ekkinish tou service se kathe pelath
        // ginetai entos twn time windows tou, kathws kai an to cost pou exei upologistei
        // kata thn kataskeuh ths lushs einai consistent me to cost pou upologizetai analutika.
        public bool CheckSolution(Objective objective)
        {
            bool status = true;
            //Check served node number
            int served_nodes = 0;
            foreach (Route rt in routes)
                served_nodes += rt.sequence.Count - 2;

            
            if (served_nodes + unvisited.Count != ProblemConfiguration.model.customers.Count)
            {
                Console.WriteLine("Unserved nodes");
                status = false;
            }
                
            
            for (int i = 0; i < routes.Count; i++)
            {
                Route r = routes[i];
                for (int j = 0; j < r.sequence.Count - 1; j++)
                {
                    Node n = r.sequence[j];
                    double service_start = n.arrivalTime + n.waitingTime;
                    if (!(service_start >= n.windowStart && service_start <= n.windowEnd)) {
                        Console.WriteLine($"{n.id} window infeasible");
                        status = false;
                        //throw new Exception("check node feasibility exception");
                    }
                    Node k = r.sequence[j + 1];
                    double distance = ProblemConfiguration.model.distances[n.id, k.id];
                    double kArrTime = service_start + n.serviceTime + distance;

                    if (Math.Abs(kArrTime - k.arrivalTime) > 0.001)
                    {
                        Console.WriteLine("ERROR IN Arrival time");
                        status = false;
                    }
                }

                //Check route cost
                if (objective == Objective.CUMULATIVE_DISTANCE)
                {
                    if (Math.Abs(r.cost - ComputeRouteCumulativeDistances(r, false)) > 1e-5)
                    {
                        Console.WriteLine("Route Cost Mismatch");
                        ComputeRouteCumulativeDistances(r, false);
                        
                        status = false;
                    }
                }
                else if(objective == Objective.CUMULATIVE_SERVICE_TIMES)
                {
                    if (Math.Abs(r.cost - ComputeRouteCumulativeServiceTimes(r)) > 1e-5)
                    {
                        Console.WriteLine("route cost   " + r.cost);
                        Console.WriteLine("YPOLOGIZEI " + ComputeRouteCumulativeServiceTimes(r));
                        throw new Exception("route cost mismatch gamo");

                        Console.WriteLine("Route Cost Mismatch");
                        status = false;
                    }
                }
                else
                {
                    throw new NotImplementedException();
                }

            }

            if (objective == Objective.CUMULATIVE_DISTANCE)
            {
                if (Math.Abs(cost - ComputeCumulativeDistances(false)) > 1e-5)
                {
                    Console.WriteLine("Solution Cost Mismatch");
                    ComputeCumulativeDistances(false);
                    status = false;
                }
            }
            else if (objective == Objective.CUMULATIVE_SERVICE_TIMES)
            {
                if (Math.Abs(cost - ComputeCumulativeServiceTimes()) > 1e-5)
                {
                    Console.WriteLine("Solution Cost Mismatch");
                    status = false;
                }
            }
                
            return status;
        }

        public static Solution ParseSolution(string path, string instance_path)
        {
            ProblemConfiguration.model = new Model(instance_path);
            Solver s = new Solver(1);

            using (StreamReader file = new StreamReader(path))
            {
                string ln;
                s.solution.cost = double.Parse(file.ReadLine(), CultureInfo.InvariantCulture);
                int solution_vehicles = Convert.ToInt32(file.ReadLine());
                s.solution.routes = new List<Route>();
                for (int i = 0; i < solution_vehicles; i++)
                {
                    Node dStart, dEnd;
                    dStart = ProblemConfiguration.model.depots[i, 0];
                    dEnd = ProblemConfiguration.model.depots[i, 1];
                    s.solution.routes.Add(new Route(dStart, dEnd, false));
                }
                
                for (int i = 0; i < solution_vehicles; i++)
                {
                    ln = file.ReadLine();
                    
                    string[] node_ids = ln.Split(" ");

                    for (int j = 0; j < node_ids.Length - 1; j++)
                    {
                        int id = Convert.ToInt32(node_ids[j]);
                        Insertion insertion = new Insertion();
                        insertion.StoreMove(ProblemConfiguration.model.nodes[id], i, j, 0);
                        s.ApplyInsertion(insertion);
                    }

                }

                file.Close();
                //Console.WriteLine(s.solution.ComputeCumulativeDistances());
            }
            return s.solution;
        }

    }



}
