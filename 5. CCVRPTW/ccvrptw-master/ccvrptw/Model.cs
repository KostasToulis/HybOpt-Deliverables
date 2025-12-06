using System.Text.RegularExpressions;
using System.Linq;
using System.IO.Enumeration;

namespace CCVRPTW
{
    public enum Objective
    {
        CUMULATIVE_DISTANCE,
        TIME_WINDOW_COMPATIBILITY,
        CUMULATIVE_SERVICE_TIMES,
        DISTANCE
    }

    public class Model
    {
        public string Name;
        public List<Node> nodes;
        public List<Node> customers;
        public Node depot;
        public Node[,] depots;
        public int vehicles;
        public int capacity;
        public double[,] distances;
        public double extraNodePenalty = 100000;
        
        public Model(string filename)
        {
            //Init
            customers = new List<Node>();
            nodes = new List<Node> ();
            Name = Path.GetFileNameWithoutExtension(filename);
            
            var logFile = File.ReadAllLines(filename);
            List<string> lines = new List<string>(logFile);

            int lineCounter = 4;
            string[] noSpaces = lines[lineCounter].Split();
            noSpaces = noSpaces.Where(val => Regex.IsMatch(val, @"^\d+$")).ToArray();
            vehicles = int.Parse(noSpaces[0]) - ProblemConfiguration.SkippedVehicles;
            capacity = int.Parse(noSpaces[1]);

            lineCounter = 9;

            for (int i = lineCounter; i < lines.Count; i++)
            {
                noSpaces = lines[i].Split();
                noSpaces = noSpaces.Where(val => Regex.IsMatch(val, @"^\d+$")).ToArray();
                int id = int.Parse(noSpaces[0]);
                double x = Double.Parse(noSpaces[1]);
                double y = Double.Parse(noSpaces[2]);
                int d = int.Parse(noSpaces[3]);

                double ws = 0;
                double we = 1000000.0;
                if (ProblemConfiguration.UseTimeWindows)
                {
                    ws = Double.Parse(noSpaces[4]);
                    we = Double.Parse(noSpaces[5]);
                }

                double st = 0.0;
                if (ProblemConfiguration.UseServiceTimes)
                    st = Double.Parse(noSpaces[6]);
                Node c = new Node(id, x, y, d, ws, we, st, false);

                if (i == 9)
                {
                    depot = c;
                    c.isDepot = true;
                }
                else
                    customers.Add(c);

                nodes.Add(c);
            }

            createDepots();
            distances = ComputeDistances();

            System.Diagnostics.Debug.Assert(depot != null);
            System.Diagnostics.Debug.Assert(depots != null);

        }

        public void createDepots()
        {
            depots = new Node[vehicles, 2];
            int depotId = nodes.Count;
            depot.pushForward = depot.windowEnd - depot.windowStart;
            Node d1, d2;
            for (int i = 0; i < vehicles; i++)
            {
                d1 = new Node(depot);
                d2 = new Node(depot);
                d1.id = depotId++;
                d2.id = depotId++;
                depots[i, 0] = d1;
                depots[i, 1] = d2;
                nodes.Add(d1);
                nodes.Add(d2);
            }
        }

        public double ComputeDistance(Node a, Node b)
        {
            return Math.Sqrt(Math.Pow((a.x - b.x), 2) + Math.Pow((a.y - b.y), 2));
        }

        public double[,] ComputeDistances()
        {
            double [,] distances = new double[nodes.Count, nodes.Count];
            for (int i = 0; i < nodes.Count; i++)
            {
                Node a = nodes[i];
                for (int j = 0; j < nodes.Count; j++)
                {
                    Node b = nodes[j];
                    distances[a.id, b.id] = ComputeDistance(a, b);
                }
            }
            return distances;
        }
        
    }

}
