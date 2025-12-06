using System.Text.RegularExpressions;
using System.Globalization;

namespace CHRVRP
{
    public class Model
    {
        public string name;
        public KPI mainKPI;
        public List<Node> nodes;
        public List<Node> customers;
        public List<Node> supply;
        public List<Node> depots;
        public int vehicles;
        public int capacity;
        public double[,] distances;
        public double averageDistanceToAllNodes;
        public Random random;

        public enum Category
        {
            customer,
            supply,
            depot
        }
        public enum KPI
        {
            kpi1=1,
            kpi2,
            kpi3,
            kpi4
        }

        public Model(string filename, int seed, int mainKPI, float p)
        {
            //Init
            nodes = new List<Node>();
            customers = new List<Node>();
            supply = new List<Node>();
            depots = new List<Node>();
            random = new Random(seed);
            name = Path.GetFileNameWithoutExtension(filename);
            this.mainKPI = (KPI) mainKPI;

            this.ReadInstance(filename);
            this.DistributeNodes();

            distances = ComputeDistances();
            AverageDistanceToAllNodes();
            FindCustomersNearestNeighbors(p);
        }

        private void AverageDistanceToAllNodes()
        {
            foreach (var currentCustomer in customers)
            {
                var sum = 0.0;

                foreach (var otherCustomer in customers)
                {
                    if (currentCustomer == otherCustomer)
                    {
                        continue;
                    }

                    sum += distances[currentCustomer.serialNumber, otherCustomer.serialNumber];
                }

                currentCustomer.averageDistanceToAllNodes = sum / (customers.Count - 1);
            }
            
        }

        private void FindCustomersNearestNeighbors(double p)
        {
            var distancesForNode = new List<(double, Node)>(customers.Count);
            foreach (var customer in customers)
            {
                distancesForNode.Clear();
                foreach (var node in customers)
                {
                    distancesForNode.Add((distances[customer.serialNumber, node.serialNumber], node));
                }

                distancesForNode = distancesForNode.OrderBy(x => x.Item1).ToList();
                foreach (var (dist, node) in distancesForNode)
                {
                    if (node == customer)
                    {
                        continue;
                    }

                    if (dist < p * node.averageDistanceToAllNodes)
                    {
                        customer.nearestNodes.Add(node);
                    }
                    else
                    {
                        break;
                    }
                }
            }
            
        }


        public void ReadInstance(string textFile)
        {
            Regex vRegex = new Regex("^V\\d+");
            Regex sRegex = new Regex("^S\\d+");
            Regex dRegex = new Regex("^D\\d+");
            Regex idRegex = new Regex("^[VSD]\\d+");
            Regex floatExtract = new Regex("^\\d+\\.\\d+");
            Regex capacityExtract = new Regex("[Y]\\d+");
            CultureInfo usCulture = new CultureInfo("en-US");
            var fileLines = File.ReadAllLines(textFile);
            int serial = 0;

            if (capacityExtract.IsMatch(this.name))
            {
                this.capacity = int.Parse(capacityExtract.Match(this.name).Value.Substring(1));  // to 1 simainei oti afinei ektos to Y
            }

            foreach (var line in fileLines)
            {
                Node newNode;
                Category category;

                if (dRegex.IsMatch(line))
                {
                    category = Category.customer;
                }
                else if (sRegex.IsMatch(line))
                {
                    category = Category.supply;
                }
                else if (vRegex.IsMatch(line))
                {
                    category = Category.depot; 
                }
                else
                {
                    continue;  // gets rid of the first lines
                }

                string[] noSpaces = line.Split(" ");

                string nodeID = idRegex.Match(noSpaces[0]).ToString();
                double xCoor = double.Parse(floatExtract.Match(noSpaces[1]).ToString(), usCulture);
                double yCoor = double.Parse(floatExtract.Match(noSpaces[2]).ToString(), usCulture);

                newNode = new Node(nodeID, serial, xCoor, yCoor, category);

                this.nodes.Add(newNode);
                serial++;
                // Console.WriteLine(newNode.ToString());
            }
        }

        public void DistributeNodes()
        {
            foreach (var node in nodes)
            {
                if (node.category.Equals(Category.depot))
                {
                    this.depots.Add(node);
                }
                else if (node.category.Equals(Category.supply))
                {
                    this.supply.Add(node);
                }
                else if (node.category.Equals(Category.customer))
                {
                    this.customers.Add(node);
                }
            }
        }

        public double ComputeDistance(Node a, Node b)
        {
            return Math.Sqrt(Math.Pow((a.x - b.x), 2) + Math.Pow((a.y - b.y), 2));
        }

        public double[,] ComputeDistances()
        {
            double[,] distances = new double[nodes.Count, nodes.Count];
            for (int i = 0; i < nodes.Count; i++)
            {
                Node a = nodes[i];
                for (int j = 0; j < nodes.Count; j++)
                {
                    Node b = nodes[j];
                    distances[a.serialNumber, b.serialNumber] = ComputeDistance(a, b);
                }
            }
            return distances;
        }

    }

}
