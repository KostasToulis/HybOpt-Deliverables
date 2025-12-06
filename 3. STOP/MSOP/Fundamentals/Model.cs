using System;
using System.Collections.Generic;
using System.Linq;

// class Model
namespace MSOP.Fundamentals
{
    public class Model
    {
        public string dataset_name;
        public int dataset_tvalue;
        public int dataset_pvalue;


        public int node_crowd;
        public int set_crowd;
        public int vehicle_number;
        public double t_max;
        public double total_available_profit;
        public Node depot;
        public List<Node> nodes;
        public List<Set> sets;
        //public Arc[,] arcs;
        public int[,] dist_matrix;
        public double min_profit_difference;
        public double max_profit_difference;
        public double min_dist_difference;
        public double max_dist_difference;

        public Random r;

        public static Model model; // the model of the problem

        public Model(string dataset_name, int node_crowd, int set_crowd, int vehicle_number, double t_max, Node depot, List<Node> nodes, List<Set> sets)
        {
            this.dataset_name = dataset_name;
            this.node_crowd = node_crowd;
            this.set_crowd = set_crowd;
            this.vehicle_number = vehicle_number;
            this.t_max = t_max;
            this.depot = depot;
            this.nodes = nodes;
            this.sets = sets;
        }

        public void Build(int seed, bool unit_clusters)
        {

            this.r = new Random(seed);

            int tIndex = dataset_name.IndexOf("_T");
            dataset_tvalue = int.Parse(dataset_name.Substring(tIndex + 2, dataset_name.IndexOf("_", tIndex + 1) - tIndex - 2));
            int pIndex = dataset_name.IndexOf("_p");
            dataset_pvalue = int.Parse(dataset_name.Substring(pIndex + 2, dataset_name.IndexOf("_", pIndex + 1) - pIndex - 2));

            // solve the CTOP with all nodes as separate clusters.
            if (unit_clusters)
            {
                // split the profit in the case of p2 category
                if (dataset_pvalue == 2)
                {
                    foreach (Set set in this.sets)
                    {
                        // Calculate the profit to distribute to each node
                        int nodeCount = set.nodes.Count;
                        int profitPerNode = set.profit / nodeCount;
                        int remainingProfit = set.profit % nodeCount;

                        // Distribute the profit equally among the nodes
                        foreach (Node node in set.nodes)
                        {
                            node.profit = profitPerNode;
                        }

                        // Distribute the remaining profit to the first nodes
                        for (int i = 0; i < remainingProfit; i++)
                        {
                            set.nodes[i].profit += 1;
                        }

                    }
                }

                // override 
                set_crowd = node_crowd;
                sets.Clear();

                // find the set at which belongs each node
                foreach (Node node in nodes)
                {
                    this.nodes[node.id].set_id = node.id;
                    if (dataset_pvalue == 1)
                    {
                        this.nodes[node.id].profit = 1;
                    } 

                    if (node.id == 0) // depot
                    {
                        this.nodes[node.id].profit = 0;
                    }

                    sets.Add(new Set(node.id, node.profit, new List<Node> { node }));
                }
            }
            else
            {
                // find the set at which belongs each node
                foreach (Set set in this.sets)
                {
                    foreach (Node node in set.nodes)
                    {
                        this.nodes[node.id].set_id = set.id;
                        this.nodes[node.id].profit = set.profit;
                    }
                }
            }

            double total_available_profit = 0;
            foreach (Set set in this.sets) { total_available_profit += set.profit; }
            this.total_available_profit = total_available_profit;

            DistMatrix();

            // generate a new distance matrix based on the normalized values
            //Scaled_DistMatrix();

            //MinAndMaxProfitDifference();
            //MinAndMaxDistanceDifference();

            //generate arcs table
            //CreateArcs();

            model = this;
        }

        //public void ReconstructModel()  // sets all necessary values back to default
        //                                // -- needs to be called when multiple solutions run sequentially
        //{
        //    foreach (Set set in this.sets)
        //    {
        //        set.in_route = false;
        //    }
        //}

        public void DistMatrix()
        {
            int[,] dist_matrix = new int[this.node_crowd, this.node_crowd];
            for (int i = 0; i < this.node_crowd; i++)
            {
                for (int j = 0; j < this.node_crowd; j++)
                {
                    dist_matrix[i, j] = 0;
                }
            }
            foreach (Node n1 in this.nodes)
            {
                foreach (Node n2 in this.nodes)
                {
                    if (n1.id < n2.id)
                    {
                        double d_x = Math.Abs(n1.x - n2.x);
                        double d_y = Math.Abs(n1.y - n2.y);
                        double dist = Math.Sqrt(Math.Pow(d_x, 2) + Math.Pow(d_y, 2));
                        dist_matrix[n1.id, n2.id] = (int)Math.Ceiling(dist);
                        dist_matrix[n2.id, n1.id] = (int)Math.Ceiling(dist);
                    }
                }
            }
            this.dist_matrix = dist_matrix;
        }

        public void MinAndMaxProfitDifference() // find the minimum and maximum profit difference in dataset (used for swap operators)
        {
            List<double> profits = new List<double>();
            foreach (Set set in sets)
            {
                if (set.id != 0)
                {
                    profits.Add(set.profit);
                }
            }
            double max_profit = ((IEnumerable<double>)new List<double>(profits)).Max();
            double min_profit = ((IEnumerable<double>)new List<double>(profits)).Min();
            this.max_profit_difference = max_profit - min_profit;
            this.min_profit_difference = min_profit - max_profit;
            //Console.WriteLine("min_profit:{0} max_profit:{1} min_difference:{2} max_difference:{3}", min_profit, max_profit, min_profit_difference, max_profit_difference);
        }

        public void MinAndMaxDistanceDifference() // find the minimum and maximum distance difference in dataset (used for swap operators)
        {
            List<double> distances = new List<double>();
            foreach (Node n1 in this.nodes)
            {
                foreach (Node n2 in this.nodes)
                {
                    if (n1.id < n2.id)
                    {
                        distances.Add(this.dist_matrix[n1.id, n2.id]);
                    }
                }
            }
            double max_distance = ((IEnumerable<double>)new List<double>(distances)).Max();
            double min_distance = ((IEnumerable<double>)new List<double>(distances)).Min();
            this.max_dist_difference = 2 * max_distance - 2 * min_distance;
            this.min_dist_difference = 2 * min_distance - 2 * max_distance;
            //Console.WriteLine("min_dist:{0} max_dist:{1} min_difference:{2} max_difference:{3}", min_distance, max_distance, min_dist_difference, max_dist_difference);
        }

        override
        public string ToString()
        {
            return "Reading Dataset: " + this.dataset_name + "\nNodes: " + this.node_crowd + "\nSets: " + this.set_crowd + "\nVehicles: " + this.vehicle_number +
                "\nTmax: " + this.t_max + "\nTotal available profit: " + this.total_available_profit;
        }

    }
}
