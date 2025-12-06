using System.Collections.Generic;
using System;
using System.Linq;

// class Model
namespace SOP_Project
{
    public class Model
    {
        public string dataset_name;
        public int node_crowd;
        public int set_crowd;
        public double t_max;
        public int total_available_profit;
        public Node depot;
        public List<Node> nodes;
        public List<Set> sets;
        //public Arc[,] arcs;
        public int[,] dist_matrix;
        public double min_profit_difference;
        public double max_profit_difference;
        public double min_dist_difference;
        public double max_dist_difference;

        public static Model m;

        public Random r;

        public Model(int node_crowd, int set_crowd, double t_max, Node depot, List<Node> nodes, List<Set> sets)
        {
            this.node_crowd = node_crowd;
            this.set_crowd = set_crowd;
            this.t_max = t_max;
            this.depot = depot;
            this.nodes = nodes;
            this.sets = sets;

            int total_available_profit = 0;
            foreach (Set set in this.sets) { total_available_profit += set.profit; }
            this.total_available_profit = total_available_profit;
        }

        //public Model DeepCopy(int seed) // generates a deep copy of a Model object
        //{
        //    List<Node> nodesDeepCopy = this.nodes.ConvertAll(node => node.DeepCopy()); // create a deep copy of the List nodes
        //    List<Set> setsDeepCopy = Set.CreateDeepCopyOfSetsListWithKnownNodes(sets, nodesDeepCopy);
        //    Model copyModel = new Model(this.node_crowd, this.set_crowd, this.t_max, nodesDeepCopy[0], nodesDeepCopy, setsDeepCopy);
        //    copyModel.Build();
        //    return copyModel;
        //}

        public Model Build()
        {
            //ScaleWithNormalDistribution(); // normalize each node's coordinates as well as each set's profit
            //ScaleWithRange();

            // find the set at which belongs each node
            foreach (Set set in this.sets)
            {
                foreach (Node node in set.nodes)
                {
                    this.nodes[node.id].set_id = set.id;
                    this.nodes[node.id].profit = set.profit;
                    this.nodes[node.id].pool_profit = set.profit;
                }
            }

            DistMatrix();

            // generate a new distance matrix based on the normalized values
            //Scaled_DistMatrix();

            //MinAndMaxProfitDifference();
            //MinAndMaxDistanceDifference();

            //generate arcs table
            //CreateArcs();

            return this;
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
                        dist_matrix[n1.id, n2.id] = (int) Math.Ceiling(dist);
                        dist_matrix[n2.id, n1.id] = (int) Math.Ceiling(dist);
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
            double max_profit = ((IEnumerable<double>) new List<double>(profits)).Max();
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
            return "Reading Dataset: " + this.dataset_name + "\nNodes: " + this.node_crowd + "\nSets: " + this.set_crowd +
                "\nTmax: " + this.t_max + "\nTotal available profit: " + this.total_available_profit;
        }

        /*
        public void CreateArcs()  // create Arc table containing the arcs between all nodes
        {
            Arc[,] arcs = new Arc[this.node_crowd, this.node_crowd];

            // execute this if you want the symmetrical arcs to behave in the same manner
            //foreach (Node n1 in this.nodes)
            //{
            //    foreach (Node n2 in this.nodes)
            //    {
            //        if (n1.id < n2.id)
            //        {
            //            Arc arc = new Arc(n1, n2, this.dist_matrix[n1.id, n2.id]);
            //            arcs[n1.id, n2.id] = arc;
            //            arcs[n2.id, n1.id] = arc;
            //        }
            //    }
            //}

            // execute this if you want the symmetrical arcs to behave in a different manner
            foreach (Node n1 in this.nodes)
            {
                foreach (Node n2 in this.nodes)
                {
                    arcs[n1.id, n2.id] = new Arc(n1, n2, this.dist_matrix[n1.id, n2.id]);
                }
            }
            this.arcs = arcs;
        }
        */
    } 
}
