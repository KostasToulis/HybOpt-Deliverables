using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SOP_Project
{
    class DataSet_Generator
    {
        public Model GenerateDataSet(int node_crowd, int set_crowd, int t_max, int profit_coefficient)
        {
            Random r = new Random(1);
            int min_range = 0;
            int max_range = 100;
            // create random nodes with coordinates ranging from (0, 100)
            List<Node> nodes = new List<Node>();
            Node depot = new Node(0, 20, 20, 0);
            for (int i=1; i < node_crowd; i++)
            {
                int x = r.Next(min_range, max_range + 1);
                int y = r.Next(min_range, max_range + 1);
                nodes.Add(new Node(i, x, y, -1));
            }

            // order created nodes in sets by applying k-means clustering method
            double[][] means = GenerateMeans(min_range, max_range, set_crowd);
            List<Set> sets = K_Means_Clustering(nodes, set_crowd, means);

            // calculate each set's profit
            foreach (Set set in sets)
            {
                set.profit = profit_coefficient * set.nodes.Count * r.Next(1, 11);
            }

            sets.Insert(0, new Set(0, 0, new List<Node> { depot }));
            nodes.Insert(0, depot);
            return new Model(node_crowd, set_crowd, t_max, depot, nodes, sets);
        }

        public static double[][] GenerateMeans(int min_range, int max_range, int set_crowd)
        {
            double[][] means = new double[set_crowd][];
            int i = 0;

            int x_axis_segments = 5; // TODO --------------------------
            int y_axis_segments = 2; // TODO --------------------------
            double x_step = max_range / x_axis_segments;
            double y_step = max_range / y_axis_segments;
            double x = min_range;
            while (x < max_range)
            {
                double y = min_range;
                while (y < max_range)
                {
                    double mean_x = x + x_step / 2;
                    double mean_y = y + y_step / 2;
                    means[i] = new double[2] { mean_x, mean_y };
                    i += 1;
                    y += y_step;
                }
                x += x_step;
            }
            return means;
        }

        public static List<Set> K_Means_Clustering(List<Node> nodes, int set_crowd, double[][] means)
        {
            // initialize empty sets
            List<Set> sets = new List<Set>();
            for (int i = 0; i < set_crowd; i++)
            {
                sets.Add(new Set(i + 1, 1, new List<Node> { }));
            }

            bool sets_not_changed = false;
            while (!sets_not_changed)
            {
                // find the mean closer to each node
                sets_not_changed = true;
                foreach (Node node in nodes)
                {
                    double min_dist = Math.Pow(10, 10);
                    int closest_mean = -1;
                    for (int i = 0; i < means.Length; i++)
                    {
                        double[] mean = means[i];
                        double d_x = Math.Abs(node.x - mean[0]);
                        double d_y = Math.Abs(node.y - mean[1]);
                        double dist = Math.Sqrt(Math.Pow(d_x, 2) + Math.Pow(d_y, 2));
                        if (dist < min_dist)
                        {
                            min_dist = dist;
                            closest_mean = i;
                        }
                    }

                    if (node.set_id != closest_mean)
                    {
                        sets_not_changed = false;  // turns false if at least one node has been removed from its previous set
                        if (node.set_id != -1)
                        {
                            sets[node.set_id].nodes.Remove(node);
                        }
                        sets[closest_mean].nodes.Add(node);
                        node.set_id = closest_mean;
                    }
                }

                if (sets_not_changed)
                {
                    continue;
                }

                // recalculate each mean's coordinates
                foreach (Set set in sets)
                {
                    double sum_x = 0;
                    double sum_y = 0;
                    foreach (Node node in set.nodes)
                    {
                        sum_x += node.x;
                        sum_y += node.y;
                    }
                    double mean_x = sum_x / set.nodes.Count;
                    double mean_y = sum_y / set.nodes.Count;
                    means[set.id - 1][0] = mean_x;
                    means[set.id - 1][1] = mean_y;
                }
            }

            return sets;
        }
    }
}
