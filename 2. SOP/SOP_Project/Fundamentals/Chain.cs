using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;
using System.IO;

namespace SOP_Project
{
    public class Chain
    {
        public static List<Chain> all_chains = new List<Chain>();
        public static List<Chain> intense_chains = new List<Chain>();
        public static List<Chain> diverse_chains = new List<Chain>();
        public static int counter = 0;
        public static int[,] chain_dist_matrix;

        public int id;
        public string pool_origin;
        public int size;
        public int useful_size;  // indicates the number of nodes in chain not already included in the solution
        public List<Node> nodes_seq;
        public List<Set> set_seq;
        public int profit;
        public int time;
        public (double, double) center;

        public Chain(Model m) // Constructor for the chain that contains only the depot
        {
            this.nodes_seq = new List<Node>() { m.depot };
            this.set_seq = new List<Set>() { m.sets[0] };
            this.size = nodes_seq.Count;
            this.profit = 0;
            this.time = 0;
            this.center = FindChainCenter();
        }
        public Chain(List<int> set_id_seq, Model m)
        {
            this.id = counter++;
            Solution route_from_chain = SolveShortestPath(set_id_seq, m);  // creates a route out of the given set chain
            this.nodes_seq = route_from_chain.route.nodes_seq;
            this.set_seq = route_from_chain.route.sets_included;
            this.size = nodes_seq.Count;
            this.profit = route_from_chain.total_profit;
            this.time = route_from_chain.total_time;
            //this.center = FindChainCenter();
        }

        private Solution SolveShortestPath(List<int> set_id_seq, Model m)  // turns the sequence of sets into a sequence of
                                                                           // nodes by solving the shortest path
        {
            // Phase 1: construct a full route in order to use shortestPath function - depot -> . -> ... -> . -> depot
            List<Set> set_seq = new List<Set>();
            int profit = 0;
            foreach (int set_id in set_id_seq)
            {
                set_seq.Add(m.sets[set_id]);
                profit += m.sets[set_id].profit;
            }
            // add the depot at the start and the end of the chain, if necessary
            //if (set_seq.Last().id != 0)
            //{
            //    set_seq.Add(m.sets[0]);
            //}
            //if (set_seq[0].id != 0)
            //{
            //    set_seq.Insert(0, m.sets[0]);
            //}

            set_seq.Insert(0, m.sets[0]);
            set_seq.Add(m.sets[0]);

            Solution chain = new Solution(new Route(0, profit, new List<Node> { }, set_seq));
            MathProgramming.SolveShortestPath(m, chain);

            // Phase 2: remove all the unnecessary depots added to form the actual chain
            int unwanted_time = 0;  // the extra time due to the depots appearance
            // remove the depots from the route that are not in the chain
            //if (set_id_seq[0] != 0)
            //{
            //    chain.route.nodes_seq.RemoveAt(0);
            //    chain.route.sets_included.RemoveAt(0);
            //    unwanted_time += m.dist_matrix[m.sets[0].id, chain.route.nodes_seq[0].id];
            //}
            //if (set_id_seq.Last() != 0)
            //{
            //    chain.route.nodes_seq.RemoveAt(chain.route.nodes_seq.Count - 1);
            //    chain.route.sets_included.RemoveAt(chain.route.sets_included.Count - 1);
            //    unwanted_time += m.dist_matrix[chain.route.nodes_seq.Last().id, m.sets[0].id];
            //}

            // remove the starting depot
            chain.route.nodes_seq.RemoveAt(0);
            chain.route.sets_included.RemoveAt(0);
            unwanted_time += m.dist_matrix[m.sets[0].id, chain.route.nodes_seq[0].id];
            // remove the end depot
            chain.route.nodes_seq.RemoveAt(chain.route.nodes_seq.Count - 1);
            chain.route.sets_included.RemoveAt(chain.route.sets_included.Count - 1);
            unwanted_time += m.dist_matrix[chain.route.nodes_seq.Last().id, m.sets[0].id];

            chain.total_time -= unwanted_time;  // actual cost of the chain

            return chain;
        }

        public override string ToString()
        {
            StringBuilder chain = new StringBuilder(this.nodes_seq[0].id + "(" + this.set_seq[0].id + ")");
            for (int i = 1; i < this.nodes_seq.Count; i++)
            {
                chain.Append(" -> " + this.nodes_seq[i].id + "(" + this.set_seq[i].id + ")");
            }
            
            return "\nchain:" + chain + "\nsize:" + this.size + "\nprofit:" + this.profit + "\ntime:" + this.time + "\n";
        }

        public static void exportChains(List<Chain> chains, string dataset, string filename)
        {
            StreamWriter writer = new StreamWriter(filename);
            writer.WriteLine(dataset);
            foreach (Chain chain in chains)
            {
                writer.Write(chain.nodes_seq[0].id);
                for (int i = 1; i < chain.nodes_seq.Count; i++)
                {
                    writer.Write("," + chain.nodes_seq[i].id);
                }
                writer.Write("\n");
            }
            writer.Close();
        }

        public (double, double) FindChainCenter()
        {
            double sum_x = 0, sum_y = 0;
            foreach (Node node in nodes_seq)
            {
                sum_x += node.x;
                sum_y += node.y;
            }
            return (sum_x / nodes_seq.Count, sum_y / nodes_seq.Count);
        }

        public static void InitializeChainDistMatrix()
        {
            int[,] ch_dist_matrix = new int[all_chains.Count, all_chains.Count];
            for (int i = 0; i < counter; i++)
            {
                for (int j = 0; j < counter; j++)
                {
                    ch_dist_matrix[i, j] = 0;
                }
            }
            for (int i = 0; i < counter; i++)
            {
                Chain ch1 = all_chains[i];
                (double ch1_x, double ch1_y) = ch1.center;
                for (int j = i + 1; j < counter; j++)
                {
                    Chain ch2 = all_chains[j];
                    (double ch2_x, double ch2_y) = ch2.center;

                    double d_x = Math.Abs(ch1_x - ch2_x);
                    double d_y = Math.Abs(ch1_y - ch2_y);
                    double dist = Math.Sqrt(Math.Pow(d_x, 2) + Math.Pow(d_y, 2));
                    ch_dist_matrix[ch1.id, ch2.id] = (int)Math.Ceiling(dist);
                    ch_dist_matrix[ch2.id, ch1.id] = (int)Math.Ceiling(dist);
                }
            }
            chain_dist_matrix = ch_dist_matrix;
        }

        public static void Initialize_Chains(Model m)
        {
            counter = 0;
            Chain.all_chains.Add(new Chain(m));
        }
    }
}
