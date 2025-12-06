using System.Collections.Generic;
using System;

//class Route
namespace SOP_Project
{
    public class Route
    {
        public static int counter = 0;
        public int id;
        public int time;
        public int total_profit;
        public List<Node> nodes_seq;
        public List<Set> sets_included;

        public Route(List<Node> nodes_seq, Model m, int id = -1)
        {
            if (id == -1)
            {
                this.id = counter++;
            }
            else
            {
                this.id = id;
            }
            time = CalcTime(nodes_seq, m);
            total_profit = TotalProfit(nodes_seq, m);
            this.nodes_seq = nodes_seq;
            sets_included = SetsIncluded(nodes_seq, m);
        }
        public Route(int time, int total_profit, List<Node> nodes_seq, List<Set> sets_included, int id = -1)
        {
            if (id == -1)
            {
                this.id = counter++;
            }
            else
            {
                this.id = id;
            }
            this.time = time;
            this.total_profit = total_profit;
            this.nodes_seq = nodes_seq;
            this.sets_included = sets_included;
        }

        public static int CalcTime(List<Node> nodes_seq, Model m)
        {
            int time = 0;
            for (int i = 0; i < nodes_seq.Count - 1; i++)
            {
                time += m.dist_matrix[nodes_seq[i].id, nodes_seq[i + 1].id];
            }
            return time;
        }

        public static int TotalProfit(List<Node> nodes_seq, Model m)
        {
            int total_profit = 0;
            foreach (Node node in nodes_seq)
            {
                total_profit += m.sets[node.set_id].profit;
            }
            return total_profit;
        }

        public bool CheckRoute(Model m) //checks if everything in route is correct, assuming that the model is correct
        {
            bool isEverythingOk = true;
            HashSet<int> set_ids_in_route = new HashSet<int>();
            HashSet<int> node_ids_in_route = new HashSet<int>();
            int total_profit = m.sets[this.nodes_seq[0].set_id].profit, //initialize total_profit at the value of the profit of the first
                                                                           //node of the sequence, as the loop starts from the second one
                   time = 0;
            int previous_node_id = this.nodes_seq[0].id;
            int counter = 0;
            if (this.nodes_seq[0].id != m.depot.id || this.nodes_seq[this.nodes_seq.Count - 1].id != m.depot.id)
            {
                Console.WriteLine("DEPOT MISSING");
                isEverythingOk = false;
            }
            foreach (Node node in this.nodes_seq.GetRange(1, this.nodes_seq.Count - 1))
            {
                counter++;
                total_profit += m.sets[node.set_id].profit;
                time += m.dist_matrix[previous_node_id, node.id];
                previous_node_id = node.id;
                if (node.set_id != this.sets_included[counter].id)
                {
                    isEverythingOk = false;
                    Console.WriteLine("DIFFERENT SET - NODE IN THE SAME POSITION: In position " + counter + " the node's set id was " + node.set_id +
                        " but the set's id was " + this.sets_included[counter].id + ".");
                }
                if (set_ids_in_route.Contains(node.set_id))
                {
                    isEverythingOk = false;
                    Console.WriteLine("SET APPEARS TWICE: The route contains two times the set with id: " + node.set_id + "."); // the HashSet does not contain the depot at the beginning
                } else
                {
                    set_ids_in_route.Add(node.set_id);
                }
                if (node_ids_in_route.Contains(node.id))
                {
                    isEverythingOk = false;
                    Console.WriteLine("NODE APEARS TWICE: The route contains two times the node with id: " + node.id + ".");
                } else
                {
                    node_ids_in_route.Add(node.id);
                }
                if (counter != this.nodes_seq.Count - 1 && node.id == m.depot.id)
                {
                    Console.WriteLine("DEPOT BETWEEN INSIDE NODES");
                    isEverythingOk = false;
                }
            }
            if (Math.Abs(total_profit - this.total_profit) > 0.0001)
            {
                isEverythingOk = false;
                Console.WriteLine("WRONG PROFIT: Route's total profit is different than the one saved.\n" +
                    "Profit found: " + total_profit + "\nProfit saved: " + this.total_profit);
            }
            if (Math.Abs(time - this.time) > 0.0001)
            {
                isEverythingOk = false;
                Console.WriteLine("WRONG TIME: Route's time is different than the one saved.\n" +
                    "Time found: " + time + "\nTime saved: " + this.time);
            }
            if (time > m.t_max)
            {
                isEverythingOk = false;
                Console.WriteLine("TIME OUT OF BOUNDS: Route's time exceeds the max available time. \nTime found: " + time + " - Tmax: " + m.t_max);
            }
            foreach (Set set in this.sets_included)
            {
                if (!set_ids_in_route.Contains(set.id))
                {
                    isEverythingOk = false;
                    Console.WriteLine("MISTAKENLY SAVED SET: Set with id: " + set.id + " is saved in the route but there is no node in it from this set.");
                }
            }
            if (set_ids_in_route.Count != this.sets_included.Count - 1) // -1 because sets_included contain 0 twice (0 is the set of the depot)
            {
                isEverythingOk = false;
                Console.WriteLine("UNEQUAL NUMBER OF SETS AND NODES: Different number of sets in the route found than the number of the saved sets. \n" +
                    "Number of sets found: " + set_ids_in_route.Count + "\nNumber of saved sets: " + this.sets_included.Count);
            }
            if (isEverythingOk)
            {
                //Console.WriteLine("No error found in the Route.");
            }
            return isEverythingOk;
        }

        public static List<Set> SetsIncluded(List<Node> nodes_seq, Model m)
        {
            List<Set> sets_included = new List<Set>();
            foreach (Node node in nodes_seq)
            {
                sets_included.Add(m.sets[node.set_id]);
            }
            return sets_included;
        }

        override
        public string ToString()
        {
            String node_ids = new String(" " + this.nodes_seq[0].id + "(" + this.nodes_seq[0].set_id + ")");

            for (int i = 1; i < this.nodes_seq.Count; i++)
            {
                node_ids += " -> " + this.nodes_seq[i].id + "(" + this.nodes_seq[i].set_id + ")";
            }

            return "\npath:" + node_ids + "\ntime:" + this.time + "\nprofit:" + this.total_profit + "\nsets included: "
                + (sets_included.Count - 1) + " sets\n";
        }
    }
}
