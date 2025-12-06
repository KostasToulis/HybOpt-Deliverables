//class Solution
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace SOP_Project
{
    public class Solution
    {
        public Route route;
        public int total_time;
        public int total_profit;
        public int iteration_best_found;
        public double duration_of_local_search; //refers to the time the local search ran ( in milliseconds )
        public double duration_of_maths; //refers to the time that exacts from Math Programming ran ( in milliseconds )
        public double duration_of_constructive; // refers to the time the constructive ran ( in milliseconds ) 
        public int iterations_of_local_search;
        public int n_exact;

        public Solution(Route route)
        {
            this.route = route;
            total_time = route.time;
            total_profit = route.total_profit;
        }

        public Solution DeepCopy(Model m) // generates a shallow copy of a solution object -- used at VNS metohd to copy best_sol
        {
            return new Solution(new Route(new List<Node>(this.route.nodes_seq), m, this.route.id));
        }

        //public static double CalcTotal_Time(List<Route> routes)
        //{
        //    double total_time = 0;
        //    foreach (Route route in routes)
        //    {
        //        total_time += route.time;
        //    }
        //    return total_time;
        //}

        //public static double CalcTotal_Profit(List<Route> routes)
        //{
        //    double total_profit = 0;
        //    foreach (Route route in routes)
        //    {
        //        total_profit += route.total_profit;
        //    }
        //    return total_profit;
        //}

        public static double NodeSimilarityPercentage(Solution sol1, Solution sol2)
        {
            HashSet<Node> sol1_nodes = sol1.route.nodes_seq.ToHashSet<Node>();
            HashSet<Node> sol2_nodes = sol2.route.nodes_seq.ToHashSet<Node>();
            HashSet<Node> set_intersection = new HashSet<Node>(sol1_nodes);
            set_intersection.IntersectWith(sol2_nodes);  // perform the A Π B node operation

            return ((double) set_intersection.Count / Math.Min(sol1_nodes.Count, sol2_nodes.Count)) * 100;
        }

        public static double SetSimilarityPercentage(Solution sol1, Solution sol2)
        {
            HashSet<Set> sol1_sets = sol1.route.sets_included.ToHashSet<Set>();
            HashSet<Set> sol2_sets = sol2.route.sets_included.ToHashSet<Set>();
            HashSet<Set> set_intersection = new HashSet<Set>(sol1_sets);
            set_intersection.IntersectWith(sol2_sets);  // perform the A Π B set operation

            return ((double)set_intersection.Count / Math.Min(sol1_sets.Count, sol2_sets.Count)) * 100;
        }

        public static double SetSimilarityPercentage2(Solution sol1, Solution sol2) // based on sets connections
        {
            HashSet<(int,int)> sol1_sets_connections = new HashSet<(int, int)>();
            HashSet<(int, int)> sol2_sets_connections = new HashSet<(int, int)>();

            for (int i = 0; i < sol1.route.sets_included.Count - 1; i++)
            {
                sol1_sets_connections.Add((sol1.route.sets_included[i].id, sol1.route.sets_included[i + 1].id));
            }

            for (int i = 0; i < sol2.route.sets_included.Count - 1; i++)
            {
                sol2_sets_connections.Add((sol2.route.sets_included[i].id, sol2.route.sets_included[i + 1].id));
            }
            HashSet<(int, int)> set_intersection = new HashSet<(int, int)>(sol1_sets_connections);
            set_intersection.IntersectWith(sol2_sets_connections);  // perform the A Π B set operation

            return ((double)set_intersection.Count / Math.Min(sol1_sets_connections.Count, sol2_sets_connections.Count)) * 100;
        }



        public bool CheckSol(Model m) 
        {
            return this.route.CheckRoute(m);
        }

        override
        public string ToString()
        {
            return this.route.ToString();
        }

        public void ReportSol(string filename, Model m) // store the solution object in a file
        {
            // generate a string object containing the nodes in solution path
            String path = new String("" + this.route.nodes_seq[0].id);
            for (int i = 1; i < this.route.nodes_seq.Count; i++)
            {
                path += "," + this.route.nodes_seq[i].id;
            }

            StreamWriter writer = new StreamWriter(filename + ".txt");
            writer.WriteLine("Dataset_name:" + m.dataset_name);
            writer.WriteLine("Tmax:" + m.t_max);
            writer.WriteLine("Sol_time:" + this.total_time);
            writer.WriteLine("Upper_Profit_Bound:" + m.total_available_profit);
            writer.WriteLine("Sol_profit:" + this.total_profit);
            writer.WriteLine("Sol_path:" + path);
            if (this.iteration_best_found != 0)
            {
                writer.WriteLine("Found at:" + this.iteration_best_found);
            }
            writer.Close();
        }

        public static void Report_Sol_Evolution(List<int> sols, string path)
        {
            try
            {
                Directory.CreateDirectory(path);
            }
            catch { }
            StreamWriter writer = new StreamWriter(path + "/Sol_Evolution.txt");
            foreach (int profit in sols)
            {
                writer.WriteLine(profit);
            }
            writer.Close();
        }

        public static void SummaryReportAsCsv(List<Solution> sols, string signature)  // call this after local search is done, at the main method
        {
            string dir = "Summary_Reports/";
            try
            {
                Directory.CreateDirectory(dir);
            }
            catch { }
            StreamWriter writer = new StreamWriter(dir + "Summary_" + signature + ".csv");
            writer.WriteLine("Restart;Best;Cost;Time;TimeExact;Best_found_at;Iterations;Sets_included;N_exact;Best_run_time;Best_run_time_exact");
            for (int i = 0; i < sols.Count; i++)
            {
                writer.Write(i + ";");
                writer.Write(sols[i].total_profit + ";");
                writer.Write(sols[i].total_time + ";");
                writer.Write(sols[i].duration_of_local_search + ";");
                writer.Write(sols[i].duration_of_maths + ";");
                writer.Write(sols[i].iteration_best_found + ";");
                writer.Write(sols[i].iterations_of_local_search + ";");
                writer.Write(sols[i].route.sets_included.Count + ";");
                writer.Write(sols[i].n_exact + "\n");

            }
            writer.Close();
        }

        public static void SummaryReportAsTxt(List<Solution> sols, string dir, string signature)  // call this after local search is done, at the main method
        {
            try
            {
                Directory.CreateDirectory(dir);
            }
            catch { }
            StreamWriter writer = new StreamWriter(dir + "Summary_" + signature + ".txt");
            for (int i = 0; i < sols.Count; i++)
            {
                writer.Write("Restart:" + i + ";");
                writer.Write("Best:" + sols[i].total_profit + ";");
                writer.Write("Cost:" + sols[i].total_time + ";");
                writer.Write("Time:" + sols[i].duration_of_local_search + ";");
                writer.Write("TimeExact:" + sols[i].duration_of_maths + ";");
                writer.Write("TimeConstructive:" + sols[i].duration_of_constructive + ";");
                writer.Write("Best_found_at:" + sols[i].iteration_best_found + ";");
                writer.Write("Iterations:" + sols[i].iterations_of_local_search + ";");
                writer.Write("N_exact:" + sols[i].n_exact + ";");
                writer.Write("Sets_included:" + sols[i].route.sets_included.Count + "\n");
            }
            writer.Close();
        }
    }
}
