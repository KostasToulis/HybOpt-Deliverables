using Microsoft.Win32.SafeHandles;
using System;
using System.Collections.Generic;
using System.Diagnostics.Metrics;
using System.IO;
using System.Linq;

namespace MSOP.Fundamentals
{
    public class Solution
    {
        public List<Route> routes;
        public int total_profit;
        public HashSet<Set> sets_included = new HashSet<Set>();
        public int iteration_best_found;
        public double duration_of_local_search; //refers to the time the local search ran ( in milliseconds )
        public double duration_of_maths; //refers to the time that exacts from Math Programming ran ( in milliseconds )
        public double duration_total;
        public int iterations_of_local_search;

        public Solution()
        {
            Model m = Model.model;
            routes = new List<Route>();
            for (int i = 0; i < m.vehicle_number; i++)
            {
                routes.Add(new Route(i));
            }
            sets_included.Add(m.sets[m.depot.set_id]);
            total_profit = 0;
        }
        public Solution(List<Route> routes)
        {
            this.routes = routes;
            this.total_profit = routes.Sum(route => route.total_profit);
            foreach (Route route in routes)
            {
                this.sets_included.UnionWith(route.sets_included);
            }
        }

        public Solution DeepCopy() // generates a deep copy of a solution object -- used at VNS metohd to copy best_sol
        {
            List<Route> copy_routes = new List<Route>();
            foreach (Route route in routes)
            {
                copy_routes.Add(new Route(new List<Node>(route.nodes_seq), route.id));
            }
            Solution sol =  new Solution(copy_routes);

            sol.total_profit = this.total_profit;
            sol.iterations_of_local_search = this.iterations_of_local_search;
            sol.iteration_best_found = this.iteration_best_found;
            sol.duration_of_maths = this.duration_of_maths;
            sol.duration_of_local_search = this.duration_of_local_search;
            sol.duration_total = this.duration_total;

            return sol;
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


        public bool CheckSol()
        {
            bool isEverythingOk = true;
            HashSet<Set> setsInSol = new HashSet<Set>();
            int profitOfSol = 0;
            foreach (Route route in routes)
            {
                // routes check
                if (!route.CheckRoute())
                {
                    isEverythingOk = false;
                }
                // different sets per route check
                if (setsInSol.Count == 0)
                {
                    setsInSol = new HashSet<Set>(route.sets_included);
                }
                else
                {
                    HashSet<Set> setsIntersection = new HashSet<Set>(setsInSol.Intersect(route.sets_included));
                    if (setsIntersection.Count > 1)
                    {
                        isEverythingOk = false;
                        foreach (Set s in setsIntersection)
                        {
                            if (s.id == 0)
                            {
                                continue;
                            }
                            Console.WriteLine("SAME SET IN DIFFERENT ROUTES: Set " + s.id + " appears in multiple routes.");
                        }

                    }
                    setsInSol.UnionWith(route.sets_included);
                }
                // total profit update
                profitOfSol += route.total_profit;
            }
            if (profitOfSol != this.total_profit)
            {
                isEverythingOk = false;
                Console.WriteLine("WRONG SOLUTION PROFIT: Solution Profit Reported: " + this.total_profit + ", Solution Profit Actual: " + profitOfSol);
            }
            if (!isEverythingOk)
            {
                Console.WriteLine("Problem");
            }
            return isEverythingOk;
        }

        override
        public string ToString()
        {
            string solString = "Solution Profit: " + this.total_profit + "\nRoutes: \n";
            foreach (Route route in routes)
            {
                solString += route.ToString();
            }
            return solString;
        }

        public static Solution ReadSolutionFromFile(string sol_extraction_path, string filename)
        {
            // Example
            //Dataset_name: 11berlin52_T100_p2_v2
            //Vehicles: 2
            //Routes
            //Route_0: 0,22,41,2,48,0
            //Route_1: 0,11,50,18,44,0
            //Profit: 2384
            //Time: 40.6827017

            //Status: Feasible
            //BestBound: 2608
            //BestRestart: 0
            //RestartTime: 17.6260049
            //Iterations: 10007
            //IterationsTillBest: 6
            //TimeOfMath: 17.442
            //Arguments: mls 100 1

            // solutionFileName
            Solution solution = new Solution();

            using (StreamReader reader = new StreamReader(sol_extraction_path + filename))
            {
                string line;
                int rtIdx = 0;
                bool startReadingNodes = false;

                while ((line = reader.ReadLine()) != null)
                {
                    if (line.StartsWith("Routes"))
                    {
                        startReadingNodes = true;
                        continue;
                    }

                    if (startReadingNodes && line.StartsWith("Route_"))
                    {
                        Route rt = solution.routes[rtIdx];
                        rtIdx++;

                        string[] nodeIds = line.Split(':', ',');
                        for (int i = 2; i < nodeIds.Length - 1; i++)
                        {
                            Node node = Model.model.nodes[int.Parse(nodeIds[i])];
                            Set set = Model.model.sets[node.set_id];
                            rt.nodes_seq.Insert(rt.nodes_seq.Count - 1, node);
                            rt.sets_included.Insert(rt.sets_included.Count - 1, set);
                            solution.sets_included.Add(set);

                            rt.total_profit += node.profit;
                            solution.total_profit += node.profit;
                        }
                    }
                }

                // update times
                foreach (Route rt in solution.routes)
                {
                    int previous_node_id = Model.model.depot.id;
                    foreach (Node node in rt.nodes_seq.GetRange(1, rt.nodes_seq.Count - 1))
                    {
                        rt.time += Model.model.dist_matrix[previous_node_id, node.id];
                        previous_node_id = node.id;
                    }
                }
            }

            solution.CheckSol();

            return solution;
        }

        public void ReportSol(string filename) // store the solution object in a file
        {
            Model m = Model.model;


            StreamWriter writer = new StreamWriter(filename + ".txt");
            writer.WriteLine("Dataset_name: " + m.dataset_name);
            writer.WriteLine("Tmax: " + m.t_max);
            writer.WriteLine("Vehicles: " + m.vehicle_number);
            writer.WriteLine("Upper_Profit_Bound: " + m.total_available_profit);
            writer.WriteLine("Sol_profit: " + this.total_profit);
            writer.WriteLine("Routes: ");
            for (int i = 0; i < this.routes.Count; i++)
            {
                Route route = this.routes[i];
                String routePath = new String("" + route.nodes_seq[0].id);
                for (int j = 1; j < route.nodes_seq.Count; j++)
                {
                    routePath += "," + route.nodes_seq[j].id;
                }
                writer.WriteLine("Route " + i + ": " + routePath);
            }
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
            writer.WriteLine("Restart;Best;Time;TimeExact;Best_found_at;Iterations");
            for (int i = 0; i < sols.Count; i++)
            {
                writer.Write(i + ";");
                writer.Write(sols[i].total_profit + ";");
                writer.Write(sols[i].duration_of_local_search + ";");
                writer.Write(sols[i].duration_of_maths + ";");
                writer.Write(sols[i].iteration_best_found + ";");
                writer.Write(sols[i].iterations_of_local_search + ";");
            }
            writer.Close();
        }

        public static void SummaryReportAsTxt(List<Solution> sols, string signature)  // call this after local search is done, at the main method
        {
            string dir = "Summary_Reports/";
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
                writer.Write("Time:" + sols[i].duration_of_local_search + ";");
                writer.Write("TimeExact:" + sols[i].duration_of_maths + ";");
                writer.Write("Best_found_at:" + sols[i].iteration_best_found + ";");
                writer.Write("Iterations:" + sols[i].iterations_of_local_search + ";");
            }
            writer.Close();
        }
    }
}
