using MSOP.Fundamentals;
using System.Collections.Generic;
using System.IO;
using System.Security.AccessControl;

namespace MSOP
{
    class Report
    {
        public static void ExtractParametersSelectionReport(Dictionary<string, List<Solution>> dataset_solutions, string export_file_name)
        {
            // Dictionary<string, int> literaure_best = Program.GetLiteratureBests("");
            StreamWriter writer = new StreamWriter(export_file_name + ".csv");
            writer.WriteLine("dataset_name;solution_1;solution_2;solution_3;solution_4;solution_5;solution_6;solution_7;solution_8;solution_9;solution_10");
            List<string> sortedDatasets = new List<string>(dataset_solutions.Keys);
            sortedDatasets.Sort();
            foreach (string dataset_name in sortedDatasets)
            {
                writer.Write(dataset_name);
                foreach (Solution sol in dataset_solutions[dataset_name])
                {
                    writer.Write(";" + sol.total_profit);
                }
                writer.Write("\n");
            }
            writer.Close();
        }

        public static void ExtractPromisesTargetTuningReport(Dictionary<string, List<Solution>> k_solutions, int[] k_values, string export_file_name)
        {
            StreamWriter writer = new StreamWriter(export_file_name + ".csv");
            string columns_names = "dataset_name";
            foreach (int k in k_values)
            {
                columns_names += ";" + k;
            }
            writer.WriteLine(columns_names);
            List<string> sortedDatasets = new List<string>(k_solutions.Keys);
            sortedDatasets.Sort();
            foreach (string dataset_name in sortedDatasets)
            {
                writer.Write(dataset_name);
                foreach (Solution sol in k_solutions[dataset_name])
                {
                    writer.Write(";" + sol.total_profit);
                }
                writer.Write("\n");
            }
            writer.Close();
        }

        public static void ExtractIntensificationDiversificationTuningReport(Dictionary<string, List<Solution>> generated_solutions, int[] l_values,
            int[] restarts_values, string export_file_name)
        {
            StreamWriter writer = new StreamWriter(export_file_name + ".csv");
            string columns_names = "dataset_name";
            for (int i = 0; i < l_values.Length; i++)
            {
                columns_names += ";l=" + l_values[i] + "restarts=" + restarts_values[i] + ";duration";
            }
            writer.WriteLine(columns_names);
            List<string> sortedDatasets = new List<string>(generated_solutions.Keys);
            sortedDatasets.Sort();
            foreach (string dataset_name in sortedDatasets)
            {
                writer.Write(dataset_name);
                foreach (Solution sol in generated_solutions[dataset_name])
                {
                    writer.Write(";" + sol.total_profit + ";" + sol.duration_total);
                }
                writer.Write("\n");
            }
            writer.Close();
        }

        public static void ExtractTunedAlgorithmReport(Dictionary<string, List<Solution>> generated_solutions, int runs, string export_file_name)
        {
            StreamWriter writer = new StreamWriter(export_file_name + ".csv");
            string columns_names = "dataset_name";
            for (int i = 1; i < runs + 1; i++)
            {
                columns_names += ";obj_" + i + ";duration_" + i;
            }
            writer.WriteLine(columns_names);
            List<string> sortedDatasets = new List<string>(generated_solutions.Keys);
            sortedDatasets.Sort();
            foreach (string dataset_name in sortedDatasets)
            {
                writer.Write(dataset_name);
                foreach (Solution sol in generated_solutions[dataset_name])
                {
                    writer.Write(";" + sol.total_profit + ";" + sol.duration_total);
                }
                writer.Write("\n");
            }
            writer.Close();
        }

        public static void ExtractSolutionInformation(Solution sol, Program.ExecutionData executionData, Model m, string export_file_path)
        {
            StreamWriter writer = new StreamWriter(export_file_path);
            writer.WriteLine("Dataset_name: " + m.dataset_name);
            writer.WriteLine("Vehicles: " + m.vehicle_number);
            List<string> routes_path = new List<string>();
            foreach (Route route in sol.routes)
            {
                string route_path = route.nodes_seq[0].id.ToString();
                for (int i = 1; i < route.nodes_seq.Count; i++)
                {
                    route_path += "," + route.nodes_seq[i].id.ToString();
                }
                routes_path.Add(route_path);
            }
            writer.WriteLine("Routes");
            for (int i = 0; i < routes_path.Count; i++)
            {
                writer.WriteLine("Route_" + i + ": " + routes_path[i]);
            }
            writer.WriteLine("Profit: " + sol.total_profit);
            writer.WriteLine("Time: " + Program.runData.execTime);
            writer.WriteLine();

            if (Program.runData.method == "exact")
            {
                writer.WriteLine("Status: " + Program.runData.status);
                writer.WriteLine("Nodes: " + Program.runData.nodeCount);
                writer.WriteLine("BestBound: " + Program.runData.BestBound_end);
                writer.WriteLine("BestBoundRoot: " + Program.runData.BestBound_root_after_cuts);
                writer.WriteLine("BestBoundRootBeforeCuts: " + Program.runData.BestBound_root);
                writer.WriteLine("FinalGap: " + Program.runData.Gap_end);
            } else if (Program.runData.method == "mls")
            {
                // Update execution data
                writer.WriteLine("Status: " + Program.runData.status);
                writer.WriteLine("BestBound: " + Program.runData.BestBound_end);
                writer.WriteLine("BestRestart: " + Program.runData.restartOfBest);
                writer.WriteLine("RestartTime: " + Program.runData.duration_of_restart);
                writer.WriteLine("Iterations: " + Program.runData.totalIters);
                writer.WriteLine("IterationsTillBest: " + Program.runData.iterOfBest);
                writer.WriteLine("TimeOfMath: " + Program.runData.duration_of_maths);
            }

            writer.WriteLine("Arguments: {0}", string.Join(" ", Program.runData.argsEx));

            writer.Close();
        }

        public static void ExtractVRPSolutionInformation(Solution sol, Program.ExecutionData executionData, Model m, string export_file_path)
        {
            int total_time = 0;

            StreamWriter writer = new StreamWriter(export_file_path);
            writer.WriteLine("Dataset_name: " + m.dataset_name);
            writer.WriteLine("Vehicles: " + sol.routes.Count);
            List<string> routes_path = new List<string>();
            foreach (Route route in sol.routes)
            {
                string route_path = route.nodes_seq[0].id.ToString();
                for (int i = 1; i < route.nodes_seq.Count; i++)
                {
                    route_path += "," + route.nodes_seq[i].id.ToString();
                }
                routes_path.Add(route_path);
            }
            writer.WriteLine("Routes");
            for (int i = 0; i < routes_path.Count; i++)
            {
                writer.WriteLine("Route_" + i + ": " + routes_path[i] + " (" + sol.routes[i].time + ")");
                total_time += sol.routes[i].time;
            }
            writer.WriteLine("Profit: " + sol.total_profit);
            writer.WriteLine("Time: " + Program.runData.execTime);
            writer.WriteLine();

            writer.WriteLine("Total distance: " + total_time);
            string output = "";
            foreach (Node node in executionData.unserved_customers)
            {
                output += node.id + " "; 
            }
            writer.WriteLine("Unserved customers: " + output + " (" + executionData.unserved_customers.Count + ")");

            //if (Program.runData.method == "exact")
            //{
            //    writer.WriteLine("Status: " + Program.runData.status);
            //    writer.WriteLine("Nodes: " + Program.runData.nodeCount);
            //    writer.WriteLine("BestBound: " + Program.runData.BestBound_end);
            //    writer.WriteLine("BestBoundRoot: " + Program.runData.BestBound_root_after_cuts);
            //    writer.WriteLine("BestBoundRootBeforeCuts: " + Program.runData.BestBound_root);
            //    writer.WriteLine("FinalGap: " + Program.runData.Gap_end);
            //}
            //else if (Program.runData.method == "mls")
            //{
            //    // Update execution data
            //    writer.WriteLine("Status: " + Program.runData.status);
            //    writer.WriteLine("BestBound: " + Program.runData.BestBound_end);
            //    writer.WriteLine("BestRestart: " + Program.runData.restartOfBest);
            //    writer.WriteLine("RestartTime: " + Program.runData.duration_of_restart);
            //    writer.WriteLine("Iterations: " + Program.runData.totalIters);
            //    writer.WriteLine("IterationsTillBest: " + Program.runData.iterOfBest);
            //    writer.WriteLine("TimeOfMath: " + Program.runData.duration_of_maths);
            //} 

            writer.WriteLine("Arguments: {0}", string.Join(" ", Program.runData.argsEx));

            writer.Close();
        }

        public static void ExtractDatasetsTotalProfit(Dictionary<string, double> datasets_total_profits, string export_file_name)
        {
            StreamWriter writer = new StreamWriter(export_file_name + ".csv");
            string columns_names = "dataset_name;total_profit\n";
            writer.WriteLine(columns_names);
            List<string> sortedDatasets = new List<string>(datasets_total_profits.Keys);
            sortedDatasets.Sort();
            foreach (string dataset_name in sortedDatasets)
            {
                writer.Write(dataset_name + ";" + datasets_total_profits[dataset_name] + "\n");
            }
            writer.Close();
        }
    }
}
