using MSOP.Fundamentals;
using MSOP.Heuristics;
using MSOP.MathematicalProgramming;
using MSOP.Solvers;
using System;
using System.Collections.Generic;
using System.Data;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using static System.Net.Mime.MediaTypeNames;


namespace MSOP
{
    class Program
    {
        /*
         * Struct to save exacution data (times, methods, results to be reported)
         */
        public struct ExecutionData
        {
            // input
            public string[] argsEx;
            public string method;
            public string solver;
            public int maxTime;

            // generic
            public double execTime;
            public double UB_end;
            public double BestBound_end;
            public string status;

            // exact
            public double BestBound_root;
            public double BestBound_root_after_cuts;
            public double Gap_end;
            public double Gap_root;
            public double Gap_root_after_cuts;
            public double nodeCount;

            // heur 
            public double totalIters;
            public double iterOfBest;
            public double restartOfBest;
            public double duration_of_maths;
            public double duration_of_restart;

            // vrp
            public List<Node> unserved_customers;
        }

        static readonly int randomSeed = 15061994; //10;
        static readonly int RCL_SIZE = 4; // for the minimum insertions algorithm
        public static bool unit_clusters = false; // this is used to make each node a separate cluster
        public static ExecutionData runData;

        static void Main(string[] args)
        {
            Console.WriteLine("=====================================================================");
            Console.WriteLine("Multi-Vehicle Set Orienteering Problem (MSOP) solver");
            Console.WriteLine("=====================================================================\n\n");

            string datasetName = "";
            int t_cat = 0;
            int profit_cat = 0;

            // read args 
            //exact grb 180 60 1
            runData.argsEx = args;
            runData.method = args[0];
            if (runData.method == "exact")
            {
                runData.solver = args[1];
                runData.maxTime = int.Parse(args[2]);
                t_cat = int.Parse(args[3]);
                profit_cat = int.Parse(args[4]);
            } else if (runData.method == "mls")
            {
                t_cat = int.Parse(args[1]);
                profit_cat = int.Parse(args[2]);
            }
            else if (runData.method == "VRP")
            {
                runData.solver = args[1];
                runData.maxTime = int.Parse(args[2]);
                t_cat = int.Parse(args[3]);
                profit_cat = int.Parse(args[4]);
            }

            // [1] Solve specific dataset
            //datasetName = "217vm1084_T60_p1_v4.msop";
            //datasetName = "39rat195_T60_p1_v2.msop";
            //datasetName = "11berlin52_T100_p2_v2.msop";
            datasetName = "80rd400_T60_p2_v4.msop";
            //datasetName = "11eil51_T60_p2_v2.msop";
            //datasetName = "64lin318_T60_p1_v2.msop";
            //datasetName = "145u724_T100_p1_v4.msop"; //574
            //datasetName = "115rat575_T100_p1_v2.msop";

            //SolveVRP("../../../MSOP_datasets/all/" + datasetName, true);
            //VrpCapacity.runVRPtest();
            //Tsp.runTSPtest();

            // Matheuristic local search 
            //Solve("../../../MSOP_datasets/all/" + datasetName, true);

            // TCF formulation Branch and cut
            //SolveMSOPTCF("../../../MSOP_datasets/all/" + datasetName, runData.solver, true, runData.maxTime);

            // VRP solver (LocalSearch)
            SolveVRP("../../../MSOP_datasets/all/" + datasetName, runData.maxTime, true);

            // [2] Solve all datasets for vehicles {2,3,4}
            //datasetName = "";
            //int vehicles = 2;
            //foreach (string file in Directory.EnumerateFiles("../../../MSOP_datasets/all/", "*v" + vehicles+ "*"))
            //{
            //    datasetName = file;
            //    Console.WriteLine("\nOptimizing instance " + datasetName + "\n");
            //    Solve(datasetName, true);
            //}


            // [3] Solve all datasets for t category {60,80,100}
            //datasetName = "";
            //int t_category = 60;
            //foreach (string file in Directory.EnumerateFiles("../../../MSOP_datasets/all/", "*T" + t_category + "*"))
            //{
            //    datasetName = file;
            //    Console.WriteLine("\nOptimizing instance " + datasetName + "\n");
            //    Solve(datasetName, true);
            //}

            // [4] Solve all datasets for profit category {1,2}
            //datasetName = "";
            //int profit_cat = 1;
            //foreach (string file in Directory.EnumerateFiles("../../../MSOP_datasets/all/", "*p" + profit_cat + "*"))
            //{
            //    datasetName = file;
            //    Console.WriteLine("\nOptimizing instance " + datasetName + "\n");
            //    //Solve(datasetName, true);
            //}

            // [5] Solve all datasets in selected folder
            //string folder = "selected";
            //datasetName = "";

            ////foreach (string file in Directory.EnumerateFiles("../../../MSOP_datasets/" + folder + "/", "*T" + t_cat + "*" + "*p" + profit_cat + "*"))
            //foreach (string file in Directory.EnumerateFiles("../../../MSOP_datasets/" + folder + "/"))
            //{
            //    datasetName = file;
            //    Console.WriteLine("\nOptimizing instance " + datasetName + "\n");
            //    if (runData.method == "mls")
            //    {
            //        Solve(datasetName, true);
            //    }
            //    else if (runData.method == "exact")
            //    {
            //        SolveMSOPTCF(datasetName, runData.solver, true, runData.maxTime);
            //    }
            //    else if (runData.method == "VRP")
            //    {
            //        SolveVRP(datasetName, runData.maxTime, true);
            //    }
            //}
        }

        public static void SolveVRP(string dataset, int maxTime, bool extractSol = false)
        {
            // Parameters
            bool silence = false;
            string sol_extraction_path = "../../../extracted_solutions/mls/";
            string sol_save_path = "../../../extracted_solutions/vrp/";
            string datasetName = dataset.Split(separator: "/").Last();
            String solutionFileName = datasetName.Substring(0, datasetName.Length - 5) + "_final.txt";
            //C:\MyWork\Repos\MSOP\MSOP\extracted_solutions\mls\11berlin52_T100_p2_v2_final

            // Read instance and initialize
            Dataset_Reader reader = new Dataset_Reader();
            Model m = reader.Read_Dataset(dataset);
            m.Build(randomSeed, unit_clusters);
            Console.WriteLine("Total available profit: " + m.total_available_profit);
            Console.WriteLine("Max route duration (t_max): " + m.t_max);

            if (!File.Exists(sol_extraction_path + solutionFileName))
            {
                Solve(dataset, extractSol);
            }

            Solution parsedSol = Solution.ReadSolutionFromFile(sol_extraction_path, solutionFileName);

            // Solve the VRP
            Stopwatch localsolver_total_time = new Stopwatch(); // counts the total time of the localsolver
            localsolver_total_time.Start();
            VRPLocalSolver vrpsolver = new VRPLocalSolver();
            Solution final_sol = vrpsolver.solveVRP(m, parsedSol, maxTime, silence);
            localsolver_total_time.Stop();
            // Update execution data
            runData.execTime = localsolver_total_time.Elapsed.TotalSeconds;

            // extract best of all restarts
            if (extractSol)
            {
                Report.ExtractVRPSolutionInformation(final_sol, runData, m, sol_save_path + m.dataset_name + "_vrp.txt");
                //vrpsolver.WriteSolution(final_sol, m, sol_extraction_path + m.dataset_name + "_vrp.txt");
                //vrpsolver.WriteSolution(sol_extraction_path + m.dataset_name + "_vrp.txt");
            }
            Console.WriteLine("\nVRP: Optimization of instance " + dataset + " is finished");
            Console.WriteLine("=====================================================================\n\n");
        }

        public static void Solve(string dataset, bool extractSol = false)
        {

            // Parameters
            string sol_extraction_path = "../../../extracted_solutions/mls/";
            int restarts = 2; // 10? 

            // 1. Read instance and initialize
            Dataset_Reader reader = new Dataset_Reader();
            Model m = reader.Read_Dataset(dataset);
            m.Build(randomSeed, unit_clusters);
            Console.WriteLine("Total available profit: " + m.total_available_profit);
            Console.WriteLine("Max route duration (t_max): " + m.t_max);

            Stopwatch Heur_total_time =  new Stopwatch(); // counts the total time of the local search
            Heur_total_time.Start();


            Solution bestSol = new Solution();
            int promiseRestart = Math.Max(40, 2 * Model.model.set_crowd);
            int maxNonImpIters = Math.Max(10000, 15 * Model.model.node_crowd);

            for (int res = 0; res < restarts; res++)
            {
                Stopwatch res_time = new Stopwatch(); // counts the total time of the local search
                res_time.Start();

                // 2. Construct initial solution
                Solution starting_sol = Initialization.Minimum_Insertions(RCL_SIZE);
                //if (extractSol)
                //{
                //    Report.ExtractSolutionInformation(starting_sol, runData, m, sol_extraction_path + m.dataset_name + "_starting.txt");
                //}

                // 3. Improve the solution
                Solution final_sol = Local_Search.GeneralLocalSearch(starting_sol, promiseRestart, maxNonImpIters);
                final_sol.CheckSol();
                res_time.Stop();


                if (final_sol.total_profit > bestSol.total_profit)
                {
                    bestSol = final_sol.DeepCopy();

                    // Update execution data
                    runData.restartOfBest = res;
                    runData.UB_end = bestSol.total_profit;
                    runData.status = "Feasible";
                    runData.BestBound_end = m.total_available_profit;
                    runData.totalIters = bestSol.iterations_of_local_search;
                    runData.iterOfBest = bestSol.iteration_best_found;
                    runData.duration_of_maths = bestSol.duration_of_maths;
                    runData.duration_of_restart = res_time.Elapsed.TotalSeconds;
                }
            }

            // write time
            Heur_total_time.Stop();
            runData.execTime = Heur_total_time.Elapsed.TotalSeconds;

            // extract best of all restarts
            if (extractSol)
            {
                Report.ExtractSolutionInformation(bestSol, runData, m, sol_extraction_path + m.dataset_name + "_final.txt");
            }
            Console.WriteLine("\nOptimization of instance " + dataset + " is finished");
            Console.WriteLine("=====================================================================\n\n");
        }

        static void SolveMSOPTCF(string dataset, string solver, bool extractSol, int maxTime)
        {
            // Parameters
            string sol_extraction_path = "../../../extracted_solutions/exact/";

            // 1. Read instance and initialize
            Dataset_Reader reader = new Dataset_Reader();
            Model m = reader.Read_Dataset(dataset);
            m.Build(randomSeed, unit_clusters);
            Console.WriteLine("Total available profit: " + m.total_available_profit);
            Console.WriteLine("Max route duration (t_max): " + m.t_max);


            // 2. Solve the exact two commodity flow formulation
            Solution final_sol = null;
            if (solver == "GRB")
            {
                final_sol = GRB_TCFFormulation.solveTCFMSOPGRB(m, maxTime);
            }
            else if (solver == "CPX")
            {
                final_sol = CPX_TCFFormulation.solveTCFMSOPCPX(m, maxTime);
            }
            else
            {
                Console.WriteLine("Solver {0} is not available. Try GRB for Gurobi or CPX for ILOG CPLEX", solver);
            }

            final_sol.CheckSol();
            if (extractSol)
            {
                Report.ExtractSolutionInformation(final_sol, runData, m, sol_extraction_path + m.dataset_name + "_TCF_final.txt");
            }
            Console.WriteLine("\nOptimization of instance " + dataset + " is finished");
            Console.WriteLine("=====================================================================\n\n");
        }


        static void ParametersSelection()
        {
            Dataset_Reader reader = new Dataset_Reader();
            string datasets_path = "C:/Users/georg/Desktop/thesis/MSOP/MSOP/MSOP_datasets/final";
            string parameters_report_path = "C:/Users/georg/Desktop/thesis/MSOP/MSOP/parameters_reports/";
            foreach (string folder in Directory.EnumerateDirectories(datasets_path))
            {
                Dictionary<string, List<Solution>> datasets_solutions = new Dictionary<string, List<Solution>>();
                foreach (string dataset in Directory.EnumerateFiles(folder))
                {
                    datasets_solutions[dataset] = new List<Solution>();
                    Model m = reader.Read_Dataset(dataset);
                    m.Build(randomSeed, unit_clusters);
                    for (int i = 0; i < 10; i++)
                    {
                        Solution starting_sol = Initialization.Minimum_Insertions(RCL_SIZE);
                        Solution final_sol = Local_Search.GeneralLocalSearch(starting_sol, 3 * Model.model.set_crowd, 10 * Model.model.node_crowd);
                        final_sol.CheckSol();
                        datasets_solutions[dataset].Add(final_sol);
                    }
                    Console.WriteLine(dataset + " finished");
                }
                string parameters_name = folder.Split('\\').Last();
                Report.ExtractParametersSelectionReport(datasets_solutions, parameters_report_path + parameters_name + "_report");
                Console.WriteLine(folder + " finished");
            }
        }

        static void PromiseTargetTuning()
        {
            Dataset_Reader reader = new Dataset_Reader();
            string datasets_path = "./final";
            string promises_report_path = "./promises_reports/";
            int[] k_values = new int[] { 1, 2, 3, 4, 5 };
            Dictionary<string, List<Solution>> k_solutions = new Dictionary<string, List<Solution>>();

            foreach (string dataset in Directory.EnumerateFiles(datasets_path))
            {
                Model m = reader.Read_Dataset(dataset);
                m.Build(randomSeed, unit_clusters);
                k_solutions[Model.model.dataset_name] = new List<Solution>();
                foreach (int k in k_values)
                {
                    Solution k_sol = AlgorithmMultipleRuns(3, k * Model.model.set_crowd, 10, 10 * Model.model.node_crowd);
                    k_solutions[Model.model.dataset_name].Add(k_sol);
                }
                Console.WriteLine(dataset + " finished");
            }
            Report.ExtractPromisesTargetTuningReport(k_solutions, k_values, promises_report_path + "promises_target_tuning_report");
        }

        static void IntensificationDiversificationTuning()
        {
            Dataset_Reader reader = new Dataset_Reader();
            string datasets_path = "./final";
            string intens_divers_report_path = "./intensification_diversification_tuning_reports/";
            int tuned_promise_target;
            int[] l_values = new int[] { 2, 5, 10, 20, 50 };
            int[] restarts_values = new int[] { 50, 20, 10, 5, 2 };
            Dictionary<string, List<Solution>> generated_solutions = new Dictionary<string, List<Solution>>();

            foreach (string dataset in Directory.EnumerateFiles(datasets_path))
            {
                Model m = reader.Read_Dataset(dataset);
                m.Build(randomSeed, unit_clusters);
                tuned_promise_target = 2 * Model.model.set_crowd;
                generated_solutions[Model.model.dataset_name] = new List<Solution>();
                for (int i = 0; i < l_values.Length; i++)
                {
                    int l = l_values[i];
                    int restarts = restarts_values[i];
                    Solution generated_sol = AlgorithmMultipleRuns(3, tuned_promise_target, restarts, l * Model.model.node_crowd);
                    generated_solutions[Model.model.dataset_name].Add(generated_sol);
                }
                Console.WriteLine(dataset + " finished");
            }
            Report.ExtractIntensificationDiversificationTuningReport(generated_solutions, l_values, restarts_values,
                intens_divers_report_path + "intensification_diversification_tuning_report");
        }

        static void TunedAlgorithmReport()
        {
            Dataset_Reader reader = new Dataset_Reader();
            string datasets_path = "./final";
            string final_report_path = "./final_reports/";
            int tuned_promise_target;
            int tuned_non_improved_iterations;
            int tuned_restarts = 5;
            Dictionary<string, List<Solution>> generated_solutions = new Dictionary<string, List<Solution>>();

            foreach (string dataset in Directory.EnumerateFiles(datasets_path))
            {
                Model m = reader.Read_Dataset(dataset);
                m.Build(randomSeed, unit_clusters);
                tuned_promise_target = 2 * Model.model.set_crowd;
                tuned_non_improved_iterations = 20 * Model.model.node_crowd;
                generated_solutions[Model.model.dataset_name] = new List<Solution>();
                for (int i = 0; i < 5; i++)
                {
                    Solution run_sol = AlgorithmSingleRun(tuned_promise_target, tuned_restarts, tuned_non_improved_iterations);
                    generated_solutions[Model.model.dataset_name].Add(run_sol);
                }
                Console.WriteLine(dataset + " finished");
            }
            Report.ExtractTunedAlgorithmReport(generated_solutions, 5, final_report_path + "tuned_algorithm_report");
        }

        static Solution AlgorithmSingleRun(int promise_target, int restarts, int not_improved_iterations)
        {
            Stopwatch watch_for_total_time = System.Diagnostics.Stopwatch.StartNew(); // counts the total time of the local search
            Solution best_sol = null;
            int best_sol_profit = 0;
            for (int i = 0; i < restarts; i++)
            {
                Solution starting_sol = Initialization.Minimum_Insertions(RCL_SIZE);
                Solution final_sol = Local_Search.GeneralLocalSearch(starting_sol, promise_target, not_improved_iterations);
                final_sol.CheckSol();
                if (final_sol.total_profit > best_sol_profit)
                {
                    best_sol = final_sol;
                    best_sol_profit = final_sol.total_profit;
                }
            }
            watch_for_total_time.Stop();
            best_sol.duration_total = watch_for_total_time.ElapsedMilliseconds / 1000.0;
            return best_sol;
        }

        static Solution AlgorithmMultipleRuns(int runs, int promise_target, int restarts, int not_improved_iterations)
        {
            Solution best_sol = null;
            int best_sol_profit = 0;
            for (int i = 0; i < runs; i++)
            {
                Solution run_sol = AlgorithmSingleRun(promise_target, restarts, not_improved_iterations);
                if (run_sol.total_profit > best_sol_profit)
                {
                    best_sol = run_sol;
                    best_sol_profit = run_sol.total_profit;
                }
            }
            return best_sol;
        }

        static void DatasetsTotalProfits()
        {
            Dataset_Reader reader = new Dataset_Reader();
            string datasets_path = "./final";
            string final_report_path = "./final_reports/";
            Dictionary<string, double> datasets_total_profits = new Dictionary<string, double>();

            foreach (string dataset in Directory.EnumerateFiles(datasets_path))
            {
                Model m = reader.Read_Dataset(dataset);
                m.Build(randomSeed, unit_clusters);
                datasets_total_profits[m.dataset_name] = m.total_available_profit;
            }
            Report.ExtractDatasetsTotalProfit(datasets_total_profits, final_report_path + "datasets_total_profits");
        }
    }
}
