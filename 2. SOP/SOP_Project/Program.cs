using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;

namespace SOP_Project
{
    class Program
    {
        static void Main(String[] args)
        {
            //string dataset_path = "Datasets/diverse/132d657_T40_p2.sop";
            //Model m = new Dataset_Reader().Read_Dataset(dataset_path).Build();
            //Console.WriteLine(m);
            //Pool.Initialize(m);
            //for (int seed = 1; seed < 21; seed++)
            //{
            //    m.r = new Random(seed);
            //    Solution sol = Initialization.Minimum_Insertions(m);
            //    Pool.sol_pool.Add(sol);
            //    Console.WriteLine(sol.total_profit);
            //}
            //SyntacticMatching.PoolToSetSeq(Pool.sol_pool);
            ////foreach (int i in SyntacticMatching.pool_as_set_seq)
            ////{
            ////    Console.Write(i + "-");
            ////}
            //Console.WriteLine("\n");
            //Stopwatch watch_for_total_time = System.Diagnostics.Stopwatch.StartNew();
            //Pool.UpdateFrequencies(m);
            //Pool.AdjustPoolCosts(m);
            //for (int i = 1; i < 6; i++)
            //{
            //    Pool.SelectMostFrequentChains(m);
            //    m.r = new Random(1);
            //    Console.WriteLine(Initialization.SolveAsKnapsack_Infeasible(m));
            //}
            //watch_for_total_time.Stop(); // stop the watch that counts total local search time
            //double time = watch_for_total_time.ElapsedMilliseconds / 1000.0;
            //Console.WriteLine("Chain list Length = " + Chain.all_chains.Count);
            //Console.WriteLine("Time: " + time);
            //Console.WriteLine("Pool Differentiation: " + Pool.DifferentiationMetric() + "%");

            //string dataset_path = "Datasets/diverse/132d657_T40_p2.sop";
            //string dataset_path = "Datasets/diverse/20kroA100_RND_T40_p2.sop";
            string dataset_path = "./Datasets/diverse/217vm1084_T40_p1.sop";
            //string dataset_path = "./Datasets/large H/sop/608pcb3038_T100_p1.sop";


            //Report.ReadAllReports("../Reports/promises/target_coef_15");

            double pr_target_coefficient = 3;
            int n_tests = 5;
            string dir_path = "./Datasets/large H/sop/";
            dir_path = "./Datasets/diverse/";
            //dir_path = "./Datasets/T100/";
            if (args.Length != 0)
            {
                pr_target_coefficient = Int32.Parse(args[0]);
                n_tests = Int32.Parse(args[1]);
                dir_path = args[2].ToString();
            }

            //foreach (string dataset in Directory.GetFiles(dir_path))
            //{
            //    RunSummaryReport(dataset, pr_target_coefficient, pr_target_coefficient, 200, n_tests);
            //}

            // test promises
            foreach (string dataset in Directory.GetFiles(dir_path))
            {
                RunSummaryReport(dataset, 1.5, 4.5, 0.5, n_tests);
            }

            //List<string> datasets_failed = new List<string>();
            //foreach (string dataset in Directory.GetFiles(dir_path))
            //{
            //    try
            //    {
            //        if (dataset.Contains("T100") && !dataset.Contains("RND"))  // run for all non-random T100
            //        {
            //            RunSummaryReport(dataset, pr_target_coefficient, pr_target_coefficient, 200, n_tests);
            //        }
            //    }
            //    catch (Exception e)
            //    {
            //        datasets_failed.Add(dataset);
            //    }
            //}
            //if (datasets_failed.Count > 0)
            //{
            //    StreamWriter writer = new StreamWriter("failed_runs.txt", true);
            //    foreach (string dataset in datasets_failed)
            //    {
            //        writer.WriteLine(dataset);
            //    }
            //    writer.Close();
            //}


            //RunAllReports(pr_target_coefficient, n_tests);
            //RunSummaryReport(dataset_path, pr_target_coefficient, pr_target_coefficient, 200, n_tests);

            // RunSingleTest(dataset_path, 600, 11);
            // CountStuff.Initialize();

            //Model m = new Dataset_Reader().Read_Dataset(dataset_path).Build();
            //for (int n = 13; n < 14; n++)
            //{
            //    Console.WriteLine("Starting test " + n);
            //    Solution sol = RunSingleTest(dataset_path, 3 * m.set_crowd, n);
            //    Console.WriteLine("Best overall:" + sol);
            //    Console.WriteLine("Total time:{0} Gurobi Time:{1}", sol.duration_of_local_search, sol.duration_of_maths);
            //}

            //for (int pr_target = 500; pr_target <= 600; pr_target += 100)
            //{
            //    CreateInstances(dataset_path, pr_target, n_tests);
            //}
            //Console.WriteLine("Total times lkh faield: {0} in {1} tests in total", CountStuff.times_lkh_failed, 23 * n_tests);
            //Console.WriteLine("Total times lkh gave different objective: {0} in {1} tests in total", CountStuff.wrong_obj_in_lkh, 23 * n_tests);

        }

        public static void RunAllReports(int pr_target_coefficient, int n_tests)
        {
            string dir_path = "./Datasets/diverse/";
            foreach (string dataset in Directory.GetFiles(dir_path))
            {
                RunSummaryReport(dataset, pr_target_coefficient, pr_target_coefficient, 200, n_tests);
            }

            // make an accumulated excel report out of the individual reports
            Report.ReadAllReports("./Summary_Reports");

        }

        public static void RunSummaryReport(string dataset_path, double promise_target_lower_bound, double promise_target_upper_bound,
            double step, int runs_for_each_target)
        {
            Model m = new Dataset_Reader().Read_Dataset(dataset_path).Build();

            string dataset_name = dataset_path[(dataset_path.LastIndexOfAny(new char[] { '/', '\\' }) + 1)..];
            dataset_name = dataset_name.Replace(".sop", "");
            m.dataset_name = dataset_name;

            // print basic information about the dataset
            Console.WriteLine(m);

            Stopwatch watch_for_constructive = new Stopwatch();
            Solution sol;
            string signature;

            for (double pr_target = promise_target_lower_bound; pr_target <= promise_target_upper_bound; pr_target += step)
            {
                for (int j = 0; j < runs_for_each_target; j++)
                {
                    // create signature for summary report file
                    signature = dataset_name + "_target_" + pr_target + "_seed_" + j;
                    Console.WriteLine("Run " + j);
                    Pool.Initialize(m);
                    //Chain.Initialize_Chains(m);
                    m.r = new Random(j);
                    watch_for_constructive.Start();
                    sol = Initialization.Minimum_Insertions(m);
                    watch_for_constructive.Stop();
                    sol.duration_of_constructive = watch_for_constructive.ElapsedMilliseconds / 1000.0;
                    sol = Local_Search.GeneralLocalSearch(m, sol, (int)Math.Round(pr_target * m.set_crowd));
                    string dir = "Summary_Reports/target_coef_" + pr_target + "/";
                    Solution.SummaryReportAsTxt(Local_Search.sols, dir, signature);
                    // initialize sols to prepare for next iteration
                    Local_Search.sols.Clear();
                }
            }
        }

        public static Dictionary<string, int> GetLiteratureBests()
        {
            Dictionary<string, int> sols = new Dictionary<string, int>();
            string[] lines = File.ReadAllLines("./Datasets/instances.csv");
            foreach (string line in lines[2..])
            {
                string[] pair = line.Split(";");
                sols.Add(pair[0], Int32.Parse(pair[1]));
            }
            return sols;
        }

        public static Solution RunSingleTest(string dataset, int pr_target, int seed)
        {
            Model m = new Dataset_Reader().Read_Dataset(dataset).Build();
            string dataset_name = dataset.Replace("./Datasets/diverse/", "");
            dataset_name = dataset_name.Replace(".sop", "");
            m.dataset_name = dataset_name;

            Pool.Initialize(m);
            Console.WriteLine(m);
            //Chain.Initialize_Chains(m);
            m.r = new Random(seed);
            Solution sol = Initialization.Minimum_Insertions(m);
            sol = Local_Search.GeneralLocalSearch(m, sol, pr_target);

            return sol;
        }
    }
}
