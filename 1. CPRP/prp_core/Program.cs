using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Security.Cryptography.X509Certificates;
using System.Text;
using System.Threading.Tasks;

namespace PRP
{

    class Program
    {
        private static int customers = 25;
        private static int periods = 3;
        private static int vehicles = 3;
        private static int index = 3;
        private static bool profiling = false;

        static void Main(string[] args)
        {
            LP.initEnv();
            MIP.initEnv();

            //Setup options
            int allowedThreads = 1;
            MIP.threadLimit = allowedThreads;
            LP.threadLimit = allowedThreads;
            MathematicalProgramming.threadLimit = allowedThreads;
            ProductionRelaxation.threadLimit = allowedThreads;
            TSP_GRB_Solver.threadLimit = allowedThreads;
            ProductionRelaxation.threadLimit = allowedThreads;

            //Suppress output
            GlobalUtils.suppress_output = false;

            if (profiling)
            {
                //Set local parameters
                LocalSearch.parameters.timeLimit = 36000000;
                LocalSearch.parameters.nonImprovIterLim = 10000;
                LocalSearch.parameters.nonImprovDiveLim = 20;
                LocalSearch.parameters.MIPBigGapLimit = 0.1;
                PRP.restarts = 20;

                RunAllBoudia(true);
                //RunAllArchetti(true);

                LP.disposeEnv();
                MIP.disposeEnv();

                return;
            }

            //Get Parameters from cmd
            if (args[0] == "exact")
            {
                // example: exact 120 cyclic true true true
                MathematicalProgramming.exactParams.algorithm = args[0];
                MathematicalProgramming.exactParams.timeLimit = int.Parse(args[1]);
                MathematicalProgramming.exactParams.periodicity = args[2]; //basic: plain PRP,  periodic: PRP with final inventories equal to starting ones. cyclic= PRP equal starting and final inventories that are decision variables 
                MathematicalProgramming.exactParams.validsInequalities = bool.Parse(args[3]);
                MathematicalProgramming.exactParams.limitedVehicles = bool.Parse(args[4]); // the max routes stored as solution to PRP
                MathematicalProgramming.exactParams.adjustedCyclicInventories = bool.Parse(args[5]); // the inventories stores as solution to the CPRP unlimited
            }
            else if (args[0] == "ls") //legacy?
            {
                // prp example: ls 3600000 3000 2 0.15 2 1 2 1 1
                LocalSearch.parameters.algorithm = args[0];
                LocalSearch.parameters.timeLimit = int.Parse(args[1]);
                LocalSearch.parameters.nonImprovIterLim = int.Parse(args[2]);
                LocalSearch.parameters.nonImprovDiveLim = int.Parse(args[3]);
                LocalSearch.parameters.MIPBigGapLimit = double.Parse(args[4], CultureInfo.InvariantCulture);
                LocalSearch.parameters.whenEnterMultiplier = int.Parse(args[5]);
                LocalSearch.parameters.whenExitMultiplier = int.Parse(args[6]);
                LocalSearch.parameters.minRepairMoves = int.Parse(args[7]);
                LocalSearch.parameters.maxRepairMoves = int.Parse(args[8]);
                PRP.restarts = int.Parse(args[8]);
            }



            bool useDefault = AskYesNoQuestion("Use default instance parameters?");
            
            Console.WriteLine("Select Instance Set (ARCH, BOUD, ADUL, BOUD_TEST)");
            string input = (Console.ReadLine());
            
            if (input == "ADUL")
                RunAllAdulyasak(useDefault);
            else if (input == "ARCH")
                RunAllArchetti(useDefault);
            else if (input == "BOUD")
                RunAllBoudia(useDefault);
            else if (input == "BOUD_TEST")
                RunAllBoudiaTesting();
            else
                Console.WriteLine("Benchmark set not Implemented Yet");
            
            LP.disposeEnv();
            MIP.disposeEnv();
        }

        private static void RunAllAdulyasak(bool useDefault)
        {
            customers = 10;
            periods = 9;
            vehicles = 2;
            index = 4;

            var productionScheduleDirFMT =
                "../../../Data/ProductionSchedules/RelSol_Adu_all regular (no saw)/RelSol_Adu_MVPRP_C{0}_P{1}_V{2}_I{3}.DAT";
            var summary_file_nameFMT = "RelSol_Adu_MVPRP_C{0}_P{1}_V{2}_I{3}.DAT_summary.txt";
            var results_summary_file_nameFMT =
                "RelSol_Adu_MVPRP_C{0}_P{1}_V{2}_I{3}.DAT_results_summary.txt";
            var prodSchedulePrefix = "RelSol_Adu_MVPRP_C{0}_P{1}_V{2}_I{3}.DAT_";
            var baseInstanceNameFMT = "MVPRP_C{0}_P{1}_V{2}_I{3}.dat";
            var fullPathInstanceNameFMT = "../../../Data/Data/PRP/data/MVPRP_Dataset_Rev/DATA_MVPRP_Rev/";
            
            if (!useDefault)
            {
                Console.WriteLine("Add periods range [3,6,9]");
                int per_start = int.Parse(Console.ReadLine());
                int per_end = int.Parse(Console.ReadLine());

                Console.WriteLine("Add vehicles range [2,3,4]");
                int vehicles_start = int.Parse(Console.ReadLine());
                int vehicles_end = int.Parse(Console.ReadLine());

                Console.WriteLine("Add customers range [10,15,20,25,30,35,40,45,50]");
                int customers_start = int.Parse(Console.ReadLine());
                int customers_end = int.Parse(Console.ReadLine());


                for (int i = 0; i < DataLoader.adul_instaces.Count; i++)
                {
                    Dictionary<string, int> instance_descr = DataLoader.adul_instaces[i];

                    // periods 3/6/9, customers 10-15-20-25-30-35-40-45-50, v2-4, i1-4
                    if (!(per_start <= instance_descr["periods"] && instance_descr["periods"]  <= per_end))
                        continue;

                    if (!(vehicles_start <= instance_descr["vehicles"] && instance_descr["vehicles"] <= vehicles_end))
                        continue;

                    if (!(customers_start <= instance_descr["customers"] && instance_descr["customers"] <= customers_end))
                        continue;

                    RunManyProductionSchedules(productionScheduleDirFMT, summary_file_nameFMT, results_summary_file_nameFMT, 
                        prodSchedulePrefix, fullPathInstanceNameFMT, baseInstanceNameFMT,
                        instance_descr["customers"], instance_descr["periods"], 
                        instance_descr["vehicles"], instance_descr["index"], PRP_DATASET_VERSION.ADULYASAK_FMT, false);
                }    
            }
            else
            {
                RunManyProductionSchedules(productionScheduleDirFMT, summary_file_nameFMT, results_summary_file_nameFMT,
                    prodSchedulePrefix, fullPathInstanceNameFMT, baseInstanceNameFMT,
                    customers, periods, vehicles, index, PRP_DATASET_VERSION.ADULYASAK_FMT, false);
            }
        }

        private static bool AskYesNoQuestion(string text)
        {
            Console.WriteLine(text + " (y/n)");
            string input = Console.ReadLine();

            if (input == "y" || input == "Y")
                return true;
            return false;

        }
        
        private static void RunAllArchetti(bool useDefault)
        {
            List<int> customerSet = new List<int>();
            int j_start = 0, j_end = 0, k_start = 0, k_end = 0;
            string input;
            bool question_status;
            if (!useDefault)
            {
                if (profiling)
                {
                    j_start = 1;
                    j_end = 96;
                    k_start = 1;
                    k_end = 5;
                    customerSet.Add(15);
                    customerSet.Add(50);
                    customerSet.Add(100);
                } else
                {
                    Console.WriteLine("Add range for j (main instance index counter)");
                    j_start = int.Parse(Console.ReadLine());
                    j_end = int.Parse(Console.ReadLine());

                    Console.WriteLine("Add range for k (instance subcounter)");
                    k_start = int.Parse(Console.ReadLine());
                    k_end = int.Parse(Console.ReadLine());

                    while (true)
                    {
                        question_status = AskYesNoQuestion("Add extra customer set?");

                        if (question_status)
                        {
                            Console.WriteLine("Enter Vehicle Amount");
                            int vehnum = int.Parse(Console.ReadLine());
                            customerSet.Add(vehnum);
                        }
                        else
                            break;
                    }
                }   


                for (int i = 0; i < customerSet.Count; i++)
                {
                    int vehicles = customerSet[i];
		            
                    //if (vehicles != 100) continue;
                    
                    for (int j = j_start; j <= j_end; j++)
                    {
                        for (int k = k_start; k <= k_end; k++)
                        {
                            //Set directory maths and format strings
                            var productionScheduleDirFMT =
                                String.Format("../../../Data/ProductionSchedules/RelSol_Arc_all regular (no saw)/RelSol_Arc_ABS{0}_{{2}}_{{3}}.DAT",
                                    j);
                            var summary_file_nameFMT = String.Format("RelSol_Arc_ABS{0}_{{2}}_{{3}}.DAT_summary.txt", j);
                            var results_summary_file_nameFMT =
                                String.Format("RelSol_Arc_ABS{0}_{{2}}_{{3}}.DAT_results_summary.txt", j);
                            var prodSchedulePrefix = String.Format("RelSol_Arc_ABS{0}_{{2}}_{{3}}.DAT_", j);
                            var baseInstanceNameFMT = String.Format("ABS{0}_{{2}}_{{3}}.DAT", j);
                            var fullPathInstanceNameFMT = "../../../Data/Data/PRP/data/Archetti_et_al/DATI_" + vehicles + "/";
                            
                            RunManyProductionSchedules(productionScheduleDirFMT, summary_file_nameFMT, results_summary_file_nameFMT, 
                                prodSchedulePrefix, fullPathInstanceNameFMT, baseInstanceNameFMT,
                                -1, -1, vehicles, k, PRP_DATASET_VERSION.ARCHETTI_FMT, false);
                            
                        }    
                    }
                }
            }
            else
            {
                //Override defaults
                int j = 68;
                int k = 1;
                vehicles = 15;
                
                //Set directory maths and format strings
                var productionScheduleDirFMT =
                    String.Format("../../../Data/ProductionSchedules/RelSol_Arc_all regular (no saw)/RelSol_Arc_ABS{0}_{{2}}_{{3}}.DAT",
                        j);
                var summary_file_nameFMT = String.Format("RelSol_Arc_ABS{0}_{{2}}_{{3}}.DAT_summary.txt", j);
                var results_summary_file_nameFMT =
                    String.Format("RelSol_Arc_ABS{0}_{{2}}_{{3}}.DAT_results_summary.txt", j);
                var prodSchedulePrefix = String.Format("RelSol_Arc_ABS{0}_{{2}}_{{3}}.DAT_", j);
                var baseInstanceNameFMT = String.Format("ABS{0}_{{2}}_{{3}}.DAT", j);
                var fullPathInstanceNameFMT = "../../../Data/Data/PRP/data/Archetti_et_al/DATI_" + vehicles + "/";
                            
                RunManyProductionSchedules(productionScheduleDirFMT, summary_file_nameFMT, results_summary_file_nameFMT, 
                    prodSchedulePrefix, fullPathInstanceNameFMT, baseInstanceNameFMT,
                    -1, -1, vehicles, k, PRP_DATASET_VERSION.ARCHETTI_FMT, false);
            }
        }

        private static void RunAllBoudiaTesting()
        {
            List<int> customerSet = new List<int>();
            int j_start = 0, j_end = 0;
            string input;
            
            customerSet.Add(50);
            customerSet.Add(100);
            customerSet.Add(200);

            Dictionary<int, List<int>> testInstances = new Dictionary<int, List<int>>();
            testInstances[50] = new List<int> { 9, 30 };
            testInstances[100] = new List<int> { 20, 27 };
            testInstances[200] = new List<int> { 9, 30 };

            j_start = 1;
            j_end = 30;

            for (int i = 0; i < customerSet.Count; i++)
            {
                int customerNum = customerSet[i];

                for (int j = j_start; j <= j_end; j++)
                {
                    if (!(testInstances[customerNum].Contains(j)))
                        continue;
                    //Set directory maths and format strings
                    var productionScheduleDirFMT =
                        String.Format("../../../Data/ProductionSchedules/RelSol_Bou_all regular (no saw)/RelSol_Bou_{0}_instance{1}",
                            customerNum, j);
                    var summary_file_nameFMT = String.Format("RelSol_Bou_instance{0}_summary.txt", j);
                    var results_summary_file_nameFMT =
                        String.Format("RelSol_BOU_{0}_{1}.DAT_results_summary.txt", customerNum, j);
                    var prodSchedulePrefix = String.Format("RelSol_Bou_instance{0}_", j);
                    var baseInstanceNameFMT = String.Format("instance{0}", j);
                    var fullPathInstanceNameFMT = "../../../Data/Data/PRP/data/Boudia_et_al/" + customerNum + " Clients/";

                    RunManyProductionSchedules(productionScheduleDirFMT, summary_file_nameFMT, results_summary_file_nameFMT,
                        prodSchedulePrefix, fullPathInstanceNameFMT, baseInstanceNameFMT,
                        -1, -1, -1, -1, PRP_DATASET_VERSION.BOUDIA_FMT, false);
                }
            }
            
        }

        private static void RunAllBoudia(bool useDefault)
        {
            if (!useDefault)
            {

                List<int> customerSet = new List<int>();
                int j_start = 0, j_end = 0;
                string input;
                bool question_status;
                
                if (profiling)
                {
                    j_start = 1;
                    j_end = 30;
                    customerSet.Add(50);
                    customerSet.Add(100);
                    customerSet.Add(200);
                }
                else
                {
                    Console.WriteLine("Add range for j (main instance index counter)");
                    j_start = int.Parse(Console.ReadLine());
                    j_end = int.Parse(Console.ReadLine());

                    while (true)
                    {
                        question_status = AskYesNoQuestion("Add extra customer set?");
                        if (question_status)
                        {
                            Console.WriteLine("Enter Customer Count");
                            int custnum = int.Parse(Console.ReadLine());
                            customerSet.Add(custnum);
                        }
                        else
                            break;
                    }
                }

                for (int i = 0; i < customerSet.Count; i++) {
                    int customerNum = customerSet[i];
		            
                    //if (customerNum != 50) continue;
                    
                    for (int j = j_start; j <= j_end; j++) {
                        //Set directory maths and format strings
                        var productionScheduleDirFMT =
                            String.Format("../../../Data/ProductionSchedules/RelSol_Bou_all regular (no saw)/RelSol_Bou_{0}_instance{1}",
                                customerNum, j);
                        var summary_file_nameFMT = String.Format("RelSol_Bou_instance{0}_summary.txt", j);
                        var results_summary_file_nameFMT =
                            String.Format("RelSol_BOU_{0}_{1}.DAT_results_summary.txt", customerNum, j);
                        var prodSchedulePrefix = String.Format("RelSol_Bou_instance{0}_", j);
                        var baseInstanceNameFMT = String.Format("instance{0}", j);
                        var fullPathInstanceNameFMT = "../../../Data/Data/PRP/data/Boudia_et_al/" + customerNum + " Clients/";
                            
                        RunManyProductionSchedules(productionScheduleDirFMT, summary_file_nameFMT, results_summary_file_nameFMT, 
                            prodSchedulePrefix, fullPathInstanceNameFMT, baseInstanceNameFMT,
                            -1, -1, -1, -1, PRP_DATASET_VERSION.BOUDIA_FMT, false);
                    }
                }
            }
            else
            {
                //Override defaults
                int customerNum = 100;
                int j = 4;
                
                //Set directory maths and format strings
                var productionScheduleDirFMT =
                    String.Format("../../../Data/ProductionSchedules/RelSol_Bou_all regular (no saw)/RelSol_Bou_{0}_instance{1}",
                        customerNum, j);
                //var productionScheduleDirFMT =
                //    String.Format("../../../Data/ProductionSchedules/RelSol_Bou_all regular (no saw) with k 3_5_5min/RelSol_Bou_{0}_instance{1}",
                //        customerNum, j);
                var summary_file_nameFMT = String.Format("RelSol_Bou_instance{0}_summary.txt", j);
                var results_summary_file_nameFMT =
                    String.Format("RelSol_BOU_{0}_{1}.DAT_results_summary.txt", customerNum, j);
                var prodSchedulePrefix = String.Format("RelSol_Bou_instance{0}_", j);
                var baseInstanceNameFMT = String.Format("instance{0}", j);
                var fullPathInstanceNameFMT = "../../../Data/Data/PRP/data/Boudia_et_al/" + customerNum + " Clients/";
                            
                RunManyProductionSchedules(productionScheduleDirFMT, summary_file_nameFMT, results_summary_file_nameFMT, 
                    prodSchedulePrefix, fullPathInstanceNameFMT, baseInstanceNameFMT,
                    -1, -1, -1, -1, PRP_DATASET_VERSION.BOUDIA_FMT, false);
            }
        }
                
        private static void RunManyProductionSchedules(string productionScheduleDirFMT, string summary_file_nameFMT, 
            string results_summary_file_nameFMT, string prodSchedulePrefix, string fullPathInstanceNameFMT, string BaseInstanceNameFMT,
            int customers, int periods, int vehicles, int index, PRP_DATASET_VERSION instanceFMT, bool prodschedule_has_routes)
        {
            int schedulesNum = 1;

            string instanceFullpathName = GenerateFullPathInstanceName(fullPathInstanceNameFMT, BaseInstanceNameFMT,
                customers, periods, vehicles, index);
            string instanceName = GenerateBaseInstanceName(BaseInstanceNameFMT, customers, periods, vehicles, index);
            Console.WriteLine("Solving Instance: " + instanceName);

            string productionScheduleDir = String.Format(productionScheduleDirFMT,
                customers, periods, vehicles, index);

            string summary_file_name = String.Format(summary_file_nameFMT,
                customers, periods, vehicles, index);
            
            string results_summary_file_name = String.Format(results_summary_file_nameFMT,
                customers, periods, vehicles, index);
            
            //Try to load summary file
            string summary_path_file = Path.Join(productionScheduleDir, summary_file_name);

            if (!File.Exists(summary_path_file))
            {
                //throw new Exception("Summary file does not exist");
				Console.WriteLine("Summary file {0} does not exist. Aborting instance", summary_path_file);
				return;
            } else {
		        Console.WriteLine("Working on summary file: {0}", summary_path_file);
            }

            //Open summery file
            StreamReader ss = File.OpenText(summary_path_file);


            SolutionResults[] result_array = new SolutionResults[10];
            
            //Fetch first 10 production schedules
            int active_pr_schedule_count = 0;
            
            for (int i = 0; i < schedulesNum; i++)
            {
                string line = ss.ReadLine();
                
                if (line == "" || line is null)
			        continue;

                /*
                if (i > 0)
                {
                    Console.WriteLine("WARNING SKIPPING SCHEDULES");
                    continue;
                }
                */
                
                //Console.WriteLine(line);
		        string variant = line.Split("\t")[0];

                string prodSchedule = String.Format(prodSchedulePrefix + variant + ".txt",
                    customers, periods, vehicles, index);

                prodSchedule = Path.Join(productionScheduleDir, prodSchedule);
                active_pr_schedule_count++;
                //This does not yield feasible solution
                //string prodSchedule = "../../../Data/ProductionSchedules/RelSol_Adu_MVPRP Results (relaxed deliv)/RelSol_Adu_MVPRP_C30_P9_V3_I2.DAT/RelSol_Adu_MVPRP_C30_P9_V3_I2.DAT_0_1_0_0_0_1_0_1_0.txt";
                //This yields feasible solution with WRONG inventory cost 21296.0 istead of 21458.0 
                //string prodSchedule = "../../../Data/ProductionSchedules/RelSol_Adu_MVPRP Results (relaxed deliv)/RelSol_Adu_MVPRP_C30_P9_V3_I2.DAT/RelSol_Adu_MVPRP_C30_P9_V3_I2.DAT_0_1_1_0_0_1_0_1_0.txt";

                Console.WriteLine("Loading Production Schedule {0}", prodSchedule);
                
                string instance_name = Path.GetFileName(prodSchedule);

                DataInput input = DataLoader.LoadingInstanceOpener(instanceFullpathName, instanceFMT);
                input.report();
                ProductionDataInput pr_scheduleInput = DataLoader.parseProductionSchedule(prodSchedule, input.customerNum, variant, input.availableVehicles, prodschedule_has_routes);

                //dirty fix for boudia instance name
                if (instanceFMT == PRP_DATASET_VERSION.BOUDIA_FMT)
                    instance_name = instance_name.Replace("instance", input.customerNum + "_instance");
                
                PRP prpModel = new PRP(input, pr_scheduleInput, instance_name, true);

                Console.WriteLine(instanceFullpathName);
                Console.WriteLine(instanceName);

                if (MathematicalProgramming.exactParams.limitedVehicles)
                {
                    int maxVehicles = parseMaxPRPVehicles(fullPathInstanceNameFMT, instanceName, vehicles);
                    prpModel.vehicles = maxVehicles;
                    prpModel.input.availableVehicles = maxVehicles;
                }

                bool breakit = false;
                if (MathematicalProgramming.exactParams.adjustedCyclicInventories)
                {
                    int nodes = prpModel.input.nodes.Count;
                    int[] startingInvs = parseAdjCyclicInventories(fullPathInstanceNameFMT, instanceName, vehicles, nodes, instanceFMT);
                    for (int ii = 0; ii < nodes; ii++)
                    {
                        Node node = prpModel.input.nodes[ii];
                        if (startingInvs[ii] != -1)
                        {
                            node.startingInventory = startingInvs[ii];
                            node.cyclicStartInv = startingInvs[ii];
                        }
                        else
                        {
                            breakit = true;
                        }
                        // else do nothing. the initial inventories apply
                    }
                }
                if (!breakit)
                {
                    result_array[i] = prpModel.solve();
                }
            }
            
            ss.Close();
            
            
            StreamWriter sw = new StreamWriter(results_summary_file_name);
            
            //Export results
            int feasible_runs = 0;
            double bestObj = double.MaxValue;
            string bestProdSchedule = "";
            double totalTime = 0;
            
            for (int i = 0; i < schedulesNum; i++)
            {
                SolutionResults res = result_array[i];
                totalTime += res.elapsedTime;

                if (res.bestSolutionObjective < bestObj)
                {
                    bestObj = res.bestSolutionObjective;
                    bestProdSchedule = res.productionSchedule;
                }
                
                if (res.feasible)
                    feasible_runs++;
            }

            // ATTENTION: silence results summaries
            if (MathematicalProgramming.exactParams.algorithm != null && MathematicalProgramming.exactParams.algorithm == "exact")
            {
                //Save to file
                sw.WriteLine("{0} {1} {2} {3}", totalTime, bestProdSchedule, bestObj, feasible_runs);
                for (int i = 0; i < 10; i++)
                {
                    SolutionResults res = result_array[i];
                    sw.WriteLine("{0} {1} {2} {3} {4}", res.productionSchedule, res.relaxedSolutionObjective, res.bestSolutionObjective, res.elapsedTime, res.feasible);
                }
                sw.Close();

                Console.WriteLine("Best Obj {0} using Prod Schedule {1}  ", bestObj, bestProdSchedule);
            }            
        }

        private static int[] parseAdjCyclicInventories(string fullPathInstanceNameFMT, string instanceName, int vehicles, int nodes, PRP_DATASET_VERSION instanceFMT)
        {
            int[] startingInvs = new int[nodes];

            for (int i = 0; i < nodes; i++)
            {
                startingInvs[i] = -1;
            }

            FileInfo src = new FileInfo(fullPathInstanceNameFMT + "cyclicStartingInvs" + vehicles + ".txt");
            if (instanceFMT == PRP_DATASET_VERSION.ADULYASAK_FMT)
            {
                src = new FileInfo(fullPathInstanceNameFMT + "cyclicStartingInvs.txt");
            }
            TextReader reader = src.OpenText();
            String str;
            char[] seperator = new char[2] { ' ', '\t' };
            List<String> data;

            //Skip lines
            while (reader.Peek() >= 0)
            {
                str = reader.ReadLine();
                data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
                if (data.First().Equals(instanceName))
                {
                    if (data.Count > 1)
                    {
                        for (int i = 0; i < nodes; i++)
                        {
                            startingInvs[i] = int.Parse(data[i + 1]);
                        }
                        break;
                    }
                }
            }
            // data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);

            return startingInvs;
        }

        private static int parseMaxPRPVehicles(string fullPathInstanceNameFMT, string instanceName, int vehicles)
        {
            int maxVehs = -1;

            FileInfo src = new FileInfo(fullPathInstanceNameFMT + "maxVehicles" + vehicles + ".txt");
            TextReader reader = src.OpenText();
            String str;
            char[] seperator = new char[2] { ' ', '\t' };
            List<String> data;

            //Skip lines
            while (reader.Peek() >= 0)
            {
                str = reader.ReadLine();
                data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
                if (data.First().Equals(instanceName))
                {
                    maxVehs = int.Parse(data.Last());
                    break;
                }
            }
            // data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);

            return maxVehs;
        }

        private static string GenerateBaseInstanceName(string fmt, int customers, int periods, int vehicles, int index)
        {
            return String.Format(fmt,
                customers, periods, vehicles, index);
        }

        private static string GenerateFullPathInstanceName(string prefix, string baseInstancenameFMT,
            int customers, int periods, int vehicles, int index)
        {
            string name = GenerateBaseInstanceName(baseInstancenameFMT, customers, periods, vehicles, index);
            return prefix + name;
        }
    }

}
