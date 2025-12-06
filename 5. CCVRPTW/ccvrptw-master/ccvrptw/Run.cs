using System;
using ILOG.CP;
using ILOG.Concert;
using System.Drawing;

namespace CCVRPTW
{
    public static class ProblemConfiguration
    {
        public static bool UseTimeWindows = true;
        public static bool UseServiceTimes = true;
        public static Model model;
        public static int SkippedVehicles = 0;
    }

    class Run
    {
        public static void Main(string[] args)
        {
            RecalculateObjectives();
            return;

            int start_instance_id = int.Parse(args[0]);
            int end_instance_id = int.Parse(args[1]);
            int restarts = int.Parse(args[2]);
            Objective objective = (Objective) Enum.Parse(typeof(Objective), args[3]);

            //Check Preprocessor Flags

            if (ProblemConfiguration.UseTimeWindows)
                Console.WriteLine("TIME WINDOWS ENABLED");
            else
                Console.WriteLine("TIME WINDOWS DISABLED");

            if (ProblemConfiguration.UseServiceTimes)
                Console.WriteLine("SERVICE TIMES ENABLED");
            else
                Console.WriteLine("SERVICE TIMES DISABLED");

            //Objective objective = Objective.CUMULATIVE_SERVICE_TIMES;
            //CUMULATIVE_DISTANCE
            //CUMULATIVE_SERVICE_TIMES
            RunAll(start_instance_id, end_instance_id, restarts, objective);
            
            //RunInstance("../../../solomon_100/C102.txt", 1, objective);
            return;

            ProblemConfiguration.model = new Model("../../../solomon_100/R102.txt");

            bool use_cp = false;
            
            if (use_cp)
            {
                ////Use CP Solver
                Solution sol = CPSolver.SolveVRPTW(300, 8);
                if (sol is null)
                    return;
                Console.WriteLine(sol);
                System.Diagnostics.Debug.Assert(sol.CheckSolution(objective));

                //sol.ComputeCumulativeDistances(d);
                //sol.ComputeCumulativeBeginServiceTimes(d);
                //sol.ComputeCumulativeLatestBeginServiceTimes();

                //Use CP solution as a warm start for Local Search

                Solver s = new Solver(1);
                s.solution = sol;
                s.PromisesImprove(objective);
                //s.solution.PlotCostProgression(s.costProgression, s.solution.cost, s.promisesLastImprovedIteration);
                Console.WriteLine(s.solution.cost);
                s.solution.CheckSolution(objective);
                Console.WriteLine(s.solution);
            }
            else
            {
                Solver s = new Solver(1);
                var watch = new System.Diagnostics.Stopwatch();
                watch.Start();
                s.SolveWithRestarts(objective);
                watch.Stop();
                Console.WriteLine($"Execution time = {watch.ElapsedMilliseconds / 1000} s");
            }
        }
        
        public static Solver RunInstance(string file, int restarts, Objective objective)
        {   
            Console.WriteLine("---------------------------------------------------------------------------------------------------------------------");
            Console.WriteLine("Running : " + file);
            ProblemConfiguration.model = new Model(file);
            Solver s = new Solver(restarts);
            s.SolveWithRestarts(objective);
            Console.WriteLine(s.solution);

            s.solution.CheckSolution(objective);
            Console.WriteLine("cum service   " + s.solution.ComputeCumulativeServiceTimes());
            Console.WriteLine("cum distances " + s.solution.ComputeCumulativeDistances());
            return s;
        }

        public static void RunAll(int start, int end, int restarts, Objective objective)
        {
            Console.WriteLine($"Input Settings: {start}, {end}, {restarts}, {objective}");
            string[] fileArray = Directory.GetFiles(@"../../../solomon_100_minvehicles/", "*.txt");
            Dictionary<String, double> kyriakakis_costs = new Dictionary<String, double>();
            addKyriakakisCosts(kyriakakis_costs);

            StreamWriter sw = new StreamWriter($"Test_{start}_{end}.csv");
            sw.WriteLine("Instance,Vehicles,Not Visited,Cost,Kyriakakis Cost," +
                "LS Last Improving Iter,Refresh Promises Iter,Execution Time (s), Best Restart, Biggest Gap");

            for (int i=start; i < Math.Min(end, fileArray.Length); i++)
            {
                string file = fileArray[i];
                Solver solver = RunInstance(file, restarts, objective);
                report(sw, file, solver, kyriakakis_costs);
            }

            sw.Close();

        }

        public static void report(StreamWriter sw, string file, Solver s, Dictionary<String, double> kyriakakis)
        {
            string instance_name = Path.GetFileNameWithoutExtension(file);
            sw.Write(instance_name);
            sw.Write(",");
            sw.Write(s.vehicles);
            sw.Write(",");
            sw.Write((int)(s.solution.cost / ProblemConfiguration.model.extraNodePenalty)); // customers in extra vehicles
            sw.Write(",");
            sw.Write(Math.Round(s.solution.cost, 2).ToString().Replace(",", "."));
            sw.Write(",");
            sw.Write(kyriakakis[instance_name].ToString().Replace(",","."));
            sw.Write(",");
            sw.Write(s.solution.lastImprovedIteration);
            sw.Write(",");
            sw.Write(s.promisesRestart);
            sw.Write(",");
            sw.Write(s.elapsedTime);
            sw.Write(",");
            sw.Write(s.bestRestart);
            sw.Write(",");
            sw.Write(s.solution.biggestIterationsGap);
            sw.WriteLine();

        }
        public static void addKyriakakisCosts(Dictionary<String, double> dict)
        {
            dict.Add("C101", 3865.78);
            dict.Add("C102", 3856.17);
            dict.Add("C103", 3855.03);
            dict.Add("C104", 3772.94);
            dict.Add("C105", 3865.78);
            dict.Add("C106", 3865.78);
            dict.Add("C107", 3865.78);
            dict.Add("C108", 3863.35);
            dict.Add("C109", 3857.22);
            dict.Add("C201", 7643.65);
            dict.Add("C202", 7640.45);
            dict.Add("C203", 7404.43);
            dict.Add("C204", 7257.49);
            dict.Add("C205", 7657.47);
            dict.Add("C206", 7620.29);
            dict.Add("C207", 7527.96);
            dict.Add("C208", 7521.26);
            dict.Add("R101", 4202.69);
            dict.Add("R102", 4009.32);
            dict.Add("R103", 4137.95);
            dict.Add("R104", 4247.65);
            dict.Add("R105", 4361.74);
            dict.Add("R106", 4463.04);
            dict.Add("R107", 4535.91);
            dict.Add("R108", 4031.05);
            dict.Add("R109", 4387.95);
            dict.Add("R110", 4177.4);
            dict.Add("R111", 4478.57);
            dict.Add("R112", 3991.25);
            dict.Add("R201", 15507.4);
            dict.Add("R202", 18849.1);
            dict.Add("R203", 10310.9);
            dict.Add("R204", 20267.5);
            dict.Add("R205", 16578.1);
            dict.Add("R206", 14607.3);
            dict.Add("R207", 12650.8);
            dict.Add("R208", 16613);
            dict.Add("R209", 14762.5);
            dict.Add("R210", 15637.7);
            dict.Add("R211", 12425.2);
            dict.Add("RC101", 5212.08);
            dict.Add("RC102", 5297.68);
            dict.Add("RC103", 5318.24);
            dict.Add("RC104", 4830.75);
            dict.Add("RC105", 4867.66);
            dict.Add("RC106", 5402.45);
            dict.Add("RC107", 5031.48);
            dict.Add("RC108", 5173.73);
            dict.Add("RC201", 17286.2);
            dict.Add("RC202", 13197.7);
            dict.Add("RC203", 15918);
            dict.Add("RC204", 11823.6);
            dict.Add("RC205", 15071.4);
            dict.Add("RC206", 19064);
            dict.Add("RC207", 15930.8);
            dict.Add("RC208", 12952.2);
        }
    
        public static void RecalculateObjectives()
        {

            StreamWriter sw = new StreamWriter("reports_cumservicetime.txt");
            sw.WriteLine($"Instance Name; Distance; CumDist; CumServiceTimes");

            foreach (string instance in Directory.GetFiles("../../../solomon_100_original"))
            {
                string instance_name = Path.GetFileNameWithoutExtension(instance);

                //Find Appropriate solution file
                string solution_file = "";
                foreach (string sol_file in Directory.GetFiles("../../Release/net6.0/CUM_SERVICE_TIMES_25_REST"))
                {
                    if (Path.GetFileNameWithoutExtension(sol_file).StartsWith(instance_name))
                    {
                        solution_file = sol_file;
                        break;
                    }
                }
                
                if (solution_file == "")
                {
                    Console.WriteLine("No solution file found for instance");
                    continue;
                }
                    
                Solution sol = Solution.ParseSolution(solution_file, instance);

                sw.WriteLine($"{instance_name}; {sol.ComputeDistances()} ; {sol.ComputeCumulativeDistances()} ;  {sol.ComputeCumulativeServiceTimes()}");

                //Console.WriteLine($"Cached Solution Cost {sol.cost}");
                Console.WriteLine($"Instance {instance_name} ; Recalculated Costs - Distance {sol.ComputeDistances()} ; Cum Dist {sol.ComputeCumulativeDistances()} ; Cum Service Times {sol.ComputeCumulativeServiceTimes()}");
            
            }

            sw.Close();

        }
    
    }
}
