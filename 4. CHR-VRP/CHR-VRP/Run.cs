using System.Diagnostics;
using OfficeOpenXml;
using System.Globalization;
using System.Text.Json;
using ScottPlot;

namespace CHRVRP
{
    internal class Run
    {
        private static void Main(string[] args)
        {
            var path = args[0];
            string text = File.ReadAllText(path);
            var config = JsonSerializer.Deserialize<Configurations>(text);
            
            ExcelPackage.LicenseContext = LicenseContext.NonCommercial;
            var package = new ExcelPackage();
            var worksheet = package.Workbook.Worksheets.Add("Comparison");
            worksheet.Cells["A1"].Value = "Instance";
            worksheet.Cells["B1"].Value = "Our Result";
            worksheet.Cells["C1"].Value = "Result File";
            worksheet.Cells["D1"].Value = "Their Result";
            worksheet.Cells["E1"].Value = "ILS Gap";
            worksheet.Cells["F1"].Value = "Ours is Better";
            worksheet.Cells["G1"].Value = "Average Run Time (Hours:Minutes:Seconds.Milliseconds)";
            worksheet.Cells["H1"].Value = "Average Math Model Calls";
            worksheet.Cells["I1"].Value = "Average Math Model Run Time (Hours:Minutes:Seconds.Milliseconds)";

            Dictionary<string, double> results = ReadResults();
            var cell = 2;
            foreach (var kvp in results)
            {
                worksheet.Cells[$"C{cell}"].Value = kvp.Key;
                worksheet.Cells[$"D{cell}"].Value = kvp.Value;
                cell++;
            }

            string instancesDirectory = "../../../Instances";

            // Retrieve all file paths in the instances directory and its subdirectories
            string[] filePaths = Directory.GetFiles(instancesDirectory, "*", SearchOption.AllDirectories);

            // Check that indeed every instance exists
            foreach (string filePath in filePaths)
            {
                Debug.Assert(File.Exists(filePath));
            }
            //var instance = filePaths[55];  // same as string instance = "../../../Instances/Medium Instances/inst100S1V6Y20PxLxn7.atsp";
            var bestSolutions = new Solution[config.NumberOfRuns];
            var rand = new Random(0);
            var seeds = new int[config.NumberOfRuns];
            for (int i = 0; i < seeds.Length; i++)
            {
                seeds[i] = rand.Next();
            }
            
            String instance;
            var ticks = new List<long>(config.NumberOfRuns);
            var mathModelTicks = new List<long>(config.NumberOfRuns);
            var mathModelTimesCalled = new List<int>(config.NumberOfRuns);
            var elapsed = new Stopwatch();
            Console.WriteLine(config.StartingPoint);
            Console.WriteLine(config.EndingPoint);
            
            for (var i = config.StartingPoint; i < config.EndingPoint; i++)
            {
                //lefmanou: no need for these to be done in each run
                instance = filePaths[i];
                
                ticks.Clear();
                mathModelTicks.Clear();
                mathModelTimesCalled.Clear();

                cell = i + 2;
                Console.WriteLine($"{i}, {instance}");
                worksheet.Cells[$"A{cell}"].Value = instance;

                for (var run = 0; run < config.NumberOfRuns; run++)
                {
                    elapsed.Start();
                    Model model;

                    int seed = seeds[run];
                    model = new Model(instance, seed, config.MainKpi, config.NearestNeighborsP);

                    List<Move> moves = new List<Move>();
                    moves.Add(new Swap());
                    moves.Add(new Relocation());
                    moves.Add(new TwoOpt());

                    Solution solution = new Solution(instance, model, false);
                    solution.Construct(config.RclSize);
                    solution.CalculateObjectives();

                    //Console.WriteLine("Solution Before Local Search");
                    //Console.WriteLine(solution.GetMainKPIObjective() + "\n");

                    LocalSearch ls = new LocalSearch(solution, moves.ToArray(), config);
                    ls.Run();
                    
                    mathModelTicks.Add(ls.mathModelTimes);
                    mathModelTimesCalled.Add(ls.mathModelCalled.Count(b => b));
                    
                    //Console.WriteLine("Solution After Local Search");
                    //Console.WriteLine(ls.solution.GetMainKPIObjective() + "\n");

                    PlotObjective(ls.objectiveValues, ls.mathModelCalled, i, run);
                    
                    bestSolutions[run] = ls.solution;
                    
                    if (Debugger.IsAttached)
                    {
                        if (ls.solution.CheckEverything())
                        {
                            Console.WriteLine($"Run {run}, Solution Cost: {ls.solution.GetMainKPIObjective()}\n");
                        }
                        else
                        {
                            Console.WriteLine("MAX = {0}, MIN = {1}", solution.max, solution.min);
                            Console.WriteLine("oopsie");
                            Environment.Exit(1);
                        }
                    }
                    //ls.solution.PlotRoutes(2);
                    elapsed.Stop();
                    ticks.Add(elapsed.ElapsedTicks);
                    elapsed.Reset();
                }

                var bestSolution = bestSolutions[0];
                var bestRun = 0;
                for (var j = 1; j < config.NumberOfRuns; j++)
                {
                    var sol = bestSolutions[j];
                    if (sol.GetMainKPIObjective() < bestSolution.GetMainKPIObjective())
                    {
                        bestSolution = sol;
                        bestRun = j;
                    }
                }

                worksheet.Cells[$"B{cell}"].Value = Math.Round(bestSolution.GetMainKPIObjective(), 2);
                var gap = ((double)worksheet.Cells[$"B{cell}"].Value - (double)worksheet.Cells[$"D{cell}"].Value) / (double)worksheet.Cells[$"D{cell}"].Value;
                worksheet.Cells[$"E{cell}"].Value = gap;
                worksheet.Cells[$"F{cell}"].Value = gap <= 0;
                var avg = ticks.Average(); // create average of ticks
                var averageTimeSpan = new TimeSpan((long)avg); // cast needed from double to long
                // Format and store the TimeSpan value.
                var elapsedTime = String.Format("{0:00}:{1:00}:{2:00}.{3:00}",
                    averageTimeSpan.Hours, averageTimeSpan.Minutes, averageTimeSpan.Seconds,
                    averageTimeSpan.Milliseconds / 10);
                worksheet.Cells[$"G{cell}"].Value = elapsedTime;
                worksheet.Cells[$"H{cell}"].Value = mathModelTimesCalled.Average();
                avg = mathModelTicks.Sum() / mathModelTimesCalled.Sum();
                averageTimeSpan = new TimeSpan((long)avg);
                elapsedTime = String.Format("{0:00}:{1:00}:{2:00}.{3:00}",
                    averageTimeSpan.Hours, averageTimeSpan.Minutes, averageTimeSpan.Seconds,
                    averageTimeSpan.Milliseconds / 10);
                worksheet.Cells[$"I{cell}"].Value = elapsedTime;

                //bestSolution.PlotRoutes(2);
                /*if (!bestSolution.CheckEverything())
                {
                    Environment.Exit(1);
                }*/

                Console.WriteLine("\n********************************************************************");
                Console.WriteLine($"Best Run {bestRun}, Solution Cost: {bestSolution.GetMainKPIObjective()}");
                Console.WriteLine("********************************************************************\n");
            }

            var fileInfo = new FileInfo("../../../../comparisons" + config.StartingPoint + "_" + config.EndingPoint + ".xlsx");
            package.SaveAs(fileInfo);
        }

        private static Dictionary<string, double> ReadResults()
        {
            Dictionary<string, double> results = new Dictionary<string, double>();

            //string MILPResultsDirectory = "../../../Results/MILP Instances";
            string smallResultDirectory = "../../../Results/Small Instances";
            string mediumResultDirectory = "../../../Results/Medium Instances";
            string largeResultDirectory = "../../../Results/Large Instances";

            //string[] MILPResultFilePaths = Directory.GetFiles(MILPResultsDirectory, "res*", SearchOption.AllDirectories);
            string[] smallResultFilePaths = Directory.GetFiles(smallResultDirectory, "res*", SearchOption.AllDirectories);
            string[] mediumResultFilePaths = Directory.GetFiles(mediumResultDirectory, "res*", SearchOption.AllDirectories);
            string[] largeResultFilePaths = Directory.GetFiles(largeResultDirectory, "res*", SearchOption.AllDirectories);

            string[] restResultFilePaths = largeResultFilePaths.Concat(mediumResultFilePaths).Concat(smallResultFilePaths).ToArray();

            /*
            foreach (string filePath in MILPResultFilePaths)
            {
                using (var fileStream = new FileStream(filePath, FileMode.Open, FileAccess.Read, FileShare.Read))
                using (var streamReader = new StreamReader(fileStream))
                {
                    streamReader.ReadLine();
                    string line = streamReader.ReadLine();
                    string value = line.Split(' ')[1];

                    results.Add(filePath, value);

                }
            }*/

            CultureInfo usCulture = new CultureInfo("en-US");

            foreach (string filePath in restResultFilePaths)
            {
                using (var fileStream = new FileStream(filePath, FileMode.Open, FileAccess.Read, FileShare.Read))
                using (var streamReader = new StreamReader(fileStream))
                {
                    streamReader.ReadLine();
                    var line = streamReader.ReadLine();
                    if (line != null)
                    {
                        var value = double.Parse(line.Split(',')[7], usCulture);
                        results.Add(filePath, value);
                    }
                }
            }

            //results.ToList().ForEach(kvp => Console.WriteLine($"FILE: {kvp.Key}, VALUE: {kvp.Value}"));

            return results;
        }

        private static void PlotObjective(List<double> objectiveValues, List<bool> mathModelCalled, int instance, int run)
        {
            var plt = new Plot(1600, 1200);
            plt.AddSignal(objectiveValues.ToArray());
            
            for (int i = 0; i < mathModelCalled.Count; i++)
            {
                if (mathModelCalled[i])
                {
                    plt.AddMarker(x: i, y: objectiveValues[i], color: System.Drawing.Color.Red, size: 10);
                }
            }
            
            // create the directory if it doesn't exist
            Directory.CreateDirectory($"../../../../signal_plots/instance_{instance}");
            plt.SaveFig($"../../../../signal_plots/instance_{instance}/run_{run}.png");
        }
    }
}
