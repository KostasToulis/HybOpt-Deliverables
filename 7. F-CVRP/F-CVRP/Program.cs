using System;
using static F_CVRP.Setup;
using static F_CVRP.Model;
using System.IO;
using static F_CVRP.Solver;
using System.Diagnostics;
using OfficeOpenXml;
using System.Collections.Generic;
using static F_CVRP.Gurobi.MIP;
using System.Threading.Tasks;

namespace F_CVRP
{
    class Program
    {
        
        public static Run_config runData;

        static void Main(string[] args)
        {
            //Benchmark instance

            //string fileName = "fcvrp_P-n55-k15_17_13_3.txt";

            //string fileName = "fcvrp_P-n101-k4_30_3_1.txt";

            string fileName = "All";


            string projectDirectory = Directory.GetParent(AppDomain.CurrentDomain.BaseDirectory).Parent.Parent.Parent.FullName;

            Solve(projectDirectory, fileName);

        }


        private static Solution SolveOne(string projectDirectory, string fileName, Stopwatch stopwatch)
        {
         
            var path = Path.Combine(projectDirectory, "instances", fileName);
            Model model = Setup.CreateModel(path);
            runData = new Run_config(fileName, model);
            model.runData = runData;

            Solver s = new Solver(model);

            Solution sol = s.Solve(model, stopwatch);

            return sol;

        }


        private static void Solve(string projectDirectory, string fileName)
        {
            string folderPath = Path.Combine(projectDirectory, "instances");
            string resultsPath = Path.Combine(projectDirectory, "results", "output.xlsx");
            string failedPath = Path.Combine(projectDirectory, "results", "failed.txt");
            string analyticalPath = Path.Combine(projectDirectory, "results/analyticalResults");

            if (Directory.Exists(folderPath))
            {

                string[] files;
                if (fileName == "All")
                {
                    files = Directory.GetFiles(folderPath);
                }
                else
                {
                    files = new string[] { Path.Combine(projectDirectory, "instances", fileName) };
                }

                ExcelPackage.License.SetNonCommercialOrganization("Kostas Toulis"); 


                //ExcelPackage.LicenseContext = LicenseContext.NonCommercial;

                using (var package = new ExcelPackage())
                {
                    var worksheet = package.Workbook.Worksheets.Add("AM_0.5_MIP_0.5");
                    int i = 0;


                    //Parallel.ForEach(string file in files)
                    //foreach (string file in files)
                    Parallel.ForEach(files, (file, state, index) =>
                    {
                        try
                        {

                            Stopwatch stopwatch = new Stopwatch();
                            stopwatch.Start();

                            Solution sol = SolveOne(projectDirectory, Path.GetFileName(file), stopwatch);

                            stopwatch.Stop();

                            Console.WriteLine($"Instance: {Path.GetFileName(file)}, Cost: {sol.cost}, Origin: {sol.origin}");
                            OutputResults(sol, resultsPath, package, worksheet, i, file, stopwatch);
                            OutputAnalyticalSol(sol, file, analyticalPath);
                            i++;
                        }
                        catch
                        {
                            using (StreamWriter writer = new StreamWriter(failedPath, true))
                            {
                                writer.WriteLine(Path.GetFileName(file));

                            }
                        }
                    });
                }
            }
        }

        private static void OutputResults(Solution sol, string resultsPath, ExcelPackage package, ExcelWorksheet worksheet, int i, string file, Stopwatch stopwatch)
        {
            worksheet.Cells[i + 1, 1].Value = Path.GetFileName(file);
            worksheet.Cells[i + 1, 2].Value = sol.cost;
            worksheet.Cells[i + 1, 3].Value = stopwatch.ElapsedMilliseconds / 1000;

            package.SaveAs(new FileInfo(resultsPath));
        }
        private static void OutputAnalyticalSol(Solution sol, string file, string analyticalPath)
        {
            file = Path.GetFileName(file);
            analyticalPath = Path.Combine(analyticalPath, file);

            using (StreamWriter writer = new StreamWriter(analyticalPath, true))
            {
                writer.WriteLine($"Cost: {sol.cost}");

                foreach (Route rt in sol.routes)
                {
                    string nodeIds = "";
                    for (int i=0; i< rt.sequenceOfNodes.Count; i++)
                    {
                        nodeIds = nodeIds + rt.sequenceOfNodes[i].id.ToString();
                        if (i != rt.sequenceOfNodes.Count - 1)
                        {
                            nodeIds = nodeIds + "-";
                        }
                    }

                    writer.WriteLine(nodeIds);
                }


            }
        }
    }
}
