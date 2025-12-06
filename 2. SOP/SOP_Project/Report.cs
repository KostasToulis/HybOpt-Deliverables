using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Linq;
using System.Globalization;

namespace SOP_Project
{
    class Report
    {
        static List<Report> reports = new List<Report>();

        Model m;
        int tests;
        int total_runs;
        int restarts;
        List<int> sols;
        List<int> costs;
        List<double> times;
        List<double> times_for_exact;
        List<double> times_for_constructive;
        List<int> best_found_at;
        List<int> iterations;
        List<int> n_exact_calls;
        List<int> best_found_at_list;
        List<int> sets_included;

        string dataset_name;
        int promise_target;
        int best_sol;
        double avg_sol;
        int best_run;
        int best_restart;
        double best_run_time;
        double best_run_time_exact;
        double best_run_time_constructive;
        int best_run_best_found_at;
        int best_run_exact_calls;
        int best_run_iterations;
        double avg_time;
        double avg_time_for_exact;
        double avg_time_for_constructive;
        double avg_best_found_at;
        double avg_iterations;
        double avg_exact_calls;
        int max_best_found_at;
        int max_iterations; 

        public Report()
        {
            tests = 0;
            total_runs = 0;
            restarts = 0;
            sols = new List<int>();
            costs = new List<int>();
            times = new List<double>();
            times_for_exact = new List<double>();
            times_for_constructive = new List<double>();
            best_found_at = new List<int>();
            iterations = new List<int>();
            n_exact_calls = new List<int>();
            best_found_at_list = new List<int>();
            sets_included = new List<int>();
        }

        public static void ReadAllReports(string target_folder) 
        {
            string[] summary_reports = Directory.GetFiles(target_folder);

            Dictionary<string, List<string>> same_dataset_reports = new Dictionary<string, List<string>>();
            foreach (string summmary_report in summary_reports)
            {
                string dataset = string.Join("_", summmary_report.Split("\\")[1].Split("_").ToList().GetRange(1, 4));
                dataset = dataset.Replace("_target", "");
                if (same_dataset_reports.ContainsKey(dataset))
                {
                    same_dataset_reports[dataset].Add(summmary_report);
                }
                else
                {
                    same_dataset_reports.Add(dataset, new List<string>() { summmary_report });
                }
            }
            foreach (string dataset in same_dataset_reports.Keys)
            {
                reports.Add(ReadSummaryReport(dataset, same_dataset_reports[dataset]));
            }

            //foreach (Report report in reports)
            //{
            //    ExportExactCallsWithObj(report, target_folder);
            //}

            string excel_report_filename = target_folder + "_complete_report";
            ExportReportFolderAsCsv(reports, excel_report_filename);
        }

        static Report ReadSummaryReport(string dataset, List<string> paths)  // creates a report object based on all summary reports around a single dataset
        {
            Report report = new Report { dataset_name = dataset, tests = paths.Count, max_best_found_at = 0, max_iterations = 0};
            // the dataset file should be either on the large H path or on the sop path
            try
            {
                report.m = new Dataset_Reader().Read_Dataset("./Datasets/sop/" + dataset + ".sop");
            }
            catch (Exception e)
            {
                report.m = new Dataset_Reader().Read_Dataset("./Datasets/large H/sop/" + dataset + ".sop");
            }

            double construct_time_to_remove = 0; // remove the construction time for non-restart solutions

            foreach (string path in paths)
            {
                string[] lines = File.ReadAllLines(path);
                report.restarts = lines.Length;
                foreach (string line in lines)
                {
                    report.total_runs++;
                    string[] line_elements = line.Split(";");
                    report.sols.Add(Int32.Parse(line_elements[1].Split(":")[1]));
                    report.costs.Add(Int32.Parse(line_elements[2].Split(":")[1]));
                    report.times.Add(Double.Parse(line_elements[3].Split(":")[1], CultureInfo.GetCultureInfo("en-US")));
                    report.times_for_exact.Add(Double.Parse(line_elements[4].Split(":")[1], CultureInfo.GetCultureInfo("en-US")));
                    report.times_for_constructive.Add(Double.Parse(line_elements[5].Split(":")[1], CultureInfo.GetCultureInfo("en-US")));
                    if (report.total_runs % (report.restarts * report.tests) == 0) // count only for the restart constructives
                    {
                        construct_time_to_remove += report.times_for_constructive[report.times_for_constructive.Count - 1];
                    }
                    report.best_found_at_list.Add(Int32.Parse(line_elements[6].Split(":")[1]));
                    report.iterations.Add(Int32.Parse(line_elements[7].Split(":")[1]));
                    report.n_exact_calls.Add(Int32.Parse(line_elements[8].Split(":")[1]));
                    report.sets_included.Add(Int32.Parse(line_elements[9].Split(":")[1]));
                }
            }
            report.best_sol = report.sols.Max();
            report.avg_sol = report.sols.Sum() / report.total_runs;
            int best_at = 0;
            foreach (int sol in report.sols)
            {
                if (sol == report.best_sol) { break; }
                best_at++;
            }
            report.best_run = (int) best_at / report.restarts;
            report.best_restart = best_at % report.restarts;
            report.best_run_time = report.times[best_at];
            report.best_run_time_exact = report.times_for_exact[best_at];
            report.best_run_time_constructive = report.times_for_constructive[best_at];
            report.best_run_best_found_at = report.best_found_at_list[best_at];
            report.best_run_iterations = report.iterations[best_at];
            report.best_run_exact_calls = report.n_exact_calls[best_at];
            report.avg_time = report.times.Sum() / report.total_runs;
            report.avg_time_for_exact = report.times_for_exact.Sum() / report.total_runs;
            report.avg_time_for_constructive = (report.times_for_constructive.Sum() - construct_time_to_remove) / (report.total_runs - report.tests);  // count only for the restart constructives
            report.avg_best_found_at = report.best_found_at_list.Sum() / report.total_runs;
            report.avg_exact_calls = report.n_exact_calls.Sum() / report.total_runs;
            report.avg_iterations = report.iterations.Sum() / report.total_runs;
            report.max_best_found_at = report.best_found_at_list.Max();
            report.max_iterations = report.iterations.Max();

            return report;
        }

        public static void ExportReportFolderAsCsv(List<Report> reports, string export_file_name)
        {
            Dictionary<string, int> literaure_best = Program.GetLiteratureBests();
            StreamWriter writer = new StreamWriter(export_file_name + ".csv");
            writer.WriteLine("dataset_name;Global_best;Liter_best;GLS;Sol.avg;Time.avg;Time_for_exact.avg;Time_for_constructive.avg;" +
                "Best_found_at.avg;Max_best_found_at;Avg_Iterations;Max_iterations;Avg_exact_calls;;Best_run;Time;Time_exact;Time_constructive;Best_found_at;Iterations;Exact_calls");
            foreach(Report report in reports)
            {
                writer.Write(report.dataset_name + ";");
                Console.WriteLine(report.m);
                writer.Write(report.m.total_available_profit + ";");
                string liter_best = "-";
                if (literaure_best.ContainsKey(report.dataset_name))
                {
                    liter_best = literaure_best[report.dataset_name].ToString();
                }
                writer.Write(liter_best + ";");
                writer.Write(report.best_sol + ";");
                writer.Write(report.avg_sol + ";");
                writer.Write(report.avg_time + ";");
                writer.Write(report.avg_time_for_exact + ";");
                writer.Write(report.avg_time_for_constructive + ";");
                writer.Write(report.avg_best_found_at + ";");
                writer.Write(report.max_best_found_at + ";");
                writer.Write(report.avg_iterations + ";");
                writer.Write(report.max_iterations + ";");
                writer.Write(report.avg_exact_calls + ";");
                writer.Write(";");
                writer.Write(report.best_run + "(" + report.best_restart + ")" + ";");
                writer.Write(report.best_run_time + ";");
                writer.Write(report.best_run_time_exact + ";");
                writer.Write(report.best_run_time_constructive + ";");
                writer.Write(report.best_run_best_found_at + ";");
                writer.Write(report.best_run_iterations + ";");
                writer.Write(report.best_run_exact_calls + "\n");

            }
            writer.Close();
        }

        public static void ExportExactCallsWithObj(Report report, string target_folder)
        {
            StreamWriter writer = new StreamWriter("../Reports/Dataset reports/" + report.dataset_name + "_exact_with_obj.csv");
            writer.WriteLine(report.dataset_name);
            writer.WriteLine("Exact_calls;Best_found_at;Objective");
            for (int i = 0; i < report.total_runs; i++)
            {
                writer.Write(report.n_exact_calls[i] + ";");
                writer.Write(report.best_found_at_list[i] + ";");
                writer.Write(report.sols[i] + "\n");
            }
            writer.Close();
        }
    }
}
