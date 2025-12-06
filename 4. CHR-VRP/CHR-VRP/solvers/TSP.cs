using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Gurobi;
using System.Diagnostics;


namespace CHRVRP
{
    class TSP
    {
        /**
         * Run the  LKH 2 TSP (LKH-2.exe) algorithm http://webhotel4.ruc.dk/~keld/research/LKH/
         *
         * The LKH2.exe needs to be in the debug or release folder as well as dlls:
         *  i) VCRUNTIME140D.DLL https://www.dll-files.com/vcruntime140d.dll.html (x64 version)
         *  ii) UCRTBASED.DLL https://www.dll-files.com/ucrtbased.dll.html (x64 version)
         */
        public static void LKHAlgorithm(Model m, Solution sol, int runs = 2, bool hide_errors = false)
        {
            // Parameters

            //Vars
        //    Dictionary<int, Node> mappingDict = new Dictionary<int, Node>();
        //    StringBuilder problemStr = new StringBuilder();

        //    // easy ref
        //    Route rt = sol.route;
        //    int n = rt.nodes_seq.Count - 1; //leave second depot outside

        //    //var watch = System.Diagnostics.Stopwatch.StartNew();
        //    // Build problem string
        //    problemStr.Append(string.Format("{0} {1}", runs, n));
        //    Node nodeCur;
        //    int tspPrecisionFactor = 1000;
        //    for (int i = 0; i < n; i++)
        //    {
        //        nodeCur = rt.nodes_seq[i];
        //        problemStr.Append(string.Format(" {0} {1} {2}", i + 1, (int) Math.Round(tspPrecisionFactor * nodeCur.x), (int) Math.Round(tspPrecisionFactor * nodeCur.y)));
        //        mappingDict.Add(i + 1, nodeCur);
        //    }
        //    //Console.WriteLine(problemStr.ToString());
        //    //watch.Stop();
        //    //var elapsedMs = watch.ElapsedMilliseconds;
        //    //Console.WriteLine("Building string input: " + elapsedMs + " ms");

        //    //var watch2 = System.Diagnostics.Stopwatch.StartNew();
        //    // call the exe
        //    // Sample input "8 5 1 50 50 2 10 11 3 20 22 4 30 33 5 40 44";
        //    // runs nodes depot x y cust2 x2 y2 cust3 x3 y3 etc. the customer id are increasing starting from depot which is 1
        //    Process process = new Process();
        //    process.StartInfo.FileName = "LKH-2.exe";
        //    process.StartInfo.Arguments = problemStr.ToString();
        //    process.StartInfo.UseShellExecute = false;
        //    process.StartInfo.RedirectStandardOutput = true;
        //    process.StartInfo.RedirectStandardError = true;
        //    process.Start();

        //    //* Read the output (or the error)
        //    string output = process.StandardOutput.ReadToEnd();
        //    string err = process.StandardError.ReadToEnd();
        //    process.WaitForExit();
        //    //watch2.Stop();
        //    //var elapsedMs2 = watch2.ElapsedMilliseconds;
        //    //Console.WriteLine("Optimizing: " + elapsedMs2 + " ms");

        //    //Console.WriteLine(err);
        //    //Console.WriteLine(output);

        //    int obj;
        //    List<int> newRoute = new List<int>();
        //    //var watch3 = System.Diagnostics.Stopwatch.StartNew();
        //    (obj, newRoute) = readObjAndTour(output);
        //    //watch3.Stop();
        //    //var elapsedMs3 = watch3.ElapsedMilliseconds;
        //    //Console.WriteLine("Reading solution: " + elapsedMs3 + " ms");

        //    //var watch4 = System.Diagnostics.Stopwatch.StartNew();
        //    // Update solution object
        //    //Old node_list
        //    //List<Node> old_route = new List<Node>();
        //    //for (int j = 0; j < n; j++)
        //    //    old_route.Add(rt.nodes_seq[j]);

        //    // chech if the solution is worse and rerun
        //    if (obj > sol.total_time)
        //    {
        //        if (runs == 2)
        //            Console.WriteLine("LKH failed to improve");
        //        //CountStuff.times_lkh_failed++;
        //            //LKHAlgorithm(m, sol, 5);
        //    }
        //    else if (obj < sol.total_time)
        //    {
        //        // check if anything is changed
        //        bool changesExists = false;
        //        for (int i = 0; i < newRoute.Count - 1; i++)
        //        {
        //            if (i + 1 != newRoute[i])
        //            {
        //                changesExists = true;
        //                break;
        //            }
        //        }

        //        if (changesExists)
        //        {
        //            // recalculate time f
        //            int old_routeCost = 0; // rt.time;
        //            int previous_node_id = rt.nodes_seq[0].id;
        //            foreach (Node node in rt.nodes_seq.GetRange(1, rt.nodes_seq.Count - 1))
        //            {
        //                old_routeCost += m.dist_matrix[previous_node_id, node.id];
        //                previous_node_id = node.id;
        //            }

        //            // 1. Profit is the same

        //            // 2. update the nodes lists
        //            rt.nodes_seq.Clear();
        //            rt.sets_included.Clear();
        //            //foreach (Set set in m.sets) { set.in_route = false; }

        //            //rt.nodes_seq.Add(depot);
        //            //rt.sets_included.Add(m.sets[depot.set_id]);
        //            //m.sets[depot.set_id].in_route = true;
        //            for (int i = 0; i < newRoute.Count; i++)
        //            {
        //                int idx = newRoute[i];
        //                Node node = mappingDict[idx];
        //                //rt.nodes_seq.Insert(rt.nodes_seq.Count - 1, node);
        //                rt.nodes_seq.Add(node);
        //                rt.sets_included.Add(m.sets[node.set_id]);
        //                //m.sets[node.set_id].in_route = true;
        //            }
        //            //rt.nodes_seq.Add(depot);
        //            //rt.sets_included.Add(m.sets[depot.set_id]);

        //            // 3. update time
        //            rt.time = 0;
        //            for (int j = 1; j < rt.nodes_seq.Count; j++)
        //            {
        //                rt.time += m.dist_matrix[rt.nodes_seq[j - 1].id, rt.nodes_seq[j].id];
        //                //Console.WriteLine(m.dist_matrix[rt.nodes_seq[j - 1].id, rt.nodes_seq[j].id]);
        //            }

        //            sol.total_time = rt.time;

        //            if (old_routeCost != rt.time)
        //            {
        //                //Console.WriteLine("Improvement from TSP: old distance = {0} --> optimized distance = {1}", old_routeCost, rt.time);
        //            }
        //            if (obj != rt.time)
        //            {
        //                Console.WriteLine("Error in LKH TSP objective: LKH = {0} vs scratch = {1}", obj, rt.time); //attention maybe due to the precision error
        //                //CountStuff.wrong_obj_in_lkh++;
        //            }
        //            //watch4.Stop();
        //            //var elapsedMs4 = watch4.ElapsedMilliseconds;
        //            //Console.WriteLine("Storing solution: " + elapsedMs4 + " ms");

        //            if (!hide_errors && !sol.route.CheckRoute(m))
        //            {
        //                Console.WriteLine("Infeasible route in LKH TSP");
        //            }
        //        }
        //        else
        //        {
        //            //Console.WriteLine("No improvement from LKH TSP");
        //        }
        //    }
        //}

        //private static (int obj, List<int> newRoute) readObjAndTour(string output)
        //{
        //    int obj = -1;
        //    List<int> newRoute = new List<int>();

        //    //split
        //    var lines = output.Split("\n");

        //    // TOUR_SECTION,1,3,2,27,51,59,50,35,36,8,28,21,18,26,34,33,19,32,17,16,31,29,20,30,25,22,23,24,14,13,55,38,43,42,12,11,10,15,9,44,39,54,56,53,52,37,41,40,58,57,47,48,46,49,45,6,7,5,4,1
        //    //COMMENT: Length = 46989

        //    for (int idx = lines.Length-2; idx > -1;  idx--)
        //    {
        //        if (lines[idx].StartsWith("TOUR_SECTION"))
        //        {
        //            var splits = lines[idx].Split(",");
        //            for (int i = 1; i < splits.Length; i++)
        //            {
        //                newRoute.Add(Int32.Parse(splits[i]));
        //            }
        //            splits = lines[idx-1].Split(" ");
        //            obj = Int32.Parse(splits[splits.Length-1].Replace("\r", ""));
        //            break;
        //        }
        //    }

        //    return (obj, newRoute);
       }
    }
}
