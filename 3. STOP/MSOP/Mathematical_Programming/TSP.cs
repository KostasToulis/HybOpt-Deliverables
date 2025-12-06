using MSOP.Fundamentals;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

namespace MSOP.MathematicalProgramming
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
        public static void LKHAlgorithm(Model m, Route route, int runs = 2, bool hide_errors = false)
        {
            // Parameters
            bool silence = true;

            //Vars
            Dictionary<int, Node> mappingDict = new Dictionary<int, Node>();
            StringBuilder problemStr = new StringBuilder();

            // easy ref
            Route rt = route;
            int n = rt.nodes_seq.Count - 1; //leave second depot outside

            //var watch = System.Diagnostics.Stopwatch.StartNew();
            // Build problem string
            problemStr.Append(string.Format("{0} {1}", runs, n));
            Node nodeCur;
            int tspPrecisionFactor = 1000;
            for (int i = 0; i < n; i++)
            {
                nodeCur = rt.nodes_seq[i];
                problemStr.Append(string.Format(" {0} {1} {2}", i + 1, (int)Math.Round(tspPrecisionFactor * nodeCur.x), (int)Math.Round(tspPrecisionFactor * nodeCur.y)));
                mappingDict.Add(i + 1, nodeCur);
            }
            //Console.WriteLine(problemStr.ToString());
            //watch.Stop();
            //var elapsedMs = watch.ElapsedMilliseconds;
            //Console.WriteLine("Building string input: " + elapsedMs + " ms");

            //var watch2 = System.Diagnostics.Stopwatch.StartNew();
            // call the exe
            // Sample input "8 5 1 50 50 2 10 11 3 20 22 4 30 33 5 40 44";
            // runs nodes depot x y cust2 x2 y2 cust3 x3 y3 etc. the customer id are increasing starting from depot which is 1
            Process process = new Process();
            process.StartInfo.FileName = "LKH-2.exe";
            process.StartInfo.Arguments = problemStr.ToString();
            process.StartInfo.UseShellExecute = false;
            process.StartInfo.RedirectStandardOutput = true;
            process.StartInfo.RedirectStandardError = true;
            process.Start();

            //* Read the output (or the error)
            string output = process.StandardOutput.ReadToEnd();
            string err = process.StandardError.ReadToEnd();
            process.WaitForExit();
            //watch2.Stop();
            //var elapsedMs2 = watch2.ElapsedMilliseconds;
            //Console.WriteLine("Optimizing: " + elapsedMs2 + " ms");

            //Console.WriteLine(err);
            //Console.WriteLine(output);

            int obj;
            List<int> newRoute = new List<int>();
            //var watch3 = System.Diagnostics.Stopwatch.StartNew();
            (obj, newRoute) = readObjAndTour(output);
            //watch3.Stop();
            //var elapsedMs3 = watch3.ElapsedMilliseconds;
            //Console.WriteLine("Reading solution: " + elapsedMs3 + " ms");

            //var watch4 = System.Diagnostics.Stopwatch.StartNew();
            // Update solution object
            //Old node_list
            //List<Node> old_route = new List<Node>();
            //for (int j = 0; j < n; j++)
            //    old_route.Add(rt.nodes_seq[j]);

            // chech if the solution is worse and rerun
            if (obj > route.time)
            {
                if (runs == 2)
                    Console.WriteLine("LKH failed to improve");
                //CountStuff.times_lkh_failed++;
                //LKHAlgorithm(m, sol, 5);
            }
            else if (obj < route.time)
            {
                // check if anything is changed
                bool changesExists = false;
                for (int i = 0; i < newRoute.Count - 1; i++)
                {
                    if (i + 1 != newRoute[i])
                    {
                        changesExists = true;
                        break;
                    }
                }

                if (changesExists)
                {
                    // recalculate time f
                    int old_routeCost = 0; // rt.time;
                    int previous_node_id = rt.nodes_seq[0].id;
                    foreach (Node node in rt.nodes_seq.GetRange(1, rt.nodes_seq.Count - 1))
                    {
                        old_routeCost += m.dist_matrix[previous_node_id, node.id];
                        previous_node_id = node.id;
                    }

                    // 1. Profit is the same

                    // 2. update the nodes lists
                    rt.nodes_seq.Clear();
                    rt.sets_included.Clear();
                    //foreach (Set set in m.sets) { set.in_route = false; }

                    //rt.nodes_seq.Add(depot);
                    //rt.sets_included.Add(m.sets[depot.set_id]);
                    //m.sets[depot.set_id].in_route = true;
                    for (int i = 0; i < newRoute.Count; i++)
                    {
                        int idx = newRoute[i];
                        Node node = mappingDict[idx];
                        //rt.nodes_seq.Insert(rt.nodes_seq.Count - 1, node);
                        rt.nodes_seq.Add(node);
                        rt.sets_included.Add(m.sets[node.set_id]);
                        //m.sets[node.set_id].in_route = true;
                    }
                    //rt.nodes_seq.Add(depot);
                    //rt.sets_included.Add(m.sets[depot.set_id]);

                    // 3. update time
                    rt.time = 0;
                    for (int j = 1; j < rt.nodes_seq.Count; j++)
                    {
                        rt.time += m.dist_matrix[rt.nodes_seq[j - 1].id, rt.nodes_seq[j].id];
                        //Console.WriteLine(m.dist_matrix[rt.nodes_seq[j - 1].id, rt.nodes_seq[j].id]);
                    }

                    route.time = rt.time;

                    if (old_routeCost != rt.time)
                    {
                        if (!silence)
                        {
                            Console.WriteLine("Improvement from TSP: old distance = {0} --> optimized distance = {1}", old_routeCost, rt.time);
                        }
                    }
                    if (obj != rt.time)
                    {
                        Console.WriteLine("Error in LKH TSP objective: LKH = {0} vs scratch = {1}", obj, rt.time); //attention maybe due to the precision error
                        //CountStuff.wrong_obj_in_lkh++;
                    }
                    //watch4.Stop();
                    //var elapsedMs4 = watch4.ElapsedMilliseconds;
                    //Console.WriteLine("Storing solution: " + elapsedMs4 + " ms");

                    if (!hide_errors && !route.CheckRoute())
                    {
                        Console.WriteLine("Infeasible route in LKH TSP");
                    }
                }
                else
                {
                    //Console.WriteLine("No improvement from LKH TSP");
                }
            }
        }

        private static (int obj, List<int> newRoute) readObjAndTour(string output)
        {
            int obj = -1;
            List<int> newRoute = new List<int>();

            //split
            var lines = output.Split("\n");

            // TOUR_SECTION,1,3,2,27,51,59,50,35,36,8,28,21,18,26,34,33,19,32,17,16,31,29,20,30,25,22,23,24,14,13,55,38,43,42,12,11,10,15,9,44,39,54,56,53,52,37,41,40,58,57,47,48,46,49,45,6,7,5,4,1
            //COMMENT: Length = 46989

            for (int idx = lines.Length - 2; idx > -1; idx--)
            {
                if (lines[idx].StartsWith("TOUR_SECTION"))
                {
                    var splits = lines[idx].Split(",");
                    for (int i = 1; i < splits.Length; i++)
                    {
                        newRoute.Add(Int32.Parse(splits[i]));
                    }
                    splits = lines[idx - 1].Split(" ");
                    obj = Int32.Parse(splits[splits.Length - 1].Replace("\r", ""));
                    break;
                }
            }

            return (obj, newRoute);
        }

        //    public static void FixedNodesTSP(Model m, Solution sol, double secLimit, double mipGapLimit, double heur)
        //    {
        //        // easy access
        //        Route rt = sol.route;
        //        int n = rt.nodes_seq.Count - 1; //leave second depot outside
        //        int customersNum = m.nodes.Count - 1;
        //        int allNodesNum = m.nodes.Count;

        //        GRBModel TSPmodel = new GRBModel(MathProgramming.gurobiEnv);

        //        try
        //        {
        //            // Must set LazyConstraints parameter when using lazy constraints
        //            TSPmodel.Parameters.LazyConstraints = 1;
        //            TSPmodel.Parameters.OutputFlag = 0; //Suppress Output
        //            TSPmodel.ModelName = "TSP" + DateTime.Now.ToString("HH:mm:ss tt");
        //            TSPmodel.Parameters.TimeLimit = secLimit;
        //            TSPmodel.Parameters.MIPGap = mipGapLimit;

        //            TSPmodel.Parameters.Heuristics = heur; // default 0.05
        //            //TSPmodel.Parameters.MIPFocus = 1; // focus on feasibility

        //            // Create variables
        //            GRBVar[,] vars = new GRBVar[n, n];

        //            for (int i = 0; i < n; i++)
        //            {
        //                for (int j = 0; j <= i; j++)
        //                {
        //                    vars[i, j] = TSPmodel.AddVar(0.0, 1.0, m.dist_matrix[rt.nodes_seq[i].id, rt.nodes_seq[j].id], GRB.BINARY, "x_" + rt.nodes_seq[i].id + "," + rt.nodes_seq[j].id);
        //                    vars[j, i] = vars[i, j];
        //                }
        //            }


        //            /*
        //             * Negative cycles detection
        //            for (int i = 0; i < n; i++)
        //            {
        //                for (int j = 0; j <= i; j++)
        //                {
        //                    for (int k = 0; k < n; k++)
        //                    {
        //                        double ac = m.dist_matrix[rt.nodes_seq[i].id, rt.nodes_seq[j].id];
        //                        double abc = m.dist_matrix[rt.nodes_seq[i].id, rt.nodes_seq[k].id] + m.dist_matrix[rt.nodes_seq[k].id, rt.nodes_seq[j].id];
        //                        if (abc < ac)
        //                        {
        //                            Console.WriteLine("cost {0}-{1} via {2}: {3} vs {4}", rt.nodes_seq[i].id, rt.nodes_seq[j].id, rt.nodes_seq[k].id, ac, abc);
        //                        }
        //                    }
        //                }
        //            }
        //            */

        //            // Degree-2 constraints
        //            for (int i = 0; i < n; i++)
        //            {
        //                GRBLinExpr expr = 0;
        //                for (int j = 0; j < n; j++)
        //                {
        //                    expr.AddTerm(1.0, vars[i, j]);
        //                }
        //                TSPmodel.AddConstr(expr == 2.0, "deg2_" + i);
        //            }

        //            // Forbid edge from node back to itself
        //            for (int i = 0; i < n; i++)
        //            {
        //                vars[i, i].UB = 0.0;
        //            }

        //            // Set starting solution
        //            for (int i = 0; i < n - 1; i++)
        //            {
        //                vars[i, i + 1].Start = 1.0;
        //                vars[i, i + 1].VarHintVal = 1.0;
        //            }

        //            /*
        //            // find a lower bound
        //            int[] used = new int[m.nodes.Count];
        //            int lb = 0;
        //            for (int i = 0; i < rt.nodes_seq.Count-1; i++)
        //            {
        //                Node nodei = rt.nodes_seq[i];
        //                int minVal = int.MaxValue;
        //                int nodeId = -1;
        //                for (int j = 0; j < rt.nodes_seq.Count; j++)
        //                {
        //                    Node nodej = rt.nodes_seq[j];
        //                    if (nodei.id != nodej.id)
        //                    {
        //                        if ( minVal > m.dist_matrix[nodei.id, nodej.id] && used[nodej.id] < 1)
        //                        {
        //                            minVal = m.dist_matrix[nodei.id, nodej.id];
        //                            nodeId = nodej.id;
        //                        }
        //                    }

        //                }
        //                used[nodeId] += 1;
        //                lb += minVal;
        //            }
        //            Console.WriteLine("LB: {0}", lb);
        //            */

        //            TSP_GRB_Callback cb = new TSP_GRB_Callback(vars);
        //            TSPmodel.SetCallback(cb);
        //            TSPmodel.Optimize();
        //            TSPmodel.Parameters.Threads = 1;

        //            int[] tour;

        //            double obj_change = 0.0;
        //            if (TSPmodel.Status == GRB.Status.INFEASIBLE)
        //            {
        //                Console.WriteLine("Model is infeasible");
        //                // compute and write out IIS
        //                TSPmodel.ComputeIIS();
        //                TSPmodel.Write(TSPmodel.ModelName + ".ilp");
        //                throw new Exception("TSP Model Failed to solve");
        //            }
        //            else if (TSPmodel.Status == GRB.Status.OPTIMAL || (TSPmodel.Status == GRB.Status.TIME_LIMIT && TSPmodel.ObjVal != 1e+100))
        //            {
        //                // store tour
        //                tour = TSP_GRB_Callback.findsubtour(TSPmodel.Get(GRB.DoubleAttr.X, vars));

        //                if (tour.Length == 0)
        //                    Console.WriteLine("WARNING ZERO LENGTH TOUR");
        //                if (tour.Length + 1 != rt.nodes_seq.Count)
        //                {
        //                    Console.WriteLine("TOUR NODE MISMATCH");
        //                    Console.WriteLine("Route: ");
        //                    for (int j = 0; j < rt.nodes_seq.Count; j++)
        //                        Console.Write(rt.nodes_seq[j] + " ");
        //                    Console.Write("\n");
        //                    Console.WriteLine("TSP Route: ");
        //                    for (int j = 0; j < tour.Length; j++)
        //                        Console.Write(rt.nodes_seq[tour[j]].id + " ");
        //                    Console.Write("\n");

        //                    TSPmodel.Dispose();
        //                }

        //                bool reorder = false;
        //                for (int i = 0; i < tour.Length - 1; i++)
        //                {
        //                    if (i != tour[i])
        //                    {
        //                        reorder = true;
        //                        break;
        //                    }
        //                }

        //                if (reorder)
        //                {
        //                    // Update solution object
        //                    //Old node_list
        //                    List<Node> old_route = new List<Node>();
        //                    for (int j = 0; j < n; j++)
        //                        old_route.Add(rt.nodes_seq[j]);

        //                    // recalculate time for when tsp is used by SolveSimulInsDelSubproblem
        //                    int old_routeCost = 0; // rt.time;
        //                    int previous_node_id = rt.nodes_seq[0].id;
        //                    foreach (Node node in rt.nodes_seq.GetRange(1, rt.nodes_seq.Count - 1))
        //                    {
        //                        old_routeCost += m.dist_matrix[previous_node_id, node.id];
        //                        previous_node_id = node.id;
        //                    }

        //                    // 1. Profit is the same

        //                    // 2. update the nodes lists
        //                    rt.nodes_seq.Clear();
        //                    rt.sets_included.Clear();
        //                    //foreach (Set set in m.sets) { set.in_route = false; }

        //                    Node depot = m.nodes[0];
        //                    rt.nodes_seq.Add(depot);
        //                    rt.nodes_seq.Add(depot);
        //                    rt.sets_included.Add(m.sets[depot.set_id]);
        //                    //m.sets[depot.set_id].in_route = true;

        //                    for (int j = 1; j < n; j++)
        //                    {
        //                        Node node = old_route[tour[j]];
        //                        rt.nodes_seq.Insert(rt.nodes_seq.Count - 1, node);
        //                        rt.sets_included.Add(m.sets[node.set_id]);
        //                        //m.sets[node.set_id].in_route = true;
        //                    }
        //                    rt.sets_included.Add(m.sets[depot.set_id]);


        //                    // 3. update time
        //                    rt.time = 0;
        //                    for (int j = 1; j < rt.nodes_seq.Count; j++)
        //                        rt.time += m.dist_matrix[rt.nodes_seq[j - 1].id, rt.nodes_seq[j].id];

        //                    sol.total_time = rt.time;

        //                    if (old_routeCost != rt.time)
        //                    {
        //                        Console.WriteLine("Improvement from TSP: old distance = {0} --> optimized distance = {1}", old_routeCost, rt.time);
        //                    }

        //                    /*
        //                    for (int nn = 1; nn < rt.nodes_seq.Count - 1; nn++)
        //                    {
        //                        Node cus = rt.nodes_seq[nn];
        //                        if (cus.id == 0)
        //                        {
        //                            Console.WriteLine("Depot in the middle");
        //                            Console.Write("Route: ");
        //                            for (int jj = 0; jj < rt.nodes_seq.Count; jj++)
        //                                Console.Write(rt.nodes_seq[jj] + " ");
        //                            Console.Write("\n");
        //                            Console.Write("Route: ");
        //                            for (int jj = 0; jj < old_route.Count; jj++)
        //                                Console.Write(old_route[jj] + " ");
        //                            Console.Write("\n");
        //                            throw new Exception("TSP error");
        //                        }
        //                    }
        //                    */
        //                }
        //            }

        //            TSPmodel.Dispose();
        //        }
        //        catch (GRBException e)
        //        {
        //            Console.WriteLine("Error code: " + e.ErrorCode + ". " + e.Message);
        //            Console.WriteLine(e.StackTrace);
        //            throw new Exception("TSP Model Failed");
        //        }
        //        catch (IndexOutOfRangeException e)
        //        {
        //            Console.WriteLine("Some index was out of range" + e.Message);
        //            Console.WriteLine(e.StackTrace);
        //            Console.WriteLine("Problematic Route: " + rt.ToString());
        //            //sol.SaveToFile("route_error");
        //            throw new Exception("TSP Model Failed");
        //        }

        //        if (!rt.CheckRoute(m))
        //        {
        //            //Console.WriteLine("TSP Error");
        //        }
        //    }
        //}

        //public class TSP_GRB_Callback : GRBCallback
        //{
        //    private GRBVar[,] vars;

        //    public TSP_GRB_Callback(GRBVar[,] xvars)
        //    {
        //        vars = xvars;
        //    }

        //    // Subtour elimination callback.  Whenever a feasible solution is found,
        //    // find the smallest subtour, and add a subtour elimination
        //    // constraint if the tour doesn't visit every node.

        //    protected override void Callback()
        //    {
        //        try
        //        {
        //            if (where == GRB.Callback.MIPSOL)
        //            {
        //                // Found an integer feasible solution - does it visit every node?

        //                int n = vars.GetLength(0);
        //                int[] tour = findsubtour(GetSolution(vars));

        //                if (tour.Length < n)
        //                {
        //                    // Add subtour elimination constraint
        //                    GRBLinExpr expr = 0;
        //                    for (int i = 0; i < tour.Length; i++)
        //                        for (int j = i + 1; j < tour.Length; j++)
        //                            expr.AddTerm(1.0, vars[tour[i], tour[j]]);
        //                    AddLazy(expr <= tour.Length - 1);
        //                }
        //            }
        //        }
        //        catch (GRBException e)
        //        {
        //            Console.WriteLine("Error code: " + e.ErrorCode + ". " + e.Message);
        //            Console.WriteLine(e.StackTrace);
        //        }
        //    }

        // Given an integer-feasible solution 'sol', return the smallest
        // sub-tour (as a list of node indices).

        //public static int[] findsubtour(double[,] sol)
        //{
        //    int n = sol.GetLength(0);
        //    bool[] seen = new bool[n];
        //    int[] tour = new int[n];
        //    int bestind, bestlen;
        //    int i, node, len, start;

        //    for (i = 0; i < n; i++)
        //        seen[i] = false;

        //    start = 0;
        //    bestlen = n + 1;
        //    bestind = -1;
        //    node = 0;
        //    while (start < n)
        //    {
        //        for (node = 0; node < n; node++)
        //            if (!seen[node])
        //                break;
        //        if (node == n)
        //            break;
        //        for (len = 0; len < n; len++)
        //        {
        //            tour[start + len] = node;
        //            seen[node] = true;
        //            for (i = 0; i < n; i++)
        //            {
        //                if (sol[node, i] > 0.5 && !seen[i])
        //                {
        //                    node = i;
        //                    break;
        //                }
        //            }
        //            if (i == n)
        //            {
        //                len++;
        //                if (len < bestlen)
        //                {
        //                    bestlen = len;
        //                    bestind = start;
        //                }
        //                start += len;
        //                break;
        //            }
        //        }
        //    }

        //    for (i = 0; i < bestlen; i++)
        //        tour[i] = tour[bestind + i];
        //    System.Array.Resize(ref tour, bestlen);

        //    return tour;
        //}
    }
}
