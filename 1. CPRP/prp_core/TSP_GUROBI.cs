/* Copyright 2020, Gurobi Optimization, LLC */

// Solve a traveling salesman problem on a randomly generated set of
// points using lazy constraints.   The base MIP model only includes
// 'degree-2' constraints, requiring each node to have exactly
// two incident edges.  Solutions to this model may contain subtours -
// tours that don't visit every node.  The lazy constraint callback
// adds new constraints to cut them off.

using System;
using System.Collections.Generic;
using Gurobi;



namespace PRP
{

    public class TSP_GRB_Solver
    {
        public static GRBEnv env = new GRBEnv();
        public static int threadLimit;

        public TSP_GRB_Solver()
        {

        }

        public static void TestSolve(Solution sol)
        {
            // works for BOUD 200_9! 
            int[] route = new int[] { 0, 5, 194, 165, 16, 93, 43, 130, 67, 200, 14, 84, 187, 168, 100, 115, 36, 127, 136, 117, 162, 135, 0 };
            //Create route
            Route rt = new Route(sol.model.capacity);
            for (int i = 0; i < route.Length; i++)
                rt.nodes.Add(sol.nodes[route[i]]);
            Console.WriteLine(rt.ToString());
            //Solve Test route
            double route_obj_improv = SolveRoute(sol, rt);
        }

        public static double SolveRoute(Solution sol, Route rt)
        {

            GRBModel model = new GRBModel(env);
            
            try
            {
                //env = new GRBEnv();
                
                PRP problem = sol.model;
                // Must set LazyConstraints parameter when using lazy constraints
                model.Parameters.LazyConstraints = 1;
                model.Parameters.OutputFlag = 0; //Suppress Output

                int n = rt.nodes.Count-1; //leave second depot outside

                // Create variables
                GRBVar[,] vars = new GRBVar[n, n];

                for (int i = 0; i < n; i++)
                {
                    for (int j = 0; j <= i; j++)
                    {
                        //Console.WriteLine("cost {0}-{1} : {2}", rt.nodes[i].uid, rt.nodes[j].uid, problem.distMatrix[rt.nodes[i].uid, rt.nodes[j].uid]);
                        vars[i, j] = model.AddVar(0.0, 1.0, problem.distMatrix[rt.nodes[i].uid, rt.nodes[j].uid],
                          GRB.BINARY, "x" + i + "_" + j);
                        vars[j, i] = vars[i, j];
                    }
                }

                /*
                 * Negative cycles detection
                for (int i = 0; i < n; i++)
                {
                    for (int j = 0; j <= i; j++)
                    {
                        for (int k = 0; k < n; k++)
                        {
                            double ac = problem.distMatrix[rt.nodes[i].uid, rt.nodes[j].uid];
                            double abc = problem.distMatrix[rt.nodes[i].uid, rt.nodes[k].uid] + problem.distMatrix[rt.nodes[k].uid, rt.nodes[j].uid];
                            if (abc < ac)
                            {
                                Console.WriteLine("cost {0}-{1} via {2}: {3} vs {4}", rt.nodes[i].uid, rt.nodes[j].uid, rt.nodes[k].uid, ac, abc);
                            }
                        }
                    }
                }
                */

                // Degree-2 constraints
                for (int i = 0; i < n; i++)
                {
                    GRBLinExpr expr = 0;
                    for (int j = 0; j < n; j++)
                        expr.AddTerm(1.0, vars[i, j]);
                    model.AddConstr(expr == 2.0, "deg2_" + i);
                }

                // Forbid edge from node back to itself

                for (int i = 0; i < n; i++)
                    vars[i, i].UB = 0.0;

                TSP_GRB_Callback cb = new TSP_GRB_Callback(vars);
                model.SetCallback(cb);
                model.Optimize();
                model.Parameters.Threads = threadLimit;

                int[] tour;

                double obj_change = 0.0;
                if (model.Status != GRB.Status.OPTIMAL)
                {
                    GlobalUtils.writeToConsole("Failed to solve model");
                    throw new Exception("TSP Model Failed to solve");
                }
                else
                {
                    tour = TSP_GRB_Callback.findsubtour(model.Get(GRB.DoubleAttr.X, vars));

                    if (tour.Length == 0)
                        Console.Write("WARNING ZERO LENGTH TOUR");
                    if (tour.Length + 1 != rt.nodes.Count)
                    {
                        Console.Write("TOUR NODE MISMATCH");
                        Console.Write("Route: ");
                        for (int j = 0; j < rt.nodes.Count; j++)
                            Console.Write(rt.nodes[j] + " ");
                        Console.Write("\n");
                        Console.Write("TSP Route: ");
                        for (int j = 0; j < tour.Length; j++)
                            Console.Write(rt.nodes[tour[j]].uid + " ");
                        Console.Write("\n");

                        model.Dispose();
                        return 0.0;
                    }

                    //Check of the tour is the same as the input route
                    /*
                    if (!GlobalUtils.IsEqual(model.ObjVal, rt.totalRoutingCost))
                    {
                        GlobalUtils.writeToConsole("Tour mismatch between route and optimal TSP {1} vs {0}",
                          model.ObjVal, rt.totalRoutingCost);
                        Console.Write("Route: ");
                        for (int j = 0; j < rt.nodes.Count; j++)
                            Console.Write(rt.nodes[j] + " ");
                        Console.Write("\n");

                        Console.Write("TSP Route: ");
                        for (int j = 0; j < rt.nodes.Count; j++)
                            Console.Write(rt.nodes[tour[j]].uid + " ");
                        Console.Write("\n");
                    }
                    */

                    //In any case apply the calculated route
                    obj_change = rt.totalRoutingCost;

                    //APPLY CHANGES TO ROUTE
                    Node depot = rt.nodes[0];
                    //Old node_list

                    List<Node> old_route = new List<Node>();
                    List<int> old_deliveries = new List<int>();

                    for (int j = 0; j < n; j++)
                        old_route.Add(rt.nodes[j]);
                    
                    rt.nodes.Clear();
                    rt.nodes.Add(depot);
                    rt.nodes.Add(depot);
                    //Insert nodes with the updated order
                    for (int j = 1; j < n; j++)
                    {
                        rt.nodes.Insert(rt.nodes.Count - 1, old_route[tour[j]]);
                    }

                    //Recalculate routing cost
                    rt.totalRoutingCost = 0.0;
                    for (int j = 1; j < rt.nodes.Count; j++)
                        rt.totalRoutingCost += sol.model.distMatrix[rt.nodes[j - 1].uid, rt.nodes[j].uid];

                    obj_change -= rt.totalRoutingCost;

                    /*
                    for (int nn = 1; nn < rt.nodes.Count - 1; nn++)
                    {
                        Node cus = rt.nodes[nn];
                        if (cus.uid == 0)
                        {
                            GlobalUtils.writeToConsole("Depot in the middle");
                            Console.Write("Route: ");
                            for (int jj = 0; jj < rt.nodes.Count; jj++)
                                Console.Write(rt.nodes[jj] + " ");
                            Console.Write("\n");
                            GlobalUtils.writeToConsole("Depot in the middle");
                            Console.Write("Route: ");
                            for (int jj = 0; jj < old_route.Count; jj++)
                                Console.Write(old_route[jj] + " ");
                            Console.Write("\n");
                            throw new Exception("TSP GUROBI FUCKED UP");
                        }
                    }
                    */


                    
                
                }

                model.Dispose();
                //env.Dispose();
                
                return obj_change;

            }
            catch (GRBException e)
            {
                Console.WriteLine("Error code: " + e.ErrorCode + ". " + e.Message);
                Console.WriteLine(e.StackTrace);
                throw new Exception("TSP Model Failed");
            }
            catch (IndexOutOfRangeException e)
            {
                Console.WriteLine("Some index was out of range" + e.Message);
                Console.WriteLine(e.StackTrace);
                Console.WriteLine("Problematic Route: " + rt.ToString());
                sol.SaveToFile("route_error");
                throw new Exception("TSP Model Failed");
            }

            return 0.0;
        }

        public static double SolveTSP(Solution sol)
        {

            //Solve a TSP for all routes of the solution
            double obj_improvement = 0.0;

            for (int i = 0; i < sol.periods.Count; i++)
            {
                Period pr = sol.periods[i];
                for (int j = 0; j < pr.periodRoutes.Count; j++)
                {
                    Route rt = pr.periodRoutes[j];

                    if (rt.nodes.Count > 4 && rt.modified)
                    {
                        //Call Solver
                        double route_obj_improv = SolveRoute(sol, rt);

                        obj_improvement += route_obj_improv;
                        //Update shit
                        sol.routingCost -= route_obj_improv;
                        sol.totalObjective -= route_obj_improv;
                        rt.SetLoadAndCostLists(i, sol.model);
                        rt.modified = false; //Reset modification status
                        if (!Solution.checkSolutionStatus(sol.TestEverythingFromScratch()))
                        {
                            Console.Write("Route: ");
                            for (int jj = 0; jj < rt.nodes.Count; jj++)
                                Console.Write(rt.nodes[jj] + " ");
                            Console.Write("\n");
                            throw new Exception("TSP GUROBI FUCKED UP");
                        }
                    }
                }
            }

            return obj_improvement;
        }


    }

    public class TSP_GRB_Callback : GRBCallback
    {
        private GRBVar[,] vars;

        public TSP_GRB_Callback(GRBVar[,] xvars)
        {
            vars = xvars;
        }

        // Subtour elimination callback.  Whenever a feasible solution is found,
        // find the smallest subtour, and add a subtour elimination
        // constraint if the tour doesn't visit every node.

        protected override void Callback()
        {
            try
            {
                if (where == GRB.Callback.MIPSOL)
                {
                    // Found an integer feasible solution - does it visit every node?

                    int n = vars.GetLength(0);
                    int[] tour = findsubtour(GetSolution(vars));

                    if (tour.Length < n)
                    {
                        // Add subtour elimination constraint
                        GRBLinExpr expr = 0;
                        for (int i = 0; i < tour.Length; i++)
                            for (int j = i + 1; j < tour.Length; j++)
                                expr.AddTerm(1.0, vars[tour[i], tour[j]]);
                        AddLazy(expr <= tour.Length - 1);
                    }
                }
            }
            catch (GRBException e)
            {
                Console.WriteLine("Error code: " + e.ErrorCode + ". " + e.Message);
                Console.WriteLine(e.StackTrace);
            }
        }

        // Given an integer-feasible solution 'sol', return the smallest
        // sub-tour (as a list of node indices).

        public static int[] findsubtour(double[,] sol)
        {
            int n = sol.GetLength(0);
            bool[] seen = new bool[n];
            int[] tour = new int[n];
            int bestind, bestlen;
            int i, node, len, start;

            for (i = 0; i < n; i++)
                seen[i] = false;

            start = 0;
            bestlen = n + 1;
            bestind = -1;
            node = 0;
            while (start < n)
            {
                for (node = 0; node < n; node++)
                    if (!seen[node])
                        break;
                if (node == n)
                    break;
                for (len = 0; len < n; len++)
                {
                    tour[start + len] = node;
                    seen[node] = true;
                    for (i = 0; i < n; i++)
                    {
                        if (sol[node, i] > 0.5 && !seen[i])
                        {
                            node = i;
                            break;
                        }
                    }
                    if (i == n)
                    {
                        len++;
                        if (len < bestlen)
                        {
                            bestlen = len;
                            bestind = start;
                        }
                        start += len;
                        break;
                    }
                }
            }

            for (i = 0; i < bestlen; i++)
                tour[i] = tour[bestind + i];
            System.Array.Resize(ref tour, bestlen);

            return tour;
        }


    }

}

