using System;
using System.Collections.Generic;
using System.Data;
using System.Diagnostics;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using Gurobi;
using OfficeOpenXml.Packaging.Ionic.Zlib;
using ScottPlot.Drawing.Colormaps;
using ScottPlot.SnapLogic;

namespace CHRVRP
{
    class Relaxations
    {
        /**
         * Simultaneous up to maxIns insertions and up to maxDel deletions with routing approximations 
         */
        public static void SolveRelaxedSupplyAndCustomerAssignmentModel(Solution sol, int maxInsDel, double minSpChange)
        {
            // params 
            bool addValidInequalities = false;
            bool silence = true;
            int maxIns = maxInsDel;
            int maxDel = maxInsDel;
            int minSPchanges = Convert.ToInt32(sol.routes.Count * minSpChange);

            // easy access
            var vehNum = sol.routes.Count;
            var nodeNum = sol.model.nodes.Count;
            var distances = sol.model.distances;
            
            var runConstraint5 = sol.model.supply.Count > 2;

            // vars
            LocalSearch ls = new LocalSearch(sol); // This is awful practice but I cannot access these some methods. They shouldn't be enclosed in the LS.

            // create model
            try
            {
                GRBModel SCRel = new GRBModel(Solver.gurobiEnv);

                // Model params
                SCRel.ModelName = "SC-Rel" + DateTime.Now.ToString("HH:mm:ss tt");
                SCRel.Parameters.OutputFlag = silence ? 0 : 1; // Gurobi logging
                SCRel.Parameters.Threads = 1; // usually we use 1 thread when solving MIPs for reasons of direct comparisons
                SCRel.Parameters.TimeLimit = 1 * 60; // termination condition in seconds 
                //SCRel.Parameters.MIPGap = gapLimit; // termination condition, stop when reaching X% difference between lower and upper bound 


                // Preprocessing - calculate removal and addition costs
                var A = new double[nodeNum, vehNum];
                var ANode = new Node[nodeNum, vehNum];
                var L = new double[nodeNum, vehNum];

               // initialize
                for (int i = 0; i < nodeNum; i++)
                {
                    var n = sol.model.nodes[i];
                    for (int k = 0; k < vehNum; k++)
                    {
                        A[n.serialNumber, k] = int.MaxValue; 
                        L[n.serialNumber, k] = 0;
                    }
                }

                // calculate removal cost
                for (int k = 0; k < sol.routes.Count; k++)
                {
                    Route rt = sol.routes[k];
                    for (int i = 1; i < rt.sequence.Count; i++) // depot cannot be removed - supply point can be
                    {
                        Node prev = rt.sequence[i - 1]; 
                        Node cur = rt.sequence[i];
                        Node next = (i == rt.sequence.Count - 1) ? cur : rt.sequence[i + 1];
                        
                        // the negative value represents savings
                        L[cur.serialNumber, k] = distances[prev.serialNumber, next.serialNumber] - distances[prev.serialNumber, cur.serialNumber] - distances[cur.serialNumber, next.serialNumber];
                    }
                }

                // calculate addition cost
                for (int k = 0; k < sol.routes.Count; k++)
                {
                    Route rt = sol.routes[k];

                    foreach(Node toAdd in sol.model.supply)
                    {
                        if (!rt.sequence.Contains(toAdd)) //eitherwise it cannot be added
                        {
                            Node prev = rt.sequence[0];
                            Node suc = (rt.sequence.Count == 2) ? prev : rt.sequence[2];

                            A[toAdd.serialNumber, k] = distances[prev.serialNumber, toAdd.serialNumber] + distances[toAdd.serialNumber, suc.serialNumber] - distances[prev.serialNumber, suc.serialNumber];
                        }
                    }

                    foreach (Node toAdd in sol.model.customers)
                    {
                        if (!rt.sequence.Contains(toAdd)) //eitherwise it cannot be added
                        {
                            for (int i = 1; i < rt.sequence.Count; i++) // depot cannot be removed - supply point can be
                            {
                                Node prev = rt.sequence[0];
                                Node suc = (i == rt.sequence.Count - 1) ? prev : rt.sequence[i + 1];

                                var addCost = distances[prev.serialNumber, toAdd.serialNumber] + distances[toAdd.serialNumber, suc.serialNumber] - distances[prev.serialNumber, suc.serialNumber];
                                if (A[toAdd.serialNumber, k] > addCost)
                                {
                                    A[toAdd.serialNumber, k] = addCost;
                                }
                            }
                        }
                    }
                }

                // ============================================================================================================================================================//
                // Decision variables declaration
                var a = new GRBVar[nodeNum, vehNum];
                var l = new GRBVar[nodeNum, vehNum];
                var z = new GRBVar();

                for (int k = 0; k < sol.routes.Count; k++)
                {
                    Route rt = sol.routes[k];

                    for (int i = 0; i < sol.model.nodes.Count; i++) // depot cannot be removed - supply point can be
                    {
                        Node node = sol.model.nodes[i];

                        // depots cannot change
                        if (sol.model.depots.Contains(node)) 
                        {
                            a[node.serialNumber, k] = SCRel.AddVar(0.0, 0.0, 0.0, GRB.BINARY, $"a_{node.serialNumber}_{k}");
                            l[node.serialNumber, k] = SCRel.AddVar(0.0, 0.0, 0.0, GRB.BINARY, $"l_{node.serialNumber}_{k}");
                            continue;
                        }

                        if (rt.sequence.Contains(node))
                        {
                            a[node.serialNumber, k] = SCRel.AddVar(0.0, 0.0, A[node.serialNumber, k], GRB.BINARY, $"a_{node.serialNumber}_{k}");
                            l[node.serialNumber, k] = SCRel.AddVar(0.0, 1.0, L[node.serialNumber, k], GRB.BINARY, $"l_{node.serialNumber}_{k}");
                        } 
                        else
                        {
                            a[node.serialNumber, k] = SCRel.AddVar(0.0, 1.0, A[node.serialNumber, k], GRB.BINARY, $"a_{node.serialNumber}_{k}");
                            l[node.serialNumber, k] = SCRel.AddVar(0.0, 0.0, L[node.serialNumber, k], GRB.BINARY, $"l_{node.serialNumber}_{k}");
                        }
                    }
                }
                z = SCRel.AddVar(0.0, Double.MaxValue, 1, GRB.CONTINUOUS, $"z");

                // ============================================================================================================================================================//
                // Objective sense
                SCRel.ModelSense = GRB.MINIMIZE;

                // ============================================================================================================================================================//
                // Constraints
                // Constraint 1: Max insertions
                for (int k = 0; k < sol.routes.Count; k++)
                {
                    GRBLinExpr exp1 = 0.0;
                    for (int i = 0; i < sol.model.nodes.Count; i++)
                    {
                        Node node = sol.model.nodes[i];
                        if (!sol.model.depots.Contains(node))
                        {
                            exp1.AddTerm(1.0, a[node.serialNumber, k]);
                        }
                    }
                    SCRel.AddConstr(exp1 <= maxIns, $"con1_max_insertions_{k}");
                }

                // Constraint 2: Max deletions
                for (int k = 0; k < sol.routes.Count; k++)
                {
                    Route rt = sol.routes[k];   
                    GRBLinExpr exp1 = 0.0;

                    for (int i = 1; i < rt.sequence.Count; i++)
                    {
                        Node node = rt.sequence[i];

                        exp1.AddTerm(1.0, l[node.serialNumber, k]);
                    }
                    SCRel.AddConstr(exp1 <= maxDel, $"con2_max_deletions_{k}");
                }

                // Constraint 3: Capacity (approximations apply)
                for (int k = 0; k < sol.routes.Count; k++)
                {
                    GRBLinExpr exp3 = 0.0;
                    Route rt = sol.routes[k];

                    for (int i = 0; i < sol.model.customers.Count; i++) 
                    {
                        Node node = sol.model.customers[i];
                        if (rt.sequence.Contains(node))
                        {
                            exp3.AddTerm(-1.0, l[node.serialNumber, k]);
                        }
                        else
                        {
                            exp3.AddTerm(1.0, a[node.serialNumber, k]);
                        }
                    }
                    SCRel.AddConstr(rt.load + exp3 <= sol.model.capacity, $"con3_capacity_{k}");
                }

                // Constraint 4: All customers must be visited
                for (int i = 0; i < sol.model.customers.Count; i++)
                {
                    GRBLinExpr exp4 = 0.0;
                    Node node = sol.model.customers[i];

                    for (int k = 0; k < sol.routes.Count; k++)
                    {
                        Route rt = sol.routes[k];

                        exp4.AddTerm(1.0, a[node.serialNumber, k]);
                        exp4.AddTerm(-1.0, l[node.serialNumber, k]);
                    }

                    SCRel.AddConstr(exp4 == 0, $"con4_visit_{node.serialNumber}");
                }

                // Constraint 5: Force supply point changes
                if (runConstraint5)
                {
                    GRBLinExpr exp5 = 0.0;
                    GRBLinExpr exp5b = 0.0;

                    for (int i = 0; i < sol.model.supply.Count; i++)
                    {
                        Node node = sol.model.supply[i];

                        for (int k = 0; k < sol.routes.Count; k++)
                        {
                            Route rt = sol.routes[k];

                            exp5.AddTerm(1.0, a[node.serialNumber, k]);
                            exp5b.AddTerm(1.0, l[node.serialNumber, k]);
                        }
                    }
                    SCRel.AddConstr(exp5 >= minSPchanges, $"con5_supply_point_changes");
                    SCRel.AddConstr(exp5b >= minSPchanges, $"con5b_supply_point_changes");
                }

                // Constraint 6: Only one supply point per vehicle
                for (int k = 0; k < sol.routes.Count; k++)
                {
                    GRBLinExpr exp6 = 0.0;
                    Route rt = sol.routes[k];

                    for (int i = 0; i < sol.model.supply.Count; i++)
                    {
                        Node node = sol.model.supply[i];

                        exp6.AddTerm(1.0, a[node.serialNumber, k]);
                        exp6.AddTerm(-1.0, l[node.serialNumber, k]);
                    }

                    SCRel.AddConstr(exp6 == 0, $"con6_single_supply_point_{k}");
                }

                // Constraint 7: Bind the max distance to the z variable
                for (int k = 0; k < sol.routes.Count; k++)
                {
                    GRBLinExpr exp7 = 0.0;
                    Route rt = sol.routes[k];

                    exp7.AddConstant(rt.totalDistance);

                    for (int i = 0; i < sol.model.nodes.Count; i++)
                    {
                        Node node = sol.model.nodes[i];

                        if (rt.sequence.Contains(node))
                        {
                            exp7.AddTerm(L[node.serialNumber, k], l[node.serialNumber, k]);
                        }
                        else
                        {
                            exp7.AddTerm(A[node.serialNumber, k], a[node.serialNumber, k]);
                        }
                    }
                    SCRel.AddConstr(exp7 <= z, $"con7_objective_{k}");
                }


                // ============================================================================================================================================================//
                // Valid inequalities
                // Valid inequalities are extra not required constraints that are used to cut down the solution space
                if (addValidInequalities)
                {
                    //TODO: write relationships of depots supply only 1 customers etc

                }


                // ==================================================Optimize====================================================================
                SCRel.Optimize();


                // ==================================================Results====================================================================
                switch (SCRel.Status)
                {
                    case GRB.Status.OPTIMAL:
                    case GRB.Status.TIME_LIMIT:
                        {
                            // parse solution
                            //var dictNodeChanges = new Dictionary<int, (List<Node>, List<Node>)>();
                            //var dictSPChanges = new Dictionary<int, bool>();

                            //for (int k = 0; k < sol.routes.Count; k++)
                            //{
                            //    Route rt = sol.routes[k];
                            //    var added = new List<Node>();
                            //    var removed = new List<Node>();

                            //    for (int i = 0; i < sol.model.nodes.Count; i++) // depot cannot be removed - supply point can be
                            //    {
                            //        Node node = sol.model.nodes[i];

                            //        if (Solver.IsEqual(a[node.serialNumber, k].X, 1.0, 1e-3))
                            //        {
                            //            added.Add(node);
                            //        }
                            //        if (Solver.IsEqual(l[node.serialNumber, k].X, 1.0, 1e-3))
                            //        {
                            //            removed.Add(node);
                            //        }
                            //    }
                            //    dictNodeChanges[k] = (added, removed);
                            //    var newsp = added.SingleOrDefault(x => sol.model.supply.Contains(x));
                            //    var oldsp = removed.SingleOrDefault(x => sol.model.supply.Contains(x));
                            //    dictSPChanges[k] = false;
                            //    if (newsp != null && oldsp != null)
                            //    {
                            //        dictSPChanges[k] = true;
                            //    }
                            //}

                            // update all routes
                            for (int k = 0; k < sol.routes.Count; k++)
                            {
                                Route rt = sol.routes[k];
                                if (!silence)
                                {
                                    Console.WriteLine($"Old route {k} => load: {rt.load}, distance={rt.totalDistance}");
                                }

                                //  1. remove nodes
                                int removed = 0;
                                for (int i = 0; i < sol.model.nodes.Count; i++)
                                {
                                    Node node = sol.model.nodes[i];

                                    if (Solver.IsEqual(l[node.serialNumber, k].X, 1.0, 1e-3))
                                    {
                                        if (sol.model.depots.Contains(node))
                                        {
                                            throw new Exception("Supply nodes cannot be removed/changed");
                                        }

                                        rt.sequence.Remove(node);
                                        removed++;
                                        if (sol.model.customers.Contains(node))
                                        {
                                            rt.load--;
                                        }
                                    }
                                }

                                // 2. add nodes
                                int added = 0;
                                for (int i = 0; i < sol.model.nodes.Count; i++)
                                {
                                    Node node = sol.model.nodes[i];

                                    if (Solver.IsEqual(a[node.serialNumber, k].X, 1.0, 1e-3))
                                    {
                                        if (sol.model.depots.Contains(node))
                                        {
                                            throw new Exception("Supply nodes cannot be removed/changed");
                                        }

                                        var idx = sol.model.supply.Contains(node) ? 1 : FindBestInsertionPoint(sol, rt, node);
                                        if (idx == -1)
                                        {
                                            rt.sequence.Add(node);
                                        }
                                        else
                                        {
                                            rt.sequence.Insert(idx, node);
                                        }
                                        
                                        added++;
                                        if (sol.model.customers.Contains(node))
                                        {
                                            rt.load++;
                                        }
                                    }
                                }


                                // 3. recalculate distances
                                rt.totalDistance = 0;
                                for (int i = 1; i < rt.sequence.Count; i++)
                                {
                                    Node prev = rt.sequence[i-1];
                                    Node suc = rt.sequence[i];
                                    rt.totalDistance += sol.model.distances[prev.serialNumber, suc.serialNumber];
                                }                                

                                if (!silence)
                                {
                                    Console.WriteLine($"New route {k} => load: {rt.load}, distance={rt.totalDistance} | Additions={added}, Removals={removed}");
                                }

                                //TSP optimization
                                // for random instances this may take a while. time limit in seconds and MIP gap may be added.
                                //int secLimit = 30;
                                //double mipGapLimit = 0.005; //0.5%
                                //double heur = 0.80; //80% 
                                //MathProgramming.SolveTSP(m, sol, secLimit, mipGapLimit, heur);

                                ////Console.WriteLine("Simultaneous Insertions/Deletions ({0},{1}) objective change: {2} (Dist: {3} --> {4})", maxIns,maxDel, sol.total_profit-oldProf, oldTime, sol.total_time);
                                ////MathProgramming.OptimizeNodesGivenSets(m, sol);
                            }


                            //TODO: lefmanou see if anything else needs to be updated
                            for (int k = 0; k < sol.routes.Count; k++)
                            {
                                Route rt = sol.routes[k];

                                // This is not good practice but I cannot access these methods. They shouldn't be enclosed in the LS.
                                ls.UpdateNodesInfo(1, rt);
                                ls.UpdateRouteInfo(rt);
                                //sol.UpdateArrivalTimes(rt);
                                sol.UpdateCumDistance(rt);
                                rt.IndexInRoute();
                            }

                            sol.UpdateRouteMinMax();
                            sol.objective1 = sol.CalculateObjective1();
                            sol.objective2 = sol.CalculateObjective2();
                            sol.objective3 = sol.CalculateObjective3();
                            sol.objective4 = sol.CalculateObjective4();

                            if (Debugger.IsAttached)
                            {
                                if (!sol.CheckEverything())
                                {
                                    Console.Error.WriteLine("Error in SCRel: the relaxation produced a possible non valid solution");
                                }
                            }

                            break;
                        }
                    case GRB.Status.INFEASIBLE:
                        {
                            Console.WriteLine("Model is infeasible");
                            // compute and write out IIS
                            SCRel.ComputeIIS();
                            SCRel.Write("SCRel" + SCRel.ModelName + ".ilp");
                            break;
                        }
                    case GRB.Status.UNBOUNDED:
                        {
                            Console.WriteLine("Model is unbounded");
                            break;
                        }
                    default:
                        {
                            Console.WriteLine("Optimization was stopped with status = " + SCRel.Status);
                            break;
                        }
                }
                // Dispose of model
                SCRel.Dispose();
                //gurobiEnv.Dispose();
            }
            catch (GRBException e)
            {
                Console.WriteLine("Error code: " + e.ErrorCode + ". " + e.Message);
            }
        }

        /*
         * The methods inserts the node in the best position
         */
        private static int FindBestInsertionPoint(Solution sol, Route rt, Node node)
        {
            int bestIdx = -1;
            double minCost = double.MaxValue;

            for (int idx = 2; idx < rt.sequence.Count; idx++) // Start from index 1 to avoid modifying the start depot and the supply point
            {
                Node prevNode = rt.sequence[idx - 1];
                Node nextNode = rt.sequence[idx];

                double costIncrease = sol.model.distances[prevNode.serialNumber, node.serialNumber] +
                                      sol.model.distances[node.serialNumber, nextNode.serialNumber] -
                                      sol.model.distances[prevNode.serialNumber, nextNode.serialNumber];

                if (costIncrease < minCost)
                {
                    bestIdx = idx;
                    minCost = costIncrease;
                }
            }

            return bestIdx;
        }
    }
}
