using ILOG.Concert;
using ILOG.CPLEX;
using MSOP.Fundamentals;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;


namespace MSOP.MathematicalProgramming
{

    public class CPX_TCFFormulation
    {
        public struct ExactParameters
        {
            public int maxTime;
        }
        /**
         * Solve the two-commodity flow formulation of Baldacci
         */
        public static Solution solveTCFMSOPCPX(Model m, int maxTime)
        {
            Solution sol = new Solution();
            Stopwatch stopwatch = new Stopwatch();


            //params
            bool addValidInequalities = true;
            bool branchingPriority = false;


            // fix for single veh to check
            // 11berlin52_T60_p1_v2.msop: 43
            //m.vehicle_number = 1;
            //m.t_max = 2424;

            //11eil51_T60_p1_v2.msop: 39 
            //m.vehicle_number = 1;
            //m.t_max = 104;


            // easy access
            int customersNum = m.nodes.Count - 1;
            int allNodesNum = m.nodes.Count;
            int endDepotsNum = m.vehicle_number;
            int extendedGraphNodesNum = allNodesNum + endDepotsNum;
            int setsNum = m.sets.Count;

            // extended graph nodes
            List<Node> nodes_ext = new List<Node>(m.nodes);
            for (int i = 0; i < m.vehicle_number; i++)
            {
                nodes_ext.Add(m.depot.DeepCopy());
                nodes_ext.Last().id = allNodesNum + i;
            }

            try
            {
                Cplex MSOPmodel = new Cplex();

                // MSOP model params
                MSOPmodel.SetParam(Cplex.Param.Simplex.Display, 1); // CPLEX logging
                //MSOPmodel.SetParam(Cplex.Param.MIP.Tolerances.MIPGap, 0.0); // termination condition, stop when reaching X% difference between lower and upper bound 
                MSOPmodel.SetParam(Cplex.Param.Threads, 1); // usually we use 1 thread when solving MIPs for reasons of direct comparisons
                MSOPmodel.SetParam(Cplex.Param.TimeLimit, maxTime * 60); // termination condition in seconds 
                //MSOPmodel.SetParam(Cplex.Param.MIP.Strategy.HeuristicFreq, 2); // not to be used with the heuristic effort simultaneously
                MSOPmodel.SetParam(Cplex.Param.MIP.Strategy.HeuristicEffort, 2);
                MSOPmodel.SetParam(Cplex.Param.ClockType, 2);


                // ============================================================================================================================================================//
                // Decision variables declaration
                // Binary
                INumVar[] w = new INumVar[allNodesNum]; // 1 if customer i is visted by a route, 0 otherwise
                INumVar[,] x = new INumVar[extendedGraphNodesNum, extendedGraphNodesNum]; // 1 if arc (i,j) is selected in solution, 0 otherwise
                INumVar[] s = new INumVar[setsNum]; // 1 if cluster g is selected in solution, 0 otherwise

                //Continuous
                INumVar[,] y = new INumVar[extendedGraphNodesNum, extendedGraphNodesNum]; // residual time of arc (i,j) 
                INumVar[,] z = new INumVar[extendedGraphNodesNum, extendedGraphNodesNum]; // accumulated time of arc (i,j) 

                for (int i = 0; i < allNodesNum; i++) // also depot
                {
                    Node from = m.nodes[i];
                    w[from.id] = MSOPmodel.BoolVar("w_" + from.id);
                }

                for (int g = 0; g < setsNum; g++)
                {
                    Set set = m.sets[g];
                    s[set.id] = MSOPmodel.BoolVar("s_" + set.id);
                    //if (g > 0)
                    //{
                    //    MSOPmodel.SetPriority(s[set.id], 1);
                    //}
                    //s[set.id].SetObjCoef(set.profit); // Set objective weight for binary variable w_i
                }

                for (int i = 0; i < allNodesNum; i++) // from depot to Vc between Vc
                {
                    Node from = m.nodes[i];

                    for (int j = 1; j < allNodesNum; j++)
                    {
                        if (i == j)
                        {
                            continue;
                        }
                        Node to = m.nodes[j];
                        x[from.id, to.id] = MSOPmodel.BoolVar("x_(" + from.id + "," + to.id + ")");
                        y[from.id, to.id] = MSOPmodel.NumVar(0.0, m.t_max, "y_(" + from.id + "," + to.id + ")");
                        z[from.id, to.id] = MSOPmodel.NumVar(0.0, m.t_max, "z_(" + from.id + "," + to.id + ")");
                    }
                }
                for (int i = 1; i < allNodesNum; i++) // from vc to end depots
                {
                    Node from = m.nodes[i];

                    for (int j = allNodesNum; j < extendedGraphNodesNum; j++)
                    {
                        Node to = nodes_ext[j]; //final depots
                        x[from.id, to.id] = MSOPmodel.BoolVar("x_(" + from.id + "," + to.id + ")");
                        y[from.id, to.id] = MSOPmodel.NumVar(0.0, m.t_max, "y_(" + from.id + "," + to.id + ")");
                        z[from.id, to.id] = MSOPmodel.NumVar(0.0, m.t_max, "z_(" + from.id + "," + to.id + ")");
                    }
                }

                // Branching priorities
                if (branchingPriority) // this needs to be fixed, it throws an exception
                {
                    for (int i = 1; i < s.Length; i++)
                    {                  
                        MSOPmodel.SetPriority(s[i], 2);
                    }

                    //int[] sPriorities = new int[setsNum];
                    //Array.Fill(sPriorities, 0);
                    //MSOPmodel.SetPriorities(s, sPriorities);

                    //int[,] xPriorities = new int[extendedGraphNodesNum, extendedGraphNodesNum];
                    //for (int i = 0; i < xPriorities.GetLength(0); i++)
                    //{
                    //    for (int j = 0; j < xPriorities.GetLength(1); j++)
                    //    {
                    //        xPriorities[i, j] = 1;
                    //        MSOPmodel.SetPriority(x[i,j], xPriorities[i,j]);
                    //    }
                    //}
                }

                // ============================================================================================================================================================//
                // Objective sense
                //MSOPmodel.Maximize();
                ILinearNumExpr exp_1a = MSOPmodel.LinearNumExpr();
                for (int g = 0; g < setsNum; g++)
                {
                    Set set = m.sets[g];
                    exp_1a.AddTerm(set.profit, s[set.id]);
                }
                MSOPmodel.AddMaximize(exp_1a, "exp_1a");


                // Constraints
                // Con 1b: Depot degree constraint
                ILinearNumExpr exp_1b = MSOPmodel.LinearNumExpr();
                for (int i = 1; i < allNodesNum; i++)
                {
                    Node to = m.nodes[i];
                    exp_1b.AddTerm(1.0, x[m.depot.id, to.id]);
                }
                MSOPmodel.AddEq(exp_1b, m.vehicle_number, "exp_1b");



                // Con 1c: Customer degree constraint
                for (int i = 1; i < allNodesNum; i++)
                {
                    Node from = m.nodes[i];
                    ILinearNumExpr exp = MSOPmodel.LinearNumExpr();

                    for (int j = 1; j < extendedGraphNodesNum; j++)
                    {
                        if (i != j)
                        {
                            Node to = nodes_ext[j];
                            exp.AddTerm(1.0, x[from.id, to.id]);
                        }
                    }

                    MSOPmodel.AddEq(exp, w[from.id], "con1c_" + from.id);
                }

                // Con 1c_2: Customer degree constraint
                for (int i = 1; i < allNodesNum; i++)
                {
                    Node to = m.nodes[i];
                    ILinearNumExpr exp = MSOPmodel.LinearNumExpr();

                    for (int j = 0; j < allNodesNum; j++)
                    {
                        if (i != j)
                        {
                            Node from = m.nodes[j];
                            exp.AddTerm(1.0, x[from.id, to.id]);
                        }
                    }
                    MSOPmodel.AddEq(exp, w[to.id], "con1c_2_" + to.id);
                }


                // Con 1d: Final depots degree constraint
                for (int j = allNodesNum; j < extendedGraphNodesNum; j++)
                {
                    Node to = nodes_ext[j];

                    ILinearNumExpr exp = MSOPmodel.LinearNumExpr();
                    for (int i = 1; i < allNodesNum; i++)
                    {
                        Node from = m.nodes[i];
                        exp.AddTerm(1.0, x[from.id, to.id]);
                    }
                    MSOPmodel.AddEq(exp, 1, "con1d_" + j);
                }


                // Con 1e: Link variables s and w
                for (int g = 0; g < setsNum; g++)
                {
                    Set set = m.sets[g];
                    ILinearNumExpr exp = MSOPmodel.LinearNumExpr();

                    for (int i = 0; i < set.nodes.Count; i++)
                    {
                        Node node = set.nodes[i];

                        exp.AddTerm(1.0, w[node.id]);
                    }

                    MSOPmodel.AddLe(s[g], exp, "con1e_" + g);
                }


                // Con 1f: Flow constraints for flow z
                for (int i = 1; i < allNodesNum; i++) // for each customer in Vc
                {
                    Node nodei = m.nodes[i];

                    ILinearNumExpr exp = MSOPmodel.LinearNumExpr();
                    for (int j = 1; j < extendedGraphNodesNum; j++) // Vc or end depot
                    {
                        if (i != j)
                        {
                            Node to = nodes_ext[j];
                            exp.AddTerm(1.0, z[nodei.id, to.id]);
                        }
                    }

                    ILinearNumExpr exp2 = MSOPmodel.LinearNumExpr();
                    for (int j = 0; j < allNodesNum; j++)
                    {
                        if (i != j)
                        {
                            Node from = m.nodes[j];
                            exp2.AddTerm(1.0, z[from.id, nodei.id]);
                        }
                    }

                    for (int j = 0; j < allNodesNum; j++)
                    {
                        if (i != j)
                        {
                            Node from = m.nodes[j];
                            exp2.AddTerm(m.dist_matrix[from.id, nodei.id], x[from.id, nodei.id]);
                        }
                    }

                    MSOPmodel.AddEq(exp, exp2, "con1f_" + nodei.id); //add constraint to model 
                }


                // Con 1g: Tmax
                for (int j = allNodesNum; j < extendedGraphNodesNum; j++)
                {
                    Node nodej = nodes_ext[j];

                    ILinearNumExpr exp = MSOPmodel.LinearNumExpr();
                    for (int i = 1; i < allNodesNum; i++) //only customers
                    {
                        if (i != j)
                        {
                            Node from = m.nodes[i];
                            exp.AddTerm(1.0, z[from.id, nodej.id]);
                        }
                    }

                    for (int i = 1; i < allNodesNum; i++)
                    {
                        if (i != j)
                        {
                            Node from = m.nodes[i];
                            exp.AddTerm(m.dist_matrix[from.id, m.depot.id], x[from.id, nodej.id]); // use the original depot for the distance matrix
                        }
                    }

                    MSOPmodel.AddLe(exp, m.t_max, "con1g_" + j);
                }


                // Con 1h: Flow constraints for flow y
                for (int i = 0; i < allNodesNum; i++)
                {
                    Node from = m.nodes[i];

                    for (int j = 1; j < extendedGraphNodesNum; j++)
                    {
                        if (i == j)
                        {
                            continue;
                        }
                        if (i == 0 && j >= allNodesNum)
                        {
                            continue;
                        }
                        Node to = nodes_ext[j];
                        ILinearNumExpr exp = MSOPmodel.LinearNumExpr();
                        exp.AddTerm(1.0, z[from.id, to.id]);
                        exp.AddTerm(1.0, y[from.id, to.id]);

                        ILinearNumExpr exp2 = MSOPmodel.LinearNumExpr();
                        exp2.AddTerm(m.t_max, x[from.id, to.id]);

                        MSOPmodel.AddEq(exp, exp2, "con1h_" + j);
                    }
                }

                // IMPORTANT: this is required. Otherwise some sets are repeated withing routes. For instance, a route contains two customers from the same set. 
                // 1. There is no constraint to minimize visits, so it can be done if it does not affect the objective function 
                // 2. possible triangular inequality fail?: i.e., d[i,j] >= d[i,k] + d[k,j], so k is visited even if his set is already covered by another node.
                // Con v1_b: at most 1 customer is visited per set
                for (int g = 1; g < setsNum; g++)
                {
                    Set set = m.sets[g];
                    ILinearNumExpr exp = MSOPmodel.LinearNumExpr();

                    for (int i = 0; i < set.nodes.Count; i++)
                    {
                        Node node = set.nodes[i];

                        exp.AddTerm(1.0, w[node.id]);
                    }

                    MSOPmodel.AddLe(exp, 1, "con_v_1x_" + g);
                }

                // ============================================================================================================================================================//
                if (addValidInequalities)
                {
                    // Con x: x_ij and x_ji 
                    for (int i = 1; i < allNodesNum; i++) // from depot to Vc between Vc
                    {
                        Node from = m.nodes[i];

                        for (int j = 1; j < allNodesNum; j++)
                        {
                            if (i == j)
                            {
                                continue;
                            }
                            Node to = nodes_ext[j];
                            ILinearNumExpr exp = MSOPmodel.LinearNumExpr();
                            exp.AddTerm(1.0, x[from.id, to.id]);
                            exp.AddTerm(1.0, x[to.id, from.id]);
                            MSOPmodel.AddLe(exp, 1.0, "con_v_1x_" + i + "," + j);

                        }
                    }
                }

                // ============================================================================================================================================================//

                // set callback to retrieve bounds at root node
                // create callback class
                CPXRootCallback cb = new CPXRootCallback();
                MSOPmodel.Use(cb);

                // ============================================================================================================================================================//
                // Solve the formulation
                stopwatch.Start();
                bool res = MSOPmodel.Solve();
                stopwatch.Stop();

                // ============================================================================================================================================================//
                //Console.WriteLine("Solution status = " + MSOPmodel.GetStatus());
                //Console.WriteLine("Solution value  = " + MSOPmodel.ObjValue);
                if (res)
                {
                    // Update execution data
                    Program.runData.status = MSOPmodel.GetStatus().ToString();
                    Program.runData.execTime = stopwatch.Elapsed.TotalSeconds;//GetCplexTime();GetDetTime
                    Program.runData.UB_end = MSOPmodel.ObjValue;
                    Program.runData.BestBound_end = MSOPmodel.GetBestObjValue();
                    Program.runData.Gap_end = MSOPmodel.GetMIPRelativeGap();
                    Program.runData.nodeCount = MSOPmodel.Nnodes;

                    // Update the sol object
                    List<int> startingNodes = new List<int>();

                    foreach (Route route in sol.routes)
                    {
                        Node prev = m.nodes[0];
                        bool endLoop = false;
                        while (!endLoop)
                        {
                            for (int i = 1; i < extendedGraphNodesNum; i++)
                            {
                                Node cur = nodes_ext[i];

                                if (prev.id != cur.id)
                                {
                                    if (MathProgramming.IsEqual(MSOPmodel.GetValue(x[prev.id, cur.id]), 1.0, 1e-3))
                                    {
                                        if (prev.id == m.depot.id)
                                        {
                                            if (!startingNodes.Contains(cur.id))
                                            {
                                                startingNodes.Add(cur.id);
                                            }
                                            else // skip
                                            {
                                                continue;
                                            }
                                        }
                                        if (cur.set_id == 0)
                                        {
                                            // Update the time of the route with the final depot
                                            route.time += m.dist_matrix[prev.id, m.depot.id];

                                            endLoop = true;
                                            break;
                                        }

                                        // 1. Insert the node at the Route object
                                        route.nodes_seq.Insert(route.nodes_seq.Count - 1, cur);

                                        // 2. Insert the set in the Route object
                                        route.sets_included.Insert(route.sets_included.Count - 1, m.sets[cur.set_id]);

                                        // 3. Update the time of the route
                                        route.time += m.dist_matrix[prev.id, cur.id];

                                        // 4. Update the profit of the route
                                        route.total_profit += m.sets[cur.set_id].profit;

                                        // 5. Update sets included in Solution object
                                        sol.sets_included.Add(m.sets[cur.set_id]);

                                        // 6. Update the profit of the Solution object
                                        sol.total_profit += m.sets[cur.set_id].profit;

                                        prev = cur;
                                        break;
                                    }
                                }
                            }
                        }
                    }

                    // run tests 
                    if (!sol.CheckSol())
                    {
                        Console.WriteLine("Error in solving the TCF formulation with CPLEX");
                    }
                }
                else
                {
                    Console.WriteLine("CPLEX optimization output :" + res);
                }

                MSOPmodel.End();
            }
            catch (ILOG.Concert.Exception e)
            {
                Console.WriteLine("CPLEX exception: " + e.Message);
            }
            return sol;
        }
    }

    class CPXRootCallback : Cplex.MIPInfoCallback 
    {
        private bool firstTime = true;

        public override void Main()
        {
            try
            {
                double nodecnt = GetNnodes();

                if (nodecnt == 0)
                {
                    if (firstTime)
                    {
                        this.firstTime = false;
                        Program.runData.BestBound_root = GetBestObjValue();
                    }
                    Program.runData.BestBound_root_after_cuts = GetBestObjValue();
                }                                
            }
            catch (ILOG.Concert.Exception e)
            {
                Console.WriteLine("CPLEX exception: " + e.Message);
                Console.WriteLine(e.StackTrace);
            }
            catch (System.Exception e)
            {
                Console.WriteLine("Error during callback");
                Console.WriteLine(e.StackTrace);
            }
        }
    }
}
