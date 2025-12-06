using Gurobi;
using MSOP.Fundamentals;
using System;
using System.Collections.Generic;
using System.Linq;

namespace MSOP.MathematicalProgramming
{
    public class GRB_TCFFormulation
    {
        public struct ExactParameters
        {
            public int maxTime;
        }

        public static Solution solveTCFMSOPGRB(Model m, int maxTime)
        {
            Solution sol = new Solution();

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
                GRBModel SOPmodel = new GRBModel(MathProgramming.gurobiEnv);

                // SOP model params
                SOPmodel.ModelName = "MSOPModelTCFGRB" + DateTime.Now.ToString("HH:mm:ss tt");
                SOPmodel.Parameters.OutputFlag = 1; // Gurobi logging
                //SOPmodel.Parameters.MIPGap = gapLimit; // termination condition, stop when reaching X% difference between lower and upper bound 
                SOPmodel.Parameters.Threads = 1; // usually we use 1 thread when solving MIPs for reasons of direct comparisons
                SOPmodel.Parameters.TimeLimit = maxTime * 60; // termination condition in seconds 
                SOPmodel.Parameters.Heuristics = 0.15; // 0-1, default = 0.05

                // Decision variables declaration
                // Binary
                GRBVar[] w = new GRBVar[allNodesNum]; // 1 if customer i is visted by a route, 0 otherwise
                GRBVar[,] x = new GRBVar[extendedGraphNodesNum, extendedGraphNodesNum]; // 1 if arc (i,j) is selected in solution, 0 otherwise
                GRBVar[] s = new GRBVar[setsNum]; // 1 if cluster g is selected in solution, 0 otherwise

                //Continuous
                GRBVar[,] y = new GRBVar[extendedGraphNodesNum, extendedGraphNodesNum]; // residual time of arc (i,j) 
                GRBVar[,] z = new GRBVar[extendedGraphNodesNum, extendedGraphNodesNum]; // accumulated time of arc (i,j) 

                for (int i = 0; i < allNodesNum; i++) // also depot
                {
                    Node from = m.nodes[i];
                    w[from.id] = SOPmodel.AddVar(0.0, 1.0, 0.0, GRB.BINARY, "w_" + from.id);
                }

                for (int g = 0; g < setsNum; g++)
                {
                    Set set = m.sets[g];
                    s[set.id] = SOPmodel.AddVar(0.0, 1.0, set.profit, GRB.BINARY, "s_" + set.id);
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
                        x[from.id, to.id] = SOPmodel.AddVar(0.0, 1.0, 0.0, GRB.BINARY, "x_(" + from.id + "," + to.id + ")");
                        y[from.id, to.id] = SOPmodel.AddVar(0.0, m.t_max, 0.0, GRB.CONTINUOUS, "y_(" + from.id + "," + to.id + ")");
                        z[from.id, to.id] = SOPmodel.AddVar(0.0, m.t_max, 0.0, GRB.CONTINUOUS, "z_(" + from.id + "," + to.id + ")");
                    }
                }
                for (int i = 1; i < allNodesNum; i++) // from vc to end depots
                {
                    Node from = m.nodes[i];

                    for (int j = allNodesNum; j < extendedGraphNodesNum; j++)
                    {
                        Node to = nodes_ext[j]; //final depots
                        x[from.id, to.id] = SOPmodel.AddVar(0.0, 1.0, 0.0, GRB.BINARY, "x_(" + from.id + "," + to.id + ")");
                        y[from.id, to.id] = SOPmodel.AddVar(0.0, m.t_max, 0.0, GRB.CONTINUOUS, "y_(" + from.id + "," + to.id + ")");
                        z[from.id, to.id] = SOPmodel.AddVar(0.0, m.t_max, 0.0, GRB.CONTINUOUS, "z_(" + from.id + "," + to.id + ")");
                    }
                }

                // Branching priorities
                if (branchingPriority)
                {
                    for (int g = 0; g < setsNum; g++)
                    {
                        Set set = m.sets[g];
                        s[set.id].BranchPriority = 2;
                    }
                }

                // ============================================================================================================================================================//
                // Objective sense
                SOPmodel.ModelSense = GRB.MAXIMIZE;

                // ============================================================================================================================================================//
                // Constraints

                // Con 1b: Depot degree constraint
                GRBLinExpr exp_1b = 0.0;
                for (int i = 1; i < allNodesNum; i++)
                {
                    Node to = m.nodes[i];
                    exp_1b.AddTerm(1.0, x[m.depot.id, to.id]);
                }
                SOPmodel.AddConstr(exp_1b == m.vehicle_number, "exp_1b");

                // Con 1c: Customer degree constraint
                for (int i = 1; i < allNodesNum; i++)
                {
                    Node from = m.nodes[i];
                    GRBLinExpr exp = 0.0;

                    for (int j = 1; j < extendedGraphNodesNum; j++)
                    {
                        if (i != j)
                        {
                            Node to = nodes_ext[j];
                            exp.AddTerm(1.0, x[from.id, to.id]);
                        }
                    }

                    SOPmodel.AddConstr(exp == w[from.id], "con1c_" + from.id);
                }

                // Con 1c_2: Customer degree constraint
                for (int i = 1; i < allNodesNum; i++)
                {
                    Node to = m.nodes[i];
                    GRBLinExpr exp = 0.0;

                    for (int j = 0; j < allNodesNum; j++)
                    {
                        if (i != j)
                        {
                            Node from = m.nodes[j];
                            exp.AddTerm(1.0, x[from.id, to.id]);
                        }
                    }
                    SOPmodel.AddConstr(exp == w[to.id], "con1c_2_" + to.id);
                }


                // Con 1d: Final depots degree constraint
                for (int j = allNodesNum; j < extendedGraphNodesNum; j++)
                {
                    Node to = nodes_ext[j];

                    GRBLinExpr exp = 0.0;
                    for (int i = 1; i < allNodesNum; i++)
                    {
                        Node from = m.nodes[i];
                        exp.AddTerm(1.0, x[from.id, to.id]);
                    }
                    SOPmodel.AddConstr(exp == 1, "con1d_" + j);
                }


                // Con 1e: Link variables s and w
                for (int g = 0; g < setsNum; g++)
                {
                    Set set = m.sets[g];
                    GRBLinExpr exp = 0.0;

                    for (int i = 0; i < set.nodes.Count; i++)
                    {
                        Node node = set.nodes[i];

                        exp.AddTerm(1.0, w[node.id]);
                    }

                    SOPmodel.AddConstr(s[g] <= exp, "con1e_" + g);
                }


                // Con 1f: Flow constraints for flow z
                for (int i = 1; i < allNodesNum; i++) // for each customer in Vc
                {
                    Node nodei = m.nodes[i];

                    GRBLinExpr exp = 0.0;
                    for (int j = 1; j < extendedGraphNodesNum; j++) // Vc or end depot
                    {
                        if (i != j)
                        {
                            Node to = nodes_ext[j];
                            exp.AddTerm(1.0, z[nodei.id, to.id]);
                        }
                    }

                    GRBLinExpr exp2 = 0.0;
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

                    SOPmodel.AddConstr(exp == exp2, "con1f_" + nodei.id); //add constraint to model 
                }


                // Con 1g: Tmax
                for (int j = allNodesNum; j < extendedGraphNodesNum; j++)
                {
                    Node nodej = nodes_ext[j];

                    GRBLinExpr exp = 0.0;
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

                    SOPmodel.AddConstr(exp <= m.t_max, "con1g_" + j);
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
                        GRBLinExpr exp = 0.0;
                        exp.AddTerm(1.0, z[from.id, to.id]);
                        exp.AddTerm(1.0, y[from.id, to.id]);

                        GRBLinExpr exp2 = 0.0;
                        exp2.AddTerm(m.t_max, x[from.id, to.id]);

                        SOPmodel.AddConstr(exp == exp2, "con1h_" + j);
                    }
                }

                // IMPORTANT: this is required. Otherwise some sets are repeated withing routes. For instance, a route contains two customers from the same set. 
                // 1. There is no constraint to minimize visits, so it can be done if it does not affect the objective function 
                // 2. possible triangular inequality fail?: i.e., d[i,j] >= d[i,k] + d[k,j], so k is visited even if his set is already covered by another node.
                // Con v1_b: at most 1 customer is visited per set
                for (int g = 1; g < setsNum; g++)
                {
                    Set set = m.sets[g];
                    GRBLinExpr exp = 0.0;

                    for (int i = 0; i < set.nodes.Count; i++)
                    {
                        Node node = set.nodes[i];

                        exp.AddTerm(1.0, w[node.id]);
                    }

                    SOPmodel.AddConstr(exp <= 1, "con_v_1x_" + g);
                }

                // ============================================================================================================================================================//
                if (addValidInequalities)
                {
                    // Con v1_a: x_ij and x_ji, at most one can be used
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
                            GRBLinExpr exp = 0.0;
                            exp.AddTerm(1.0, x[from.id, to.id]);
                            exp.AddTerm(1.0, x[to.id, from.id]);
                            SOPmodel.AddConstr(exp <= 1.0, "con_v_1x_" + i + "," + j);
                        }
                    }
                }

                // ============================================================================================================================================================//
                // set callback to retrieve bounds at root node
                // create callback class
                GRBRootCallback cb = new GRBRootCallback();

                // set callback to retrieve bounds of the solution
                SOPmodel.SetCallback(cb);


                // ============================================================================================================================================================//
                //solve the formulation
                // ================================================== Optimize ====================================================================
                SOPmodel.Optimize();

                // ================================================== Results ====================================================================
                switch (SOPmodel.Status)
                {
                    case GRB.Status.OPTIMAL:
                    case GRB.Status.TIME_LIMIT:
                        {
                            // Update execution data
                            Program.runData.status = SOPmodel.Status.ToString();
                            Program.runData.execTime = SOPmodel.Runtime;
                            Program.runData.UB_end = SOPmodel.ObjVal;
                            Program.runData.BestBound_end = SOPmodel.ObjBound;
                            Program.runData.Gap_end = SOPmodel.MIPGap;
                            Program.runData.nodeCount = SOPmodel.NodeCount;

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
                                            if (MathProgramming.IsEqual(x[prev.id, cur.id].X, 1.0, 1e-3))
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
                                Console.WriteLine("Error in solving the TCF formulation with Gurobi");
                            }
                            break;
                        }
                    case GRB.Status.INFEASIBLE:
                        {
                            Console.WriteLine("Model is infeasible");
                            // compute and write out IIS
                            SOPmodel.ComputeIIS();
                            SOPmodel.Write(SOPmodel.ModelName + ".ilp");
                            break;
                        }
                    case GRB.Status.UNBOUNDED:
                        {
                            Console.WriteLine("Model is unbounded");
                            break;
                        }
                    default:
                        {
                            Console.WriteLine("Optimization status = " + SOPmodel.Status);
                            break;
                        }
                }
                // Dispose of model
                SOPmodel.Dispose();
                //gurobiEnv.Dispose();
            }
            catch (GRBException e)
            {
                Console.WriteLine("Error code: " + e.ErrorCode + ". " + e.Message);
            }
            return sol;
        }
    }

    class GRBRootCallback : GRBCallback
    {
        private bool firstTime = true;

        protected override void Callback()
        {
            try
            {
                // where
                if (where == GRB.Callback.MIP)
                {
                    // General MIP callback
                    double nodecnt = GetDoubleInfo(GRB.Callback.MIP_NODCNT);
                    
                    if (nodecnt == 0)
                    {
                        //rootUb = GetDoubleInfo(GRB.Callback.MIP_OBJBST);
                        if (firstTime)
                        {
                            this.firstTime = false;
                            Program.runData.BestBound_root = GetDoubleInfo(GRB.Callback.MIP_OBJBND);
                        }
                        Program.runData.BestBound_root_after_cuts = GetDoubleInfo(GRB.Callback.MIP_OBJBND);
                    }

                }
            } catch (GRBException e) {
              Console.WriteLine("Error code: " + e.ErrorCode);
              Console.WriteLine(e.Message);
              Console.WriteLine(e.StackTrace);
            } catch (Exception e) {
              Console.WriteLine("Error during callback");
              Console.WriteLine(e.StackTrace);
            }
        }
    }
}
