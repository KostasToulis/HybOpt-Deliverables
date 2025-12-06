using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Gurobi;

namespace SOP_Project
{
    class SOP
    {
        /**
         * Full SOP model solver with MTZ constraints for subtours
         */
        public static Solution SOPModelMTZ(Model m)
        {
            Solution sol = new Solution(new Route(new List<Node> { m.depot, m.depot }, m));

            //params
            bool addValidInequalities = false;

            // easy access
            int customersNum = m.nodes.Count - 1;
            int allNodesNum = m.nodes.Count;
            int setsNum = m.sets.Count;

            // create model
            try
            {
                GRBModel SOPmodel = new GRBModel(MathProgramming.gurobiEnv);

                // SOP model params
                SOPmodel.ModelName = "SOMModelMTZ" + DateTime.Now.ToString("HH:mm:ss tt");
                SOPmodel.Parameters.OutputFlag = 0; // Gurobi logging
                //PPRPmodel.Parameters.MIPGap = gapLimit; // termination condition, stop when reaching X% difference between lower and upper bound 
                SOPmodel.Parameters.Threads = 1; // usually we use 1 thread when solving MIPs for reasons of direct comparisons
                SOPmodel.Parameters.TimeLimit = 5 * 60; // termination condition in seconds 

                // ============================================================================================================================================================//
                // Decision variables declaration
                // Binary
                GRBVar[,] x = new GRBVar[allNodesNum, allNodesNum];
                GRBVar[,] u = new GRBVar[allNodesNum, allNodesNum];
                GRBVar[] y = new GRBVar[allNodesNum];
                GRBVar[] z = new GRBVar[setsNum];

                for (int i = 0; i < allNodesNum; i++)
                {
                    Node from = m.nodes[i];
                    y[from.id] = SOPmodel.AddVar(0.0, 1.0, 0.0, GRB.BINARY, "y_" + from.id); // lower bound, upper bound, factor in objective function, type, name

                    for (int j = 0; j < allNodesNum; j++)
                    {
                        if (i != j) // Speedup: i < j if we transform it to symmetric we will have half x variables 
                        {
                            Node to = m.nodes[j];
                            x[from.id, to.id] = SOPmodel.AddVar(0.0, 1.0, 0.0, GRB.BINARY, "x_" + from.id + "," + to.id);
                            u[from.id, to.id] = SOPmodel.AddVar(0.0, allNodesNum, 0.0, GRB.CONTINUOUS, "u_" + from.id + "," + to.id);
                        }
                    }
                }
                for (int g = 0; g < setsNum; g++)
                {
                    Set set = m.sets[g];
                    z[set.id] = SOPmodel.AddVar(0.0, 1.0, set.profit, GRB.BINARY, "z_" + set.id); 
                }

                /*
                // possible speedup: branching priority (we wish to branch on y_i first - to be tested)
                for (int i = 0; i < allNodesNum; i++)
                {
                    y[i].BranchPriority = 2;
                }
                */

                // ============================================================================================================================================================//
                // Objective sense
                SOPmodel.ModelSense = GRB.MAXIMIZE;

                // ============================================================================================================================================================//
                // Constraints
                // Reference [1] C. Archetti, F. Carrabs, and R. Cerulli, “The Set Orienteering Problem,” Eur. J. Oper. Res., vol. 267, no. 1, pp. 264–272, 2018.  
                // Constraint 1 (con 2 of [1]): the sum of outgoing edges of node i to every node j equals the number of visits to y_i (0/1)
                for (int i = 0; i < allNodesNum; i++) //Speed up: this could be cut down by nodes in the given sets only
                {
                    Node from = m.nodes[i];

                    GRBLinExpr exp = 0.0; //Initialize a linear expression
                    for (int j = 0; j < allNodesNum; j++) //Speed up: this could be cut down by nodes in the given sets only
                    {
                        if (i != j) //speedup symmetric vars
                        {
                            Node to = m.nodes[j];
                            exp.AddTerm(1.0, x[from.id, to.id]);
                        }
                    }
                    SOPmodel.AddConstr(exp == y[from.id], "con1_" + from.id); //add constraint to model 
                }

                // Constraint 2 (con 3 of [1]): the sum of ingoing edges to node i from every node j equals the number of visits to y_i (0/1)
                for (int i = 0; i < allNodesNum; i++) //Speed up: this could be cut down by nodes in the given sets only
                {
                    Node to = m.nodes[i];

                    GRBLinExpr exp = 0.0; //Initialize a linear expression
                    for (int j = 0; j < allNodesNum; j++) //Speed up: this could be cut down by nodes in the given sets only
                    {
                        if (i != j) //speedup symmetric vars
                        {
                            Node from = m.nodes[j];
                            exp.AddTerm(1.0, x[from.id, to.id]);
                        }
                    }
                    SOPmodel.AddConstr(exp == y[to.id], "con2_" + to.id); //add constraint to model 
                }

                /*
                //Speedup: constraints 1 and 2 may be summed up to one
                for (int i = 0; i < allNodesNum; i++) //Speed up: this could be cut down by nodes in the given sets only
                {
                    Node from = m.nodes[i];

                    GRBLinExpr exp = 0.0; //Initialize a linear expression
                    for (int j = 0; j < allNodesNum; j++) //Speed up: this could be cut down by nodes in the given sets only
                    {
                        if (i != j) //speedup symmetric vars
                        {
                            Node to = m.nodes[j];
                            exp.AddTerm(1.0, x[to.id, from.id]);
                            exp.AddTerm(1.0, x[from.id, to.id]);
                        }
                    }
                    SOPmodel.AddConstr(exp == 2*y[from.id], "con1_" + from.id); //add constraint to model 
                }
                */

                // Constraint 4 (con 5 of [1]): tour length 
                GRBLinExpr exp2 = 0.0;
                for (int i = 0; i < allNodesNum; i++) //Speed up: this could be cut down by nodes in the given sets only
                {
                    Node from = m.nodes[i];

                    for (int j = 0; j < allNodesNum; j++) //Speed up: this could be cut down by nodes in the given sets only
                    {
                        if (i != j) //speedup symmetric vars
                        {
                            Node to = m.nodes[j];
                            exp2.AddTerm(m.dist_matrix[from.id,to.id], x[from.id, to.id]);
                        }
                    }
                }
                SOPmodel.AddConstr(exp2 <= m.t_max, "con4_Tmax"); //add constraint to model 

                // Constraint 5 (con 6 of [1])
                for (int g = 0; g < setsNum; g++) 
                {
                    Set set = m.sets[g];
                    GRBLinExpr exp = 0.0;
                    for (int i = 0; i < set.nodes.Count; i++)
                    {
                        Node node = set.nodes[i];
                        exp.AddTerm(1.0, y[node.id]);
                    }
                    SOPmodel.AddConstr(z[set.id] <= exp, "con5_" + set.id); 
                }

                // ATTENTION
                // Constraint 5b: As distances is not included the solution has duplicate sets. This constraint forces only one customer per set
                for (int g = 0; g < setsNum; g++)
                {
                    Set set = m.sets[g];
                    GRBLinExpr exp = 0.0;
                    for (int i = 0; i < set.nodes.Count; i++)
                    {
                        Node node = set.nodes[i];
                        exp.AddTerm(1.0, y[node.id]);
                    }
                    SOPmodel.AddConstr(exp <= 1, "con5b_" + set.id);
                }

                // MTZ Constraints for subtour elimination
                // Constraint 6 (con 10 of [1]): MTZ subtour elimination
                for (int i = 1; i < allNodesNum; i++) //exclude depot
                {
                    Node from = m.nodes[i];

                    GRBLinExpr exp = 0.0;
                    for (int j = 0; j < allNodesNum; j++)
                    {
                        if (i != j) //speedup symmetric vars
                        {
                            Node to = m.nodes[j];
                            exp.AddTerm(1.0, u[from.id, to.id]);
                            exp.AddTerm(-1.0, u[to.id, from.id]);
                        }
                    }
                    SOPmodel.AddConstr(exp == y[from.id], "con5_flow_" + from.id);
                }

                // Constraint 7 (con 11 of [1]): MTZ subtour elimination
                for (int i = 0; i < allNodesNum; i++)
                {
                    Node from = m.nodes[i];
                    for (int j = 0; j < allNodesNum; j++)
                    {

                        if (i != j) //speedup symmetric vars
                        {
                            Node to = m.nodes[j];
                            SOPmodel.AddConstr(u[from.id, to.id] <= (allNodesNum - 1) * x[from.id, to.id], "con6_flow_" + from.id);
                        }
                    }
                }

                // ============================================================================================================================================================//
                // Valid inequalities
                // Valid inequalities are extra not required constraints that are used to cut down the solution space
                if (addValidInequalities)
                {
                    // Valid inequality 1
                    for (int i = 0; i < allNodesNum; i++) //Speed up: this could be cut down by nodes in the given sets only
                    {
                        Node to = m.nodes[i];

                        for (int j = 0; j < allNodesNum; j++) //Speed up: this could be cut down by nodes in the given sets only
                        {
                            if (i != j)
                            {
                                Node from = m.nodes[j];
                                SOPmodel.AddConstr(x[from.id, to.id] + x[to.id, from.id] <= 1, "vi_1_" + from.id + "," + to.id); //add constraint to model 
                            }
                        }
                    }
                }

                // ==================================================Optimize====================================================================
                SOPmodel.Optimize();

                // ==================================================Results====================================================================
                switch (SOPmodel.Status)
                {
                    case GRB.Status.OPTIMAL:
                    case GRB.Status.TIME_LIMIT:
                        {
                            // 1. update sol profit
                            sol.total_profit = (int)Math.Round(SOPmodel.ObjVal);
                            sol.route.total_profit = (int)Math.Round(SOPmodel.ObjVal);

                            // 2. update time
                            int totalDistance = 0;
                            for (int i = 0; i < allNodesNum; i++)
                            {
                                Node from = m.nodes[i];
                                for (int j = 0; j < allNodesNum; j++)
                                {
                                    if (i != j) 
                                    {
                                        Node to = m.nodes[j];
                                        if (MathProgramming.IsEqual(x[from.id, to.id].X, 1.0, 1e-3))
                                        {
                                            totalDistance += m.dist_matrix[from.id, to.id];
                                        }
                                    }
                                }
                            }

                            sol.total_time = totalDistance;
                            sol.route.time = totalDistance;

                            // 3. update the nodes lists
                            sol.route.nodes_seq.Clear();
                            sol.route.sets_included.Clear();
                            //foreach (Set set in m.sets) { set.in_route = false; }

                            Node prev = m.nodes[0];
                            sol.route.nodes_seq.Add(prev);
                            sol.route.sets_included.Add(m.sets[prev.set_id]);
                            //m.sets[prev.set_id].in_route = true;

                            bool endLoop = false;
                            while (!endLoop)
                            {
                                for (int i = 0; i < allNodesNum; i++)
                                {
                                    Node cur = m.nodes[i];

                                    if (prev.id!= cur.id)
                                    {
                                        if (MathProgramming.IsEqual(x[prev.id, cur.id].X, 1.0, 1e-3))
                                        {
                                            sol.route.nodes_seq.Add(cur);
                                            sol.route.sets_included.Add(m.sets[cur.set_id]);
                                            //m.sets[cur.set_id].in_route = true;
                                            if (cur.id == m.depot.id)
                                            {
                                                endLoop = true;
                                            }
                                            prev = cur;
                                            break;
                                        }
                                    }
                                }
                            }
                            
                            
                             //Reporting 
                            for (int i = 0; i < allNodesNum; i++)
                            {
                                Node from = m.nodes[i];
                                if (MathProgramming.IsEqual(y[from.id].X, 1.0, 1e-3))
                                {
                                    Console.WriteLine("y_{0}=1", from.id);
                                }
                            }
                            for (int i = 0; i < allNodesNum; i++)
                            {
                                Node from = m.nodes[i];
                                for (int j = 0; j < allNodesNum; j++)
                                {
                                    if (i != j) // Speedup: i < j if we transform it to symmetric we will have half x variables 
                                    {
                                        Node to = m.nodes[j];
                                        if (MathProgramming.IsEqual(x[from.id, to.id].X, 1.0, 1e-3))
                                        {
                                            Console.WriteLine("x_{0},{1}=1", from.id, to.id);
                                            //Console.WriteLine("u_{0},{1}={2}", from.id, to.id,u[from.id, to.id].X);

                                        }
                                    }
                                }
                            }
                            

                            for (int i = 0; i < allNodesNum; i++)
                            {
                                Node from = m.nodes[i];
                                for (int j = 0; j < allNodesNum; j++)
                                {
                                    if (i != j) // Speedup: i < j if we transform it to symmetric we will have half x variables 
                                    {
                                        Node to = m.nodes[j];
                                        if (MathProgramming.IsEqual(x[from.id, to.id].X, 1.0, 1e-3))
                                        {
                                            Console.WriteLine("u_{0},{1}={2}", from.id, to.id, u[from.id, to.id].X);
                                        }
                                    }
                                }
                            }
                            

                            // run tests 
                            if (!sol.route.CheckRoute(m))
                            {
                                Console.WriteLine("Error");
                                return new Solution(new Route(new List<Node> { m.depot, m.depot }, m));
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
                            Console.WriteLine("Optimization was stopped with status = " + SOPmodel.Status);
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

        /**
         * Full SOP model solver with callback and lazy subtour elimination constraints
         */
        public static Solution SOPModelLazy(Model m)
        {
            //Solution sol = new Solution(new Route(0, new List<Node> { m.depot, m.depot }, m));
            throw new NotImplementedException();
            //return sol;
        }
    }
}
