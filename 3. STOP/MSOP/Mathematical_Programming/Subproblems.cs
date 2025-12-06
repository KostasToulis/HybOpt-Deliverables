using Gurobi;
using MSOP.Fundamentals;
using System;
using System.Collections.Generic;
using System.Linq;

namespace MSOP.MathematicalProgramming
{
    class Subproblems
    {
        /**
         * Given a solution, the methods preserves the clusters and its order and optimizes the nodes visited in its cluster with respect to the distance travelled
         */
        public static void OptimizeNodesGivenSetsSubproblem(Model m, Solution sol)
        {
            //params
            bool addValidInequalities = false;
            bool silence = true;

            // easy access
            int customersNum = m.nodes.Count - 1;
            int allNodesNum = m.nodes.Count;


            // create model
            try
            {
                // separately for each route
                foreach (Route route in sol.routes)
                {
                    //Console.WriteLine(route);
                    GRBModel SOPmodel = new GRBModel(MathProgramming.gurobiEnv);

                    // SOP model params
                    SOPmodel.ModelName = "OptimizeNodesGivenSets" + DateTime.Now.ToString("HH:mm:ss tt");
                    SOPmodel.Parameters.OutputFlag = 0; // Gurobi logging
                    //PPRPmodel.Parameters.MIPGap = gapLimit; // termination condition, stop when reaching X% difference between lower and upper bound 
                    SOPmodel.Parameters.Threads = 1; // usually we use 1 thread when solving MIPs for reasons of direct comparisons
                    SOPmodel.Parameters.TimeLimit = 1 * 60; // termination condition in seconds 

                    // Preprocessing
                    //edgeset
                    bool[,] edgeArray = new bool[allNodesNum, allNodesNum];

                    for (int g = 1; g < route.sets_included.Count; g++)
                    {
                        Set set_pred = route.sets_included[g - 1];
                        Set set_suc = route.sets_included[g];

                        for (int i = 0; i < set_pred.nodes.Count; i++)
                        {
                            Node from = set_pred.nodes[i];

                            for (int j = 0; j < set_suc.nodes.Count; j++)
                            {
                                Node to = set_suc.nodes[j];

                                if (from.id != to.id)
                                {
                                    edgeArray[from.id, to.id] = true;
                                    edgeArray[to.id, from.id] = true;
                                }
                            }
                        }
                    }

                    // ============================================================================================================================================================//
                    // Decision variables declaration
                    // Binary
                    GRBVar[,] x = new GRBVar[allNodesNum, allNodesNum];
                    GRBVar[] y = new GRBVar[allNodesNum];

                    for (int i = 0; i < allNodesNum; i++)
                    {
                        Node from = m.nodes[i];
                        y[from.id] = SOPmodel.AddVar(0.0, 1.0, 0.0, GRB.BINARY, "y_" + from.id); // lower bound, upper bound, factor in objective function, type, name

                        for (int j = 0; j < allNodesNum; j++)
                        {
                            Node to = m.nodes[j];

                            if (edgeArray[from.id, to.id])
                            {
                                x[from.id, to.id] = SOPmodel.AddVar(0.0, 1.0, m.dist_matrix[from.id, to.id], GRB.BINARY, "x_" + from.id + "," + to.id);
                            }
                        }
                    }

                    // possible speedup: branching priority (we wish to branch on y_i first - to be tested)
                    for (int i = 0; i < allNodesNum; i++)
                    {
                        y[i].BranchPriority = 2;
                    }

                    // ============================================================================================================================================================//
                    // Objective sense
                    SOPmodel.ModelSense = GRB.MINIMIZE;

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
                            Node to = m.nodes[j];

                            if (edgeArray[from.id, to.id])
                            {
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
                            Node from = m.nodes[j];

                            if (edgeArray[from.id, to.id])
                            {
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
                            Node to = m.nodes[j];

                            if (edgeArray[from.id,to.id])
                            {
                                exp.AddTerm(1.0, x[to.id, from.id]);
                                exp.AddTerm(1.0, x[from.id, to.id]);
                            }
                        }
                        SOPmodel.AddConstr(exp == 2*y[from.id], "con1aggr_" + from.id); //add constraint to model 
                    }
                    */


                    // Constraint 3 (similar to con 6 of [1]): exactly one node of every given (from sol) set must be selected
                    for (int g = 0; g < route.sets_included.Count; g++)
                    {
                        Set set = route.sets_included[g];

                        GRBLinExpr exp = 0.0; //Initialize a linear expression
                        for (int i = 0; i < set.nodes.Count; i++)
                        {
                            Node node = set.nodes[i];
                            exp.AddTerm(1.0, y[node.id]);
                        }
                        SOPmodel.AddConstr(exp == 1, "con3_" + set.id);
                    }


                    // Constraint 4: forces the sum of the edges between two consecutive sets to be 1.
                    for (int g = 1; g < route.sets_included.Count; g++)
                    {
                        Set set_pred = route.sets_included[g - 1];
                        Set set_suc = route.sets_included[g];

                        GRBLinExpr exp = 0.0; //Initialize a linear expression
                        for (int i = 0; i < set_pred.nodes.Count; i++)
                        {
                            Node from = set_pred.nodes[i];

                            for (int j = 0; j < set_suc.nodes.Count; j++)
                            {
                                Node to = set_suc.nodes[j];
                                if (edgeArray[from.id, to.id])
                                {
                                    exp.AddTerm(1.0, x[from.id, to.id]);

                                }
                            }
                        }
                        SOPmodel.AddConstr(exp == 1, "con4_" + set_suc.id); //add constraint to model 
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
                                Node from = m.nodes[j];
                                if (edgeArray[from.id, to.id])
                                {
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
                                if (!silence)
                                {
                                    Console.WriteLine("Before optimizing nodes given the sets");
                                    Console.WriteLine(route);
                                }

                                // 1. update route time (profits are the same)
                                int oldTime = route.time;
                                route.time = (int)Math.Round(SOPmodel.ObjVal);

                                // 2. update the nodes lists
                                List<Node> oldRouteDC = route.nodes_seq.ConvertAll(node => new Node(node.id, node.x, node.y, node.set_id));
                                //List<Node> optimizedNodes = sol.route.nodes_seq;

                                route.nodes_seq.Clear();

                                // ATTENTION: assumption that the sets included list is always in order of visit!
                                for (int g = 0; g < route.sets_included.Count; g++)
                                {
                                    Set set = route.sets_included[g];
                                    for (int i = 0; i < set.nodes.Count; i++) //Speed up: this could be cut down by nodes in the given sets only
                                    {
                                        Node node = set.nodes[i];
                                        if (MathProgramming.IsEqual(y[node.id].X, 1.0, 1e-3)) // .X accesses the value give to the variable
                                        {
                                            route.nodes_seq.Add(node);
                                            break; //save time from checking the rest
                                        }
                                    }
                                }


                                //for (int i = 0; i < allNodesNum; i++)
                                //{
                                //    Node from = m.nodes[i];
                                //    if (MathProgramming.IsEqual(y[from.id].X, 1.0, 1e-3))
                                //    {
                                //        Console.WriteLine("y_{0}=1", from.id);
                                //    }
                                //}
                                //for (int i = 0; i < allNodesNum; i++)
                                //{
                                //    Node from = m.nodes[i];
                                //    for (int j = 0; j < allNodesNum; j++)
                                //    {
                                //        Node to = m.nodes[j];

                                //        if (edgeArray[from.id,to.id])
                                //        { 
                                //            if (MathProgramming.IsEqual(x[from.id, to.id].X, 1.0, 1e-3))
                                //            {
                                //                Console.WriteLine("x_{0},{1}=1", from.id, to.id);
                                //            }
                                //        }
                                //    }
                                //}


                                // Count differences 
                                //int count = 0;
                                //Console.WriteLine();
                                //for (int i = 0; i < oldRouteDC.Count; i++)
                                //{
                                //    Node oldNode = oldRouteDC[i];
                                //    Node newNode = sol.route.nodes_seq[i];
                                //    if (oldNode.id == newNode.id)
                                //    {
                                //        count++;
                                //        Console.Write("1");
                                //    } else
                                //    {
                                //        Console.Write("0");
                                //    }
                                //}
                                //Console.WriteLine();
                                //Console.WriteLine("The IP changed " + count + " nodes given the sets of the solution");

                                // update sol and run tests (can be removed for runs)
                                if (!route.CheckRoute())
                                {
                                    Console.WriteLine("Error");
                                }
                                if (oldTime != route.time)
                                {
                                    if (!silence)
                                    {
                                        Console.WriteLine("Improvement from OptNodes: old distance = {0} --> optimized distance = {1}", oldTime, route.time);
                                    }
                                }
                                if (!silence)
                                {
                                    Console.WriteLine("After optimizing nodes given the sets");
                                    Console.WriteLine(route);
                                }

                                break;
                            }
                        case GRB.Status.INFEASIBLE:
                            {
                                Console.WriteLine("Model is infeasible");
                                // compute and write out IIS
                                SOPmodel.ComputeIIS();
                                SOPmodel.Write("OptimizeNodesGivenSets_" + SOPmodel.ModelName + ".ilp");
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

            }
            catch (GRBException e)
            {
                Console.WriteLine("Error code: " + e.ErrorCode + ". " + e.Message);
            }
        }

        /**
         * Simultaneous up to maxIns insertions and up to maxDel deletions with routing approximations. The method works on sets. Only the best insertion position for each set is considered.
         * The best node in the best route. 
         */
        public static void SimulInsDelSubproblem(Model m, Solution sol, int maxIns, int maxDel, double custom_tmax, bool hide_tmax_constraint = false)
        {
            // params 
            bool addValidInequalities = false;
            bool silence = true;
            int maxRouteIns = 5;
            int maxRouteDels = 5;


            // easy access
            int customersNum = m.nodes.Count - 1;
            int allNodesNum = m.nodes.Count;
            int setsNum = m.sets.Count;
            int vehsNum = sol.routes.Count;

            // create model
            try
            {
                GRBModel SIDSubproblem = new GRBModel(MathProgramming.gurobiEnv);

                // SOP model params
                SIDSubproblem.ModelName = "SIDSubproblem" + DateTime.Now.ToString("HH:mm:ss tt");
                SIDSubproblem.Parameters.OutputFlag = 0; // Gurobi logging
                //PPRPmodel.Parameters.MIPGap = gapLimit; // termination condition, stop when reaching X% difference between lower and upper bound 
                SIDSubproblem.Parameters.Threads = 1; // usually we use 1 thread when solving MIPs for reasons of direct comparisons
                SIDSubproblem.Parameters.TimeLimit = 1 * 60; // termination condition in seconds 

                // Preprocessing
                bool[] existsInSol = new bool[setsNum];

                for (int g = 0; g < setsNum; g++)
                {
                    Set set = m.sets[g];
                    foreach (Route route in sol.routes)
                    {
                        if (route.sets_included.Contains(set))
                        {
                            existsInSol[set.id] = true;
                            break; // it cannot be visited by more than one vehicles (suboptimal)
                        }
                    }
                }

                // auxiliary structures initialization
                int[,] addCost = new int[setsNum, vehsNum];
                Node[,] addNode = new Node[setsNum, vehsNum];
                //Route[,] addRoute = new Route[setsNum, vehsNum];
                int[] remCost = new int[setsNum];

                for (int l = 1; l < setsNum; l++)
                {
                    Set set = m.sets[l];

                    foreach (Route route in sol.routes)
                    {
                        addCost[set.id, route.id] = int.MaxValue;
                        addNode[set.id, route.id] = null;
                        //addRoute[set.id, route.id] = null;
                    }
                    remCost[set.id] = 0;
                }

                // populate auxiliary structures - calculate the removal savings
                foreach (Route route in sol.routes)
                {
                    for (int g = 1; g < route.sets_included.Count - 1; g++) // nodes except depots
                    {
                        Set set = route.sets_included[g];
                        Set set_pred = route.sets_included[g - 1];
                        Set set_suc = route.sets_included[g + 1];
                        Node node = route.nodes_seq[g];
                        Node node_pred = route.nodes_seq[g - 1];
                        Node node_suc = route.nodes_seq[g + 1];

                        remCost[set.id] = m.dist_matrix[node_pred.id, node_suc.id] - m.dist_matrix[node_pred.id, node.id] - m.dist_matrix[node.id, node_suc.id]; // Caution: negative cost (saving)
                    }
                }

                // populate auxiliary structures - calculate the insertion costs
                for (int l = 0; l < setsNum; l++)
                {
                    Set set = m.sets[l];

                    foreach (Route route in sol.routes)
                    {
                        //Note: check to allow removal and insertion or not
                        // if route does not contain a set it can be added (even if it is deleted from another route)
                        if (!route.sets_included.Contains(set)) //!existsInSol[set.id]) 
                        {
                            for (int g = 1; g < route.sets_included.Count; g++)
                            {
                                Set set_pred = route.sets_included[g - 1];
                                Set set_suc = route.sets_included[g];
                                Node node_pred = route.nodes_seq[g - 1];
                                Node node_suc = route.nodes_seq[g];

                                for (int i = 0; i < set.nodes.Count; i++) // check the best node if the set is to be visited
                                {
                                    Node node = set.nodes[i];
                                    if (m.dist_matrix[node_pred.id, node.id] + m.dist_matrix[node.id, node_suc.id] -
                                        m.dist_matrix[node_pred.id, node_suc.id] < addCost[set.id, route.id])
                                    {
                                        addCost[set.id, route.id] = m.dist_matrix[node_pred.id, node.id] + m.dist_matrix[node.id, node_suc.id] -
                                            m.dist_matrix[node_pred.id, node_suc.id];
                                        addNode[set.id, route.id] = node; // ref
                                        //addRoute[set.id, route.id] = route; // ref
                                    }
                                }
                            }
                        }
                        else
                        {

                        }
                    }
                }

                // ============================================================================================================================================================//
                // Decision variables declaration
                GRBVar[,] add = new GRBVar[setsNum, vehsNum];
                GRBVar[] rem = new GRBVar[setsNum];

                for (int g = 1; g < setsNum; g++)
                {
                    Set set = m.sets[g];

                    if (existsInSol[g])
                    {
                        rem[set.id] = SIDSubproblem.AddVar(0.0, 1.0, -set.profit, GRB.BINARY, "rem_" + set.id);
                    }
                    else
                    {
                        rem[set.id] = SIDSubproblem.AddVar(0.0, 0.0, -set.profit, GRB.BINARY, "rem_" + set.id); // cannot be deleted
                    }

                    // whether it exists in sol or not
                    foreach (Route route in sol.routes)
                    {
                        //NOTE: && !(existsInSol[g]: adding this means that a set cannot be removed from a route and added to another route
                        // at the same time. Use this or constraint 5
                        //if (!route.sets_included.Contains(set) && !(existsInSol[g])) 
                        if (!route.sets_included.Contains(set)) // && !(existsInSol[g]))
                        {
                            add[set.id, route.id] = SIDSubproblem.AddVar(0.0, 1.0, set.profit, GRB.BINARY, "add_" + set.id + "_" + route.id);
                        }
                        else
                        {
                            add[set.id, route.id] = SIDSubproblem.AddVar(0.0, 0.0, set.profit, GRB.BINARY, "add_" + set.id + "_" + route.id);
                        }
                    }
                }

                // ============================================================================================================================================================//
                // Objective sense
                SIDSubproblem.ModelSense = GRB.MAXIMIZE;

                // ============================================================================================================================================================//
                // Constraints
                // Constraint 1: Max insertions
                GRBLinExpr exp1 = 0.0;
                for (int g = 1; g < setsNum; g++)
                {
                    Set set = m.sets[g];
                    foreach (Route route in sol.routes)
                    {
                        if (!route.sets_included.Contains(set))
                        {
                            exp1.AddTerm(1.0, add[set.id, route.id]);
                        }
                    }
                }
                SIDSubproblem.AddConstr(exp1 <= maxIns, "con1_max_insertions");

                // OR Constraint 1: Max insertions per vehicle
                foreach (Route route in sol.routes)
                {
                    GRBLinExpr exp1b = 0.0;
                    for (int g = 1; g < setsNum; g++)
                    {
                        Set set = m.sets[g];
                        if (!route.sets_included.Contains(set))
                        {
                            exp1b.AddTerm(1.0, add[set.id, route.id]);
                        }
                    }
                    // maxRouteIns = (int) Math.Ceiling(route.sets_included.Count / 5.0));
                    maxRouteIns = (int)Math.Ceiling(route.sets_included.Count / (m.r.NextDouble() * (10.0 - 5.0) + 5.0));
                    SIDSubproblem.AddConstr(exp1b <= maxRouteIns, "con1_max_insertions_" + route.id);
                }

                // Constraint 2: Max deletions
                GRBLinExpr exp2 = 0.0;
                for (int g = 1; g < setsNum; g++)
                {
                    Set set = m.sets[g];
                    if (existsInSol[g]) //(route.sets_included.Contains(set))
                    {
                        exp2.AddTerm(1.0, rem[set.id]);
                    }
                }
                SIDSubproblem.AddConstr(exp2 <= maxDel, "con2_max_deletions");

                // OR Constraint 2: Max deletions per route
                foreach (Route route in sol.routes)
                {
                    GRBLinExpr exp2b = 0.0;
                    for (int g = 1; g < setsNum; g++)
                    {
                        Set set = m.sets[g];
                        if (route.sets_included.Contains(set))
                        {
                            exp2b.AddTerm(1.0, rem[set.id]);
                        }
                    }
                    // maxRouteDels = (int)Math.Ceiling(route.sets_included.Count / 5.0);
                    maxRouteDels = (int)Math.Ceiling(route.sets_included.Count / (m.r.NextDouble() * (10.0 - 5.0) + 5.0));
                    SIDSubproblem.AddConstr(exp2b <= maxRouteDels, "con2_max_deletions" + route.id);
                }

                // Constraint 3: ΤMax (approximations apply)
                if (!hide_tmax_constraint)
                {
                    foreach (Route route in sol.routes)
                    {
                        GRBLinExpr exp3 = 0.0;

                        for (int g = 1; g < setsNum; g++)
                        {
                            Set set = m.sets[g];

                            if (route.sets_included.Contains(set))
                            {
                                exp3.AddTerm(remCost[set.id], rem[set.id]);
                            }
                            else
                            {
                                exp3.AddTerm(addCost[set.id, route.id], add[set.id, route.id]);
                            }
                        }
                        SIDSubproblem.AddConstr(route.time + exp3 <= m.t_max, "con3_Tmax_" + route.id);
                    }
                }

                // Constraint 4: Forbid duplicate sets
                for (int g = 1; g < setsNum; g++)
                {
                    Set set = m.sets[g];

                    GRBLinExpr exp4 = 0.0;
                    foreach (Route route in sol.routes)
                    {
                        exp4.AddTerm(1.0, add[set.id, route.id]);
                    }

                    if (existsInSol[g])
                    {
                        SIDSubproblem.AddConstr(exp4 <= rem[set.id], "con4_setDublicate_" + set.id);
                    }
                    else
                    {
                        SIDSubproblem.AddConstr(exp4 <= 1, "con4_setDublicate_" + set.id);
                    }
                }

                // Forbid for a set that is removed to be inserted again
                // Constraint 5: 
                //for (int g = 1; g < setsNum; g++)
                //{
                //    if (existsInSol[g])
                //    {
                //        Set set = m.sets[g];
                //        GRBLinExpr exp5 = 0.0;
                //        foreach (Route route in sol.routes)
                //        {
                //            exp5.AddTerm(1.0, add[set.id, route.id]);
                //        }
                //        SIDSubproblem.AddConstr(exp5 <= 1 - rem[set.id], "con5_simAddandRemoval_" + set.id);
                //    }
                //}

                // ============================================================================================================================================================//
                // Valid inequalities
                // Valid inequalities are extra not required constraints that are used to cut down the solution space
                if (addValidInequalities)
                {

                }

                // ==================================================Optimize====================================================================
                SIDSubproblem.Optimize();

                // ==================================================Results====================================================================
                switch (SIDSubproblem.Status)
                {
                    case GRB.Status.OPTIMAL:
                    case GRB.Status.TIME_LIMIT:
                        {
                            foreach (Route route in sol.routes)
                            {
                                if (!silence)
                                {
                                    //Console.WriteLine("Before insertion-deletion MIP");
                                    //Console.WriteLine(route);
                                }

                                // 1. update the nodes lists and profit
                                List<Node> oldRouteDC = route.nodes_seq.ConvertAll(node => new Node(node.id, node.x, node.y, node.set_id));
                                int oldTime = route.time;
                                int oldProf = route.total_profit;

                                // removal
                                for (int g = 1; g < setsNum; g++)
                                {
                                    Set set = m.sets[g];

                                    if (route.sets_included.Contains(set)) //existsInSol[g])
                                    {
                                        if (MathProgramming.IsEqual(rem[set.id].X, 1.0, 1e-3))
                                        {
                                            if (!silence)
                                            {
                                                Console.WriteLine("Removing set {0} from route {3} with saving of {1} and profit {2}",
                                                set.id, remCost[set.id], set.profit, route.id);
                                            }
                                            for (int idx = 0; idx < route.sets_included.Count; idx++)
                                            {
                                                Set setRem = route.sets_included[idx];
                                                if (setRem.id == set.id)
                                                {
                                                    // 1. update route
                                                    route.sets_included.RemoveAt(idx);
                                                    route.nodes_seq.RemoveAt(idx);
                                                    route.total_profit -= set.profit;
                                                    //route.time

                                                    // 2. update sol
                                                    sol.sets_included.Remove(set);
                                                    //m.sets[set.id].in_route = false;
                                                    sol.total_profit -= set.profit;
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                            foreach (Route route in sol.routes)
                            {
                                // insertions
                                for (int g = 0; g < setsNum; g++)
                                {
                                    Set set = m.sets[g];

                                    if (!route.sets_included.Contains(set))
                                    {
                                        if (MathProgramming.IsEqual(add[set.id, route.id].X, 1.0, 1e-3)) // Attention: random order
                                        {
                                            if (!silence)
                                            {
                                                Console.WriteLine("Inserting set {0} to route {3} with cost of {1} and profit {2}",
                                                    set.id, addCost[set.id, route.id], set.profit, route.id);
                                            }
                                            //Route toBeInsertedInto = addRoute[set.id];
                                            //Node ins = addNode[g];
                                            Node ins = addNode[set.id, route.id];

                                            // 1. update route
                                            InsertNodeToRoute(m, sol, route, ins, set);
                                            route.total_profit += set.profit;

                                            // 2. update sol
                                            sol.total_profit += set.profit;
                                        }
                                    }
                                }


                                // distance update until TSP runs
                                int previous_node_id = route.nodes_seq[0].id;
                                int time = 0;
                                foreach (Node node in route.nodes_seq.GetRange(1, route.nodes_seq.Count - 1))
                                {
                                    time += m.dist_matrix[previous_node_id, node.id];
                                    previous_node_id = node.id;
                                }
                                route.time = time;

                                // for random instances this may take a while. time limit in seconds and MIP gap may be added.
                                int secLimit = 30;
                                double mipGapLimit = 0.005; //0.5%
                                double heur = 0.80; //80% 

                                // TSP here can result in route.time > t_max. We set hide_errors var to true to avoid route checks
                                MathProgramming.SolveTSP(m, sol, secLimit, mipGapLimit, heur, true);

                                if (route.time > m.t_max) //infeasible due to approximation
                                {
                                    RestoreDistanceInfeasibility(m, sol, route, add, custom_tmax);
                                }

                                MathProgramming.SolveTSP(m, sol, secLimit, mipGapLimit, heur, false);

                                if (!route.CheckRoute())
                                {
                                    Console.WriteLine("Error SIDSubproblem");
                                }

                                if (!silence)
                                {
                                    //Console.WriteLine("After insertion-deletion MIP");
                                    //Console.WriteLine(route);
                                    //Console.WriteLine("Route {5}: Simultaneous Insertions/Deletions ({0},{1}) objective change: {2} (Dist: {3} --> {4})",
                                    //    maxIns,maxDel, sol.total_profit-oldProf, oldTime, route.time, route.id);
                                }
                                //MathProgramming.OptimizeNodesGivenSets(m, sol);
                            }

                            break;
                        }
                    case GRB.Status.INFEASIBLE:
                        {
                            Console.WriteLine("Model is infeasible");
                            // compute and write out IIS
                            SIDSubproblem.ComputeIIS();
                            SIDSubproblem.Write("SIDSubproblem" + SIDSubproblem.ModelName + ".ilp");
                            break;
                        }
                    case GRB.Status.UNBOUNDED:
                        {
                            Console.WriteLine("Model is unbounded");
                            break;
                        }
                    default:
                        {
                            Console.WriteLine("Optimization was stopped with status = " + SIDSubproblem.Status);
                            break;
                        }

                }
                // Dispose of model
                SIDSubproblem.Dispose();
                //gurobiEnv.Dispose();
            }
            catch (GRBException e)
            {
                Console.WriteLine("Error code: " + e.ErrorCode + ". " + e.Message);
            }
        }

        /**
          * Simultaneous up to maxIns insertions and up to maxDel deletions with routing approximations. The method works on sets. Only the best insertion position for each set is considered.
          * The best node in the best route. 
          */
        public static void SimulInsDelSubproblemBestVeh(Model m, Solution sol, int maxIns, int maxDel, double custom_tmax, bool hide_tmax_constraint = false)
        {
            // params 
            bool addValidInequalities = false;
            bool silence = true;

            // easy access
            int customersNum = m.nodes.Count - 1;
            int allNodesNum = m.nodes.Count;
            int setsNum = m.sets.Count;

            // create model
            try
            {
                GRBModel SIDSubproblem = new GRBModel(MathProgramming.gurobiEnv);

                // SOP model params
                SIDSubproblem.ModelName = "SIDSubproblem" + DateTime.Now.ToString("HH:mm:ss tt");
                SIDSubproblem.Parameters.OutputFlag = 0; // Gurobi logging
                //PPRPmodel.Parameters.MIPGap = gapLimit; // termination condition, stop when reaching X% difference between lower and upper bound 
                SIDSubproblem.Parameters.Threads = 1; // usually we use 1 thread when solving MIPs for reasons of direct comparisons
                SIDSubproblem.Parameters.TimeLimit = 1 * 60; // termination condition in seconds 

                // Preprocessing
                bool[] existsInSol = new bool[setsNum];

                for (int g = 0; g < setsNum; g++)
                {
                    Set set = m.sets[g];
                    foreach (Route route in sol.routes)
                    {
                        if (route.sets_included.Contains(set))
                        {
                            existsInSol[set.id] = true;
                            break; // it cannot be visited by more than one vehicles (suboptimal)
                        }
                    }
                }

                // auxiliary and initialization
                int[] addCost = new int[setsNum];
                Node[] addNode = new Node[setsNum];
                Route[] addRoute = new Route[setsNum];
                int[] remCost = new int[setsNum];

                for (int l = 0; l < setsNum; l++)
                {
                    Set set = m.sets[l];

                    addCost[set.id] = int.MaxValue;
                    remCost[set.id] = 0;
                    addNode[set.id] = null;
                    addRoute[set.id] = null;
                }

                // calculate the removal savings
                foreach (Route route in sol.routes)
                {
                    for (int g = 1; g < route.sets_included.Count - 1; g++) // nodes except depots
                    {
                        Set set = route.sets_included[g];
                        Set set_pred = route.sets_included[g - 1];
                        Set set_suc = route.sets_included[g + 1];
                        Node node = route.nodes_seq[g];
                        Node node_pred = route.nodes_seq[g - 1];
                        Node node_suc = route.nodes_seq[g + 1];

                        remCost[set.id] = m.dist_matrix[node_pred.id, node_suc.id] - m.dist_matrix[node_pred.id, node.id] - m.dist_matrix[node.id, node_suc.id]; // Caution: negative cost (saving)
                    }
                }

                // calculate the insertion costs
                for (int l = 0; l < setsNum; l++)
                {
                    Set set = m.sets[l];
                    if (!existsInSol[set.id])
                    {
                        foreach (Route route in sol.routes)
                        {
                            for (int g = 1; g < route.sets_included.Count; g++)
                            {
                                Set set_pred = route.sets_included[g - 1];
                                Set set_suc = route.sets_included[g];
                                Node node_pred = route.nodes_seq[g - 1];
                                Node node_suc = route.nodes_seq[g];

                                for (int i = 0; i < set.nodes.Count; i++) // check the best node if the set is to be visited
                                {
                                    Node node = set.nodes[i];
                                    if (m.dist_matrix[node_pred.id, node.id] + m.dist_matrix[node.id, node_suc.id] - m.dist_matrix[node_pred.id, node_suc.id] < addCost[set.id])
                                    {
                                        addCost[set.id] = m.dist_matrix[node_pred.id, node.id] + m.dist_matrix[node.id, node_suc.id] - m.dist_matrix[node_pred.id, node_suc.id];
                                        addNode[set.id] = node; // ref
                                        addRoute[set.id] = route; // ref
                                    }
                                }
                            }
                        }
                    }
                }

                // ============================================================================================================================================================//
                // Decision variables declaration
                GRBVar[] add = new GRBVar[setsNum];
                GRBVar[] rem = new GRBVar[setsNum];

                for (int g = 1; g < setsNum; g++)
                {
                    Set set = m.sets[g];

                    if (existsInSol[g])
                    {
                        rem[set.id] = SIDSubproblem.AddVar(0.0, 1.0, -set.profit, GRB.BINARY, "rem_" + set.id);
                        add[set.id] = SIDSubproblem.AddVar(0.0, 0.0, set.profit, GRB.BINARY, "add_" + set.id);
                    }
                    else
                    {
                        rem[set.id] = SIDSubproblem.AddVar(0.0, 0.0, -set.profit, GRB.BINARY, "rem_" + set.id);
                        add[set.id] = SIDSubproblem.AddVar(0.0, 1.0, set.profit, GRB.BINARY, "add_" + set.id);
                    }
                }

                // ============================================================================================================================================================//
                // Objective sense
                SIDSubproblem.ModelSense = GRB.MAXIMIZE;

                // ============================================================================================================================================================//
                // Constraints
                // Constraint 1: Max insertions
                GRBLinExpr exp1 = 0.0;
                for (int g = 1; g < setsNum; g++)
                {
                    if (!existsInSol[g])
                    {
                        Set set = m.sets[g];
                        exp1.AddTerm(1.0, add[set.id]);
                    }
                }
                SIDSubproblem.AddConstr(exp1 <= maxIns, "con1_max_insertions");

                // Constraint 2: Max deletions
                GRBLinExpr exp2 = 0.0;
                for (int g = 1; g < setsNum; g++)
                {
                    if (existsInSol[g])
                    {
                        Set set = m.sets[g];
                        exp2.AddTerm(1.0, rem[set.id]);
                    }
                }
                SIDSubproblem.AddConstr(exp2 <= maxDel, "con2_max_deletions");

                // Constraint 3: ΤMax (approximations apply)
                if (!hide_tmax_constraint)
                {
                    foreach (Route route in sol.routes)
                    {
                        GRBLinExpr exp3 = 0.0;

                        for (int g = 1; g < setsNum; g++)
                        {
                            Set set = m.sets[g];

                            if (route.sets_included.Contains(set))
                            {
                                exp3.AddTerm(remCost[set.id], rem[set.id]);
                            }
                            else
                            {
                                if (addRoute[set.id] != null && addRoute[set.id].id == route.id) // is to be inserted in this route
                                {
                                    exp3.AddTerm(addCost[set.id], add[set.id]);
                                }
                            }
                        }
                        SIDSubproblem.AddConstr(route.time + exp3 <= m.t_max, "con3_Tmax_" + route.id);
                    }
                }

                // ============================================================================================================================================================//
                // Valid inequalities
                // Valid inequalities are extra not required constraints that are used to cut down the solution space
                if (addValidInequalities)
                {

                }

                // ==================================================Optimize====================================================================
                SIDSubproblem.Optimize();

                // ==================================================Results====================================================================
                switch (SIDSubproblem.Status)
                {
                    case GRB.Status.OPTIMAL:
                    case GRB.Status.TIME_LIMIT:
                        {
                            foreach (Route route in sol.routes)
                            {
                                if (!silence)
                                {
                                    Console.WriteLine("Before insertion-deletion MIP");
                                    Console.WriteLine(route);
                                }

                                // 1. update the nodes lists and profit
                                List<Node> oldRouteDC = route.nodes_seq.ConvertAll(node => new Node(node.id, node.x, node.y, node.set_id));
                                int oldTime = route.time;
                                int oldProf = route.total_profit;

                                // removal
                                for (int g = 1; g < setsNum; g++)
                                {
                                    Set set = m.sets[g];

                                    if (existsInSol[g])
                                    {
                                        if (MathProgramming.IsEqual(rem[set.id].X, 1.0, 1e-3))
                                        {
                                            //Console.WriteLine("Removing set {0} with saving of {1} and profit {2}", set.id, remCost[set.id], set.profit);
                                            for (int idx = 0; idx < route.sets_included.Count; idx++)
                                            {
                                                Set setRem = route.sets_included[idx];
                                                if (setRem.id == set.id)
                                                {
                                                    route.sets_included.RemoveAt(idx);
                                                    route.nodes_seq.RemoveAt(idx);
                                                    sol.sets_included.Remove(set);
                                                    //m.sets[set.id].in_route = false;
                                                    sol.total_profit -= set.profit;
                                                    route.total_profit -= set.profit;
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                }

                                // insertions
                                for (int g = 0; g < setsNum; g++)
                                {
                                    Set set = m.sets[g];

                                    if (!existsInSol[g])
                                    {
                                        if (MathProgramming.IsEqual(add[set.id].X, 1.0, 1e-3)) // Attention: random order
                                        {
                                            //Console.WriteLine("Inserting set {0} with cost of {1} and profit {2}", set.id, addCost[set.id], set.profit);
                                            Route toBeInsertedInto = addRoute[set.id];

                                            if (toBeInsertedInto.id == route.id)
                                            {
                                                Node ins = addNode[g];
                                                InsertNodeToRoute(m, sol, route, ins, set);
                                                sol.total_profit += set.profit;
                                                route.total_profit += set.profit;
                                            }
                                        }
                                    }
                                }


                                // distance update until TSP runs
                                int previous_node_id = route.nodes_seq[0].id;
                                int time = 0;
                                foreach (Node node in route.nodes_seq.GetRange(1, route.nodes_seq.Count - 1))
                                {
                                    time += m.dist_matrix[previous_node_id, node.id];
                                    previous_node_id = node.id;
                                }
                                route.time = time;

                                // for random instances this may take a while. time limit in seconds and MIP gap may be added.
                                int secLimit = 30;
                                double mipGapLimit = 0.005; //0.5%
                                double heur = 0.80; //80% 

                                // MathProgramming.SolveTSP(m, sol, secLimit, mipGapLimit, heur);

                                if (route.time > m.t_max) //infeasible due to approximation
                                {
                                    RestoreDistanceInfeasibility(m, sol, route, add, custom_tmax);
                                }

                                MathProgramming.SolveTSP(m, sol, secLimit, mipGapLimit, heur);



                                if (!route.CheckRoute())
                                {
                                    Console.WriteLine("Error SIDSubproblem");
                                }

                                if (!silence)
                                {
                                    Console.WriteLine("After insertion-deletion MIP");
                                    Console.WriteLine(route);
                                    Console.WriteLine("Simultaneous Insertions/Deletions ({0},{1}) objective change: {2} (Dist: {3} --> {4})", maxIns, maxDel, sol.total_profit - oldProf, oldTime, route.time);
                                }
                                //MathProgramming.OptimizeNodesGivenSets(m, sol);
                            }

                            break;
                        }
                    case GRB.Status.INFEASIBLE:
                        {
                            Console.WriteLine("Model is infeasible");
                            // compute and write out IIS
                            SIDSubproblem.ComputeIIS();
                            SIDSubproblem.Write("SIDSubproblem" + SIDSubproblem.ModelName + ".ilp");
                            break;
                        }
                    case GRB.Status.UNBOUNDED:
                        {
                            Console.WriteLine("Model is unbounded");
                            break;
                        }
                    default:
                        {
                            Console.WriteLine("Optimization was stopped with status = " + SIDSubproblem.Status);
                            break;
                        }

                }
                // Dispose of model
                SIDSubproblem.Dispose();
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
        private static void InsertNodeToRoute(Model m, Solution sol, Route route, Node ins, Set set)
        {
            // find all feasible insertions for each position
            int bestIdx = -1;
            int bestCost = int.MaxValue;

            for (int i = 1; i < route.nodes_seq.Count; i++)
            {
                int cost_added = m.dist_matrix[route.nodes_seq[i - 1].id, ins.id] + m.dist_matrix[ins.id, route.nodes_seq[i].id] - m.dist_matrix[route.nodes_seq[i - 1].id, route.nodes_seq[i].id];
                if (bestCost > cost_added)
                {
                    bestCost = cost_added;
                    bestIdx = i;
                }
            }

            route.sets_included.Insert(bestIdx, set);
            route.nodes_seq.Insert(bestIdx, ins);
            sol.sets_included.Add(set);
        }

        // Heuristic method to restore the t_max infeasibility by sequentially removing the node with the best distance change/profit ratio
        public static void RestoreDistanceInfeasibility(Model m, Solution sol, Route route, GRBVar[] add, double custom_tmax)
        {
            //int[] savings = new int[m.sets.Count];
            while (route.time > custom_tmax) //infeasible due to approximation
            {
                double[] savings = new double[route.sets_included.Count];

                //Calculate savings
                for (int l = 0; l < route.sets_included.Count; l++)
                {
                    Set set = route.sets_included[l];
                    savings[l] = 0;
                }
                for (int g = 1; g < route.sets_included.Count - 1; g++) // nodes except depots
                {
                    Set set = route.sets_included[g];
                    Set set_pred = route.sets_included[g - 1];
                    Set set_suc = route.sets_included[g + 1];
                    Node node = route.nodes_seq[g];
                    Node node_pred = route.nodes_seq[g - 1];
                    Node node_suc = route.nodes_seq[g + 1];
                    savings[g] = (double)(-m.dist_matrix[node_pred.id, node_suc.id] + m.dist_matrix[node_pred.id, node.id] + m.dist_matrix[node.id, node_suc.id]) / set.profit;
                    if (MathProgramming.IsEqual(add[set.id].X, 1.0, 1e-3)) // if the node has just been added avoid removing again
                    {
                        savings[g] = 0.0001;
                    }
                }

                //Find best set to remove
                double max = savings.Max();
                int idx = Array.IndexOf(savings, max);
                Set setRem = null;
                for (int l = 0; l < route.sets_included.Count; l++)
                {
                    if (l == idx)
                    {
                        setRem = route.sets_included[l];
                        break;
                    }
                }
                int saving = 0;
                Node nodeRem = route.nodes_seq[idx];
                Node nodeRem_pred = route.nodes_seq[idx - 1];
                Node nodeRem_suc = route.nodes_seq[idx + 1];
                saving = m.dist_matrix[nodeRem_pred.id, nodeRem_suc.id] - m.dist_matrix[nodeRem_pred.id, nodeRem.id] - m.dist_matrix[nodeRem.id, nodeRem_suc.id];

                //Remove set and update
                route.sets_included.RemoveAt(idx);
                route.nodes_seq.RemoveAt(idx);
                //m.sets[setRem.id].in_route = false;
                sol.total_profit -= setRem.profit;
                route.total_profit -= setRem.profit;
                route.time += saving;
                // Console.WriteLine("Removing set {0} with saving of {1} and profit {2}", setRem.id, saving, setRem.profit);
            }

            if (!route.CheckRoute())
            {
                Console.WriteLine("Error in infeasibility repair of SIDSubproblem");
            }
        }


        public static void RestoreDistanceInfeasibility(Model m, Solution sol, Route route, GRBVar[,] add, double custom_tmax)
        {
            //int[] savings = new int[m.sets.Count];
            while (route.time > custom_tmax) //infeasible due to approximation
            {
                double[] savings = new double[route.sets_included.Count];

                //Calculate savings
                for (int l = 0; l < route.sets_included.Count; l++)
                {
                    Set set = route.sets_included[l];
                    savings[l] = 0;
                }
                for (int g = 1; g < route.sets_included.Count - 1; g++) // nodes except depots
                {
                    Set set = route.sets_included[g];
                    Set set_pred = route.sets_included[g - 1];
                    Set set_suc = route.sets_included[g + 1];
                    Node node = route.nodes_seq[g];
                    Node node_pred = route.nodes_seq[g - 1];
                    Node node_suc = route.nodes_seq[g + 1];
                    savings[g] = (double)(-m.dist_matrix[node_pred.id, node_suc.id] + m.dist_matrix[node_pred.id, node.id]
                        + m.dist_matrix[node.id, node_suc.id]) / set.profit;
                    if (MathProgramming.IsEqual(add[set.id, route.id].X, 1.0, 1e-3)) // if the node has just been added avoid removing again
                    {
                        savings[g] = 0.0001;
                    }
                }

                //Find best set to remove
                double max = savings.Max();
                int idx = Array.IndexOf(savings, max);
                Set setRem = null;
                for (int l = 0; l < route.sets_included.Count; l++)
                {
                    if (l == idx)
                    {
                        setRem = route.sets_included[l];
                        break;
                    }
                }
                int saving = 0;
                Node nodeRem = route.nodes_seq[idx];
                Node nodeRem_pred = route.nodes_seq[idx - 1];
                Node nodeRem_suc = route.nodes_seq[idx + 1];
                saving = m.dist_matrix[nodeRem_pred.id, nodeRem_suc.id] - m.dist_matrix[nodeRem_pred.id, nodeRem.id] - m.dist_matrix[nodeRem.id, nodeRem_suc.id];

                //Remove set and update
                route.sets_included.RemoveAt(idx);
                route.nodes_seq.RemoveAt(idx);
                //m.sets[setRem.id].in_route = false;
                sol.total_profit -= setRem.profit;
                route.total_profit -= setRem.profit;
                route.time += saving;
                // Console.WriteLine("Removing set {0} with saving of {1} and profit {2}", setRem.id, saving, setRem.profit);
            }

            if (!route.CheckRoute())
            {
                Console.WriteLine("Error in infeasibility repair of SIDSubproblem");
            }
        }

        //// Needs to be revisited - not to be used
        //public static void SubOptimizeNodesGivenSetsNoOrderSubproblem(Model m, Solution sol)
        //{
        //    // it updates the given sol (do we need to create and return a deepcopy of sol?)
        //    //params
        //    bool addValidInequalities = false;

        //    // easy access
        //    int customersNum = m.nodes.Count - 1;
        //    int allNodesNum = m.nodes.Count;

        //    // create model
        //    try
        //    {
        //        GRBModel SOPmodel = new GRBModel(MathProgramming.gurobiEnv);

        //        // SOP model params
        //        SOPmodel.ModelName = "OptimizeNodesGivenSets" + DateTime.Now.ToString("HH:mm:ss tt");
        //        SOPmodel.Parameters.OutputFlag = 1; // Gurobi logging
        //        //PPRPmodel.Parameters.MIPGap = gapLimit; // termination condition, stop when reaching X% difference between lower and upper bound 
        //        SOPmodel.Parameters.Threads = 1; // usually we use 1 thread when solving MIPs for reasons of direct comparisons
        //        SOPmodel.Parameters.TimeLimit = 10 * 60; // termination condition in seconds 

        //        // ============================================================================================================================================================//
        //        // Decision variables declaration
        //        // Binary
        //        GRBVar[,] x = new GRBVar[allNodesNum, allNodesNum];
        //        GRBVar[,] u = new GRBVar[allNodesNum, allNodesNum];
        //        GRBVar[] y = new GRBVar[allNodesNum];

        //        for (int i = 0; i < allNodesNum; i++)
        //        {
        //            Node from = m.nodes[i];
        //            y[from.id] = SOPmodel.AddVar(0.0, 1.0, 0.0, GRB.BINARY, "y_" + from.id); // lower bound, upper bound, factor in objective function, type, name

        //            for (int j = 0; j < allNodesNum; j++)
        //            {
        //                if (i != j) // Speedup: i < j if we transform it to symmetric we will have half x variables 
        //                {
        //                    Node to = m.nodes[j];
        //                    x[from.id, to.id] = SOPmodel.AddVar(0.0, 1.0, m.dist_matrix[from.id, to.id], GRB.BINARY, "x_" + from.id + "," + to.id);
        //                    u[from.id, to.id] = SOPmodel.AddVar(0.0, allNodesNum, 0.0, GRB.CONTINUOUS, "u_" + from.id + "," + to.id);
        //                }
        //            }
        //        }

        //        // possible speedup: branching priority (we wish to branch on y_i first - to be tested)
        //        for (int i = 0; i < allNodesNum; i++)
        //        {
        //            y[i].BranchPriority = 2;
        //        }
        //        // ============================================================================================================================================================//
        //        // Objective sense
        //        SOPmodel.ModelSense = GRB.MINIMIZE;

        //        // ============================================================================================================================================================//
        //        // Constraints
        //        // Reference [1] C. Archetti, F. Carrabs, and R. Cerulli, “The Set Orienteering Problem,” Eur. J. Oper. Res., vol. 267, no. 1, pp. 264–272, 2018.  
        //        // Constraint 1 (con 2 of [1]): the sum of outgoing edges of node i to every node j equals the number of visits to y_i (0/1)
        //        for (int i = 0; i < allNodesNum; i++) //Speed up: this could be cut down by nodes in the given sets only
        //        {
        //            Node from = m.nodes[i];

        //            GRBLinExpr exp = 0.0; //Initialize a linear expression
        //            for (int j = 0; j < allNodesNum; j++) //Speed up: this could be cut down by nodes in the given sets only
        //            {
        //                if (i != j) //speedup symmetric vars
        //                {
        //                    Node to = m.nodes[j];
        //                    exp.AddTerm(1.0, x[from.id, to.id]);
        //                }
        //            }
        //            SOPmodel.AddConstr(exp == y[from.id], "con1_" + from.id); //add constraint to model 
        //        }

        //        // Constraint 2 (con 3 of [1]): the sum of ingoing edges to node i from every node j equals the number of visits to y_i (0/1)
        //        for (int i = 0; i < allNodesNum; i++) //Speed up: this could be cut down by nodes in the given sets only
        //        {
        //            Node to = m.nodes[i];

        //            GRBLinExpr exp = 0.0; //Initialize a linear expression
        //            for (int j = 0; j < allNodesNum; j++) //Speed up: this could be cut down by nodes in the given sets only
        //            {
        //                if (i != j) //speedup symmetric vars
        //                {
        //                    Node from = m.nodes[j];
        //                    exp.AddTerm(1.0, x[from.id, to.id]);
        //                }
        //            }
        //            SOPmodel.AddConstr(exp == y[to.id], "con2_" + to.id); //add constraint to model 
        //        }

        //        /*
        //        //Speedup: constraints 1 and 2 may be summed up to one
        //        for (int i = 0; i < allNodesNum; i++) //Speed up: this could be cut down by nodes in the given sets only
        //        {
        //            Node from = m.nodes[i];

        //            GRBLinExpr exp = 0.0; //Initialize a linear expression
        //            for (int j = 0; j < allNodesNum; j++) //Speed up: this could be cut down by nodes in the given sets only
        //            {
        //                if (i != j) //speedup symmetric vars
        //                {
        //                    Node to = m.nodes[j];
        //                    exp.AddTerm(1.0, x[to.id, from.id]);
        //                    exp.AddTerm(1.0, x[from.id, to.id]);
        //                }
        //            }
        //            SOPmodel.AddConstr(exp == 2*y[from.id], "con1_" + from.id); //add constraint to model 
        //        }
        //        */

        //        // Constraint 3 (similar to con 6 of [1]): exactly one node of every given (from sol) set must be selected
        //        for (int g = 0; g < sol.route.sets_included.Count; g++) //Speed up: this could be cut down by nodes in the given sets only
        //        {
        //            Set set = sol.route.sets_included[g];

        //            GRBLinExpr exp = 0.0; //Initialize a linear expression
        //            for (int i = 0; i < set.nodes.Count; i++) //Speed up: this could be cut down by nodes in the given sets only
        //            {
        //                Node node = set.nodes[i];
        //                exp.AddTerm(1.0, y[node.id]);
        //            }
        //            SOPmodel.AddConstr(exp == 1, "con3_" + set.id); //add constraint to model 
        //        }

        //        // This is redundant if sol is always feasible.
        //        // Constraint 4 (con 5 of [1]): tour length 
        //        /*
        //        GRBLinExpr exp2 = 0.0;
        //        for (int i = 0; i < allNodesNum; i++) //Speed up: this could be cut down by nodes in the given sets only
        //        {
        //            Node from = m.nodes[i];

        //            for (int j = 0; j < allNodesNum; j++) //Speed up: this could be cut down by nodes in the given sets only
        //            {
        //                if (i != j) //speedup symmetric vars
        //                {
        //                    Node to = m.nodes[j];
        //                    exp2.AddTerm(m.dist_matrix[from.id,to.id], x[i,j]);
        //                }
        //            }
        //        }
        //        SOPmodel.AddConstr(exp2 <= m.t_max, "con4_Tmax"); //add constraint to model 
        //        */

        //        // Constraint 5 (con 10 of [1]): MTZ subtour elimination
        //        for (int i = 1; i < allNodesNum; i++) //exclude depot
        //        {
        //            Node from = m.nodes[i];

        //            GRBLinExpr exp = 0.0;
        //            for (int j = 0; j < allNodesNum; j++)
        //            {
        //                if (i != j) //speedup symmetric vars
        //                {
        //                    Node to = m.nodes[j];
        //                    exp.AddTerm(1.0, u[from.id, to.id]);
        //                    exp.AddTerm(-1.0, u[to.id, from.id]);
        //                }
        //            }
        //            SOPmodel.AddConstr(exp == y[from.id], "con5_flow_" + from.id);
        //        }

        //        // Constraint 6 (con 11 of [1]): MTZ subtour elimination
        //        for (int i = 1; i < allNodesNum; i++) //exclude depot
        //        {
        //            Node from = m.nodes[i];
        //            for (int j = 0; j < allNodesNum; j++)
        //            {

        //                if (i != j) //speedup symmetric vars
        //                {
        //                    Node to = m.nodes[j];
        //                    SOPmodel.AddConstr(u[from.id, to.id] <= (allNodesNum - 1) * x[from.id, to.id], "con6_flow_" + from.id);
        //                }
        //            }
        //        }

        //        // ============================================================================================================================================================//
        //        // Valid inequalities
        //        // Valid inequalities are extra not required constraints that are used to cut down the solution space
        //        if (addValidInequalities)
        //        {

        //        }

        //        // ==================================================Optimize====================================================================
        //        SOPmodel.Optimize();

        //        // ==================================================Results====================================================================
        //        switch (SOPmodel.Status)
        //        {
        //            case GRB.Status.OPTIMAL:
        //            case GRB.Status.TIME_LIMIT:
        //                {
        //                    // 1. update route time (profits are the same)
        //                    sol.total_time = (int) Math.Round(SOPmodel.ObjVal);
        //                    sol.route.time = (int) Math.Round(SOPmodel.ObjVal);

        //                    // 2. update the nodes lists
        //                    List<Node> oldRouteDC = sol.route.nodes_seq.ConvertAll(node => new Node(node.id, node.x, node.y, node.set_id));
        //                    //List<Node> optimizedNodes = sol.route.nodes_seq;

        //                    sol.route.nodes_seq.Clear();

        //                    // ATTENTION: assumption that the sets included list is always in order of visit!
        //                    for (int g = 0; g < sol.route.sets_included.Count; g++)
        //                    {
        //                        Set set = sol.route.sets_included[g];
        //                        for (int i = 0; i < set.nodes.Count; i++) //Speed up: this could be cut down by nodes in the given sets only
        //                        {
        //                            Node node = set.nodes[i];
        //                            if (MathProgramming.IsEqual(y[node.id].X, 1.0, 1e-3)) // .X accesses the value give to the variable
        //                            {
        //                                sol.route.nodes_seq.Add(node);
        //                                break; //save time from checking the rest
        //                            }
        //                        }
        //                    }
        //                    /*
        //                    for (int i = 0; i < allNodesNum; i++)
        //                    {
        //                        Node from = m.nodes[i];
        //                        if (MathProgramming.IsEqual(y[from.id].X, 1.0, 1e-3))
        //                        {
        //                            Console.WriteLine("y_{0}=1", from.id);
        //                        }
        //                    }
        //                    for (int i = 0; i < allNodesNum; i++)
        //                    {
        //                        Node from = m.nodes[i];
        //                        for (int j = 0; j < allNodesNum; j++)
        //                        {
        //                            if (i != j) // Speedup: i < j if we transform it to symmetric we will have half x variables 
        //                            {
        //                                Node to = m.nodes[j];
        //                                if (MathProgramming.IsEqual(x[from.id, to.id].X, 1.0, 1e-3))
        //                                {
        //                                    Console.WriteLine("x_{0},{1}=1", from.id, to.id);
        //                                    //Console.WriteLine("u_{0},{1}={2}", from.id, to.id,u[from.id, to.id].X);

        //                                }
        //                            }
        //                        }
        //                    }


        //                    for (int i = 0; i < allNodesNum; i++)
        //                    {
        //                        Node from = m.nodes[i];
        //                        for (int j = 0; j < allNodesNum; j++)
        //                        {
        //                            if (i != j) // Speedup: i < j if we transform it to symmetric we will have half x variables 
        //                            {
        //                                Node to = m.nodes[j];
        //                                if (MathProgramming.IsEqual(x[from.id, to.id].X, 1.0, 1e-3))
        //                                {
        //                                    Console.WriteLine("u_{0},{1}={2}", from.id, to.id, u[from.id, to.id].X);
        //                                }
        //                            }
        //                        }
        //                    }
        //                    */
        //                    /*
        //                    // Count differences - To be commented out
        //                    int count = 0;
        //                    Console.WriteLine();
        //                    for (int i = 0; i < oldRouteDC.Count; i++)
        //                    {
        //                        Node oldNode = oldRouteDC[i];
        //                        Node newNode = sol.route.nodes_seq[i];
        //                        if (oldNode.id == newNode.id)
        //                        {
        //                            count++;
        //                            Console.Write("1");
        //                        }
        //                        else
        //                        {
        //                            Console.Write("0");
        //                        }
        //                    }
        //                    Console.WriteLine();

        //                    Console.WriteLine("The IP changed " + count + " nodes given the sets of the solution");
        //                    */

        //                    // update sol and run tests (can be removed for runs)
        //                    if (!sol.route.CheckRoute(m))
        //                    {
        //                        Console.WriteLine("Error");
        //                    }

        //                    break;
        //                }
        //            case GRB.Status.INFEASIBLE:
        //                {
        //                    Console.WriteLine("Model is infeasible");
        //                    // compute and write out IIS
        //                    SOPmodel.ComputeIIS();
        //                    SOPmodel.Write("OptimizeNodesGivenSets_" + SOPmodel.ModelName + ".ilp");
        //                    break;
        //                }
        //            case GRB.Status.UNBOUNDED:
        //                {
        //                    Console.WriteLine("Model is unbounded");
        //                    break;
        //                }
        //            default:
        //                {
        //                    Console.WriteLine("Optimization was stopped with status = " + SOPmodel.Status);
        //                    break;
        //                }

        //        }
        //        // Dispose of model
        //        SOPmodel.Dispose();
        //        //gurobiEnv.Dispose();
        //    }
        //    catch (GRBException e)
        //    {
        //        Console.WriteLine("Error code: " + e.ErrorCode + ". " + e.Message);
        //    }
        //}
    }
}
