using Gurobi;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace F_CVRP.Gurobi
{
    public class MIP
    {

        public void SolveWithMathModel(Model m, Solution sol)
        {
            try
            {
                // Create the environment and model
                GRBEnv env = new GRBEnv(true);
                env.Set("LogFile", "mip1.log");
                env.Start();
                GRBModel model = new GRBModel(env);

                int n = m.numNodes + 1;
                int k = m.vehicles;

                // Decision variables
                GRBVar[,,] x = new GRBVar[k, n, n];
                GRBVar[] y = new GRBVar[n];
                GRBVar[] u = new GRBVar[n]; // For MTZ subtour elimination


                for (int v = 0; v < k; v++)
                {
                    for (int i = 0; i < n; i++)
                    {
                        y[i] = model.AddVar(0.0, 1.0, 0.0, GRB.BINARY, $"y_{i}");
                        for (int j = 0; j < n; j++)
                        {
                            x[v, i, j] = model.AddVar(0, 1, m.costMatrix[i][j], GRB.BINARY, $"x_{v}_{i}_{j}");
                        }
                    }
                }


                for (int i = 0; i < n; i++)
                {
                    
                    if (i > 0)
                    {
                        u[i] = model.AddVar(0, n, 0, GRB.CONTINUOUS, $"u_{i}");
                    }
                }
                u[0] = model.AddVar(0, 0, 0, GRB.CONTINUOUS, "u_0");


                

                // Objective: Minimize total cost
                GRBLinExpr objective = new GRBLinExpr();
                for (int v = 0; v < k; v++)
                {
                    for (int i = 0; i < n; i++)
                    {
                        for (int j = 0; j < n; j++)
                        {
                            objective.AddTerm(m.costMatrix[i][j], x[v, i, j]);
                        }
                    }
                }
                model.SetObjective(objective, GRB.MINIMIZE);



                // Constraints

                // 1. Exactly k vehicles leave and return to the depot
                for (int v = 0; v < k; v++)
                {
                    GRBLinExpr outgoingDepot = new GRBLinExpr();
                    GRBLinExpr incomingDepot = new GRBLinExpr();
                    for (int j = 1; j < n; j++)
                    {
                        outgoingDepot.AddTerm(1.0, x[v, 0, j]);
                        incomingDepot.AddTerm(1.0, x[v, j, 0]);
                    }
                    model.AddConstr(outgoingDepot == 1, $"depot_outgoing_{v}");
                    model.AddConstr(incomingDepot == 1, $"depot_incoming_{v}");
                }



                // Customer uniqueness constraint. Each customer is visited at most once.

                for (int j = 1; j < n; j++)  // Excluding depot (assuming depot is indexed at 0)
                {
                    GRBLinExpr visitConstraint = new GRBLinExpr();
                    for (int v = 0; v < k; v++)  // Iterate over vehicles
                    {
                        for (int i = 0; i < n; i++)
                        {
                            if (i != j)  // Exclude self-references
                            {
                                visitConstraint.AddTerm(1.0, x[v, i, j]);
                            }
                        }
                    }
                    model.AddConstr(visitConstraint <= 1, $"visit_constraint_{j}");
                }




                // 2. Flow continuity: if a vehicle visits a customer, it must leave
                for (int v = 0; v < k; v++)
                {
                    for (int j = 1; j < n; j++)
                    {
                        GRBLinExpr inflow = new GRBLinExpr();
                        GRBLinExpr outflow = new GRBLinExpr();
                        for (int i = 0; i < n; i++)
                        {
                            if (i != j)
                            {
                                inflow.AddTerm(1.0, x[v, i, j]);
                                outflow.AddTerm(1.0, x[v, j, i]);
                            }
                        }
                        model.AddConstr(inflow == outflow, $"flow_balance_{v}_{j}");
                    }
                }

                // 3. Vehicle capacity constraint

                for (int v = 0; v < k; v++)
                {
                    GRBLinExpr dem = new GRBLinExpr();
                    for (int i = 1; i < n; i++)
                    {
                        for (int j = 0; j < n; j++)
                        {
                            dem.AddTerm(m.nodes[i].demand, x[v, i, j]);
                        }
                    }
                    model.AddConstr(dem <= m.capacity, $"vehicle_capacity_{v}");
                }



                // 4. Subtour elimination using MTZ constraints

                for (int v = 0; v < k; v++)
                {
                    for (int i = 1; i < n; i++)
                    {
                        for (int j = 1; j < n; j++)
                        {
                            if (i != j)
                            {
                                model.AddConstr(u[j] >= u[i] + 1 - n * (1 - x[v, i, j]), $"subtour_{i}_{j}");
                            }
                        }
                    }
                }


                // 5. Family visits constraint

                for (int l = 0; l < m.numFam; l++)
                {
                    GRBLinExpr visit = new GRBLinExpr();
                    for (int v = 0; v < k; v++)
                    {
                        for (int i = 1; i < n; i++)
                        {
                            for (int j = 0; j < n; j++)
                            {
                                if (m.nodes[i].family.id == l)
                                {
                                    visit.AddTerm(1.0, x[v, i, j]);
                                }

                            }
                        }
                    }
                    model.AddConstr(visit == m.families[l].required, $"family_requirements_{l}");
                }


                //for (int l = 0; l < m.numFam; l++)
                //{
                //    GRBLinExpr visit = new GRBLinExpr();
                //    for (int i = 1; i < n; i++)
                //    {
                //        if (m.nodes[i].family.id == l)
                //        {
                //            visit.AddTerm(1.0, y[i]);
                //        }
                //    }
                //    model.AddConstr(visit == m.families[l].required, $"family_requirements_{l}");
                //}

                // Optimize the model
                model.Optimize();
                bool silence = false;

                // Check and display the solution
                if (model.Status == GRB.Status.OPTIMAL)
                {
                    Console.WriteLine("Optimal solution found:");

                    Console.WriteLine("Selected Routes:");
                    for (int v = 0; v < k; v++)
                    {
                        for (int i = 0; i < n; i++)
                        {
                            for (int j = 0; j < n; j++)
                            {
                                if (x[v, i, j].X > 0.5)
                                {
                                    Console.WriteLine($"Vehicle {v} travels from {i} to {j}");
                                }
                            }
                        }
                    }
                    Console.WriteLine($"Total Cost: {model.ObjVal}");
                }
                else
                {
                    //Console.WriteLine("No optimal solution found.");
                }

                // Dispose of model and environment
                model.Dispose();
                env.Dispose();
            }

            
            catch (GRBException e)
            {
                Console.WriteLine("Error code: " + e.ErrorCode + ". " + e.Message);
            }
        }


        public void SimulInsDel(Model m, Solution sol, int maxRem)
        {
            int n = m.numNodes + 1;
            int k = m.vehicles;

            //Solution newSol = sol.DeepCopy();

            try
            {
                GRBEnv env = new GRBEnv(true);
                env.Set("OutputFlag", "0");
                //env.Set("LogFile", "mip2.log");
                env.Start();
                GRBModel model = new GRBModel(env);

                //SetupFCVRPModel(RISubproblem, m);

                // Model params

                //model.ModelName = "SimulInsDel" + DateTime.Now.ToString("HH:mm:ss tt");
                model.Parameters.OutputFlag = 0; // Gurobi logging
                model.Parameters.Threads = 1; // usually we use 1 thread when solving MIPs for reasons of direct comparisons
                model.Parameters.TimeLimit = 1; // termination condition in seconds

                // Preprocessing

                bool[] existsInSol = new bool[n];
                existsInSol[0] = true;

                foreach (Route rt in sol.routes)
                {
                    for (int i = 1; i < rt.sequenceOfNodes.Count - 1; i++)
                    {
                        existsInSol[rt.sequenceOfNodes[i].id] = true;
                    }
                }

                GRBVar[,] sub = new GRBVar[n, n];

                float[,] subCost = new float[n, n];


                foreach (Route rt in sol.routes)
                {
                    for (int i = 1; i < rt.sequenceOfNodes.Count - 1; i++)
                    {
                        int prev = rt.sequenceOfNodes[i - 1].id;
                        int id = rt.sequenceOfNodes[i].id;
                        int next = rt.sequenceOfNodes[i + 1].id;

                        float removeCost = (float)((m.costMatrix[prev][id] + m.costMatrix[id][next]) / (1 + 0.1 * rt.sequenceOfNodes[i].timesRemoved));

                        //float beta = 0.5f;  // Scaling factor for logarithmic increase
                        //float removeCost = (float)((m.costMatrix[prev][id] + m.costMatrix[id][next]) * Math.Log(1 + beta * rt.sequenceOfNodes[i].timesRemoved));


                        for (int j = 1; j < n-1; j++)
                        {                           

                            if (existsInSol[j])
                            {
                                sub[id, j] = model.AddVar(0.0, 0.0, 0.0, GRB.BINARY, $"sub_{id}_{j}");
                            }
                            else
                            {
                                float addCost = m.costMatrix[prev][j] + m.costMatrix[j][next];
                                subCost[id, j] = addCost - removeCost;
                                //float subCost = addCost - removeCost;

                                sub[id, j] = model.AddVar(0.0, 1.0, subCost[id,j], GRB.BINARY, $"sub_{id}_{j}");
                            }
                        }
                    }
                }



                // Constraint: Maintain the capacity constraint after substitution

                for (int v = 0; v < k; v++)  
                {
                    GRBLinExpr demandChange = new GRBLinExpr();

                    foreach (Route rt in sol.routes)
                    {
                        for (int i = 1; i < rt.sequenceOfNodes.Count - 1; i++)
                        {
                            int id = rt.sequenceOfNodes[i].id;

                            for (int j = 1; j < n-1; j++)  
                            {
                                if (!existsInSol[j])  
                                {
                                    demandChange.AddTerm(m.nodes[j].demand, sub[id, j]); 
                                    demandChange.AddTerm(-m.nodes[id].demand, sub[id, j]); 
                                }
                            }
                        }

                        model.AddConstr(rt.load + demandChange <= m.capacity, $"vehicle_capacity_after_substitution_{v}");

                    }

                }


                // Constraint: Maintain equal family distributions

                for (int l = 0; l < m.numFam; l++)
                {
                    GRBLinExpr removedFromFamily = new GRBLinExpr();
                    GRBLinExpr insertedIntoFamily = new GRBLinExpr();

                    for (int i = 1; i < n-1; i++)  // Iterate over existing solution customers
                    {
                        for (int j = 1; j < n-1; j++)  // Iterate over potential insertions
                        {
                            if (m.nodes[i].family.id == l && existsInSol[i])
                            {
                                removedFromFamily.AddTerm(1.0, sub[i, j]);  // i belongs to family l and is being removed
                            }
                            if (m.nodes[j].family.id == l && !existsInSol[j] && existsInSol[i])
                            {
                                insertedIntoFamily.AddTerm(1.0, sub[i, j]);  // j belongs to family l and is being inserted
                            }
                        }
                    }

                    model.AddConstr(removedFromFamily == insertedIntoFamily, $"family_distribution_{l}");
                }



                // Constraint: Set substitution range

                GRBLinExpr removeExpr = new GRBLinExpr();
                for (int i = 1; i < n-1; i++)
                {
                    for (int j = 1; j < n-1; j++)
                    {

                        if (existsInSol[i])
                        {
                            removeExpr.AddTerm(1.0, sub[i, j]);
                        }

                    }

                }
                model.AddConstr(removeExpr <= maxRem, "max_removals");
                model.AddConstr(removeExpr >= (int)maxRem * 0.75, "min_removals");



                // Constaint: Each customer may be removed at most once

                for (int i = 1; i < n-1; i++)
                {
                    GRBLinExpr removedSum = new GRBLinExpr();
                    for (int j = 1; j < n-1; j++)
                    {
                        if (existsInSol[i] && !existsInSol[j])
                        {
                            removedSum.AddTerm(1.0, sub[i, j]);
                        }
                    }
                    model.AddConstr(removedSum <= 1, $"removed_once_{i}");
                }

                // Constaint: Each customer may be added at most once

                for (int j = 1; j < n-1; j++)
                {
                    GRBLinExpr addedSum = new GRBLinExpr();
                    for (int i = 1; i < n-1; i++)
                    {
                        if (existsInSol[i] && !existsInSol[j])
                        {
                            addedSum.AddTerm(1.0, sub[i, j]);
                        }
                    }
                    model.AddConstr(addedSum <= 1, $"added_once_{j}");
                }


                model.ModelSense = GRB.MINIMIZE;


                model.Optimize();
                bool silence = true;

                // Check and display the solution
                if (model.Status == GRB.Status.OPTIMAL || model.Status == GRB.Status.TIME_LIMIT || model.Status == GRB.Status.SUBOPTIMAL)
                {

                    List<(int, int)> substitutions = new List<(int, int)>();

                    for (int i = 1; i < n-1; i++) 
                    {
                        for (int j = 1; j < n-1; j++)
                        {
                            if (existsInSol[i] && !existsInSol[j] && sub[i,j].X > 0.5)
                            {
                                //Console.WriteLine($"Substitute {i} with {j}");

                                substitutions.Add((i,j));
                            }
                        }
                        
                    }

                    MakeSubstitutions(sol, m, substitutions);
                }
                else
                {
                    //Console.WriteLine("No optimal solution found.");
                }


                model.Dispose();
                env.Dispose();

                
            }
            catch (Exception e)
            {
                Console.WriteLine(e.Message);
            }
        }

        public void MakeSubstitutions(Solution sol, Model m, List<(int,int)> subIds)
        {
            foreach (Route rt in sol.routes)
            {
                for (int i = 1; i < rt.sequenceOfNodes.Count - 1; i++)
                {
                    Node node = rt.sequenceOfNodes[i];

                    var pair = subIds.FirstOrDefault(p => p.Item1 == node.id);

                    if (pair != default)
                    {
                        rt.sequenceOfNodes.Remove(node);
                        node.isRouted = false;
                        node.timesRemoved++;
                        rt.sequenceOfNodes.Insert(i, m.nodes[pair.Item2]);
                        m.nodes[pair.Item2].isRouted = true;
                    }
                }
                rt.CalculateRouteCostDemand();
            }
            sol.UpdateCost();

            m.CheckSolutionFeasibility(sol);

            //Console.WriteLine($"New solution cost: {sol.cost}");
            //return sol;
        }


        public void RestorePartialSol(Model m, Solution partialSol)
        {
            int n = m.numNodes + 1;
            int k = m.vehicles;

            //Solution newSol = sol.DeepCopy();

            try
            {
                GRBEnv env = new GRBEnv(true);
                env.Set("OutputFlag", "0");
                //env.Set("LogFile", "mip2.log");
                env.Start();
                GRBModel model = new GRBModel(env);

                //SetupFCVRPModel(RISubproblem, m);

                // Model params

                //model.ModelName = "SimulInsDel" + DateTime.Now.ToString("HH:mm:ss tt");
                model.Parameters.OutputFlag = 0; // Gurobi logging
                model.Parameters.Threads = 1; // usually we use 1 thread when solving MIPs for reasons of direct comparisons
                model.Parameters.TimeLimit = 1; // termination condition in seconds

                ConstructiveHeuristic heuristic = new ConstructiveHeuristic(m);

                // Preprocessing

                bool[] existsInSol = new bool[n];
                existsInSol[0] = true;

                foreach (Route rt in partialSol.routes)
                {
                    for (int i = 1; i < rt.sequenceOfNodes.Count - 1; i++)
                    {
                        existsInSol[rt.sequenceOfNodes[i].id] = true;
                    }
                }


                int[] includedMembers = new int[m.numFam];
                foreach (Route rt in partialSol.routes)
                {
                    for (int i = 1; i < rt.sequenceOfNodes.Count - 1; i++)
                    {
                        includedMembers[rt.sequenceOfNodes[i].family.id]++;
                    }
                }


                int[] remainingMembers = new int[m.numFam];
                for (int i = 0; i < m.numFam; i++)
                {
                    remainingMembers[i] = m.famReq[i] - includedMembers[i];
                }



                // Decision Variable

                GRBVar[,] add = new GRBVar[n, k];

                float[,] addCost = new float[n, k];

                // Sum Cost

                //for (int v = 0; v < k; v++)
                //{

                //    Route rt = partialSol.routes[v];

                //    for (int i = 1; i < rt.sequenceOfNodes.Count; i++)
                //    {
                //        int prev = rt.sequenceOfNodes[i - 1].id;
                //        int current = rt.sequenceOfNodes[i].id;


                //        for (int j = 1; j < n; j++)
                //        {

                //            if (existsInSol[j])
                //            {
                //                add[j, v] = model.AddVar(0.0, 0.0, 0.0, GRB.BINARY, $"add_{j}_{v}");
                //            }
                //            else
                //            {
                //                addCost[j, v] = m.costMatrix[prev][j] + m.costMatrix[j][current];
                //                //addCost[j, v] = (float)((m.costMatrix[prev][j] + m.costMatrix[j][current]) * (1+0.1*m.nodes[j].timesInserted));

                //                add[j, v] = model.AddVar(0.0, 1.0, addCost[j, v], GRB.BINARY, $"add_{j}_{v}");
                //            }
                //        }
                //    }
                //}
                
                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                
                // Min Cost

                for (int i = 1; i < n-1; i++)
                {
                    for (int v = 0; v < k; v++)
                    {

                        Route rt = partialSol.routes[v];
                        float minCost = 1000000;

                        if (existsInSol[i])
                        {
                            add[i, v] = model.AddVar(0.0, 0.0, 0.0, GRB.BINARY, $"add_{i}_{v}");
                            break;
                        }
                        else
                        {
                            (int pos, float cost) = heuristic.FindBestInsertion(rt, m.nodes[i]);

                            add[i, v] = model.AddVar(0.0, 1.0, cost, GRB.BINARY, $"add_{i}_{v}");

                        }
                    }
                }



                // Constaint: Each customer may be added at most once

                for (int i = 1; i < n-1; i++)
                {
                    if (!existsInSol[i])
                    {
                        GRBLinExpr addedSum = new GRBLinExpr();
                        for (int v = 0; v < k; v++)
                        {

                            addedSum.AddTerm(1.0, add[i, v]);

                        }
                        model.AddConstr(addedSum <= 1, $"added_once_{i}");
                    }
                }

                // Constraint: Maintain the capacity constraint after addition

                for (int v = 0; v < k; v++)
                {
                    GRBLinExpr demandChange = new GRBLinExpr();
                    Route rt = partialSol.routes[v];

                    for (int i = 1; i < n-1; i++)
                    {
                        if (!existsInSol[i])
                        {
                            demandChange.AddTerm(m.nodes[i].demand, add[i, v]);
                        }
                    }


                    model.AddConstr(rt.load + demandChange <= m.capacity, $"vehicle_capacity_after_substitution_{v}");

                }


                // Constraint: Add customers equal to the missing required number by each family

                for (int l = 0; l < m.numFam; l++)
                {

                    GRBLinExpr insertedIntoFamily = new GRBLinExpr();

                    for (int i = 1; i < n-1; i++)
                    {
                        if (!existsInSol[i] && m.nodes[i].family.id == l)
                        {
                            for (int v = 0; v < k; v++)
                            {
                                insertedIntoFamily.AddTerm(1.0, add[i, v]);
                            }
                        }

                    }

                    model.AddConstr(insertedIntoFamily == remainingMembers[l], $"family_requirements_{l}");
                }



                



                // Objective: Minimize the total substitution cost
                //GRBLinExpr substitutionCost = new GRBLinExpr();

                //for (int i = 1; i < n; i++)  
                //{
                //    for (int j = 1; j < n; j++)  
                //    {
                //        if (existsInSol[i] && !existsInSol[j])  
                //        {
                //            substitutionCost.AddTerm(subCost[i, j], sub[i, j]);  
                //        }
                //    }
                //}

                //model.SetObjective(substitutionCost, GRB.MINIMIZE);


                // Objective: Minimize addition cost
                model.ModelSense = GRB.MINIMIZE;


                model.Optimize();
                bool silence = true;

                if (model.Status == GRB.Status.OPTIMAL || model.Status == GRB.Status.TIME_LIMIT || model.Status == GRB.Status.SUBOPTIMAL)
                {
                    
                    for (int v = 0; v < k; v++)
                    {
                        for (int i = 1; i < n-1; i++)
                        {
                            if (!existsInSol[i] && add[i,v].X > 0.5)
                            {
                                //Console.WriteLine($"Add node {i} to route {v}");
                                (int pos, float cost) = heuristic.FindBestInsertion(partialSol.routes[v], m.nodes[i]);
                                partialSol.routes[v].sequenceOfNodes.Insert(pos, m.nodes[i]);
                                m.nodes[i].isRouted = true;
                                m.nodes[i].timesInserted++;
                            }
                        }
                    }

                    if (!m.CheckSolutionFeasibility(partialSol))
                    {
                        Console.WriteLine("MIP fail");
                    }

                    foreach(Route rt in partialSol.routes)
                    {
                        rt.CalculateRouteCostDemand();
                    }
                    partialSol.UpdateCost();

                }
                else
                {
                    //Console.WriteLine("No optimal solution found.");
                }


                model.Dispose();
                env.Dispose();


            }
            catch (Exception e)
            {
                Console.WriteLine(e.Message);
            }
            
        }


        public void SetupFCVRPModel(GRBModel model, Model m)
        {
            int n = m.numNodes + 1;
            int k = m.vehicles;

            // Decision variables
            GRBVar[,,] x = new GRBVar[k, n, n];
            GRBVar[] y = new GRBVar[n];
            GRBVar[] u = new GRBVar[n]; // For MTZ subtour elimination


            for (int v = 0; v < k; v++)
            {
                for (int i = 0; i < n; i++)
                {
                    y[i] = model.AddVar(0.0, 1.0, 0.0, GRB.BINARY, $"y_{i}");
                    for (int j = 0; j < n; j++)
                    {
                        x[v, i, j] = model.AddVar(0, 1, m.costMatrix[i][j], GRB.BINARY, $"x_{v}_{i}_{j}");
                    }
                }
            }


            for (int i = 0; i < n; i++)
            {

                if (i > 0)
                {
                    u[i] = model.AddVar(0, n, 0, GRB.CONTINUOUS, $"u_{i}");
                }
            }
            u[0] = model.AddVar(0, 0, 0, GRB.CONTINUOUS, "u_0");




            // Objective: Minimize total cost
            GRBLinExpr objective = new GRBLinExpr();
            for (int v = 0; v < k; v++)
            {
                for (int i = 0; i < n; i++)
                {
                    for (int j = 0; j < n; j++)
                    {
                        objective.AddTerm(m.costMatrix[i][j], x[v, i, j]);
                    }
                }
            }
            model.SetObjective(objective, GRB.MINIMIZE);



            // Constraints

            // 1. Exactly k vehicles leave and return to the depot
            for (int v = 0; v < k; v++)
            {
                GRBLinExpr outgoingDepot = new GRBLinExpr();
                GRBLinExpr incomingDepot = new GRBLinExpr();
                for (int j = 1; j < n; j++)
                {
                    outgoingDepot.AddTerm(1.0, x[v, 0, j]);
                    incomingDepot.AddTerm(1.0, x[v, j, 0]);
                }
                model.AddConstr(outgoingDepot == 1, $"depot_outgoing_{v}");
                model.AddConstr(incomingDepot == 1, $"depot_incoming_{v}");
            }



            // Customer uniqueness constraint. Each customer is visited at most once.

            for (int j = 1; j < n; j++)  // Excluding depot (assuming depot is indexed at 0)
            {
                GRBLinExpr visitConstraint = new GRBLinExpr();
                for (int v = 0; v < k; v++)  // Iterate over vehicles
                {
                    for (int i = 0; i < n; i++)
                    {
                        if (i != j)  // Exclude self-references
                        {
                            visitConstraint.AddTerm(1.0, x[v, i, j]);
                        }
                    }
                }
                model.AddConstr(visitConstraint <= 1, $"visit_constraint_{j}");
            }




            // 2. Flow continuity: if a vehicle visits a customer, it must leave
            for (int v = 0; v < k; v++)
            {
                for (int j = 1; j < n; j++)
                {
                    GRBLinExpr inflow = new GRBLinExpr();
                    GRBLinExpr outflow = new GRBLinExpr();
                    for (int i = 0; i < n; i++)
                    {
                        if (i != j)
                        {
                            inflow.AddTerm(1.0, x[v, i, j]);
                            outflow.AddTerm(1.0, x[v, j, i]);
                        }
                    }
                    model.AddConstr(inflow == outflow, $"flow_balance_{v}_{j}");
                }
            }

            // 3. Vehicle capacity constraint

            for (int v = 0; v < k; v++)
            {
                GRBLinExpr dem = new GRBLinExpr();
                for (int i = 1; i < n; i++)
                {
                    for (int j = 0; j < n; j++)
                    {
                        dem.AddTerm(m.nodes[i].demand, x[v, i, j]);
                    }
                }
                model.AddConstr(dem <= m.capacity, $"vehicle_capacity_{v}");
            }



            // 4. Subtour elimination using MTZ constraints

            for (int v = 0; v < k; v++)
            {
                for (int i = 1; i < n; i++)
                {
                    for (int j = 1; j < n; j++)
                    {
                        if (i != j)
                        {
                            model.AddConstr(u[j] >= u[i] + 1 - n * (1 - x[v, i, j]), $"subtour_{i}_{j}");
                        }
                    }
                }
            }


            // 5. Family visits constraint

            for (int l = 0; l < m.numFam; l++)
            {
                GRBLinExpr visit = new GRBLinExpr();
                for (int v = 0; v < k; v++)
                {
                    for (int i = 1; i < n; i++)
                    {
                        for (int j = 0; j < n; j++)
                        {
                            if (m.nodes[i].family.id == l)
                            {
                                visit.AddTerm(1.0, x[v, i, j]);
                            }

                        }
                    }
                }
                model.AddConstr(visit == m.families[l].required, $"family_requirements_{l}");
            }

        }

    }
}
