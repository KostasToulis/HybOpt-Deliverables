using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Net.Http.Headers;
using System.Net.Sockets;
using System.Xml;
using Gurobi;
using System.Linq;

namespace PRP
{
    public struct ExactParameters
    {
        public string algorithm;
        public int timeLimit;
        public string periodicity;
        public bool validsInequalities;
        public bool limitedVehicles;
        public bool adjustedCyclicInventories;        
    }
    public class MathematicalProgramming
    {
        public static GRBEnv gurobiEnv = new GRBEnv();
        public static Stopwatch stopwatch = new Stopwatch();
        public static int threadLimit;
        public static Random rand = new Random();
        public static ExactParameters exactParams;

        //==================================================== MIPs ====================================================//
        public static (Solution, Dictionary<string, double>) SolveCPRP(PRP model, bool forcedIntegrality,
            GRBVar[,,] x_fix = null, GRBVar[] y_fix = null, GRBVar[,] z_fix = null, Dictionary<string, double> dictRes = null)
        {
            //params
            bool addValidInequalities = exactParams.validsInequalities;
            string periodicity = exactParams.periodicity;
            //bool limitedVehicles = exactParams.limitedVehicles;

            // easy access
            int periodsNum = model.input.horizonDays;
            int custNum = model.input.customerNum;
            int nodesNum = model.input.nodes.Count;
            int allNodesNum = model.input.nodes.Count + 1;
            int vehCap = model.input.dayVehicleCapacity;
            double productionCapacity = 1;
            var contOrINT = GRB.CONTINUOUS; // GRB.CONTINUOUS GRB.INTEGER
            if (forcedIntegrality)
            {
                contOrINT = GRB.INTEGER;
            }

            // declaration
            Dictionary<string, double> dict = new Dictionary<String, double>();
            Solution optimizedSolutionFixed = null;
            Dictionary<string, double> dictFixed = null;

            if (dictRes == null)
            {
                dict.Add("Status", -1.0);
                dict.Add("Runtime", -1.0);
                dict.Add("ObjBound", -1.0);
                dict.Add("ObjVal", -1.0);
                dict.Add("NodeCount", -1.0);
                dict.Add("NumBinVars", -1.0);
                dict.Add("NumVars", -1.0);
                dict.Add("NumConstrs", -1.0);
                dict.Add("MIPGap", -1.0);
                dict.Add("Solution validity", -1.0);
                dict.Add("routingCostFromMIP", -1.0);
                dict.Add("holdingCostFromMIP", -1.0);
                dict.Add("totalUnitProductionCostFromMIP", -1.0);
                dict.Add("setupProductionCostFromMIP", -1.0);
                dict.Add("totalObjectiveFromMIP", -1.0);

                //TODO how to get inventories
            }

            Solution optimizedSolution = new Solution(model);
            double[,] UBq = new double[nodesNum, periodsNum]; //upper bound of delivered quantities to customer
            double[] UBp = new double[periodsNum]; //upper bound of delivered quantities to customer

            // Define production capacity based on the inventories

            productionCapacity += optimizedSolution.depot.startingInventory; //zero for archetti ATTENTION
            for (int i = 0; i < optimizedSolution.customers.Count; i++)
            {
                Node cust = optimizedSolution.customers[i];
                productionCapacity += cust.stockMaximumLevel; 
                // TODO remove this 
                for (int t = 0; t < periodsNum; ++t)
                {
                    productionCapacity += cust.productRate[t];
                }
            }

            // ATTENTION: modify depot maximum stock level
            optimizedSolution.depot.stockMaximumLevel = (int) Math.Min((int)productionCapacity, optimizedSolution.depot.stockMaximumLevel);
            optimizedSolution.depot.productionCapacity = (int)Math.Min(productionCapacity, optimizedSolution.depot.productionCapacity);

            // create model
            try
            {
                //GRBEnv gurobiEnv = new GRBEnv();
                GRBModel PPRPmodel = new GRBModel(gurobiEnv);

                // Params
                PPRPmodel.ModelName = "FullMIP_" + model.instanceName;
                PPRPmodel.Parameters.OutputFlag = (GlobalUtils.suppress_output) ? 0 : 1;
                PPRPmodel.Parameters.MIPGap = 1e-8;
                PPRPmodel.Parameters.Heuristics = 0.25;
                //PPRPmodel.Parameters.IntFeasTol = 1e-8;
                //PPRPmodel.Parameters.FeasibilityTol = 1e-8;
                //PPRPmodel.Parameters.OptimalityTol = 1e-8;
                PPRPmodel.Parameters.Threads = threadLimit;
                PPRPmodel.Parameters.TimeLimit = exactParams.timeLimit * 60; //0.2*60
                PPRPmodel.Parameters.MIPFocus = 1; // try to find feasible solutions

                // Decision variables
                // Binary
                GRBVar[,,] x = new GRBVar[allNodesNum, allNodesNum, periodsNum];
                GRBVar[] y = new GRBVar[periodsNum];
                GRBVar[,] z = new GRBVar[nodesNum, periodsNum];

                // Cont
                GRBVar[,,] f = new GRBVar[allNodesNum, allNodesNum, periodsNum];
                GRBVar[] p = new GRBVar[periodsNum];
                GRBVar[,] q = new GRBVar[nodesNum, periodsNum];
                GRBVar[,] I = new GRBVar[nodesNum, periodsNum]; //no need for extra inventory day. It is implied by last period I^0==I^|T|
                //GRBVar[] I_depot = new GRBVar[periodsNum]; // depot inventory
                //GRBVar[,] I_cust = new GRBVar[custNum, periodsNum]; // customer inventory

                // calculate bounds
                for (int t = 0; t < periodsNum; ++t)
                {
                    UBq[0, t] = 0;
                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];

                        int aggrCap = cust.stockMaximumLevel + cust.productRate[t];
                        /*
                        int totalDemandUntilEnd = 0;
                        for (int tt = t; tt < periodsNum; ++tt)
                        {
                            totalDemandUntilEnd += cust.productRate[tt]; //TODO:
                        }

                        totalDemandUntilEnd += I[i, periodsNum-1]; 
                        
                        UBq[cust.uid, t] = (double)GlobalUtils.Min(aggrCap, vehCap, totalDemandUntilEnd);

                        */
                        UBq[cust.uid, t] = Math.Min(aggrCap, vehCap);
                    }
                }

                for (int t = 0; t < periodsNum; ++t)
                {
                    /*
                     double totalToBeProdQuantity = 0.0;

                     for (int i = 0; i < optimizedSolution.customers.Count; i++)
                     {
                         Node cust = optimizedSolution.customers[i];
                         for (int tt = t; tt < periodsNum; ++tt)
                         {
                             totalToBeProdQuantity += cust.productRate[tt]; //TODO: careful for error
                         }
                         totalToBeProdQuantity += cust.startingInventory;
                     }
                     totalToBeProdQuantity += optimizedSolution.depot.startingInventory;

                     UBp[t] = Math.Min(productionCapacity, totalToBeProdQuantity);
                    */
                    UBp[t] = productionCapacity;
                }


                // ============================================================================================================================================================//
                // Variable Initialization
                for (int t = 0; t < periodsNum; ++t)
                {
                    y[t] = PPRPmodel.AddVar(0.0, 1.0, optimizedSolution.depot.productionSetupCost, GRB.BINARY, "y^" + t);
                    //p[t] = PPRPmodel.AddVar(0.0, productionCapacity, optimizedSolution.depot.unitProductionCost, contOrINT, "p^" + t); //continuous 
                    p[t] = PPRPmodel.AddVar(0.0, UBp[t], optimizedSolution.depot.unitProductionCost, contOrINT, "p^" + t); //continuous 
                    //p[t] = PPRPmodel.AddVar(0.0, productionCapacity, optimizedSolution.depot.unitProductionCost, GRB.INTEGER, "p^" + t); //continuous

                    //I_depot[t] = PPRPmodel.AddVar(0.0, optimizedSolution.depot.stockMaximumLevel, optimizedSolution.depot.unitHoldingCost, GRB.CONTINUOUS, "I_depot^" + t);

                    for (int i = 0; i < nodesNum; i++)
                    {
                        Node node = optimizedSolution.nodes[i];
                        I[i, t] = PPRPmodel.AddVar(0.0, node.stockMaximumLevel, node.unitHoldingCost, contOrINT, "I_" + i + "^" + t);
                        if (i > 0) // customer
                        {
                            z[i, t] = PPRPmodel.AddVar(0.0, 1.0, 0.0, GRB.BINARY, "z_" + i + "^" + t);
                            q[i, t] = PPRPmodel.AddVar(0.0, UBq[i, t], 0.0, contOrINT, "q_" + i + "^" + t);
                            //q[i, t] = PPRPmodel.AddVar(0.0, UBq[i, t], 0.0, GRB.INTEGER, "q_" + i + "^" + t);
                        }
                        else
                        {
                            z[i, t] = PPRPmodel.AddVar(0.0, 0.0, 0.0, GRB.BINARY, "z_" + i + "^" + t);
                        }
                    }

                    for (int i = 0; i < allNodesNum; i++)
                    {
                        Node from = optimizedSolution.augNodes[i];
                        for (int j = 0; j < allNodesNum; j++)
                        {
                            Node to = optimizedSolution.augNodes[j];
                            if (i != j) //symmetric?
                            {
                                if (i < j)
                                {
                                    x[i, j, t] = PPRPmodel.AddVar(0.0, 1.0, model.distMatrix[from.uid, to.uid], GRB.BINARY, "x_" + i + "," + j + "^" + t);
                                }
                                f[i, j, t] = PPRPmodel.AddVar(0.0, vehCap, 0.0, contOrINT, "f_" + i + "," + j + "^" + t);
                            }
                        }
                    }
                }

                // branching priority
                for (int t = 0; t < periodsNum; t++)
                {
                    y[t].BranchPriority = 10;
                }

                // ============================================================================================================================================================//

                // Objective sense
                PPRPmodel.ModelSense = GRB.MINIMIZE;
                // ============================================================================================================================================================//

                // Routing and Flow Constraints 
                // 2. Visit schedule                
                for (int t = 0; t < periodsNum; t++)
                {
                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];

                        GRBLinExpr exp = 0.0;

                        for (int j = 0; j < optimizedSolution.augNodes.Count; j++)
                        {
                            Node custj = optimizedSolution.augNodes[j];

                            if (cust.uid > j) //if (cust.uid > custj.uid)
                            {
                                exp.AddTerm(1.0, x[j, cust.uid, t]);
                            }
                            else if (cust.uid < j)
                            {
                                exp.AddTerm(1.0, x[cust.uid, j, t]);
                            }
                        }

                        PPRPmodel.AddConstr(exp == 2 * z[cust.uid, t], "con2_" + cust.uid + "^" + t);
                    }
                }

                // 3. Routes start
                for (int t = 0; t < periodsNum; t++)
                {
                    GRBLinExpr exp = 0.0;

                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];
                        exp.AddTerm(1.0, x[0, cust.uid, t]);
                    }
                    PPRPmodel.AddConstr(exp <= model.vehicles, "con3_^" + t);
                }

                // 4. Routes end
                for (int t = 0; t < periodsNum; t++)
                {
                    GRBLinExpr exp = 0.0;
                    GRBLinExpr rhs = 0.0;

                    for (int j = 0; j < optimizedSolution.customers.Count; j++)
                    {
                        Node cust = optimizedSolution.customers[j];
                        exp.AddTerm(1.0, x[0, cust.uid, t]);
                    }
                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];
                        rhs.AddTerm(1.0, x[cust.uid, allNodesNum - 1, t]);
                    }

                    PPRPmodel.AddConstr(exp == rhs, "con4_^" + t);
                }

                // 5. Flow summation
                for (int t = 0; t < periodsNum; t++)
                {
                    for (int i = 0; i < optimizedSolution.augNodes.Count; i++)
                    {
                        Node custi = optimizedSolution.augNodes[i];

                        for (int j = 0; j < optimizedSolution.augNodes.Count; j++)
                        {
                            Node custj = optimizedSolution.augNodes[j];

                            if (i < j)
                            {
                                PPRPmodel.AddConstr(f[i, j, t] + f[j, i, t] == vehCap * x[i, j, t], "con5_" + i + "," + j + "^" + t);
                            }
                        }
                    }
                }

                // 6. Flow defination
                for (int t = 0; t < periodsNum; t++)
                {
                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];

                        GRBLinExpr exp = 0.0;

                        for (int j = 0; j < optimizedSolution.augNodes.Count; j++)
                        {
                            Node custj = optimizedSolution.augNodes[j];

                            if (cust.uid != custj.uid)
                            {
                                exp.AddTerm(1.0, f[cust.uid, j, t]);
                            }
                        }

                        PPRPmodel.AddConstr(exp == vehCap * z[cust.uid, t] - q[cust.uid, t], "con6_" + i + "^" + t);
                    }
                }

                // 7. Flow and deliveries
                for (int t = 0; t < periodsNum; t++)
                {
                    GRBLinExpr exp = 0.0;
                    GRBLinExpr rhs = 0.0;
                    for (int j = 0; j < optimizedSolution.customers.Count; j++)
                    {
                        Node cust = optimizedSolution.customers[j];
                        exp.AddTerm(1.0, f[0, cust.uid, t]);
                    }
                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];
                        rhs.AddTerm(1.0, q[cust.uid, t]);
                    }
                    PPRPmodel.AddConstr(exp == rhs, "con7_^" + t);
                }

                // 8. End flow
                for (int t = 0; t < periodsNum; t++)
                {
                    GRBLinExpr exp = 0.0;
                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];
                        exp.AddTerm(1.0, f[cust.uid, allNodesNum - 1, t]);
                    }

                    PPRPmodel.AddConstr(exp == 0.0, "con8_^" + t);
                }

                // Production Constraints 
                // 9. Production binaries
                for (int t = 0; t < periodsNum; ++t)
                {
                    PPRPmodel.AddConstr(p[t] <= y[t] * productionCapacity, "con9a_" + t);
                }

                /*
                for (int t = 0; t < periodsNum; ++t)
                {
                    GRBLinExpr exp = 0.0;
                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];
                        for (int tt = t; tt < periodsNum; ++tt)
                        {
                            exp.AddConstant(cust.productRate[tt]);
                        }
                        exp.AddConstant(cust.startingInventory);
                       
                        //totalToBeProdQuantity += cust.startingInventory;
                    }
                    exp.AddConstant(optimizedSolution.depot.startingInventory);

                    PPRPmodel.AddConstr(p[t] <= exp * y[t], "con9b_" + t);
                }
                */

                // ATTENTION: BOUDIA fix
                if (model.input.type == PRP_DATASET_VERSION.BOUDIA_FMT)
                {
                    PPRPmodel.AddConstr(p[0] == 0, "boudia_no_prod");
                    PPRPmodel.AddConstr(y[0] == 0, "boudia_no_prod");
                }


                // 10. Connection with previous production periods
                // Could be stricter for non periodic. This is not useful
                /*
                for (int t = 1; t < periodsNum; ++t)
                {
                    GRBLinExpr exp = 0.0;
                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];
                        for (int tt = 0; tt < periodsNum; ++tt)
                        {
                            exp.AddConstant(cust.productRate[tt]);
                        }
                    }
                    for (int tt = 0; tt < t - 1; ++tt)
                    {
                        exp.AddTerm(-1.0, p[tt]);
                    }
                    PPRPmodel.AddConstr(p[t] <= exp, "con10_" + t);
                }
                */

                // Inventory Flow Constraints 
                // 13. Depot inventory flow 
                for (int t = 0; t < periodsNum; ++t)
                {
                    
                    GRBLinExpr lhs = 0.0;

                    lhs.AddTerm(1.0, I[0, t]);
                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];

                        lhs.AddTerm(1.0, q[cust.uid, t]);
                    }

                    // Disallow the depot to have init inventory as decision variable
                    /*
                    if (t == 0)
                        PPRPmodel.AddConstr(lhs == optimizedSolution.depot.startingInventory + p[t], "depot_inv_" + t);
                    else
                        PPRPmodel.AddConstr(lhs == I[0, t - 1] + p[t], "depot_inv_" + t);
                     */
                    PPRPmodel.AddConstr(lhs == I[0, previousPeriodIdx(periodsNum, t)] + p[t], "depot_inv_" + t);
                   
                }

                // 14. Customer inventory flow 
                for (int t = 0; t < periodsNum; t++)
                {
                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];
                        GRBLinExpr lhs = 0.0;
                        lhs.AddTerm(1.0, I[cust.uid, t]);
                        PPRPmodel.AddConstr(lhs == I[cust.uid, previousPeriodIdx(periodsNum, t)] + q[cust.uid, t] - cust.productRate[t], "cust_inv" + cust.uid + "^" + t);
                    }
                }

                // ML Policy
                // 15. ML policy (relaxed UB)
                for (int t = 0; t < periodsNum; t++)
                {
                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];
                        PPRPmodel.AddConstr(q[cust.uid, t] <= cust.stockMaximumLevel + cust.productRate[t] - I[cust.uid, previousPeriodIdx(periodsNum, t)], "con15_" + i + "^" + t);
                    }
                }

                // Periodicity
                // 16. Same initial and end inventory (cycle)
                /*
                if (periodicity == "periodic")
                {
                    for (int i = 0; i < nodesNum; i++) // check for clone depot inventory
                    {
                        Node node = optimizedSolution.nodes[i];
                        PPRPmodel.AddConstr(I[node.uid, periodsNum - 1] == node.startingInventory, "con16_" + i);
                    }
                }
                */

                // 17. non zero quantity visits: the data set distance matrices do not respect triangular inequalities
                /*
                for (int t = 0; t < periodsNum; t++)
                {
                    for (int i = 0; i < custNum; i++) // check for clone depot inventory
                    {
                        Node cust = optimizedSolution.customers[i];
                        PPRPmodel.AddConstr(q[cust.uid, t] >=  z[cust.uid, t], "con17_" + i + "^" + t);
                    }
                }
                */

                // ============================================================================================================================================================//
                // Valid inequalities
                // Subdeliveries by 
                if (addValidInequalities)
                {
                    // Desaulnier Subdeliveries inequalities
                    List<int>[,] Tminus = new List<int>[nodesNum, periodsNum]; //upper bound of delivered quantities to customer

                    // initialization
                    for (int t = 0; t < periodsNum; ++t)
                    {
                        Tminus[0, t] = new List<int> { };
                        for (int i = 0; i < optimizedSolution.customers.Count; i++)
                        {
                            Node cust = optimizedSolution.customers[i];
                            Tminus[cust.uid, t] = new List<int> { };
                        }
                    }

                    
                    //populate sets
                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];
                        int startPeriod = findSafeCusStartPeriodVI(cust, model); // findCusStartPeriodVI(cust, model); // this cannot be used now because it needs constant initial inventory 

                        for (int t = 0; t < periodsNum; t++)
                        {
                            double totalDemand = 0.0;
                            
                            //if (periodicity == "periodic")
                            //{
                            //    if (t == periodsNum - 1) //last period may take into account the inventory as well
                            //    {
                            //        totalDemand += cust.startingInventory;
                            //    }
                            //}
                            
                            //for (int tt = t; tt > -1; tt--)
                            for (int backstep = 0; backstep < periodsNum; backstep++)
                            {
                                int cyclictt = actualPeriod(periodsNum, t, backstep);
                                totalDemand += cust.productRate[cyclictt];
                                if (totalDemand <= cust.stockMaximumLevel)
                                {
                                    Tminus[cust.uid, t].Add(cyclictt);
                                }
                                else
                                {
                                    Tminus[cust.uid, t].Add(cyclictt);
                                    break;
                                }
                            }
                        }
                    }
                    

                    
                    
                    //populate sets
                    /*
                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];
                        int startPeriod = findSafeCusStartPeriodVI(cust, model); // findCusStartPeriodVI(cust, model); // this cannot be used now because it needs constant initial inventory 

                        for (int t = startPeriod; t < periodsNum; t++)
                        {
                            double totalDemand = 0.0;
                                        
                            for (int tt = t; tt > -1; tt--)
                            {
                                totalDemand += cust.productRate[tt];
                                if (totalDemand <= cust.stockMaximumLevel)
                                {
                                    Tminus[cust.uid, t].Add(tt);
                                }
                                else
                                {
                                    Tminus[cust.uid, t].Add(tt);
                                    break;
                                }
                            }
                         }
                    }
                    */
                    
                     
                    

                    //add constraints
                    for (int t = 0; t < periodsNum; ++t)
                    {
                        for (int i = 0; i < optimizedSolution.customers.Count; i++)
                        {
                            Node cust = optimizedSolution.customers[i];
                            //Console.WriteLine("cust {3}: starting={0}, demand={1}, max level={2}", cust.startingInventory, cust.productRate[1], cust.stockMaximumLevel, cust.uid);

                            if (Tminus[cust.uid, t].Count > 0)
                            {
                                GRBLinExpr exp = 0.0;
                                foreach (int elem in Tminus[cust.uid, t])
                                {
                                    exp.AddTerm(1.0, z[cust.uid, elem]);
                                    //Console.WriteLine("cust {0} {1}:+{2}",cust.uid,t,elem);
                                }
                                PPRPmodel.AddConstr(exp >= 1, "vi44_" + cust.uid + "^" + t);
                            }
                        }
                    }


                    // 1. Visit and arcs depot
                    for (int t = 0; t < periodsNum; t++)
                    {
                        for (int i = 0; i < optimizedSolution.customers.Count; i++)
                        {
                            Node cust = optimizedSolution.customers[i];

                            PPRPmodel.AddConstr(x[0, cust.uid, t] <= 1 * z[cust.uid, t], "vi26_" + cust.uid + "^" + t); // TODO: or 2
                        }
                    }

                    // 2-3. Visits and arcs
                    for (int t = 0; t < periodsNum; t++)
                    {
                        for (int i = 0; i < optimizedSolution.customers.Count; i++)
                        {
                            Node custi = optimizedSolution.customers[i];

                            for (int j = 0; j < optimizedSolution.customers.Count; j++)
                            {
                                Node custj = optimizedSolution.customers[j];

                                if (custi.uid < custj.uid)
                                {
                                    PPRPmodel.AddConstr(x[custi.uid, custj.uid, t] <= z[custi.uid, t], "vi27_" + custi.uid + "," + custj.uid + "^" + t);
                                    PPRPmodel.AddConstr(x[custi.uid, custj.uid, t] <= z[custj.uid, t], "vi28_" + custi.uid + "," + custj.uid + "^" + t);
                                }
                            }
                        }
                    }

                    // 4-5. Flows and arcs
                    for (int t = 0; t < periodsNum; t++)
                    {
                        for (int i = 0; i < optimizedSolution.customers.Count; i++)
                        {
                            Node custi = optimizedSolution.customers[i];

                            for (int j = 0; j < optimizedSolution.customers.Count; j++)
                            {
                                Node custj = optimizedSolution.customers[j];

                                if (custi.uid < custj.uid)
                                {
                                    PPRPmodel.AddConstr(f[custi.uid, custj.uid, t] <= x[custi.uid, custj.uid, t] * vehCap, "vi29_" + custi.uid + "," + custj.uid + "^" + t);
                                    PPRPmodel.AddConstr(f[custj.uid, custi.uid, t] <= x[custi.uid, custj.uid, t] * vehCap, "vi30_" + custi.uid + "," + custj.uid + "^" + t);
                                }
                            }
                        }
                    }

                    // 12. Conditional flow  bounds 
                    for (int t = 0; t < periodsNum; t++)
                    {
                        for (int i = 0; i < optimizedSolution.customers.Count; i++)
                        {
                            Node custi = optimizedSolution.customers[i];

                            for (int j = 0; j < optimizedSolution.customers.Count; j++)
                            {
                                Node custj = optimizedSolution.customers[j];

                                if (custi.uid < custj.uid)
                                {
                                    PPRPmodel.AddConstr(f[custi.uid, custj.uid, t] >= custj.productRate[t] * x[custi.uid, custj.uid, t] - I[custj.uid, previousPeriodIdx(periodsNum, t)], "con37a_" + custi.uid + "," + custj.uid + "^" + t);
                                    PPRPmodel.AddConstr(f[custj.uid, custi.uid, t] >= custi.productRate[t] * x[custi.uid, custj.uid, t] - I[custi.uid, previousPeriodIdx(periodsNum, t)], "con37b_" + custi.uid + "," + custj.uid + "^" + t);
                                }
                            }
                        }
                    }

                    // 6. Delivered quantity and visits
                    //periodicity is captured in the UBq calculation
                    for (int t = 0; t < periodsNum; ++t)
                    {
                        for (int i = 0; i < optimizedSolution.customers.Count; i++)
                        {
                            Node custi = optimizedSolution.customers[i];
                            PPRPmodel.AddConstr(q[custi.uid, t] <= UBq[custi.uid, t] * z[custi.uid, t], "vi31_" + custi.uid + "^" + t);
                        }
                    }

                    // Remove all are flows 
                    // 7. Delivered quantity and visits with inventories
                    /*
                    for (int t = 0; t < periodsNum; ++t)
                    {
                        for (int i = 0; i < optimizedSolution.customers.Count; i++)
                        {
                            Node custi = optimizedSolution.customers[i];
                            int totalDemandUntilEnd = 0;
                            for (int tt = t; tt < periodsNum; ++tt)
                            {
                                totalDemandUntilEnd += custi.productRate[tt]; //TODO:
                            }
                            if (t == 0)
                            {
                                PPRPmodel.AddConstr(q[custi.uid, t] <= custi.stockMaximumLevel + custi.productRate[t] - custi.startingInventory, "vi32a_" + custi.uid + "^" + t);
                                if (periodicity == "periodic")
                                {
                                    PPRPmodel.AddConstr(q[custi.uid, t] <= totalDemandUntilEnd - custi.startingInventory + custi.startingInventory, "vi32b_" + custi.uid + "^" + t);
                                }
                                else
                                {
                                    PPRPmodel.AddConstr(q[custi.uid, t] <= totalDemandUntilEnd - custi.startingInventory, "vi32b_" + custi.uid + "^" + t);
                                }
                            }
                            else
                            {
                                PPRPmodel.AddConstr(q[custi.uid, t] <= custi.stockMaximumLevel + custi.productRate[t] - I[custi.uid, t - 1], "vi32a_" + custi.uid + "^" + t);
                                if (periodicity == "periodic")
                                {
                                    PPRPmodel.AddConstr(q[custi.uid, t] <= totalDemandUntilEnd - I[custi.uid, t - 1] + custi.startingInventory, "vi32b_" + custi.uid + "^" + t);
                                }
                                else
                                {
                                    PPRPmodel.AddConstr(q[custi.uid, t] <= totalDemandUntilEnd - I[custi.uid, t - 1], "vi32b_" + custi.uid + "^" + t);
                                }
                            }
                        }
                    }
                    */



                    // 9. Minimum remaining visits
                    for (int t1 = 0; t1 < periodsNum; ++t1)
                    {
                        for (int t2 = 0; t2 < periodsNum; ++t2)
                        {
                            if (t1 <= t2)
                            {
                                for (int i = 0; i < optimizedSolution.customers.Count; i++)
                                {
                                    Node custi = optimizedSolution.customers[i];
                                    int maxDemand = custi.productRate.Max();

                                    double numerator = 0.0;
                                    double denominator = (double)Math.Min(vehCap, custi.stockMaximumLevel + maxDemand);

                                    GRBLinExpr exp2 = 0.0;

                                    for (int tt = t1; tt < t2; ++tt) //Note: t+1 according to equation
                                    {
                                        exp2.AddTerm(1.0, z[custi.uid, tt]);
                                        numerator += custi.productRate[tt];

                                    }

                                    numerator -= custi.stockMaximumLevel;

                                    double fraction = Math.Ceiling(numerator / denominator);
                                    PPRPmodel.AddConstr(exp2 >= fraction, "vi34" + custi.uid + "^" + t1 + "^" + t2);
                                }
                            }
                        }
                    }

                    // 10. Minimum remaining visits
                    for (int t1 = 0; t1 < periodsNum; ++t1)
                    {
                        for (int t2 = 0; t2 < periodsNum; ++t2)
                        {
                            if (t1 <= t2)
                            {
                                for (int i = 0; i < optimizedSolution.customers.Count; i++)
                                {
                                    Node custi = optimizedSolution.customers[i];
                                    int maxDemand = custi.productRate.Max();

                                    GRBLinExpr exp = 0.0;
                                    double denominator = (double)Math.Min(vehCap, custi.stockMaximumLevel + maxDemand);

                                    GRBLinExpr exp2 = 0.0;

                                    for (int tt = t1; tt < t2; ++tt) //Note: t+1 according to equation
                                    {
                                        exp2.AddTerm(1.0, z[custi.uid, tt]);
                                        exp.AddConstant(custi.productRate[tt]);
                                    }
                                    exp.AddTerm(-1.0, I[custi.uid, previousPeriodIdx(periodsNum, t1)]);
                                    
                                    PPRPmodel.AddConstr(denominator * exp2 >= exp, "vi35" + custi.uid + "^" + t1 + "^" + t2);
                                }
                            }
                        }
                    }

                    // 11. Minimum remaining visits
                    for (int t1 = 0; t1 < periodsNum; ++t1)
                    {
                        for (int t2 = 0; t2 < periodsNum; ++t2)
                        {
                            if (t1 <= t2)
                            {
                                for (int i = 0; i < optimizedSolution.customers.Count; i++)
                                {
                                    Node custi = optimizedSolution.customers[i];
                                    int maxDemand = custi.productRate.Max();

                                    GRBLinExpr numerator = 0.0;
                                    double denominator = 0.0;

                                    GRBLinExpr exp2 = 0.0;

                                    for (int tt = t1; tt < t2; ++tt) //Note: t+1 according to equation
                                    {
                                        exp2.AddTerm(1.0, z[custi.uid, tt]);
                                        numerator.AddConstant(custi.productRate[tt]);
                                        denominator += custi.productRate[tt];
                                    }
                                    numerator.AddTerm(-1.0, I[custi.uid, previousPeriodIdx(periodsNum, t1)]);
                                    
                                    PPRPmodel.AddConstr(denominator * exp2 >= numerator, "vi36" + custi.uid + "^" + t1 + "^" + t2);
                                }
                            }
                        }
                    }


                    // 13. Number of production days 1
                    for (int t = 0; t < periodsNum; t++)
                    {
                        GRBLinExpr exp = 0.0;

                        for (int i = 0; i < optimizedSolution.customers.Count; i++)
                        {
                            Node custi = optimizedSolution.customers[i];
                            exp.AddConstant(custi.productRate[t]);
                            exp.AddTerm(-1.0, I[custi.uid, previousPeriodIdx(periodsNum, t)]);
                        }
                        PPRPmodel.AddConstr(y[t] * optimizedSolution.depot.productionCapacity >= exp - I[0, previousPeriodIdx(periodsNum, t)], "con39_" + "^" + t);
                    }


                    // This is wrong because of it counts from start. Not for decision variable initial stock 
                    /*
                    // 14. Number of production days 2
                    for (int t = 1; t < periodsNum; ++t)
                    {
                        double numerator = 0.0;
                        double denominator = optimizedSolution.depot.productionCapacity;

                        GRBLinExpr exp = 0.0;
                        for (int tt = 0; tt < t; ++tt) //Note: t+1 
                        {
                            exp.AddTerm(1.0, y[tt]);
                        }

                        for (int i = 0; i < optimizedSolution.customers.Count; i++)
                        {
                            Node custi = optimizedSolution.customers[i];

                            for (int tt = 0; tt < t; ++tt) //Note: t+1 
                            {
                                numerator += custi.productRate[tt];
                            }

                            numerator -= custi.stockMaximumLevel;
                        }
                        double fraction = Math.Ceiling(numerator / denominator);

                        PPRPmodel.AddConstr(exp >= fraction, "vi40_" + "^" + t);
                    }
                    */

                    // 15. Number of production days 3
                    for (int t1 = 0; t1 < periodsNum; ++t1)
                    {
                        for (int t2 = 0; t2 < periodsNum; ++t2)
                        {
                            if (t1 <= t2)
                            {
                                double numerator = 0.0;
                                double denominator = optimizedSolution.depot.productionCapacity;

                                GRBLinExpr exp = 0.0;
                                for (int tt = t1; tt < t2; ++tt) //Note: t+1 
                                {
                                    exp.AddTerm(1.0, y[tt]);
                                }

                                for (int i = 0; i < optimizedSolution.customers.Count; i++)
                                {
                                    Node custi = optimizedSolution.customers[i];

                                    for (int tt = t1; tt < t2; ++tt) //Note: t+1 
                                    {
                                        numerator += custi.productRate[tt];
                                    }

                                    numerator -= custi.stockMaximumLevel;
                                }
                                numerator -= optimizedSolution.depot.stockMaximumLevel;

                                double fraction = Math.Ceiling(numerator / denominator);

                                PPRPmodel.AddConstr(exp >= fraction, "vi41_" + "^" + t1 + "^" + t2);
                            }
                        }
                    }

                    // 16. Number of production days 4 
                    for (int t1 = 0; t1 < periodsNum; ++t1)
                    {
                        for (int t2 = 0; t2 < periodsNum; ++t2)
                        {
                            if (t1 <= t2)
                            {
                                GRBLinExpr numerator = 0.0;
                                double denominator = optimizedSolution.depot.productionCapacity;

                                GRBLinExpr exp = 0.0;
                                for (int tt = t1; tt < t2; ++tt) //Note: t+1 
                                {
                                    exp.AddTerm(1.0, y[tt]);
                                }

                                for (int i = 0; i < optimizedSolution.customers.Count; i++)
                                {
                                    Node custi = optimizedSolution.customers[i];

                                    for (int tt = t1; tt < t2; ++tt) //Note: t+1 
                                    {
                                        numerator.AddConstant(custi.productRate[tt]);
                                    }
                                    numerator.AddTerm(-1.0, I[custi.uid, previousPeriodIdx(periodsNum, t1)]);
                                }
                                numerator.AddTerm(-1.0, I[0, previousPeriodIdx(periodsNum, t1)]);
                              
                                // ERROR
                                //numerator -= optimizedSolution.depot.stockMaximumLevel;

                                PPRPmodel.AddConstr(denominator * exp >= numerator, "vi42_" + "^" + t1 + "^" + t2);
                            }
                        }
                    }

                }


                // ===========================================Fix binary variables (when rerunning for integrality)===============================
                //bool varFixing = true;
                if (x_fix != null)
                {
                    for (int t = 0; t < periodsNum; ++t)
                    {
                        for (int i = 0; i < allNodesNum; i++)
                        {
                            Node from = optimizedSolution.augNodes[i];
                            for (int j = 0; j < allNodesNum; j++)
                            {
                                Node to = optimizedSolution.augNodes[j];
                                if (i != j) //symmetric?
                                {
                                    if (i < j)
                                    {
                                        if (GlobalUtils.IsEqual(x_fix[i, j, t].X, 1.0, 1e-3))
                                        {
                                            x[i, j, t].LB = 1.0;
                                            x[i, j, t].UB = 1.0;
                                        }
                                        else
                                        {
                                            x[i, j, t].LB = 0.0;
                                            x[i, j, t].UB = 0.0;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                if (y_fix != null)
                {
                    for (int t = 0; t < periodsNum; ++t)
                    {
                        if (GlobalUtils.IsEqual(y_fix[t].X, 1.0, 1e-3))
                        {
                            y[t].LB = 1.0;
                            y[t].UB = 1.0;
                        }
                        else
                        {
                            y[t].LB = 0.0;
                            y[t].UB = 0.0;
                        }
                    }
                }

                if (z_fix != null)
                {
                    for (int t = 0; t < periodsNum; ++t)
                    {
                        for (int i = 0; i < nodesNum; i++)
                        {
                            if (i > 0)
                            {
                                if (GlobalUtils.IsEqual(z_fix[i, t].X, 1.0, 1e-3))
                                {
                                    z[i, t].LB = 1.0;
                                    z[i, t].UB = 1.0;
                                }
                                else
                                {
                                    z[i, t].LB = 0.0;
                                    z[i, t].UB = 0.0;
                                }
                            }
                        }
                    }
                }

                // ==================================================Optimize====================================================================
                PPRPmodel.Optimize();
                int optimstatus = PPRPmodel.Status;
                switch (PPRPmodel.Status)
                {
                    case GRB.Status.OPTIMAL:
                    case GRB.Status.TIME_LIMIT:
                        {
                            double objval = PPRPmodel.ObjVal;

                            dict["ObjVal"] = PPRPmodel.ObjVal;
                            if (forcedIntegrality) // if forced integrality, override the dict
                            {
                                dict["Status"] = dictRes["Status"];
                                dict["Runtime"] = dictRes["Runtime"];
                                dict["ObjBound"] = dictRes["ObjBound"];
                                dict["NodeCount"] = dictRes["NodeCount"];
                                dict["NumBinVars"] = dictRes["NumBinVars"];
                                dict["NumVars"] = dictRes["NumVars"];
                                dict["NumConstrs"] = dictRes["NumConstrs"];
                                dict["MIPGap"] = dictRes["MIPGap"];
                            }
                            else
                            {
                                dict["Status"] = PPRPmodel.Status;
                                dict["Runtime"] = PPRPmodel.Runtime;
                                dict["ObjBound"] = PPRPmodel.ObjBound;
                                dict["NodeCount"] = PPRPmodel.NodeCount;
                                dict["NumBinVars"] = PPRPmodel.NumBinVars;
                                dict["NumVars"] = PPRPmodel.NumVars;
                                dict["NumConstrs"] = PPRPmodel.NumConstrs;
                                dict["MIPGap"] = PPRPmodel.MIPGap;
                                //dict["ObjVal"] = PPRPmodel.ObjVal;
                            }

                            bool integrality = checkVariableIntegrality(optimizedSolution, model, PPRPmodel, x, y, z, f, p, q, I);

                            if (!integrality) // Imposed rounding error
                            {
                                if (forcedIntegrality) // implied failure 
                                {
                                    throw new Exception("Integrality constraints were not respected.");
                                }
                                bool forceIntegralityVar = true;

                                (optimizedSolutionFixed, dictFixed) = SolveCPRP(model, forceIntegralityVar, x, y, z, dict);
                                return (optimizedSolutionFixed, dictFixed);
                                break;
                            }

                            // Store the initial inventory (cyclic)
                            for (int i = 0; i < optimizedSolution.nodes.Count; i++)
                            {
                                Node node = optimizedSolution.nodes[i];
                                node.cyclicStartInv = (int)Math.Round(I[i, periodsNum - 1].X, 2);
                                node.startingInventory = (int)Math.Round(I[i, periodsNum - 1].X, 2);
                                if (!GlobalUtils.IsEqual(node.startingInventory, Math.Round(I[i, periodsNum - 1].X, 2)))
                                {
                                    Console.WriteLine("Problem");
                                }
                            }


                            // Update depot 
                            for (int t = 0; t < periodsNum; t++)
                            {
                                optimizedSolution.depot.deliveredQuantities[t] = 0;
                                optimizedSolution.depot.productionRates[t] = (int)Math.Round(p[t].X); //ATTENTION not Int
                                optimizedSolution.depot.open[t] = false;
                                if (GlobalUtils.IsEqual(y[t].X, 1.0, 1e-3))
                                {
                                    optimizedSolution.depot.open[t] = true;
                                }
                            }

                            //report 
                            /*
                            String prodSch = "\nProduction schedule: ";
                            String quant = "\nProduction quantities: ";
                            for (int t = 0; t < periodsNum; ++t)
                            {
                                prodSch += ((int)Math.Round(y[t].X));
                                prodSch += "_";
                                quant += ((int)Math.Round(p[t].X));
                                quant += "_";
                            }
                            Console.WriteLine(prodSch);
                            Console.WriteLine(quant);

                            for (int t = 0; t < periodsNum; t++)
                            {
                                for (int i = 0; i < allNodesNum; i++)
                                {
                                    Node from = optimizedSolution.augNodes[i];
                                    for (int j = 0; j < allNodesNum; j++)
                                    {
                                        Node to = optimizedSolution.augNodes[j];
                                        if (i != j) //symmetric?
                                        {
                                            if (i < j)
                                            {
                                                if (GlobalUtils.IsEqual(x[i,j,t].X, 1.0, 1e-3))
                                                    Console.WriteLine("x_{0}-{1}^{2}",i, j,t); 
                                            }
                                            //if (GlobalUtils.IsEqual(f[i, j, t].X, 1.0, 1e-3))
                                                //Console.WriteLine("f_{0}-{1}^{2}", from.uid, to.uid, t);
                                        }
                                    }
                                }
                            }

                            Console.WriteLine("\n\nVisits and quantities");

                            for (int t = 0; t < periodsNum; t++)
                            {
                                for (int i = 1; i < nodesNum; i++)
                                {
                                    Node cust = optimizedSolution.augNodes[i];
                                    if (GlobalUtils.IsEqual(z[i, t].X, 1.0, 1e-3))
                                    {
                                        //Console.WriteLine("z_{0}^{1}", cust.uid, t);
                                        Console.WriteLine("q_{0}^{1}={2}", cust.uid, t, (int)Math.Round(q[cust.uid,t].X));

                                    }
                                }
                            }
                            */

                            // routes parsing 
                            List<List<Node>>[] periodRouteArray = new List<List<Node>>[periodsNum];
                            for (int t = 0; t < periodsNum; t++)
                            {
                                periodRouteArray[t] = findPeriodRoutes(model, PPRPmodel, x, z, q, optimizedSolution, t);
                            }

                            for (int t = 0; t < periodsNum; t++)
                            {
                                Period pr = optimizedSolution.periods[t];

                                int rt_idx = 0;

                                foreach (List<Node> list in periodRouteArray[t])
                                {
                                    Route rt = pr.periodRoutes[rt_idx];

                                    //Console.Write("\nPeriod {0} Route: ", t);
                                    foreach (Node node in list)
                                    {
                                        rt.nodes.Add(node);

                                        if (node.uid != 0)
                                        {
                                            node.visitSchedule[t] = true;

                                            CustDelivery cd = node.horizonDeliveryServices[t];
                                            node.deliveredQuantities[t] = (int)Math.Round(q[node.uid, t].X); //ATTENTION not Int
                                            if ((int)Math.Round(q[node.uid, t].X) == 00) //ATTENTION not Int
                                            {
                                                GlobalUtils.writeToConsole(" Customer {0} has zero quantity delivery on period {1}:" +
                                                    "Possible shortcut due to triangular inequality failure from roundings.", node.uid, t);
                                            }
                                            cd.quantity = node.deliveredQuantities[t];
                                            optimizedSolution.depot.deliveredQuantities[t] -= node.deliveredQuantities[t]; // OR +=;

                                            cd.route = rt;
                                            cd.route.load += cd.quantity;

                                        }
                                        //Console.Write("{0}>", node.uid);
                                    }
                                    rt.SetLoadAndCostLists(t, model);
                                    rt.calculateRoutingCost(model);
                                    rt_idx++;
                                }
                            }

                            // Reporting: Route printing
                            for (int t = 0; t < periodsNum; t++)
                            {
                                Period pr = optimizedSolution.periods[t];

                                int rt_idx = 0;

                                foreach (List<Node> list in periodRouteArray[t])
                                {
                                    Route rt = pr.periodRoutes[rt_idx];

                                    Console.Write("\nPeriod {0} Route: ", t);
                                    foreach (Node node in list)
                                    {
                                        Console.Write("{0}>", node.uid);
                                    }
                                }
                            }



                            // Update inventory levels
                            for (int i = 0; i < optimizedSolution.customers.Count; i++)
                            {
                                Node cust = optimizedSolution.customers[i];
                                cust.CalculateInventoryLevels();
                            }
                            optimizedSolution.depot.CalculateInventoryLevels();

                            // Testing calculate objectives from MIP variables
                            double routingCostFromMIP = 0.0;
                            double holdingCostFromMIP = 0.0;
                            double holdingCostFromMIP2 = 0.0;
                            double totalUnitProductionCostFromMIP = 0.0;
                            double setupProductionCostFromMIP = 0.0;
                            double totalObjectiveFromMIP = 0.0;

                            for (int t = 0; t < periodsNum; ++t)
                            {
                                if (GlobalUtils.IsEqual(y[t].X, 1.0, 1e-3))
                                {
                                    setupProductionCostFromMIP += optimizedSolution.depot.productionSetupCost;
                                }
                                totalUnitProductionCostFromMIP += (int)Math.Round(p[t].X) * optimizedSolution.depot.unitProductionCost;

                                for (int i = 0; i < nodesNum; i++)
                                {
                                    Node node = optimizedSolution.nodes[i];
                                    holdingCostFromMIP += node.unitHoldingCost * (int)Math.Round(I[i, t].X); //ATTENTION not Int
                                    holdingCostFromMIP2 += node.unitHoldingCost * (double)Math.Round(I[i, t].X, 2);
                                }
                                Console.WriteLine("");
                                if (holdingCostFromMIP != holdingCostFromMIP2)
                                {
                                    Console.WriteLine("Possible rounding errors");
                                }

                                for (int i = 0; i < allNodesNum; i++)
                                {
                                    Node from = optimizedSolution.augNodes[i];
                                    for (int j = 0; j < allNodesNum; j++)
                                    {
                                        Node to = optimizedSolution.augNodes[j];
                                        if (i < j) //symmetric?
                                        {
                                            if (GlobalUtils.IsEqual(x[i, j, t].X, 1.0, 1e-3))
                                            {
                                                //routingCostFromMIP += model.distMatrix[from.uid, to.uid];
                                                routingCostFromMIP += model.distMatrix[from.uid, to.uid];
                                            }
                                        }
                                    }
                                }
                            }

                            totalObjectiveFromMIP = routingCostFromMIP + holdingCostFromMIP + totalUnitProductionCostFromMIP
                                + setupProductionCostFromMIP;

                            //Set solution objective
                            optimizedSolution.routingCost = Solution.EvaluateRoutingObjectivefromScratch(optimizedSolution);
                            optimizedSolution.holdingCost = Solution.EvaluateInventoryObjectivefromScratch(optimizedSolution);
                            optimizedSolution.totalUnitProductionCost = Solution.EvaluateUnitProductionObjectivefromScratch(optimizedSolution);
                            optimizedSolution.setupProductionCost = Solution.EvaluateSetupProductionObjectivefromScratch(optimizedSolution);
                            optimizedSolution.totalObjective = optimizedSolution.routingCost + optimizedSolution.holdingCost + optimizedSolution.totalUnitProductionCost + optimizedSolution.setupProductionCost;

                            if (forcedIntegrality) // if forced integrality, override the dict
                            {
                                dict["routingCostFromMIP"] = routingCostFromMIP;
                                dict["holdingCostFromMIP"] = holdingCostFromMIP;
                                dict["totalUnitProductionCostFromMIP"] = totalUnitProductionCostFromMIP;
                                dict["setupProductionCostFromMIP"] = setupProductionCostFromMIP;
                                dict["totalObjectiveFromMIP"] = totalObjectiveFromMIP;
                            }
                            else
                            {
                                dict["routingCostFromMIP"] = routingCostFromMIP;
                                dict["holdingCostFromMIP"] = holdingCostFromMIP;
                                dict["totalUnitProductionCostFromMIP"] = totalUnitProductionCostFromMIP;
                                dict["setupProductionCostFromMIP"] = setupProductionCostFromMIP;
                                dict["totalObjectiveFromMIP"] = totalObjectiveFromMIP;
                            }

                            GlobalUtils.writeToConsole("\nRouting {0} == {1}", routingCostFromMIP, optimizedSolution.routingCost);
                            GlobalUtils.writeToConsole("Holding {0} == {1}", holdingCostFromMIP, optimizedSolution.holdingCost);
                            GlobalUtils.writeToConsole("Unit production {0} == {1}", totalUnitProductionCostFromMIP, optimizedSolution.totalUnitProductionCost);
                            GlobalUtils.writeToConsole("Setup production {0} == {1}", setupProductionCostFromMIP, optimizedSolution.setupProductionCost);
                            GlobalUtils.writeToConsole("Total {0} == {1}", totalObjectiveFromMIP, optimizedSolution.totalObjective);

                            if (MathematicalProgramming.exactParams.periodicity == "periodic")
                            {
                                optimizedSolution.status = optimizedSolution.TestEverythingFromScratchPeriodic();
                            }
                            else if (MathematicalProgramming.exactParams.periodicity == "cyclic")
                            {
                                optimizedSolution.status = optimizedSolution.TestEverythingFromScratchCyclic();
                            }
                            else if (MathematicalProgramming.exactParams.periodicity == "no")
                            {
                                optimizedSolution.status = optimizedSolution.TestEverythingFromScratch();
                            } else
                            {
                                Console.WriteLine("no: PRP, periodic: periodic equal final and starting inventories, cyclic: same but with decision variable inventories");
                            }

                            //TSP_GRB_Solver.SolveTSP(optimizedSolution);

                            break;
                        }
                    case GRB.Status.INFEASIBLE:
                        {
                            Console.WriteLine("Model is infeasible");
                            // it could call itself with increased number of total insertions
                            //runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemoval(ref sol);


                            // compute and write out IIS
                            PPRPmodel.ComputeIIS();
                            PPRPmodel.Write(model.instanceName + "_model.ilp");
                            return (optimizedSolution, dict);
                        }
                    case GRB.Status.UNBOUNDED:
                        {
                            Console.WriteLine("Model is unbounded");
                            return (optimizedSolution, dict);
                        }
                    default:
                        {
                            Console.WriteLine("Optimization was stopped with status = " + optimstatus);
                            return (optimizedSolution, dict);
                        }

                }

                // Dispose of model
                PPRPmodel.Dispose();
                //gurobiEnv.Dispose();
            }
            catch (GRBException e)
            {
                Console.WriteLine("Error code: " + e.ErrorCode + ". " + e.Message);
            }
            if (optimizedSolutionFixed != null)
            {
                return (optimizedSolutionFixed, dictFixed);
            }
            return (optimizedSolution, dict);
        }

        public static int actualPeriod(int periodsNum, int t, int backstep)
        {
            int actualperiod = t;
            for (int i=0; i<backstep; i++)
            {
                actualperiod = actualperiod - 1;
                if (actualperiod == -1)
                {
                    actualperiod = periodsNum - 1; // zero index base
                }
            }
            return actualperiod;
        }

        public static int previousPeriodIdx(int periodsNum, int t)
        {
            int prev = -1;
            if (t==0)
            {
                prev = periodsNum - 1; // six periods: 0,1,2,3,4,5 
            } 
                else
            {
                prev = t-1;
            }
            return prev;
        }

        public static (Solution, Dictionary<string, double>) SolvePPRP(PRP model, bool forcedIntegrality, 
            GRBVar[,,] x_fix = null, GRBVar[] y_fix = null, GRBVar[,] z_fix = null, Dictionary<string, double> dictRes = null)
        {
            //params
            bool addValidInequalities = exactParams.validsInequalities;
            string periodicity = exactParams.periodicity;
            //bool limitedVehicles = exactParams.limitedVehicles;

            // easy access
            int periodsNum = model.input.horizonDays;
            int custNum = model.input.customerNum;
            int nodesNum = model.input.nodes.Count;
            int allNodesNum= model.input.nodes.Count + 1;
            int vehCap = model.input.dayVehicleCapacity;
            double productionCapacity = 1;
            var contOrINT = GRB.CONTINUOUS; // GRB.CONTINUOUS GRB.INTEGER
            if (forcedIntegrality)
            {
                contOrINT = GRB.INTEGER;
            }

            // declaration
            Dictionary<string, double> dict = new Dictionary<String, double>();
            Solution optimizedSolutionFixed = null;
            Dictionary<string, double> dictFixed = null;

            if (dictRes == null)
            {
                dict.Add("Status", -1.0);
                dict.Add("Runtime", -1.0);
                dict.Add("ObjBound", -1.0);
                dict.Add("ObjVal", -1.0);
                dict.Add("NodeCount", -1.0);
                dict.Add("NumBinVars", -1.0);
                dict.Add("NumVars", -1.0);
                dict.Add("NumConstrs", -1.0);
                dict.Add("MIPGap", -1.0);
                dict.Add("Solution validity", -1.0);
                dict.Add("routingCostFromMIP", -1.0);
                dict.Add("holdingCostFromMIP", -1.0);
                dict.Add("totalUnitProductionCostFromMIP", -1.0);
                dict.Add("setupProductionCostFromMIP", -1.0);
                dict.Add("totalObjectiveFromMIP", -1.0);
            }

            Solution optimizedSolution = new Solution(model);
            double[,] UBq = new double[nodesNum, periodsNum]; //upper bound of delivered quantities to customer
            double[] UBp = new double[periodsNum]; //upper bound of delivered quantities to customer

            productionCapacity += optimizedSolution.depot.startingInventory;
            for (int i = 0; i < optimizedSolution.customers.Count; i++)
            {
                Node cust = optimizedSolution.customers[i];
                productionCapacity += cust.startingInventory;
                for (int t = 0; t < periodsNum; ++t)
                {
                    productionCapacity += cust.productRate[t]; 
                }
            }

            // ATTENTION: modify depot maximum stock level
            optimizedSolution.depot.stockMaximumLevel = (int)Math.Min((int)productionCapacity, optimizedSolution.depot.stockMaximumLevel);
            optimizedSolution.depot.productionCapacity = (int)Math.Min(productionCapacity, optimizedSolution.depot.productionCapacity);

            // create model
            try
            {
                //GRBEnv gurobiEnv = new GRBEnv();
                GRBModel PPRPmodel = new GRBModel(gurobiEnv);

                // Params
                PPRPmodel.ModelName = "FullMIP_" + model.instanceName;
                PPRPmodel.Parameters.OutputFlag = (GlobalUtils.suppress_output) ? 0 : 1;
                PPRPmodel.Parameters.MIPGap = 1e-8;
                PPRPmodel.Parameters.Heuristics = 0.25;
                //PPRPmodel.Parameters.IntFeasTol = 1e-8;
                //PPRPmodel.Parameters.FeasibilityTol = 1e-8;
                //PPRPmodel.Parameters.OptimalityTol = 1e-8;
                PPRPmodel.Parameters.Threads = threadLimit;
                PPRPmodel.Parameters.TimeLimit = exactParams.timeLimit * 60; //0.2*60
                PPRPmodel.Parameters.MIPFocus = 1;

                // Decision variables
                // Binary
                GRBVar[,,] x = new GRBVar[allNodesNum, allNodesNum, periodsNum];
                GRBVar[] y = new GRBVar[periodsNum];
                GRBVar[,] z = new GRBVar[nodesNum, periodsNum];

                // Cont
                GRBVar[,,] f = new GRBVar[allNodesNum, allNodesNum, periodsNum];
                GRBVar[] p = new GRBVar[periodsNum];
                GRBVar[,] q = new GRBVar[nodesNum, periodsNum];
                GRBVar[,] I = new GRBVar[nodesNum, periodsNum];
                //GRBVar[] I_depot = new GRBVar[periodsNum]; // depot inventory
                //GRBVar[,] I_cust = new GRBVar[custNum, periodsNum]; // customer inventory

                // calculate bounds
                for (int t = 0; t < periodsNum; ++t)
                {
                    UBq[0, t] = 0;
                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];

                        int aggrCap = cust.stockMaximumLevel + cust.productRate[t];
                        int totalDemandUntilEnd = 0;
                        for (int tt = t; tt < periodsNum; ++tt)
                        {
                            totalDemandUntilEnd += cust.productRate[tt]; //TODO:
                        }
                        if (periodicity == "periodic")
                        {
                            totalDemandUntilEnd += cust.startingInventory; //periodic
                        }
                        UBq[cust.uid, t] = (double)GlobalUtils.Min(aggrCap, vehCap, totalDemandUntilEnd);
                    }
                }

                for (int t = 0; t < periodsNum; ++t)
                {
                    double totalToBeProdQuantity = 0.0;

                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];
                        for (int tt = t; tt < periodsNum; ++tt)
                        {
                            totalToBeProdQuantity += cust.productRate[tt]; //TODO: careful for error
                        }
                        if (periodicity == "periodic")
                        {
                            totalToBeProdQuantity += cust.startingInventory;
                        }
                    }
                    if (periodicity == "periodic")
                    {
                        totalToBeProdQuantity += optimizedSolution.depot.startingInventory;
                    }
                    UBp[t] = Math.Min(productionCapacity, totalToBeProdQuantity);
                }


                // ============================================================================================================================================================//
                // Variable Initialization
                for (int t = 0; t < periodsNum; ++t)
                {
                    y[t] = PPRPmodel.AddVar(0.0, 1.0, optimizedSolution.depot.productionSetupCost, GRB.BINARY, "y^" + t);
                    //p[t] = PPRPmodel.AddVar(0.0, productionCapacity, optimizedSolution.depot.unitProductionCost, contOrINT, "p^" + t); //continuous 
                    p[t] = PPRPmodel.AddVar(0.0, UBp[t], optimizedSolution.depot.unitProductionCost, contOrINT, "p^" + t); //continuous 
                    //p[t] = PPRPmodel.AddVar(0.0, productionCapacity, optimizedSolution.depot.unitProductionCost, GRB.INTEGER, "p^" + t); //continuous

                    //I_depot[t] = PPRPmodel.AddVar(0.0, optimizedSolution.depot.stockMaximumLevel, optimizedSolution.depot.unitHoldingCost, GRB.CONTINUOUS, "I_depot^" + t);

                    for (int i = 0; i < nodesNum; i++)
                    {
                        Node node = optimizedSolution.nodes[i];
                        I[i, t] = PPRPmodel.AddVar(0.0, node.stockMaximumLevel, node.unitHoldingCost, contOrINT, "I_" + i + "^" + t);
                        if ( i > 0) // customer
                        {
                            z[i, t] = PPRPmodel.AddVar(0.0, 1.0, 0.0, GRB.BINARY, "z_" + i + "^" + t);
                            q[i, t] = PPRPmodel.AddVar(0.0, UBq[i,t], 0.0, contOrINT, "q_" + i + "^" + t);
                            //q[i, t] = PPRPmodel.AddVar(0.0, UBq[i, t], 0.0, GRB.INTEGER, "q_" + i + "^" + t);
                        }
                        else
                        {
                            z[i, t] = PPRPmodel.AddVar(0.0, 0.0, 0.0, GRB.BINARY, "z_" + i + "^" + t);
                        }
                    }

                    for (int i = 0; i < allNodesNum; i++)
                    {
                        Node from = optimizedSolution.augNodes[i];
                        for (int j = 0; j < allNodesNum; j++)
                        {
                            Node to = optimizedSolution.augNodes[j];
                            if (i!=j) //symmetric?
                            {
                                if (i<j)
                                {
                                    x[i, j, t] = PPRPmodel.AddVar(0.0, 1.0, model.distMatrix[from.uid, to.uid], GRB.BINARY, "x_" + i + "," + j + "^" + t);
                                }
                                f[i, j, t] = PPRPmodel.AddVar(0.0, vehCap, 0.0, contOrINT, "f_" + i + "," + j + "^" + t);
                            }
                        }
                    }
                }

                // branching priority
                for (int t = 0; t < periodsNum; t++)
                {
                    y[t].BranchPriority = 10;
                }

                // ============================================================================================================================================================//

                // Objective sense
                PPRPmodel.ModelSense = GRB.MINIMIZE;
                // ============================================================================================================================================================//

                // Routing and Flow Constraints 
                // 2. Visit schedule                
                for (int t = 0; t < periodsNum; t++)
                {
                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];

                        GRBLinExpr exp = 0.0;

                        for (int j = 0; j < optimizedSolution.augNodes.Count; j++)
                        {
                            Node custj = optimizedSolution.augNodes[j];

                            if (cust.uid > j) //if (cust.uid > custj.uid)
                            {
                                    exp.AddTerm(1.0, x[j, cust.uid, t]);
                            } 
                            else if (cust.uid < j)
                            {
                                exp.AddTerm(1.0, x[cust.uid, j, t]);
                            }
                        }

                        PPRPmodel.AddConstr(exp == 2 * z[cust.uid, t], "con2_" + cust.uid + "^" + t);
                    }
                }

                // 3. Routes start
                for (int t = 0; t < periodsNum; t++)
                {
                    GRBLinExpr exp = 0.0;

                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];
                        exp.AddTerm(1.0, x[0, cust.uid, t]);
                    }
                    PPRPmodel.AddConstr(exp <= model.vehicles, "con3_^" + t);
                }

                // 4. Routes end
                for (int t = 0; t < periodsNum; t++)
                {
                    GRBLinExpr exp = 0.0;
                    GRBLinExpr rhs = 0.0;

                    for (int j = 0; j < optimizedSolution.customers.Count; j++)
                    {
                        Node cust = optimizedSolution.customers[j];
                        exp.AddTerm(1.0, x[0, cust.uid, t]);
                    }
                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];
                        rhs.AddTerm(1.0, x[cust.uid, allNodesNum-1, t]);
                    }

                    PPRPmodel.AddConstr(exp == rhs, "con4_^" + t);
                }

                // 5. Flow summation
                for (int t = 0; t < periodsNum; t++)
                {
                    for (int i = 0; i < optimizedSolution.augNodes.Count; i++)
                    {
                        Node custi = optimizedSolution.augNodes[i];

                        for (int j = 0; j < optimizedSolution.augNodes.Count; j++)
                        {
                            Node custj = optimizedSolution.augNodes[j];

                            if (i<j)
                            {
                                PPRPmodel.AddConstr(f[i,j,t] + f[j,i,t] == vehCap*x[i,j,t], "con5_" + i + "," + j + "^" + t);
                            }
                        }
                    }
                }

                // 6. Flow defination
                for (int t = 0; t < periodsNum; t++)
                {
                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];

                        GRBLinExpr exp = 0.0;

                        for (int j = 0; j < optimizedSolution.augNodes.Count; j++)
                        {
                            Node custj = optimizedSolution.augNodes[j];

                            if (cust.uid != custj.uid)
                            {
                                exp.AddTerm(1.0, f[cust.uid, j, t]);
                            }
                        }

                        PPRPmodel.AddConstr(exp == vehCap*z[cust.uid,t] - q[cust.uid, t], "con6_" + i + "^" + t);
                    }
                }

                // 7. Flow and deliveries
                for (int t = 0; t < periodsNum; t++)
                {
                    GRBLinExpr exp = 0.0;
                    GRBLinExpr rhs = 0.0;
                    for (int j = 0; j < optimizedSolution.customers.Count; j++)
                    {
                        Node cust = optimizedSolution.customers[j];
                        exp.AddTerm(1.0, f[0, cust.uid, t]);
                    }
                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];
                        rhs.AddTerm(1.0, q[cust.uid, t]);
                    }
                    PPRPmodel.AddConstr(exp == rhs, "con7_^" + t);
                }

                // 8. End flow
                for (int t = 0; t < periodsNum; t++)
                {
                    GRBLinExpr exp = 0.0;
                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];
                        exp.AddTerm(1.0, f[cust.uid, allNodesNum - 1, t]);
                    }

                    PPRPmodel.AddConstr(exp == 0.0, "con8_^" + t);
                }

                // Production Constraints 
                // 9. Production binaries
                for (int t = 0; t < periodsNum; ++t)
                {
                    PPRPmodel.AddConstr(p[t] <= y[t]* productionCapacity, "con9a_"+t);
                }
                for (int t = 0; t < periodsNum; ++t)
                {
                    GRBLinExpr exp = 0.0;
                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];
                        for (int tt = t; tt < periodsNum; ++tt)
                        {
                            exp.AddConstant(cust.productRate[tt]);
                        }
                        if (periodicity == "periodic")
                        {
                            exp.AddConstant(cust.startingInventory);
                        }
                        //totalToBeProdQuantity += cust.startingInventory;
                    }
                    if (periodicity == "periodic")
                    {
                        exp.AddConstant(optimizedSolution.depot.startingInventory);
                    }
                    PPRPmodel.AddConstr(p[t] <= exp*y[t], "con9b_" + t);
                }
                if (model.input.type == PRP_DATASET_VERSION.BOUDIA_FMT)
                {
                    PPRPmodel.AddConstr(p[0] == 0, "boudia_no_prod");
                    PPRPmodel.AddConstr(y[0] == 0, "boudia_no_prod");
                }

              
                // 10. Connection with previous production periods
                // Could be stricter for non periodic. This is not useful
                /*
                for (int t = 1; t < periodsNum; ++t)
                {
                    GRBLinExpr exp = 0.0;
                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];
                        for (int tt = 0; tt < periodsNum; ++tt)
                        {
                            exp.AddConstant(cust.productRate[tt]);
                        }
                    }
                    for (int tt = 0; tt < t - 1; ++tt)
                    {
                        exp.AddTerm(-1.0, p[tt]);
                    }
                    PPRPmodel.AddConstr(p[t] <= exp, "con10_" + t);
                }
                */

                // Inventory Flow Constraints 
                // 13. Depot inventory flow 
                for (int t = 0; t < periodsNum; ++t)
                {
                    GRBLinExpr lhs = 0.0;

                    lhs.AddTerm(1.0, I[0,t]);
                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];

                        lhs.AddTerm(1.0, q[cust.uid, t]);
                    }
                    if (t == 0)
                        PPRPmodel.AddConstr(lhs == optimizedSolution.depot.startingInventory + p[t], "depot_inv_" + t);
                    else
                        PPRPmodel.AddConstr(lhs == I[0,t - 1] + p[t], "depot_inv_" + t);
                }

                // 14. Customer inventory flow 
                for (int t = 0; t < periodsNum; t++)
                {
                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];
                        GRBLinExpr lhs = 0.0;
                        lhs.AddTerm(1.0, I[cust.uid, t]);

                        if (t == 0)
                        {
                            PPRPmodel.AddConstr(lhs == cust.startingInventory + q[cust.uid, t] - cust.productRate[t], "cust_inv" + cust.uid + "^" + t);
                        }
                        else
                        {
                            PPRPmodel.AddConstr(lhs == I[cust.uid, t - 1] + q[cust.uid, t] - cust.productRate[t], "cust_inv" + cust.uid + "^" + t);
                        }
                    }
                }

                // ML Policy
                // 15. ML policy (relaxed UB)
                for (int t = 0; t < periodsNum; t++)
                {
                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];

                        if (t == 0)
                            PPRPmodel.AddConstr(q[cust.uid, t] <= cust.stockMaximumLevel + cust.productRate[t] - cust.startingInventory, "con15_" + i + " ^ " + t);
                        else
                            PPRPmodel.AddConstr(q[cust.uid, t] <= cust.stockMaximumLevel + cust.productRate[t] - I[cust.uid,t-1], "con15_" + i + "^" + t);
                    }
                }

                // Periodicity
                // 16. Same initial and end inventory (cycle)       
                if (periodicity == "periodic")
                {
                    for (int i = 0; i < nodesNum; i++) // check for clone depot inventory
                    {
                        Node node = optimizedSolution.nodes[i];
                        PPRPmodel.AddConstr(I[node.uid, periodsNum - 1] == node.startingInventory, "con16_" + i);
                    }
                }

                // 17. non zero visits)  
                /*
                for (int t = 0; t < periodsNum; t++)
                {
                    for (int i = 0; i < custNum; i++) // check for clone depot inventory
                    {
                        Node cust = optimizedSolution.customers[i];
                        PPRPmodel.AddConstr(q[cust.uid, t] >=  z[cust.uid, t], "con17_" + i + "^" + t);
                    }
                }
                */

                // ============================================================================================================================================================//
                // Valid inequalities
                if (addValidInequalities)
                {
                    // Desaulnier Subdeliveries inequalities
                    List<int>[,] Tminus = new List<int>[nodesNum, periodsNum]; //upper bound of delivered quantities to customer

                    // initialization
                    for (int t = 0; t < periodsNum; ++t)
                    {
                        Tminus[0, t] = new List<int> { };
                        for (int i = 0; i < optimizedSolution.customers.Count; i++)
                        {
                            Node cust = optimizedSolution.customers[i];
                            Tminus[cust.uid, t] = new List<int> { };
                        }
                    }

                    //populate sets
                    
                    for (int i = 0; i < optimizedSolution.customers.Count; i++)
                    {
                        Node cust = optimizedSolution.customers[i];
                        int startPeriod = findCusStartPeriodVI(cust, model);

                        for (int t = startPeriod; t < periodsNum; t++)
                        {
                            double totalDemand = 0.0;
                            if (periodicity == "periodic")
                            {
                                if (t == periodsNum - 1) //last period may take into account the inventory as well
                                {
                                    totalDemand += cust.startingInventory;
                                }
                            }
                            for (int tt = t; tt > -1; tt--)
                            {
                                totalDemand += cust.productRate[tt];
                                if (totalDemand <= cust.stockMaximumLevel)
                                {
                                    Tminus[cust.uid, t].Add(tt);
                                }
                                else
                                {
                                    Tminus[cust.uid, t].Add(tt);
                                    break;
                                }
                            }
                        }
                    }
                    

                    //add constraints
                    
                    for (int t = 0; t < periodsNum; ++t)
                    {
                        for (int i = 0; i < optimizedSolution.customers.Count; i++)
                        {
                            Node cust = optimizedSolution.customers[i];
                            //Console.WriteLine("cust {3}: starting={0}, demand={1}, max level={2}", cust.startingInventory, cust.productRate[1], cust.stockMaximumLevel, cust.uid);

                            if (Tminus[cust.uid, t].Count > 0)
                            {
                                GRBLinExpr exp = 0.0;
                                foreach (int elem in Tminus[cust.uid, t])
                                {
                                    exp.AddTerm(1.0, z[cust.uid, elem]);
                                    //Console.WriteLine("cust {0} {1}:+{2}",cust.uid,t,elem);
                                }
                                PPRPmodel.AddConstr(exp >= 1, "vi44_" + cust.uid + "^" + t);
                            }
                        }
                    }
                    
                    
                    // 1. Visit and arcs depot
                    for (int t = 0; t < periodsNum; t++)
                    {
                        for (int i = 0; i < optimizedSolution.customers.Count; i++)
                        {
                            Node cust = optimizedSolution.customers[i];

                            PPRPmodel.AddConstr(x[0, cust.uid, t] <= 1 * z[cust.uid, t], "vi26_" + cust.uid + "^" + t); // TODO: or 2
                        }
                    }

                    // 2-3. Visits and arcs
                    for (int t = 0; t < periodsNum; t++)
                    {
                        for (int i = 0; i < optimizedSolution.customers.Count; i++)
                        {
                            Node custi = optimizedSolution.customers[i];

                            for (int j = 0; j < optimizedSolution.customers.Count; j++)
                            {
                                Node custj = optimizedSolution.customers[j];

                                if (custi.uid < custj.uid)
                                {
                                    PPRPmodel.AddConstr(x[custi.uid, custj.uid, t] <= z[custi.uid, t], "vi27_" + custi.uid + "," + custj.uid + "^" + t);
                                    PPRPmodel.AddConstr(x[custi.uid, custj.uid, t] <= z[custj.uid, t], "vi28_" + custi.uid + "," + custj.uid + "^" + t);
                                }
                            }
                        }
                    }
                    
                    // 4-5. Flows and arcs
                    for (int t = 0; t < periodsNum; t++)
                    {
                        for (int i = 0; i < optimizedSolution.customers.Count; i++)
                        {
                            Node custi = optimizedSolution.customers[i];

                            for (int j = 0; j < optimizedSolution.customers.Count; j++)
                            {
                                Node custj = optimizedSolution.customers[j];

                                if (custi.uid < custj.uid)
                                {
                                    PPRPmodel.AddConstr(f[custi.uid, custj.uid, t] <= x[custi.uid, custj.uid, t] * vehCap, "vi29_" + custi.uid + "," + custj.uid + "^" + t);
                                    PPRPmodel.AddConstr(f[custj.uid, custi.uid, t] <= x[custi.uid, custj.uid, t] * vehCap, "vi30_" + custi.uid + "," + custj.uid + "^" + t);
                                }
                            }
                        }
                    }
                    
                    // 6. Delivered quantity and visits
                    //periodicity is captured in the UBq calculation
                    for (int t = 0; t < periodsNum; ++t)
                    {
                        for (int i = 0; i < optimizedSolution.customers.Count; i++)
                        {
                            Node custi = optimizedSolution.customers[i];
                            PPRPmodel.AddConstr(q[custi.uid, t] <= UBq[custi.uid, t] * z[custi.uid, t], "vi31_" + custi.uid + "^" + t);
                        }
                    }    
                    
                    
                    // 7. Delivered quantity and visits with inventories
                    /*                    
                    for (int t = 0; t < periodsNum; ++t)
                    {
                        for (int i = 0; i < optimizedSolution.customers.Count; i++)
                        {
                            Node custi = optimizedSolution.customers[i];
                            int totalDemandUntilEnd = 0;
                            for (int tt = t; tt < periodsNum; ++tt)
                            {
                                totalDemandUntilEnd += custi.productRate[tt]; //TODO:
                            }
                            if (t == 0)
                            {
                                PPRPmodel.AddConstr(q[custi.uid, t] <= custi.stockMaximumLevel + custi.productRate[t] - custi.startingInventory, "vi32a_" + custi.uid + "^" + t);
                                if (periodicity == "periodic")
                                {
                                    PPRPmodel.AddConstr(q[custi.uid, t] <= totalDemandUntilEnd - custi.startingInventory + custi.startingInventory, "vi32b_" + custi.uid + "^" + t);
                                }
                                else
                                {
                                    PPRPmodel.AddConstr(q[custi.uid, t] <= totalDemandUntilEnd - custi.startingInventory, "vi32b_" + custi.uid + "^" + t);
                                }
                            }
                            else
                            {
                                PPRPmodel.AddConstr(q[custi.uid, t] <= custi.stockMaximumLevel + custi.productRate[t] - I[custi.uid, t - 1], "vi32a_" + custi.uid + "^" + t);
                                if (periodicity == "periodic")
                                {
                                    PPRPmodel.AddConstr(q[custi.uid, t] <= totalDemandUntilEnd - I[custi.uid, t - 1] + custi.startingInventory, "vi32b_" + custi.uid + "^" + t);
                                }
                                else
                                {
                                    PPRPmodel.AddConstr(q[custi.uid, t] <= totalDemandUntilEnd - I[custi.uid, t - 1], "vi32b_" + custi.uid + "^" + t);
                                }
                            }
                        }
                    }      
                    */
                    
                    
                    
                    // 8. Minimum remaining visits
                    for (int t = 0; t < periodsNum; ++t)
                    {
                        for (int i = 0; i < optimizedSolution.customers.Count; i++)
                        {
                            Node custi = optimizedSolution.customers[i];
                            int maxDemand = custi.productRate.Max();

                            double numerator = 0.0;
                            double denominator = (double)Math.Min(vehCap, custi.stockMaximumLevel + maxDemand);

                            GRBLinExpr exp2 = 0.0;

                            for (int tt = 0; tt < t; ++tt) //Note: t+1 according to equation
                            {
                                exp2.AddTerm(1.0, z[custi.uid, tt]);
                            }

                            for (int tt = 0; tt < t; ++tt) //Note: t+1 according to equation
                            {
                                numerator += custi.productRate[tt];
                            }
                            numerator -= custi.startingInventory;

                            double fraction = Math.Ceiling(numerator / denominator);
                            PPRPmodel.AddConstr(exp2 >= fraction, "vi33" + custi.uid + "^" + t);
                        }
                    }
                    
                    // 9. Minimum remaining visits
                    for (int t1 = 0; t1 < periodsNum; ++t1)
                    {
                        for (int t2 = 0; t2 < periodsNum; ++t2)
                        {
                            if (t1 <= t2)
                            {
                                for (int i = 0; i < optimizedSolution.customers.Count; i++)
                                {
                                    Node custi = optimizedSolution.customers[i];
                                    int maxDemand = custi.productRate.Max();

                                    double numerator = 0.0;
                                    double denominator = (double)Math.Min(vehCap, custi.stockMaximumLevel + maxDemand);

                                    GRBLinExpr exp2 = 0.0;

                                    for (int tt = t1; tt < t2; ++tt) //Note: t+1 according to equation
                                    {
                                        exp2.AddTerm(1.0, z[custi.uid, tt]);
                                        numerator += custi.productRate[tt];

                                    }

                                    numerator -= custi.stockMaximumLevel;

                                    double fraction = Math.Ceiling(numerator / denominator);
                                    PPRPmodel.AddConstr(exp2 >= fraction, "vi34" + custi.uid + "^" + t1 + "^" + t2);
                                }
                            }
                        }
                    }
                    
                    // 10. Minimum remaining visits
                    for (int t1 = 0; t1 < periodsNum; ++t1)
                    {
                        for (int t2 = 0; t2 < periodsNum; ++t2)
                        {
                            if (t1 <= t2)
                            {
                                for (int i = 0; i < optimizedSolution.customers.Count; i++)
                                {
                                    Node custi = optimizedSolution.customers[i];
                                    int maxDemand = custi.productRate.Max();

                                    GRBLinExpr exp = 0.0;
                                    double denominator = (double)Math.Min(vehCap, custi.stockMaximumLevel + maxDemand);

                                    GRBLinExpr exp2 = 0.0;

                                    for (int tt = t1; tt < t2; ++tt) //Note: t+1 according to equation
                                    {
                                        exp2.AddTerm(1.0, z[custi.uid, tt]);
                                        exp.AddConstant(custi.productRate[tt]);
                                    }

                                    if (t1 == 0)
                                    {
                                        exp.AddConstant(-custi.startingInventory);
                                    }
                                    else
                                    {
                                        exp.AddTerm(-1.0, I[custi.uid, t1 - 1]);
                                    }
                                    PPRPmodel.AddConstr(denominator * exp2 >= exp, "vi35" + custi.uid + "^" + t1 + "^" + t2);
                                }
                            }
                        }
                    }

                    // 11. Minimum remaining visits
                    for (int t1 = 0; t1 < periodsNum; ++t1)
                    {
                        for (int t2 = 0; t2 < periodsNum; ++t2)
                        {
                            if (t1 <= t2)
                            {
                                for (int i = 0; i < optimizedSolution.customers.Count; i++)
                                {
                                    Node custi = optimizedSolution.customers[i];
                                    int maxDemand = custi.productRate.Max();

                                    GRBLinExpr numerator = 0.0;
                                    double denominator = 0.0;

                                    GRBLinExpr exp2 = 0.0;

                                    for (int tt = t1; tt < t2; ++tt) //Note: t+1 according to equation
                                    {
                                        exp2.AddTerm(1.0, z[custi.uid, tt]);
                                        numerator.AddConstant(custi.productRate[tt]);
                                        denominator += custi.productRate[tt];
                                    }

                                    if (t1 == 0)
                                    {
                                        numerator.AddConstant(-custi.startingInventory);
                                    }
                                    else
                                    {
                                        numerator.AddTerm(-1.0, I[custi.uid, t1 - 1]);
                                    }
                                    PPRPmodel.AddConstr(denominator * exp2 >= numerator, "vi36" + custi.uid + "^" + t1 + "^" + t2);
                                }
                            }
                        }
                    }
                    
                   
                    // 12. Conditional flow  bounds 
                    for (int t = 0; t < periodsNum; t++)
                    {
                        for (int i = 0; i < optimizedSolution.customers.Count; i++)
                        {
                            Node custi = optimizedSolution.customers[i];

                            for (int j = 0; j < optimizedSolution.customers.Count; j++)
                            {
                                Node custj = optimizedSolution.customers[j];

                                if (custi.uid < custj.uid)
                                {
                                    if (t == 0)
                                    {
                                        PPRPmodel.AddConstr(f[custi.uid, custj.uid, t] >= custj.productRate[t] * x[custi.uid, custj.uid, t] - custj.startingInventory, "con37a_" + custi.uid + "," + custj.uid + "^" + t);
                                        PPRPmodel.AddConstr(f[custj.uid, custi.uid, t] >= custi.productRate[t] * x[custi.uid, custj.uid, t] - custi.startingInventory, "con37b_" + custi.uid + "," + custj.uid + "^" + t);
                                    }
                                    else
                                    {
                                        PPRPmodel.AddConstr(f[custi.uid, custj.uid, t] >= custj.productRate[t] * x[custi.uid, custj.uid, t] - I[custj.uid, t-1], "con37a_" + custi.uid + "," + custj.uid + "^" + t);
                                        PPRPmodel.AddConstr(f[custj.uid, custi.uid, t] >= custi.productRate[t] * x[custi.uid, custj.uid, t] - I[custi.uid, t-1], "con37b_" + custi.uid + "," + custj.uid + "^" + t);
                                    }
                                }
                            }
                        }
                    }
                    
                   
                    
                    // 13. Conditional flow  bounds 
                    for (int t = 0; t < periodsNum; t++)
                    {
                        GRBLinExpr exp = 0.0;

                        for (int i = 0; i < optimizedSolution.customers.Count; i++)
                        {
                            Node custi = optimizedSolution.customers[i];
                            exp.AddConstant(custi.productRate[t]);
                            if (t == 0)
                            {
                                exp.AddConstant(-custi.startingInventory);
                            }
                            else
                            {
                                exp.AddTerm(-1.0, I[custi.uid, t-1]);
                            }
                        }
                        if (t == 0)
                            PPRPmodel.AddConstr(y[t] * optimizedSolution.depot.productionCapacity >= exp - optimizedSolution.depot.startingInventory, "con39_" + "^" + t);
                        else
                            PPRPmodel.AddConstr(y[t] * optimizedSolution.depot.productionCapacity >= exp - I[0, t - 1], "con39_" + "^" + t);
                    }
                    
                    
                    // WRONG only works with zero depot inventory 
                    // 14. Conditional flow  bounds 
                    for (int t = 1; t < periodsNum; ++t)
                    {
                        double numerator = 0.0;
                        double denominator = optimizedSolution.depot.productionCapacity;

                        GRBLinExpr exp = 0.0;
                        for (int tt = 0; tt < t; ++tt) //Note: t+1 
                        {
                            exp.AddTerm(1.0, y[tt]);
                        }

                        for (int i = 0; i < optimizedSolution.customers.Count; i++)
                        {
                            Node custi = optimizedSolution.customers[i];

                            for (int tt = 0; tt < t; ++tt) //Note: t+1 
                            {
                                numerator += custi.productRate[tt];
                            }

                            numerator -= custi.startingInventory;
                        }
                        numerator -= optimizedSolution.depot.startingInventory;

                        double fraction = Math.Ceiling(numerator / denominator);

                        PPRPmodel.AddConstr(exp >= fraction, "vi40_" + "^" + t);
                    }
                    
                    // 15. Conditional flow  bounds 
                    for (int t1 = 0; t1 < periodsNum; ++t1)
                    {
                        for (int t2 = 0; t2 < periodsNum; ++t2)
                        {
                            if (t1 <= t2)
                            {
                                double numerator = 0.0;
                                double denominator = optimizedSolution.depot.productionCapacity;

                                GRBLinExpr exp = 0.0;
                                for (int tt = t1; tt < t2; ++tt) //Note: t+1 
                                {
                                    exp.AddTerm(1.0, y[tt]);
                                }

                                for (int i = 0; i < optimizedSolution.customers.Count; i++)
                                {
                                    Node custi = optimizedSolution.customers[i];

                                    for (int tt = t1; tt < t2; ++tt) //Note: t+1 
                                    {
                                        numerator += custi.productRate[tt];
                                    }

                                    numerator -= custi.stockMaximumLevel;
                                }
                                numerator -= optimizedSolution.depot.stockMaximumLevel;

                                double fraction = Math.Ceiling(numerator / denominator);

                                PPRPmodel.AddConstr(exp >= fraction, "vi41_" + "^" + t1 + "^" + t2);
                            }
                        }
                    }
                    

                    // 16. Conditional flow  bounds 
                    for (int t1 = 0; t1 < periodsNum; ++t1)
                    {
                        for (int t2 = 0; t2 < periodsNum; ++t2)
                        {
                            if (t1 <= t2)
                            {
                                GRBLinExpr numerator = 0.0;
                                double denominator = optimizedSolution.depot.productionCapacity;

                                GRBLinExpr exp = 0.0;
                                for (int tt = t1; tt < t2; ++tt) //Note: t+1 
                                {
                                    exp.AddTerm(1.0, y[tt]);
                                }

                                for (int i = 0; i < optimizedSolution.customers.Count; i++)
                                {
                                    Node custi = optimizedSolution.customers[i];

                                    for (int tt = t1; tt < t2; ++tt) //Note: t+1 
                                    {
                                        numerator.AddConstant(custi.productRate[tt]);
                                    }
                                    if (t1 == 0)
                                    {
                                        numerator.AddConstant(-custi.stockMaximumLevel);

                                    }
                                    else
                                    {
                                        numerator.AddTerm(-1.0, I[custi.uid, t1 - 1]);
                                    }

                                }
                                if (t1 == 0)
                                {
                                    numerator.AddConstant(-optimizedSolution.depot.startingInventory);
                                }
                                else
                                {
                                    numerator.AddTerm(-1.0, I[0, t1 - 1]);
                                }
                                // try this ERROR
                                //numerator -= optimizedSolution.depot.stockMaximumLevel;

                                PPRPmodel.AddConstr(denominator * exp >= numerator, "vi42_" + "^" + t1 + "^" + t2);
                            }
                        }
                    }
                    
                
                }


                // ===========================================Fix binary variables (when rerunning for integrality)===============================
                //bool varFixing = true;
                if (x_fix != null)
                {
                    for (int t = 0; t < periodsNum; ++t)
                    {
                        for (int i = 0; i < allNodesNum; i++)
                        {
                            Node from = optimizedSolution.augNodes[i];
                            for (int j = 0; j < allNodesNum; j++)
                            {
                                Node to = optimizedSolution.augNodes[j];
                                if (i != j) //symmetric?
                                {
                                    if (i < j)
                                    {
                                        if (GlobalUtils.IsEqual(x_fix[i,j,t].X, 1.0, 1e-3))
                                        {
                                            x[i, j, t].LB = 1.0;
                                            x[i, j, t].UB = 1.0;
                                        }
                                        else
                                        {
                                            x[i, j, t].LB = 0.0;
                                            x[i, j, t].UB = 0.0;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                if (y_fix != null)
                {
                    for (int t = 0; t < periodsNum; ++t)
                    {
                        if (GlobalUtils.IsEqual(y_fix[t].X, 1.0, 1e-3))
                        {
                            y[t].LB = 1.0;
                            y[t].UB = 1.0;
                        }
                        else
                        {
                            y[t].LB = 0.0;
                            y[t].UB = 0.0;
                        }
                    }
                }

                if (z_fix != null)
                {
                    for (int t = 0; t < periodsNum; ++t)
                    {
                        for (int i = 0; i < nodesNum; i++)
                        {
                            if (i > 0)
                            {
                                if (GlobalUtils.IsEqual(z_fix[i, t].X, 1.0, 1e-3))
                                {
                                    z[i, t].LB = 1.0;
                                    z[i, t].UB = 1.0;
                                }
                                else
                                {
                                    z[i, t].LB = 0.0;
                                    z[i, t].UB = 0.0;
                                }
                            }
                        }
                    }
                }

                // ==================================================Optimize====================================================================
                PPRPmodel.Optimize();
                int optimstatus = PPRPmodel.Status;
                switch (PPRPmodel.Status)
                {
                    case GRB.Status.OPTIMAL:
                    case GRB.Status.TIME_LIMIT:
                        {
                            double objval = PPRPmodel.ObjVal;

                            dict["ObjVal"] = PPRPmodel.ObjVal;
                            if (forcedIntegrality) // if forced integrality, override the dict
                            {
                                dict["Status"] = dictRes["Status"];
                                dict["Runtime"] = dictRes["Runtime"];
                                dict["ObjBound"] = dictRes["ObjBound"];
                                dict["NodeCount"] = dictRes["NodeCount"];
                                dict["NumBinVars"] = dictRes["NumBinVars"];
                                dict["NumVars"] = dictRes["NumVars"];
                                dict["NumConstrs"] = dictRes["NumConstrs"];
                                dict["MIPGap"] = dictRes["MIPGap"];
                            } else
                            {
                                dict["Status"] = PPRPmodel.Status;
                                dict["Runtime"] = PPRPmodel.Runtime;
                                dict["ObjBound"] = PPRPmodel.ObjBound;
                                dict["NodeCount"] = PPRPmodel.NodeCount;
                                dict["NumBinVars"] = PPRPmodel.NumBinVars;
                                dict["NumVars"] = PPRPmodel.NumVars;
                                dict["NumConstrs"] = PPRPmodel.NumConstrs;
                                dict["MIPGap"] = PPRPmodel.MIPGap;
                                //dict["ObjVal"] = PPRPmodel.ObjVal;
                            }

                            bool integrality = checkVariableIntegrality(optimizedSolution, model, PPRPmodel, x, y, z, f, p, q, I);

                            if (!integrality) // Imposed rounding error
                            {
                                if (forcedIntegrality) // implied failure 
                                {
                                    throw new Exception("Integrality constraints were not respected.");
                                }
                                bool forceIntegralityVar = true;

                                (optimizedSolutionFixed, dictFixed) = SolvePPRP(model, forceIntegralityVar, x, y, z, dict);
                                return (optimizedSolutionFixed, dictFixed);
                                break;
                            }

                            // Update depot 
                            for (int t = 0; t < periodsNum; t++)
                            {
                                optimizedSolution.depot.deliveredQuantities[t] = 0;
                                optimizedSolution.depot.productionRates[t] = (int)Math.Round(p[t].X); //ATTENTION not Int
                                optimizedSolution.depot.open[t] = false;
                                if (GlobalUtils.IsEqual(y[t].X, 1.0, 1e-3))
                                {
                                    optimizedSolution.depot.open[t] = true;
                                }
                            }

                            //report 
                            /*
                            String prodSch = "\nProduction schedule: ";
                            String quant = "\nProduction quantities: ";
                            for (int t = 0; t < periodsNum; ++t)
                            {
                                prodSch += ((int)Math.Round(y[t].X));
                                prodSch += "_";
                                quant += ((int)Math.Round(p[t].X));
                                quant += "_";
                            }
                            Console.WriteLine(prodSch);
                            Console.WriteLine(quant);

                            for (int t = 0; t < periodsNum; t++)
                            {
                                for (int i = 0; i < allNodesNum; i++)
                                {
                                    Node from = optimizedSolution.augNodes[i];
                                    for (int j = 0; j < allNodesNum; j++)
                                    {
                                        Node to = optimizedSolution.augNodes[j];
                                        if (i != j) //symmetric?
                                        {
                                            if (i < j)
                                            {
                                                if (GlobalUtils.IsEqual(x[i,j,t].X, 1.0, 1e-3))
                                                    Console.WriteLine("x_{0}-{1}^{2}",i, j,t); 
                                            }
                                            //if (GlobalUtils.IsEqual(f[i, j, t].X, 1.0, 1e-3))
                                                //Console.WriteLine("f_{0}-{1}^{2}", from.uid, to.uid, t);
                                        }
                                    }
                                }
                            }

                            Console.WriteLine("\n\nVisits and quantities");

                            for (int t = 0; t < periodsNum; t++)
                            {
                                for (int i = 1; i < nodesNum; i++)
                                {
                                    Node cust = optimizedSolution.augNodes[i];
                                    if (GlobalUtils.IsEqual(z[i, t].X, 1.0, 1e-3))
                                    {
                                        //Console.WriteLine("z_{0}^{1}", cust.uid, t);
                                        Console.WriteLine("q_{0}^{1}={2}", cust.uid, t, (int)Math.Round(q[cust.uid,t].X));

                                    }
                                }
                            }
                            */

                            // routes parsing 
                            List<List<Node>>[] periodRouteArray = new List<List<Node>>[periodsNum];
                            for (int t = 0; t < periodsNum; t++)
                            {
                                periodRouteArray[t] = findPeriodRoutes(model, PPRPmodel, x, z, q, optimizedSolution, t);
                            }

                            for (int t = 0; t < periodsNum; t++)
                            {
                                Period pr = optimizedSolution.periods[t];

                                int rt_idx = 0;

                                foreach (List<Node> list in periodRouteArray[t])
                                {
                                    Route rt = pr.periodRoutes[rt_idx];

                                    //Console.Write("\nPeriod {0} Route: ", t);
                                    foreach (Node node in list)
                                    {
                                        rt.nodes.Add(node);

                                        if (node.uid != 0)
                                        {
                                            node.visitSchedule[t] = true;

                                            CustDelivery cd = node.horizonDeliveryServices[t];
                                            node.deliveredQuantities[t] = (int)Math.Round(q[node.uid, t].X); //ATTENTION not Int
                                            if ((int)Math.Round(q[node.uid, t].X) == 00) //ATTENTION not Int
                                            {
                                                GlobalUtils.writeToConsole(" Customer {0} has zero quantity delivery on period {1}:" +
                                                    "Possible shortcut due to triangular inequality failure from roundings.", node.uid, t);
                                            }
                                            cd.quantity = node.deliveredQuantities[t];
                                            optimizedSolution.depot.deliveredQuantities[t] -= node.deliveredQuantities[t]; // OR +=;

                                            cd.route = rt;
                                            cd.route.load += cd.quantity;

                                        }
                                        //Console.Write("{0}>", node.uid);
                                    }
                                    rt.SetLoadAndCostLists(t, model);
                                    rt.calculateRoutingCost(model);
                                    rt_idx++;
                                }
                            }

                            // Reporting: Route printing
                            for (int t = 0; t < periodsNum; t++)
                            {
                                Period pr = optimizedSolution.periods[t];

                                int rt_idx = 0;

                                foreach (List<Node> list in periodRouteArray[t])
                                {
                                    Route rt = pr.periodRoutes[rt_idx];

                                    Console.Write("\nPeriod {0} Route: ", t);
                                    foreach (Node node in list)
                                    {
                                        Console.Write("{0}>", node.uid);
                                    }
                                }
                            }


                            // Update inventory levels
                            for (int i = 0; i < optimizedSolution.customers.Count; i++)
                            {
                                Node cust = optimizedSolution.customers[i];
                                cust.CalculateInventoryLevels();
                            }
                            optimizedSolution.depot.CalculateInventoryLevels();

                            // Testing calculate objectives from MIP variables
                            double routingCostFromMIP = 0.0;
                            double holdingCostFromMIP = 0.0;
                            double holdingCostFromMIP2 = 0.0;
                            double totalUnitProductionCostFromMIP = 0.0;
                            double setupProductionCostFromMIP = 0.0;
                            double totalObjectiveFromMIP = 0.0;

                            for (int t = 0; t < periodsNum; ++t)
                            {
                                if (GlobalUtils.IsEqual(y[t].X, 1.0, 1e-3))
                                {
                                    setupProductionCostFromMIP += optimizedSolution.depot.productionSetupCost;
                                }
                                totalUnitProductionCostFromMIP += (int)Math.Round(p[t].X) * optimizedSolution.depot.unitProductionCost;

                                for (int i = 0; i < nodesNum; i++)
                                {
                                    Node node = optimizedSolution.nodes[i];
                                    holdingCostFromMIP += node.unitHoldingCost * (int)Math.Round(I[i, t].X); //ATTENTION not Int
                                    holdingCostFromMIP2 += node.unitHoldingCost * (double)Math.Round(I[i, t].X, 2);
                                }
                                Console.WriteLine("");
                                if (holdingCostFromMIP != holdingCostFromMIP2)
                                {
                                    Console.WriteLine("Possible rounding errors");
                                }

                                for (int i = 0; i < allNodesNum; i++)
                                {
                                    Node from = optimizedSolution.augNodes[i];
                                    for (int j = 0; j < allNodesNum; j++)
                                    {
                                        Node to = optimizedSolution.augNodes[j];
                                        if (i < j) //symmetric?
                                        {
                                            if (GlobalUtils.IsEqual(x[i, j, t].X, 1.0, 1e-3))
                                            {
                                                //routingCostFromMIP += model.distMatrix[from.uid, to.uid];
                                                routingCostFromMIP += model.distMatrix[from.uid, to.uid];
                                            }
                                        }
                                    }
                                }
                            }

                            totalObjectiveFromMIP = routingCostFromMIP + holdingCostFromMIP + totalUnitProductionCostFromMIP
                                + setupProductionCostFromMIP;

                            //Set solution objective
                            optimizedSolution.routingCost = Solution.EvaluateRoutingObjectivefromScratch(optimizedSolution);
                            optimizedSolution.holdingCost = Solution.EvaluateInventoryObjectivefromScratch(optimizedSolution);
                            optimizedSolution.totalUnitProductionCost = Solution.EvaluateUnitProductionObjectivefromScratch(optimizedSolution);
                            optimizedSolution.setupProductionCost = Solution.EvaluateSetupProductionObjectivefromScratch(optimizedSolution);
                            optimizedSolution.totalObjective = optimizedSolution.routingCost + optimizedSolution.holdingCost + optimizedSolution.totalUnitProductionCost + optimizedSolution.setupProductionCost;

                            if (forcedIntegrality) // if forced integrality, override the dict
                            {
                                dict["routingCostFromMIP"] = routingCostFromMIP;
                                dict["holdingCostFromMIP"] = holdingCostFromMIP;
                                dict["totalUnitProductionCostFromMIP"] = totalUnitProductionCostFromMIP;
                                dict["setupProductionCostFromMIP"] = setupProductionCostFromMIP;
                                dict["totalObjectiveFromMIP"] = totalObjectiveFromMIP;
                            }
                            else
                            {
                                dict["routingCostFromMIP"] = routingCostFromMIP;
                                dict["holdingCostFromMIP"] = holdingCostFromMIP;
                                dict["totalUnitProductionCostFromMIP"] = totalUnitProductionCostFromMIP;
                                dict["setupProductionCostFromMIP"] = setupProductionCostFromMIP;
                                dict["totalObjectiveFromMIP"] = totalObjectiveFromMIP;
                            }

                            GlobalUtils.writeToConsole("\nRouting {0} == {1}", routingCostFromMIP, optimizedSolution.routingCost);
                            GlobalUtils.writeToConsole("Holding {0} == {1}", holdingCostFromMIP, optimizedSolution.holdingCost);
                            GlobalUtils.writeToConsole("Unit production {0} == {1}", totalUnitProductionCostFromMIP, optimizedSolution.totalUnitProductionCost);
                            GlobalUtils.writeToConsole("Setup production {0} == {1}", setupProductionCostFromMIP, optimizedSolution.setupProductionCost);
                            GlobalUtils.writeToConsole("Total {0} == {1}", totalObjectiveFromMIP, optimizedSolution.totalObjective);
                            
                            if (MathematicalProgramming.exactParams.periodicity == "periodic")
                            {
                                optimizedSolution.status = optimizedSolution.TestEverythingFromScratchPeriodic();
                            } 
                            else if (MathematicalProgramming.exactParams.periodicity == "cyclic")
                            {
                                optimizedSolution.status = optimizedSolution.TestEverythingFromScratchCyclic();
                            }
                            else
                            {
                                optimizedSolution.status = optimizedSolution.TestEverythingFromScratch();
                            }

                            //TSP_GRB_Solver.SolveTSP(optimizedSolution);

                            break;
                        }
                    case GRB.Status.INFEASIBLE:
                        {
                            Console.WriteLine("Model is infeasible");
                            // it could call itself with increased number of total insertions
                            //runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemoval(ref sol);


                            // compute and write out IIS
                            PPRPmodel.ComputeIIS();
                            PPRPmodel.Write(model.instanceName+"_model.ilp");
                            return (optimizedSolution, dict);
                        }
                    case GRB.Status.UNBOUNDED:
                        {
                            Console.WriteLine("Model is unbounded");
                            return (optimizedSolution, dict);
                        }
                    default:
                        {
                            Console.WriteLine("Optimization was stopped with status = " + optimstatus);
                            return (optimizedSolution, dict);
                        }

                }

                // Dispose of model
                PPRPmodel.Dispose();
                //gurobiEnv.Dispose();
            }
            catch (GRBException e)
            {
                Console.WriteLine("Error code: " + e.ErrorCode + ". " + e.Message);
            }
            if (optimizedSolutionFixed != null)
            {
                return (optimizedSolutionFixed, dictFixed);
            }
            return (optimizedSolution, dict);
        }


        private static bool checkVariableIntegrality(Solution sol, PRP model, GRBModel PPRPmodel, GRBVar[,,] x, GRBVar[] y, GRBVar[,] z, GRBVar[,,] f, GRBVar[] p, GRBVar[,] q, GRBVar[,] I)
        {
            bool integrality = true;
            int nodesNum = model.input.nodes.Count;
            int allNodesNum = model.input.nodes.Count + 1;

            for (int t = 0; t < model.input.horizonDays; ++t)
            {
                if (!(GlobalUtils.IsEqual(y[t].X, 1.0, 1e-3) || GlobalUtils.IsEqual(y[t].X, 0.0, 1e-3)))
                {
                    Console.WriteLine("ERROR: Variable y integrity value error: y[{0}]={1}", t, y[t].X);
                    integrality = false;
                }
                if (Math.Abs((int)Math.Round(p[t].X)- p[t].X) > 0.001)
                {
                    Console.WriteLine("ERROR: Variable p integrity value error: p[{0}]={1}", t, p[t].X);
                    integrality = false;
                }  

                for (int i = 0; i < nodesNum; i++)
                {
                    if (Math.Abs((int)Math.Round(I[i,t].X) - I[i,t].X) > 0.001)
                    {
                        Console.WriteLine("ERROR: Variable Ι integrity value error: Ι[{0}^{1}]={2}", i, t, I[i,t].X);
                        integrality = false;
                    }
                    if (i > 0) // customer
                    {
                        if (!(GlobalUtils.IsEqual(z[i,t].X, 1.0, 1e-3) || GlobalUtils.IsEqual(z[i,t].X, 0.0, 1e-3)))
                        {
                            Console.WriteLine("ERROR: Variable z integrity value error: z[{0}^{1}]={2}", i,t, z[i,t].X);
                            integrality = false;
                        }
                        if (Math.Abs((int)Math.Round(q[i, t].X) - q[i, t].X) > 0.001)
                        {
                            Console.WriteLine("ERROR: Variable q integrity value error: q[{0}^{1}]={2}", i, t, q[i, t].X);
                            integrality = false;
                        }
                    }
                }

                for (int i = 0; i < allNodesNum; i++)
                {
                    Node from = sol.augNodes[i];
                    for (int j = 0; j < allNodesNum; j++)
                    {
                        Node to = sol.augNodes[j];
                        if (i != j) //symmetric?
                        {
                            if (i < j)
                            {
                                if (!(GlobalUtils.IsEqual(x[i, j, t].X, 1.0, 1e-3) || GlobalUtils.IsEqual(x[i, j, t].X, 0.0, 1e-3)))
                                {
                                    Console.WriteLine("ERROR: Variable x integrity value error: x[{0},{1}^{2}]={3}", i, j, t, x[i, j, t].X);
                                    integrality = false;
                                }
                            }
                            if (Math.Abs((int)Math.Round(f[i, j,t].X) - f[i,j, t].X) > 0.001)
                            {
                                Console.WriteLine("ERROR: Variable f integrity value error: f[{0},{1}^{2}]={3}", i, j, t, f[i, j,t].X);
                                integrality = false;
                            }
                        }
                    }
                }
            }

            return integrality;
        }

        private static List<List<Node>> findPeriodRoutes(PRP model, GRBModel PRPmodel, GRBVar[,,] x, GRBVar[,] z, GRBVar[,] q, Solution sol, int t)
        {
            int allNodesNum = model.input.nodes.Count + 1;
            Node depot = sol.depot;
            int custNum = model.input.customerNum;
            int cloneDepotUID = allNodesNum - 1;

            List<List<Node>> routes = new List<List<Node>>();
            List<int> seen = new List<int>();
            HashSet<int> periodVisits = new HashSet<int>();

            for (int j = 0; j < custNum; j++) // equal to K
            {
                Node node = sol.customers[j];
                if (GlobalUtils.IsEqual(z[node.uid, t].X, 1.0, 1e-3))
                    periodVisits.Add(node.uid);
            }

            for (int j = 0; j < custNum; j++) // equal to K
            {
                Node node = sol.customers[j];

                List<Node> rt = new List<Node>();

                if (GlobalUtils.IsEqual(x[depot.uid, node.uid, t].X, 1.0, 1e-3) && periodVisits.Contains(node.uid)) // route start
                {
                    Node curNode = depot;
                    Node nxtNode = node;
                    rt.Add(curNode);
                    rt.Add(nxtNode);
                    periodVisits.Remove(node.uid);

                    while ((nxtNode.uid != depot.uid) && (nxtNode.uid != cloneDepotUID))
                    {
                        for (int ii = 0; ii < allNodesNum; ii++)
                        {
                            Node jNode = sol.augNodes[ii];
                            int node1 = Math.Min(jNode.uid, nxtNode.uid);
                            int node2 = Math.Max(jNode.uid, nxtNode.uid);
                            if (jNode.uid != nxtNode.uid && jNode.uid != curNode.uid && GlobalUtils.IsEqual(x[node1, node2, t].X, 1.0, 1e-3))
                            {
                                curNode = nxtNode;
                                nxtNode = jNode;
                                periodVisits.Remove(jNode.uid);
                                rt.Add(jNode);
                                break;
                            }

                            if (jNode.uid == 0)
                            {
                                if (jNode.uid != nxtNode.uid && GlobalUtils.IsEqual(x[nxtNode.uid, cloneDepotUID, t].X, 1.0, 1e-3))
                                {
                                    curNode = nxtNode;
                                    nxtNode = jNode;
                                    periodVisits.Remove(jNode.uid);
                                    rt.Add(jNode);
                                    break;
                                }
                            }
                        }
                    }
                    routes.Add(rt);
                }
            }

            if(periodVisits.Count>0)
            {
                for (int j = 0; j < custNum; j++) // equal to K
                {
                    Node node = sol.customers[j];

                    List<Node> rt = new List<Node>();

                    if (GlobalUtils.IsEqual(x[node.uid, cloneDepotUID, t].X, 1.0, 1e-3) && periodVisits.Contains(node.uid)) // route start
                    {
                        Node curNode = depot;
                        Node nxtNode = node;
                        periodVisits.Remove(node.uid);
                        rt.Add(curNode);
                        rt.Add(nxtNode);

                        while ((nxtNode.uid != depot.uid) && (nxtNode.uid != cloneDepotUID))
                        {
                            for (int ii = allNodesNum-2; ii > -1; ii--)
                            {
                                Node jNode = sol.augNodes[ii];
                                int node1 = Math.Min(jNode.uid, nxtNode.uid);
                                int node2 = Math.Max(jNode.uid, nxtNode.uid);
                                if (jNode.uid != nxtNode.uid && jNode.uid != curNode.uid && GlobalUtils.IsEqual(x[node1, node2, t].X, 1.0, 1e-3))
                                {
                                    curNode = nxtNode;
                                    nxtNode = jNode;
                                    periodVisits.Remove(jNode.uid);
                                    rt.Add(jNode);
                                    break;
                                }
                                if (jNode.uid == 0)
                                {
                                    if (jNode.uid != nxtNode.uid && GlobalUtils.IsEqual(x[nxtNode.uid, cloneDepotUID, t].X, 1.0, 1e-3))
                                    {
                                        curNode = nxtNode;
                                        nxtNode = jNode;
                                        periodVisits.Remove(jNode.uid);
                                        rt.Add(jNode);
                                        break;
                                    }
                                }
                            }
                        }
                        routes.Add(rt);
                    }
                }
            }

            //julia code for IRP
            /*
            for i = indices.V_cus

                list = []
                if isapprox(x[1, i, t], 1, atol = 0.0001) && i in visits
                        vehicle_id += 1
                    cur_node = 1
                    cur_node2 = i
                    setdiff!(visits, i) # remove i from remaining visits
                    append!(list, 1) # update route
                    append!(list, i)
                    #println("list=$list")

                    while cur_node2 != length(indices.V_aug) && cur_node2 != 1 # while the route is not closed
                        #if cur_node2 == 1
                        #    same_depot = true
                        #end
                        for j = indices.V_aug
                            #println("i=$cur_node2, j=$j, min(i,j)=$(min(cur_node2,j)), max(i,j)=$(max(cur_node2,j))")
                            if j != cur_node2 && isapprox(x[min(cur_node2, j), max(cur_node2, j), t], 1, atol = 0.0001) && cur_node != j  # && (!(cur_node2 in list) || !(j in list))   #&& (min(i,j) || max(i,j) != cur_node2)
                                cur_node = cur_node2
                                cur_node2 = j
                                setdiff!(visits, j) # remove j from remaining visits
                                #println("i=$cur_node2, j=$j, min(i,j)=$(min(cur_node2,j)), max(i,j)=$(max(cur_node2,j)), cur_node=$cur_node, cur_node2=$cur_node2")
                                append!(list, j) # update route
                                #println("list=$list")
                                break
                            end
                        end
                    end
                    veh_dict[vehicle_id] = copy(list)
                end
            end
            */

            /*
            for (int j = 1; j < allNodesNum; j++)
            {
                Node to = sol.augNodes[j];
                int toUID = to.uid;
                if (to.uid == 0)
                    toUID = allNodesNum - 1;

                Node prev = depot;
                if (GlobalUtils.IsEqual(x[prev.uid, toUID, t].X, 1.0, 1e-3)) // route start
                {
                    List<Node> rt = new List<Node>();
                    rt.Add(prev);
                    seen.Add(prev.uid);

                    bool termination = false;
                    int nextIDAdj = -1;
                    while (!termination)
                    {
                        Node next;
                        for (int ii = 0; ii < allNodesNum; ii++)
                        {
                            next = sol.augNodes[ii];

                            if (prev.uid == next.uid)
                                continue;

                            nextIDAdj = next.uid;
                            if (next.uid == 0)
                            {
                                if (GlobalUtils.IsEqual(x[next.uid, prev.uid, t].X, 1.0, 1e-3))
                                {
                                    termination = true;
                                    break;
                                }
                                if (GlobalUtils.IsEqual(x[prev.uid, allNodesNum - 1, t].X, 1.0, 1e-3))
                                {
                                    termination = true;
                                    break;
                                }
                            }
                            int prevID = Math.Min(prev.uid, nextIDAdj);
                            int nextID = Math.Max(prev.uid, nextIDAdj);
                            
                            if (GlobalUtils.IsEqual(x[prevID, nextID, t].X, 1.0, 1e-3) && !seen.Contains(next.uid))
                            {
                                rt.Add(next);
                                seen.Add(next.uid);
                                prev = next;
                                break;
                            }
                        }
                        if (nextIDAdj == allNodesNum - 1)
                        {
                            termination = true;
                        }
                    }
                    rt.Add(depot);
                    routes.Add(rt);
                }
            }
            */
            if (periodVisits.Count > 0)
            {
                Console.WriteLine("{0} visits remaining unrouted for period {1}: {2}", periodVisits.Count, t, periodVisits.ToString());
                foreach (int node in periodVisits) 
                {
                    Console.WriteLine("node {0} --> quantity {1}", node, q[node,t].X);
                }
            }

            return routes;
        }
        

        private static int findCusStartPeriodVI(Node cust, PRP model)
        {
            int st = model.input.horizonDays;
            double totalDemand = 0.0;
            for (int t = 0; t < model.input.horizonDays; ++t)
            {
                totalDemand += cust.productRate[t];
                if (totalDemand > cust.startingInventory)
                {
                    return t;
                }
            }
            return st;
        }

        private static int findSafeCusStartPeriodVI(Node cust, PRP model)
        {
            int st = model.input.horizonDays;
            double totalDemand = 0.0;
            for (int t = 0; t < model.input.horizonDays; ++t)
            {
                totalDemand += cust.productRate[t];
                if (totalDemand > cust.stockMaximumLevel)
                {
                    return t;
                }
            }
            return st;
        }


        private static bool existsNonIntegerQuantity(ref Solution sol, GRBVar[,] q, GRBVar[] p)
        {
            var dict = new Dictionary<int, int>();

            for (int t = 0; t < sol.periods.Count; t++)
            {
                if (Math.Abs(Math.Round(p[t].X) - p[t].X) > 0.1)
                {
                    //GlobalUtils.writeToConsole("{0} vs {1}", Math.Round(p[t].X), p[t].X);
                    return true;
                }
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    if (Math.Abs(Math.Round(q[i, t].X) - q[i, t].X) > 0.1)
                    {
                        if (dict.ContainsKey(i))
                            dict[i]++;
                        else
                            dict[i] = 1;
                        Console.WriteLine("{0} vs {1} {2},{3}", Math.Round(q[i,t].X), q[i,t].X, i, t);
                        //return true;
                    }
                }

            }
            foreach (var pair in dict)
            {
                if (pair.Value != 2)
                    return true;
            }
            return false;
        }

        private static bool existsViolatedQuantity(ref Solution sol, GRBVar[,] q, GRBVar[] p)
        {
            for (int t = 0; t < sol.periods.Count; t++)
            {
                if (Math.Abs(Math.Round(p[t].X) - p[t].X) > 0.1)
                {
                    Console.WriteLine("{0} vs {1}", Math.Round(p[t].X), p[t].X);
                    return true;
                }
            }
            for (int i = 0; i < sol.customers.Count; i++)
            {
                Node cust = sol.customers[i];
                int deliveries = 0;
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    deliveries += (int)Math.Round(q[i, t].X);
                }
                if (deliveries != cust.totalDemand - cust.startingInventory)
                {
                    Console.WriteLine("{0} vs {1}", deliveries, cust.totalDemand - cust.startingInventory);
                    return true;
                }
            }
            return false;
        }

        private static bool existsViolatedQuantity(ref Solution sol, GRBVar[,] q, GRBVar[,] qrlc, GRBVar[] p)
        {
            for (int t = 0; t < sol.periods.Count; t++)
            {
                if (Math.Abs(Math.Round(p[t].X) - p[t].X) > 0.1)
                {
                    Console.WriteLine("{0} vs {1}", Math.Round(p[t].X), p[t].X);
                    return true;
                }
            }
            for (int i = 0; i < sol.customers.Count; i++)
            {
                Node cust = sol.customers[i];
                int deliveries = 0;
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    deliveries += (int)Math.Round(q[i, t].X);
                    if (cust.visitSchedule[t])
                        deliveries += (int)Math.Round(qrlc[i, t].X);
                }
                if (deliveries != cust.totalDemand - cust.startingInventory)
                {
                    Console.WriteLine("{0} vs {1}", deliveries, cust.totalDemand - cust.startingInventory);
                    return true;
                }
            }
            return false;
        }


        public static void initEnv()
        {
            gurobiEnv = new GRBEnv();
            if (gurobiEnv == null)
                throw new Exception("Could not initialize gurobi environment");
        }

        public static void disposeEnv()
        {
            gurobiEnv.Dispose();
        }

    }
}
