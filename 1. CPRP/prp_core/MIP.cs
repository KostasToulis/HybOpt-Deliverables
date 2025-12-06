using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Net.Http.Headers;
using System.Net.Sockets;
using System.Xml;
using Gurobi;

namespace PRP
{
    public class MIP
    {
        public static GRBEnv gurobiEnv;
        //public static Stopwatch stopwatch = new Stopwatch();
        public static int threadLimit;
        public static Random rand = new Random();

        //==================================================== MIPs ====================================================//
        //MIP formulation for simultaneously re-optimizing the delivery quantities of all customers and the production quantities as well. 
        //Allows one customer removal per route. Note: It can be  transformed to allow more
        public static bool runSimultaneousDeliveryProductionReoptimizationLPwithCustRemoval(Solution sol)
        {
            int numberOfPeriods = sol.periods.Count;
            int numberOfCustomers = sol.customers.Count;
            double[,] UBq = new double[numberOfCustomers, numberOfPeriods]; //upper bound of delivered quantities to customer
            double[,] rtSaves = new double[numberOfCustomers, numberOfPeriods]; //upper bound of delivered quantities to customer
            double[] UBp = new double[numberOfPeriods]; //upper bound of delivered quantities to customer
            double totalProdQuantity = 0.0;
            bool rerun = false;

            //calculate UBq 
            for (int i = 0; i < sol.customers.Count; i++)
            {
                Node cust = sol.customers[i];
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    int aggrCap = cust.stockMaximumLevel + cust.productRate[t]; //@greg cap + demand (OK)
                    int vehCap = sol.model.capacity; //@greg veh capacity (OK)
                    int totalDemandUntilEnd = 0;
                    for (int tt = t; tt < numberOfPeriods; ++tt)
                    {
                        totalDemandUntilEnd += cust.productRate[tt];  //@greg remaining demand until end of horizon (OK)
                    }
                    if (!cust.visitSchedule[t])
                        UBq[i, t] = 0;
                    else
                        UBq[i, t] = (double)GlobalUtils.Min(aggrCap, vehCap, totalDemandUntilEnd);
                }
            }
            for (int t = 0; t < numberOfPeriods; ++t)
            {
                totalProdQuantity += sol.depot.productionRates[t];
                if (!sol.depot.open[t]) //@greg is the depot open bool
                    UBp[t] = 0;
                else
                    UBp[t] = sol.depot.productionCapacity;
            }

            for (int i = 0; i < sol.customers.Count; i++)
            {
                Node cust = sol.customers[i];
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    rtSaves[i, t] = 0.0;
                }
            }

            // calculate the possible savings
            for (int t = 0; t < sol.periods.Count; t++)
            {
                Period pr = sol.periods[t];
                for (int r = 0; r < pr.periodRoutes.Count; r++)
                {
                    Route rt = pr.periodRoutes[r];
                    for (int i = 1; i < rt.nodes.Count - 1; i++)
                    {
                        Node prev = rt.nodes[i - 1];
                        Node me = rt.nodes[i];
                        Node next = rt.nodes[i + 1];

                        double costRemoved = sol.model.distMatrix[prev.uid, me.uid] + sol.model.distMatrix[me.uid, next.uid];
                        double costAdded = sol.model.distMatrix[prev.uid, next.uid];

                        rtSaves[me.uid - 1, t] = costAdded - costRemoved;
                    }
                }
            }



            try
            {
                gurobiEnv = new GRBEnv();
                GRBModel model = new GRBModel(gurobiEnv);

                // Params
                model.ModelName = "allCustomersAndProductionQuantitiesLPwithCustRemoval";
                model.Parameters.OutputFlag = 0;
                model.Parameters.Threads = threadLimit;

                // Decision variables
                GRBVar[] I_depot = new GRBVar[numberOfPeriods]; // depot inventory
                GRBVar[] p = new GRBVar[numberOfPeriods]; // depot production
                GRBVar[,] I_cust = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer inventory
                GRBVar[,] q = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer delivery quantities
                GRBVar[,] remZ = new GRBVar[numberOfCustomers, numberOfPeriods]; 

                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    I_depot[t] = model.AddVar(0.0, sol.depot.stockMaximumLevel, sol.depot.unitHoldingCost, GRB.CONTINUOUS, "I_d^" + t);
                    p[t] = model.AddVar(0.0, UBp[t], 0, GRB.CONTINUOUS, "p_d^" + t); //@greg productionCapacity: is  this the maximum quantity we can produce per dayq
                }
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        q[i, t] = model.AddVar(0.0, UBq[i, t], 0.0, GRB.CONTINUOUS, "q_" + i + "^" + t);
                        I_cust[i, t] = model.AddVar(0.0, cust.stockMaximumLevel, cust.unitHoldingCost, GRB.CONTINUOUS, "I_" + i + " ^ " + t);
                        if (cust.visitSchedule[t]) // if is not route it cannot be removed 
                            remZ[i,t] = model.AddVar(0.0, 1.0, rtSaves[i,t], GRB.BINARY, "remZ_" + i + " ^ " + t);
                        else
                            remZ[i, t] = model.AddVar(0.0, 0.0, rtSaves[i, t], GRB.BINARY, "remZ_" + i + " ^ " + t);
                    }
                }

                // Objective
                model.ModelSense = GRB.MINIMIZE;

                // Constraints 
                // 1. Depot inventory flow 
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    GRBLinExpr lhs = 0.0;
                    lhs.AddTerm(1.0, I_depot[t]);
                    for (int i = 0; i < sol.customers.Count; i++)
                    {
                        lhs.AddTerm(1.0, q[i, t]);
                    }
                    if (t == 0)
                        model.AddConstr(lhs == sol.depot.startingInventory + p[t], "depot_inv_" + t);
                    else
                        model.AddConstr(lhs == I_depot[t - 1] + p[t], "depot_inv_" + t);
                }

                // 2. Customer inventory flow
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        GRBLinExpr lhs = 0.0;
                        lhs.AddTerm(1.0, I_cust[i, t]);
                        if (t == 0)
                            model.AddConstr(lhs == cust.startingInventory + q[i, t] - cust.productRate[t], "cust_inv" + i + " ^ " + t);
                        else
                            model.AddConstr(lhs == I_cust[i, t - 1] + q[i, t] - cust.productRate[t], "cust_inv" + i + " ^ " + t);
                    }
                }

                // 3. Customer delivery capacity
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        GRBLinExpr lhs = 0.0;
                        lhs.AddTerm(1.0, q[i, t]);
                        if (t == 0)
                            model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - cust.startingInventory, "cust_ml" + i + " ^ " + t);
                        else
                            model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - I_cust[i, t - 1], "cust_ml" + i + " ^ " + t);
                    }
                }

                // 4. Customer delivery vehicle capacity
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    Period per = sol.periods[t];

                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route route = per.periodRoutes[r];
                        GRBLinExpr lhs = 0.0;

                        for (int i = 1; i < route.nodes.Count - 1; i++)
                        {
                            Node cust = route.nodes[i];
                            int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                            lhs.AddTerm(1.0, q[ii, t]);
                        }
                        model.AddConstr(lhs <= sol.model.capacity, "cust_vehcap_" + r + " ^ " + t);
                    }
                }

                // 5. Total production quantity
                GRBLinExpr lhsp = 0.0;
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    lhsp.AddTerm(1.0, p[t]);
                }
                model.AddConstr(lhsp == totalProdQuantity, "total_prod");


                // 6. removal constraints
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        GRBLinExpr lhs = q[i, t];
                        model.AddConstr(lhs <= (1-remZ[i, t])*UBq[i,t] , "rem_visits" + i + " ^ " + t);
                    }
                }

                // 7. One removal per period
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    Period per = sol.periods[t];

                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route route = per.periodRoutes[r];
                        GRBLinExpr lhs = 0.0;

                        for (int i = 1; i < route.nodes.Count - 1; i++)
                        {
                            Node cust = route.nodes[i];
                            int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                            lhs.AddTerm(1.0, remZ[ii, t]);
                        }
                        model.AddConstr(lhs <= 1.0, "one_removal" + r + " ^ " + t);
                    }
                }

                // Optimize
                model.Optimize();
                int optimstatus = model.Status;

                switch (model.Status)
                {
                    case GRB.Status.OPTIMAL:
                        {
                            double objval = model.ObjVal;

                            /*
                            for (int i = 0; i < sol.customers.Count; i++)
                            {
                                Node cust = sol.customers[i];

                                Console.Write("{0,20}", "Cust Deliveries");
                                for (int t = 0; t < numberOfPeriods; t++)
                                    Console.Write(" {0,8:0.0} ", (int)q[i, t].X);
                                Console.Write("\n");

                                Console.Write("{0,20}", "Cust Inv");
                                for (int t = 0; t < numberOfPeriods; t++)
                                    Console.Write(" {0,8:0.0} ", (int)I_cust[i, t].X);
                                Console.Write("\n");
                            }
                            */

                            // Update solution with new calculated quantities
                            for (int t = 0; t < numberOfPeriods; t++)
                            {
                                sol.depot.deliveredQuantities[t] = 0;
                                sol.depot.productionRates[t] = (int)Math.Round(p[t].X);
                            }

                            for (int i = 0; i < sol.customers.Count; i++)
                            {
                                Node cust = sol.customers[i];

                                for (int t = 0; t < numberOfPeriods; t++)
                                {
                                    CustDelivery cd = cust.horizonDeliveryServices[t];

                                    if (cd.route != null)
                                        cd.route.load -= cd.quantity;

                                    //cust.auxiliary_deliveries[t] = (int)q[i, t].X; //update delivery quantity
                                    cust.deliveredQuantities[t] = (int)Math.Round(q[i, t].X); //update delivery quantity
                                    sol.depot.deliveredQuantities[t] -= cust.deliveredQuantities[t]; // OR +=;
                                                                                                     // update customer delivery and vehicle load
                                    if (cd.route != null)
                                    {
                                        cd.quantity = cust.deliveredQuantities[t];
                                        cd.route.load += cd.quantity;
                                        cd.route.SetLoadAndCostLists(t, sol.model);
                                    }

                                    if (GlobalUtils.IsEqual(remZ[i, t].X, 1)) // if customer is affected by removal
                                    {
                                        Route old_rt = cd.route;
                                        //Console.Write("\n{0} {1}: {2}", i, t, rtSaves[i,t]);
                                        rerun = true;

                                        for (int ii = 1; ii < old_rt.nodes.Count - 1; ii++)
                                        {
                                            Node me = old_rt.nodes[ii];
                                            if (me.uid == cust.uid)
                                            {
                                                //Route removed from
                                                old_rt.nodes.RemoveAt(ii);
                                                cust.visitSchedule[t] = false;
                                                old_rt.totalRoutingCost += rtSaves[i, t];
                                                old_rt.SetLoadAndCostLists(t, sol.model);

                                                //objval -= rtSaves[i, t];
                                                //sol.routingCost += rtSaves[i, t];
                                            }
                                        }

                                        // remove cust delivery
                                        cd.route = null;
                                    }
                                }
                                cust.CalculateInventoryLevels();
                            }
                            sol.depot.CalculateInventoryLevels();

                            //Fix solution objective                            
                            //sol.totalObjective -= sol.holdingCost;
                            //sol.totalObjective += objval;
                            //sol.holdingCost = objval;

                            sol.routingCost = Solution.EvaluateRoutingObjectivefromScratch(sol);
                            sol.holdingCost = Solution.EvaluateInventoryObjectivefromScratch(sol);
                            sol.totalUnitProductionCost = Solution.EvaluateUnitProductionObjectivefromScratch(sol);
                            sol.setupProductionCost = Solution.EvaluateSetupProductionObjectivefromScratch(sol);
                            sol.totalObjective = sol.routingCost + sol.holdingCost + sol.totalUnitProductionCost +sol.setupProductionCost;


                            if (!Solution.checkSolutionStatus(sol.TestEverythingFromScratch()))
                                Console.WriteLine("Lefteri ftiakse tis malakies sou sto production lp");
                            TSP_GRB_Solver.SolveTSP(sol);

                            //if (rerun) //COMMENT TO DISABLE RE-RUNS
                            //    runSimultaneousDeliveryProductionReoptimizationLPwithCustRemoval(sol);
                            break;
                        }
                    case GRB.Status.INFEASIBLE:
                        {
                            Console.WriteLine("Model is infeasible");

                            // compute and write out IIS
                            //model.ComputeIIS();
                            //model.Write("model.ilp");
                            return false;
                        }
                    case GRB.Status.UNBOUNDED:
                        {
                            Console.WriteLine("Model is unbounded");
                            return false;
                        }
                    default:
                        {
                            Console.WriteLine("Optimization was stopped with status = "
                                              + optimstatus);
                            return false;
                        }

                }

                // Dispose of model
                model.Dispose();
                gurobiEnv.Dispose();
            }
            catch (GRBException e)
            {
                Console.WriteLine("NEW LP Error code: " + e.ErrorCode + ". " + e.Message);
            }

            return true;
        }

        //MIP formulation for simultaneously re-optimizing the delivery quantities of all customers and the production quantities as well given new visits schedules 
        // The routing is totally random, so it causes heavy diversification and it ca 
        public static bool runSimultaneousDeliveryProductionReoptimizationLPFlippedVisits(Solution sol)
        {
            int numberOfPeriods = sol.periods.Count;
            int numberOfCustomers = sol.customers.Count;
            int margin = 1;

            // find the number of vehicles to be used. This is necessary for the unlimited in order to avoid fragmente routes  
            int numberOfVehicles = 0;
            int maxNumberVehUsed = 0;
            for (int t = 0; t < sol.periods.Count; t++)
            {
                Period per = sol.periods[t];
                maxNumberVehUsed = Math.Max(maxNumberVehUsed, per.periodRoutes.Count);
            }
            if (sol.model.vehicles == sol.customers.Count) // if vehicle  fleet is unlimited
                numberOfVehicles = Math.Min(maxNumberVehUsed + margin, sol.model.vehicles);
            else
                numberOfVehicles = sol.model.vehicles;
            
            double[,] UBq = new double[numberOfCustomers, numberOfPeriods]; //upper bound of delivered quantities to customer
            double[] UBp = new double[numberOfPeriods]; //upper bound of produced quantities
            double totalProdQuantity = 0.0;

            //calculate UBq 
            for (int i = 0; i < sol.customers.Count; i++)
            {
                Node cust = sol.customers[i];
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    int aggrCap = cust.stockMaximumLevel + cust.productRate[t]; //@greg cap + demand (OK)
                    int vehCap = sol.model.capacity; //@greg veh capacity (OK)
                    int totalDemandUntilEnd = 0;
                    for (int tt = t; tt < numberOfPeriods; ++tt)
                    {
                        totalDemandUntilEnd += cust.productRate[tt];  //@greg remaining demand until end of horizon (OK)
                    }
                    if (!cust.visitSchedule[t]) //@GREG this where it reads the visit schedule
                        UBq[i, t] = 0;
                    else
                        UBq[i, t] = (double)GlobalUtils.Min(aggrCap, vehCap, totalDemandUntilEnd);
                }
            }
            for (int t = 0; t < numberOfPeriods; ++t)
            {
                totalProdQuantity += sol.depot.productionRates[t];
                if (!sol.depot.open[t]) //@greg is the depot open bool
                    UBp[t] = 0;
                else
                    UBp[t] = sol.depot.productionCapacity;
            }

            try
            {
                gurobiEnv = new GRBEnv();
                GRBModel model = new GRBModel(gurobiEnv);

                // Params
                model.ModelName = "allCustomersAndProductionQuantitiesLPFlippedVisits";
                model.Parameters.OutputFlag = 0;
                model.Parameters.Threads = threadLimit;

                // Decision variables
                GRBVar[] I_depot = new GRBVar[numberOfPeriods]; // depot inventory
                GRBVar[] p = new GRBVar[numberOfPeriods]; // depot production
                GRBVar[,] I_cust = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer inventory
                GRBVar[,,] q = new GRBVar[numberOfCustomers, numberOfVehicles, numberOfPeriods]; // customer delivery quantities
                GRBVar[,,] z = new GRBVar[numberOfCustomers, numberOfVehicles, numberOfPeriods];

                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    I_depot[t] = model.AddVar(0.0, sol.depot.stockMaximumLevel, sol.depot.unitHoldingCost, GRB.CONTINUOUS, "I_d^" + t);
                    p[t] = model.AddVar(0.0, UBp[t], 0, GRB.CONTINUOUS, "p_d^" + t); 
                }
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        I_cust[i, t] = model.AddVar(0.0, cust.stockMaximumLevel, cust.unitHoldingCost, GRB.CONTINUOUS, "I_" + i + " ^ " + t);
                        for (int k = 0; k < numberOfVehicles; ++k)
                        {
                            if (cust.visitSchedule[t])
                            {
                                z[i, k, t] = model.AddVar(0.0, 1.0, 0.0, GRB.BINARY, "z_" + i + "_" + k + "^" + t + "^" + k);
                                q[i, k, t] = model.AddVar(0.0, UBq[i, t], 0.0, GRB.CONTINUOUS, "q_" + i + "^" + t + "^" + k);
                            }
                            else
                            {
                                z[i, k, t] = model.AddVar(0.0, 0.0, 0.0, GRB.BINARY, "z_" + i + "_" + k + "^" + t + "^" + k);
                                q[i, k, t] = model.AddVar(0.0, 0.0, 0.0, GRB.CONTINUOUS, "q_" + i + "^" + t + "^" + k);
                            }
                        }
                    }
                }
                // Objective
                model.ModelSense = GRB.MINIMIZE;

                // Constraints 
                // 1. Depot inventory flow 
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    GRBLinExpr lhs = 0.0;
                    lhs.AddTerm(1.0, I_depot[t]);
                    for (int i = 0; i < sol.customers.Count; i++)
                    {
                        for (int k = 0; k < numberOfVehicles; ++k)
                            lhs.AddTerm(1.0, q[i, k, t]);
                    }
                    if (t == 0)
                        model.AddConstr(lhs == sol.depot.startingInventory + p[t], "depot_inv_" + t);
                    else
                        model.AddConstr(lhs == I_depot[t - 1] + p[t], "depot_inv_" + t);
                }

                // 2. Customer inventory flow
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        GRBLinExpr lhs = 0.0;
                        for (int k = 0; k < numberOfVehicles; ++k)
                            lhs.AddTerm(1.0, q[i, k, t]);
                    
                        if (t == 0)
                            model.AddConstr(I_cust[i, t] == cust.startingInventory + lhs - cust.productRate[t], "cust_inv" + i + " ^ " + t);
                        else
                            model.AddConstr(I_cust[i, t] == I_cust[i, t - 1] + lhs - cust.productRate[t], "cust_inv" + i + " ^ " + t);
                    }
                }

                // 3. Customer delivery capacity
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        GRBLinExpr lhs = 0.0;

                        for (int k = 0; k < numberOfVehicles; ++k)
                            lhs.AddTerm(1.0, q[i, k, t]);
                        if (t == 0)
                            model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - cust.startingInventory, "cust_ml" + i + " ^ " + t);
                        else
                            model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - I_cust[i, t - 1], "cust_ml" + i + " ^ " + t);
                    }
                }

                // 4. Customer delivery vehicle capacity
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    for (int k = 0; k < numberOfVehicles; k++)
                    {
                        GRBLinExpr lhs = 0.0;
                        for (int i = 0; i < sol.customers.Count; i++)
                        {
                            lhs.AddTerm(1.0, q[i, k, t]);
                        }
                        model.AddConstr(lhs <= sol.model.capacity, "cust_vehcap_" + k + " ^ " + t);
                    }
                }

                // 6. Customers assignment to vehicles
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    for (int i = 0; i < sol.customers.Count; i++)
                    {
                        GRBLinExpr lhs = 0.0;
                        Node cust = sol.customers[i];

                        for (int k = 0; k < numberOfVehicles; k++)
                        {
                            lhs.AddTerm(1.0, z[i, k, t]);
                        }
                        if (cust.visitSchedule[t])
                            model.AddConstr(lhs <= 1, "cust_vehass_" + i + " ^ " + t); //Note if <= the mip is allowed to remove visits
                        else
                            model.AddConstr(lhs == 0, "cust_vehass_" + i + " ^ " + t);
                    }
                }

                // 7. Forbid split delivery (bind q and z)
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    for (int i = 0; i < sol.customers.Count; i++)
                    {
                        Node cust = sol.customers[i];
                        for (int k = 0; k < numberOfVehicles; k++)
                        {
                            GRBLinExpr lhs = 0.0;
                            if (cust.visitSchedule[t])
                                model.AddConstr(q[i,k,t] <= sol.model.capacity*z[i,k,t], "splitdeliv_" + " ^ " + t + " ^ " + k);
                            else
                                model.AddConstr(q[i, k, t] == 0, "splitdeliv_" + " ^ " + t + " ^ " + k);
                        }
                    }
                }

                // 5. Total production quantity
                GRBLinExpr lhsp = 0.0;
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    lhsp.AddTerm(1.0, p[t]);
                }
                model.AddConstr(lhsp == totalProdQuantity, "total_prod");

                // Optimize
                model.Optimize();
                int optimstatus = model.Status;

                switch (model.Status)
                {
                    case GRB.Status.OPTIMAL:
                        {
                            // Update solution with new calculated quantities
                            for (int t = 0; t < numberOfPeriods; t++)
                            {
                                sol.depot.deliveredQuantities[t] = 0;
                                sol.depot.productionRates[t] = (int)Math.Round(p[t].X);
                            }

                            // update customer deliveries and routes
                            for (int t = 0; t < sol.periods.Count; t++)
                            {
                                Period per = sol.periods[t];
                                per.periodRoutes.Clear();
                                for (int k = 0; k < numberOfVehicles; k++)
                                {
                                    Route rt = new Route(sol.model.input.dayVehicleCapacity);
                                    rt.initialize(sol.depot);
                                    per.periodRoutes.Add(rt);
                                }
                            }

                            for (int t = 0; t < sol.periods.Count; t++)
                            {
                                for (int i = 0; i < sol.customers.Count; i++)
                                {
                                    Node cust = sol.customers[i];
                                    CustDelivery cd = cust.horizonDeliveryServices[t];

                                    cust.visitSchedule[t] = false;
                                    cd.route = null;
                                    cd.quantity = 0;

                                    for (int k = 0; k < numberOfVehicles; k++)
                                    {
                                        if (GlobalUtils.IsEqual(z[i, k, t].X, 1.0))
                                        {
                                            cust.visitSchedule[t] = true;
                                            Period per = sol.periods[t];
                                            Route rt = per.periodRoutes[k];
                                            rt.nodes.Insert(1, cust);
                                            cd.route = rt;

                                            cust.deliveredQuantities[t] = (int)Math.Round(q[i, k, t].X); //update delivery quantity
                                            sol.depot.deliveredQuantities[t] -= cust.deliveredQuantities[t]; // OR +=;
                                            cd.quantity = cust.deliveredQuantities[t];
                                            cd.route.load += cd.quantity;
                                            cd.route.SetLoadAndCostLists(t, sol.model);
                                            break;
                                        }
                                        else
                                        {
                                            if (z[i,k,t].X > 0.9)
                                                Console.WriteLine(z[i,k,t].X);
                                        }
                                    }
                                }
                            }

                            // calculate new routes costs
                            for (int t = 0; t < sol.periods.Count; t++)
                            {
                                Period per = sol.periods[t];

                                for (int k = 0; k < numberOfVehicles; k++)
                                {
                                    Route rt = per.periodRoutes[k];
                                    rt.totalRoutingCost = 0;
                                    rt.load = 0;

                                    for (int ii = 0; ii < rt.nodes.Count-1; ii++)
                                    {
                                        Node me = rt.nodes[ii];
                                        Node next = rt.nodes[ii+1];
                                        rt.totalRoutingCost += sol.model.distMatrix[me.uid, next.uid];
                                        if (ii>0)
                                            rt.load += me.deliveredQuantities[t];

                                    }
                                }
                            }

                            for (int i = 0; i < sol.customers.Count; i++)
                            {
                                Node cust = sol.customers[i];
                                cust.CalculateInventoryLevels();
                            }
                            sol.depot.CalculateInventoryLevels();

                            //Fix solution routing
                            //TSP_GRB_Solver.SolveTSP(sol);

                            sol.routingCost = Solution.EvaluateRoutingObjectivefromScratch(sol);
                            sol.holdingCost = Solution.EvaluateInventoryObjectivefromScratch(sol);
                            sol.totalUnitProductionCost = Solution.EvaluateUnitProductionObjectivefromScratch(sol);
                            sol.setupProductionCost = Solution.EvaluateSetupProductionObjectivefromScratch(sol);
                            sol.totalObjective = sol.routingCost + sol.holdingCost + sol.totalUnitProductionCost + sol.setupProductionCost;
                            
                            if (!Solution.checkSolutionStatus(sol.TestEverythingFromScratch()))
                                Console.WriteLine("Lefteri ftiakse tis malakies sou sto production lp");

                            TSP_GRB_Solver.SolveTSP(sol);

                            break;
                        }
                    case GRB.Status.INFEASIBLE:
                        {
                            Console.WriteLine("Model is infeasible");

                            // compute and write out IIS
                            //model.ComputeIIS();
                            //model.Write("model.ilp");
                            return false;
                        }
                    case GRB.Status.UNBOUNDED:
                        {
                            Console.WriteLine("Model is unbounded");
                            return false;
                        }
                    default:
                        {
                            Console.WriteLine("Optimization was stopped with status = "
                                              + optimstatus);
                            return false;
                        }

                }

                // Dispose of model
                model.Dispose();
                gurobiEnv.Dispose();
            }
            catch (GRBException e)
            {
                Console.WriteLine("NEW LP Error code: " + e.ErrorCode + ". " + e.Message);
            }

            return true;
        }


        //MIP formulation for simultaneously re-optimizing the delivery quantities of all customers and the production quantities 
        // as well allowing vehicle capacity to be violated. The violation is penalized and one insertion per route or customer is allowed
        public static bool runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertion(Solution sol)
        {
            int numberOfPeriods = sol.periods.Count;
            int numberOfCustomers = sol.customers.Count;

            int unitCapPenalty = 100; //Note the coefficient for penalizing capacity violation (per unit)
            int maxRouteAdditions = 1; // number of max additions per route
            int maxTotalAdditions = 10;
            int effectiveCapacity = sol.periods[0].periodRoutes[0].effectiveCapacity;
            int actualCapacity = sol.periods[0].periodRoutes[0].realCapacity;
            int excessSlack = effectiveCapacity - actualCapacity; // the max excess of capacity per route allowed

            int maxNumberVehUsed = 0;
            for (int t = 0; t < sol.periods.Count; t++)
            {
                Period per = sol.periods[t];
                maxNumberVehUsed = Math.Max(maxNumberVehUsed, per.periodRoutes.Count);
            }
            double[,] UBq = new double[numberOfCustomers, numberOfPeriods]; //upper bound of delivered quantities to customer
            double[] UBp = new double[numberOfPeriods]; //upper bound of delivered quantities to customer
            double totalProdQuantity = 0.0;
            double[,] rtAddCost = new double[numberOfCustomers, numberOfPeriods];
            Route[,] rtAddCostRoute = new Route[numberOfCustomers, numberOfPeriods];

            //calculate UBq 
            for (int i = 0; i < sol.customers.Count; i++)
            {
                Node cust = sol.customers[i];
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    int aggrCap = cust.stockMaximumLevel + cust.productRate[t];
                    int totalDemandUntilEnd = 0;
                    for (int tt = t; tt < numberOfPeriods; ++tt)
                    {
                        totalDemandUntilEnd += cust.productRate[tt];
                    }
                    //if (!cust.visitSchedule[t])
                    //    UBq[i, t] = 0;
                    //else
                    UBq[i, t] = (double)GlobalUtils.Min(aggrCap, actualCapacity, totalDemandUntilEnd);
                }
            }

            for (int t = 0; t < numberOfPeriods; ++t)
            {
                totalProdQuantity += sol.depot.productionRates[t];
                if (!sol.depot.open[t])
                    UBp[t] = 0;
                else
                    UBp[t] = sol.depot.productionCapacity;
            }

            for (int i = 0; i < sol.customers.Count; i++)
            {
                Node cust = sol.customers[i];
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    rtAddCost[i, t] = int.MaxValue;
                    rtAddCostRoute[i, t] = null;
                }
            }

            // calculate the possible savings
            for (int t = 0; t < sol.periods.Count; t++)
            {
                Period pr = sol.periods[t];
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    if (!cust.visitSchedule[t]) //if customer is not serviced then and only then can be added
                    {
                        for (int r = 0; r < pr.periodRoutes.Count; r++)
                        {
                            Route rt = pr.periodRoutes[r];
                            if (!rt.nodes.Contains(cust)) //always true
                            {
                                for (int ii = 0; ii < rt.nodes.Count - 1; ii++)
                                {
                                    Node prev = rt.nodes[ii];
                                    Node next = rt.nodes[ii + 1];

                                    double costRemoved = sol.model.distMatrix[prev.uid, next.uid];
                                    double costAdded = sol.model.distMatrix[prev.uid, cust.uid] + sol.model.distMatrix[cust.uid, next.uid];

                                    if (costAdded - costRemoved < rtAddCost[cust.uid - 1, t])
                                    {
                                        rtAddCost[cust.uid - 1, t] = costAdded - costRemoved;
                                        rtAddCostRoute[cust.uid - 1, t] = rt;
                                    }
                                }
                            }
                        }
                    }
                }
            }


            try
            {
                GRBEnv gurobiEnv = new GRBEnv();
                GRBModel model = new GRBModel(gurobiEnv);

                // Params
                model.ModelName = "allCustomersAndProductionQuantitiesLPVehCapInfeas" + sol.ellapsedMs;
                model.Parameters.OutputFlag = 0;
                model.Parameters.Threads = threadLimit;

                // Decision variables
                GRBVar[] I_depot = new GRBVar[numberOfPeriods]; // depot inventory
                GRBVar[] p = new GRBVar[numberOfPeriods]; // depot production
                GRBVar[,] I_cust = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer inventory
                GRBVar[,] q = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer delivery quantities
                GRBVar[,] e = new GRBVar[maxNumberVehUsed, numberOfPeriods]; // capacity violations
                GRBVar[,] addZ = new GRBVar[numberOfCustomers, numberOfPeriods]; // binary addition of visits


                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    I_depot[t] = model.AddVar(0.0, sol.depot.stockMaximumLevel, sol.depot.unitHoldingCost, GRB.CONTINUOUS, "I_d^" + t);
                    p[t] = model.AddVar(0.0, UBp[t], 0, GRB.CONTINUOUS, "p_d^" + t); //@greg productionCapacity: is  this the maximum quantity we can produce per dayq

                    Period per = sol.periods[t];
                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route rt = per.periodRoutes[r];
                        e[r, t] = model.AddVar(0.0, excessSlack, unitCapPenalty, GRB.CONTINUOUS, "e_" + r + "^" + t);
                    }
                }

                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        q[i, t] = model.AddVar(0.0, UBq[i, t], 0.0, GRB.CONTINUOUS, "q_" + i + "^" + t);
                        I_cust[i, t] = model.AddVar(0.0, cust.stockMaximumLevel, cust.unitHoldingCost, GRB.CONTINUOUS, "I_" + i + " ^ " + t);
                        if (cust.visitSchedule[t]) // if is not route it cannot be removed 
                            addZ[i, t] = model.AddVar(0.0, 0.0, rtAddCost[i, t], GRB.BINARY, "addZ_" + i + " ^ " + t);
                        else
                            addZ[i, t] = model.AddVar(0.0, 1.0, rtAddCost[i, t], GRB.BINARY, "addZ_" + i + " ^ " + t);
                    }
                }

                // Objective
                model.ModelSense = GRB.MINIMIZE;

                // Constraints 
                // 1. Depot inventory flow 
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    GRBLinExpr lhs = 0.0;
                    lhs.AddTerm(1.0, I_depot[t]);
                    for (int i = 0; i < sol.customers.Count; i++)
                    {
                        lhs.AddTerm(1.0, q[i, t]);
                    }
                    if (t == 0)
                        model.AddConstr(lhs == sol.depot.startingInventory + p[t], "depot_inv_" + t);
                    else
                        model.AddConstr(lhs == I_depot[t - 1] + p[t], "depot_inv_" + t);
                }

                // 2. Customer inventory flow
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        GRBLinExpr lhs = 0.0;
                        lhs.AddTerm(1.0, I_cust[i, t]);
                        if (t == 0)
                            model.AddConstr(lhs == cust.startingInventory + q[i, t] - cust.productRate[t], "cust_inv" + i + " ^ " + t);
                        else
                            model.AddConstr(lhs == I_cust[i, t - 1] + q[i, t] - cust.productRate[t], "cust_inv" + i + " ^ " + t);
                    }
                }

                // 3. Customer delivery capacity
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        GRBLinExpr lhs = 0.0;
                        lhs.AddTerm(1.0, q[i, t]);
                        if (t == 0)
                            model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - cust.startingInventory, "cust_ml" + i + " ^ " + t);
                        else
                            model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - I_cust[i, t - 1], "cust_ml" + i + " ^ " + t);
                    }
                }

                // 5. Total production quantity
                GRBLinExpr lhsp = 0.0;
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    lhsp.AddTerm(1.0, p[t]);
                }
                model.AddConstr(lhsp == totalProdQuantity, "total_prod");

                // 4. Customer delivery vehicle capacity
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    Period per = sol.periods[t];

                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route route = per.periodRoutes[r];
                        GRBLinExpr lhs = 0.0;

                        //for (int i = 1; i < route.nodes.Count - 1; i++)
                        for (int i = 0; i < sol.customers.Count; i++)
                        {
                            Node cust = sol.customers[i];
                            if (route.nodes.Contains(cust) || Object.ReferenceEquals(rtAddCostRoute[i,t], route))
                            {
                                int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                lhs.AddTerm(1.0, q[ii, t]);
                            }
                        }
                        model.AddConstr(lhs <= effectiveCapacity, "cust_vehcap_" + r + " ^ " + t); //NOTE: effective capacity
                    }
                }

                // 6. excessive slack
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    Period per = sol.periods[t];
                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route route = per.periodRoutes[r];
                        GRBLinExpr lhs = e[r, t];
                        //for (int i = 1; i < route.nodes.Count - 1; i++)
                        for (int i = 0; i < sol.customers.Count; i++)
                        {
                            Node cust = sol.customers[i];
                            if (route.nodes.Contains(cust) || Object.ReferenceEquals(rtAddCostRoute[i, t], route))
                            {
                                // if q{ii,t)is positive while not serviced it is surely in this route
                                int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                lhs.AddTerm(-1.0, q[ii, t]);
                            }
                        }
                        model.AddConstr(lhs >= -actualCapacity, "e" + r + " ^ " + t); //NOTE: effective capacity
                    }
                }

                // 7. addition constraints
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        if (!cust.visitSchedule[t])
                        {
                            GRBLinExpr lhs = q[i, t];
                            model.AddConstr(lhs <= (addZ[i, t]) * UBq[i, t], "add_visits" + i + " ^ " + t);
                        }
                    }
                }

                // 8. One addition per route (to be tuned)
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    Period per = sol.periods[t];

                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route route = per.periodRoutes[r];
                        GRBLinExpr lhs = 0.0;

                        //for (int i = 1; i < route.nodes.Count - 1; i++)
                        for (int i = 0; i < sol.customers.Count; i++)
                        {
                            Node cust = sol.customers[i];
                            if (Object.ReferenceEquals(rtAddCostRoute[i, t], route))
                            {
                                int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                lhs.AddTerm(1.0, addZ[ii, t]);
                            }
                        }
                        model.AddConstr(lhs <= maxRouteAdditions, "one_addition" + r + " ^ " + t);
                    }
                }

                // 9. Total addition bounds 
                /*
                GRBLinExpr lhsa = 0.0;
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    for (int i = 0; i < sol.customers.Count; i++)
                    {
                        Node cust = sol.customers[i];
                        int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                        lhsa.AddTerm(1.0, addZ[ii, t]);
                    }
                }
                model.AddConstr(lhsa <= maxTotalAdditions, "totalAdditions");
                */


                // Optimize
                model.Optimize();
                int optimstatus = model.Status;

                switch (model.Status)
                {
                    case GRB.Status.OPTIMAL:
                        {
                            double objval = model.ObjVal;
                            int counter = 0;

                            // Update solution with new calculated quantities
                            for (int t = 0; t < numberOfPeriods; t++)
                            {
                                sol.depot.deliveredQuantities[t] = 0;
                                sol.depot.productionRates[t] = (int)Math.Round(p[t].X);
                            }

                            for (int i = 0; i < sol.customers.Count; i++)
                            {
                                Node cust = sol.customers[i];

                                for (int t = 0; t < numberOfPeriods; t++)
                                {
                                    CustDelivery cd = cust.horizonDeliveryServices[t];
                                    if (cd.route != null)
                                        cd.route.load -= cd.quantity;

                                    cust.deliveredQuantities[t] = (int)Math.Round(q[i, t].X); //update delivery quantity
                                    sol.depot.deliveredQuantities[t] -= cust.deliveredQuantities[t]; // OR +=;
                                    // update customer delivery and vehicle load
                                    if (cd.route != null)
                                    {
                                        cd.quantity = cust.deliveredQuantities[t];
                                        cd.route.load += cd.quantity;
                                        cd.route.SetLoadAndCostLists(t, sol.model);
                                    }
                                    else if (cd.route == null && GlobalUtils.IsEqual(addZ[i, t].X, 1)) // add this 
                                    {
                                        counter += 1;
                                        cust.visitSchedule[t] = true;
                                        cd.route = rtAddCostRoute[i, t];
                                        cd.quantity = cust.deliveredQuantities[t];
                                        cd.route.load += cd.quantity;
                                        cd.route.nodes.Insert(1, cust);

                                        //fix route routing cost
                                        Node me = cd.route.nodes[0];
                                        Node next = cd.route.nodes[2];
                                        cd.route.totalRoutingCost -= sol.model.distMatrix[me.uid, next.uid];
                                        cd.route.totalRoutingCost += sol.model.distMatrix[me.uid, cust.uid];
                                        cd.route.totalRoutingCost += sol.model.distMatrix[cust.uid, next.uid];

                                        cd.route.SetLoadAndCostLists(t, sol.model);
                                    }
                                }
                                cust.CalculateInventoryLevels();
                            }
                            sol.depot.CalculateInventoryLevels();
                            GlobalUtils.writeToConsole("Total additions {0}", counter);

                            //Fix solution objective
                            sol.routingCost = Solution.EvaluateRoutingObjectivefromScratch(sol);
                            sol.holdingCost = Solution.EvaluateInventoryObjectivefromScratch(sol);
                            //sol.totalUnitProductionCost = Solution.EvaluateUnitProductionObjectivefromScratch(sol);
                            //sol.setupProductionCost = Solution.EvaluateSetupProductionObjectivefromScratch(sol);
                            sol.totalObjective = sol.routingCost + sol.holdingCost + sol.totalUnitProductionCost + sol.setupProductionCost;

                            if (!Solution.checkSolutionStatus(sol.TestEverythingFromScratch()))
                                Console.WriteLine("Lefteri ftiakse tis malakies sou sto production Mip with infeasibilities and insertion");

                            //GlobalUtils.writeToConsole("VIOLATIONS AFTER LP {0}", sol.calcVehicleLoadViolations());
                            TSP_GRB_Solver.SolveTSP(sol);
                            break;
                        }
                    case GRB.Status.INFEASIBLE:
                        {
                            Console.WriteLine("Model is infeasible");
                            // it could call itself with increased number of total insertions

                            // compute and write out IIS
                            //model.ComputeIIS();
                            //model.Write("model.ilp");
                            return false;
                        }
                    case GRB.Status.UNBOUNDED:
                        {
                            Console.WriteLine("Model is unbounded");
                            return false;
                        }
                    default:
                        {
                            Console.WriteLine("Optimization was stopped with status = "
                                              + optimstatus);
                            return false;
                        }

                }

                // Dispose of model
                model.Dispose();
                //gurobiEnv.Dispose();
            }
            catch (GRBException e)
            {
                Console.WriteLine("Error code: " + e.ErrorCode + ". " + e.Message);
            }

            return true;
        }


        //MIP formulation for simultaneously re-optimizing the delivery quantities of all customers and the production quantities 
        // as well allowing vehicle capacity to be violated. The violation is penalized and insertions and removals per route are allowed
        public static bool runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemoval(ref Solution sol, bool isCont=true)
        {            
            //Solution back_sol = new Solution(sol, 0.0);
            //if (!Solution.checkSolutionStatus(sol.TestEverythingFromScratch()))
                //throw new Exception("Production Mip with infeasibilities and insertion/removal - ERROR IN INPUT SOLUTION");
            
            int numberOfPeriods = sol.periods.Count;
            int numberOfCustomers = sol.customers.Count;

            int unitCapPenalty = 1000; //Note the coefficient for penalizing capacity violation (per unit)
            int maxRouteAdditions = 3; // number of max additions per route
            int maxTotalAdditions = 10;
            int maxRouteRemovals = 3; // number of max removals per route
            int maxTotalRemovals = 10;
            int effectiveCapacity = sol.periods[0].periodRoutes[0].effectiveCapacity;
            int actualCapacity = sol.periods[0].periodRoutes[0].realCapacity;
            int excessSlack = effectiveCapacity - actualCapacity; // the max excess of capacity per route allowed

            int maxNumberVehUsed = 0;
            for (int t = 0; t < sol.periods.Count; t++)
            {
                Period per = sol.periods[t];
                maxNumberVehUsed = Math.Max(maxNumberVehUsed, per.periodRoutes.Count);
            }
            double[,] UBq = new double[numberOfCustomers, numberOfPeriods]; //upper bound of delivered quantities to customer
            double[] UBp = new double[numberOfPeriods]; //upper bound of delivered quantities to customer
            double totalProdQuantity = 0.0;
            double[,] rtAddCost = new double[numberOfCustomers, numberOfPeriods];
            Route[,] rtAddCostRoute = new Route[numberOfCustomers, numberOfPeriods];
            double[,] rtRemCost = new double[numberOfCustomers, numberOfPeriods];
            Route[,] rtRemCostRoute = new Route[numberOfCustomers, numberOfPeriods];

            //calculate UBq 
            for (int i = 0; i < sol.customers.Count; i++)
            {
                Node cust = sol.customers[i];
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    int aggrCap = cust.stockMaximumLevel + cust.productRate[t];
                    int totalDemandUntilEnd = 0;
                    for (int tt = t; tt < numberOfPeriods; ++tt)
                    {
                        totalDemandUntilEnd += cust.productRate[tt];
                    }
                    //if (!cust.visitSchedule[t])
                    //    UBq[i, t] = 0;
                    //else
                    UBq[i, t] = (double)GlobalUtils.Min(aggrCap, actualCapacity, totalDemandUntilEnd);
                }
            }

            for (int t = 0; t < numberOfPeriods; ++t)
            {
                totalProdQuantity += sol.depot.productionRates[t];
                if (!sol.depot.open[t])
                    UBp[t] = 0;
                else
                    UBp[t] = sol.depot.productionCapacity;
            }

            for (int i = 0; i < sol.customers.Count; i++)
            {
                Node cust = sol.customers[i];
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    rtAddCost[i, t] = int.MaxValue;
                    rtAddCostRoute[i, t] = null;
                    rtRemCost[i, t] = 0;
                    rtRemCostRoute[i, t] = null;
                 }
            }

            // calculate the possible savings
            for (int t = 0; t < sol.periods.Count; t++)
            {
                Period pr = sol.periods[t];
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    if (!cust.visitSchedule[t]) //if customer is not serviced then and only then can be added
                    {
                        for (int r = 0; r < pr.periodRoutes.Count; r++)
                        {
                            Route rt = pr.periodRoutes[r];
                            if (!rt.nodes.Contains(cust)) //always true
                            {
                                for (int ii = 0; ii < rt.nodes.Count - 1; ii++)
                                {
                                    Node prev = rt.nodes[ii];
                                    Node next = rt.nodes[ii + 1];

                                    double costRemoved = sol.model.distMatrix[prev.uid, next.uid];
                                    double costAdded = sol.model.distMatrix[prev.uid, cust.uid] + sol.model.distMatrix[cust.uid, next.uid];

                                    if (costAdded - costRemoved < rtAddCost[cust.uid - 1, t])
                                    {
                                        rtAddCost[cust.uid - 1, t] = costAdded - costRemoved;
                                        rtAddCostRoute[cust.uid - 1, t] = rt;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            for (int t = 0; t < sol.periods.Count; t++)
            {
                Period pr = sol.periods[t];
                for (int r = 0; r < pr.periodRoutes.Count; r++)
                {
                    Route rt = pr.periodRoutes[r];
                    for (int i = 1; i < rt.nodes.Count - 1; i++)
                    {
                        Node prev = rt.nodes[i - 1];
                        Node me = rt.nodes[i];
                        Node next = rt.nodes[i + 1];

                        double costRemoved = sol.model.distMatrix[prev.uid, me.uid] + sol.model.distMatrix[me.uid, next.uid];
                        double costAdded = sol.model.distMatrix[prev.uid, next.uid];

                        rtRemCost[me.uid - 1, t] = costAdded - costRemoved;
                        rtRemCostRoute[me.uid - 1, t] = rt;
                    }
                }
            }


            try
            {
                GRBEnv gurobiEnv = new GRBEnv();
                GRBModel model = new GRBModel(gurobiEnv);

                // Params
                model.ModelName = "allCustomersAndProductionQuantitiesLPVehCapInfeas" + sol.ellapsedMs;
                model.Parameters.OutputFlag = 0;
                model.Parameters.Threads = threadLimit;

                // Decision variables
                GRBVar[] I_depot = new GRBVar[numberOfPeriods]; // depot inventory
                GRBVar[] p = new GRBVar[numberOfPeriods]; // depot production
                GRBVar[,] I_cust = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer inventory
                GRBVar[,] q = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer delivery quantities
                GRBVar[,] e = new GRBVar[maxNumberVehUsed, numberOfPeriods]; // capacity violations
                GRBVar[,] addZ = new GRBVar[numberOfCustomers, numberOfPeriods]; // binary addition of visits
                GRBVar[,] remZ = new GRBVar[numberOfCustomers, numberOfPeriods]; // binary deletion of visits
                               
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    I_depot[t] = model.AddVar(0.0, sol.depot.stockMaximumLevel, sol.depot.unitHoldingCost, GRB.CONTINUOUS, "I_d^" + t);
                    if (isCont)
                        p[t] = model.AddVar(0.0, UBp[t], 0, GRB.CONTINUOUS, "p_d^" + t); //@greg productionCapacity: is  this the maximum quantity we can produce per dayq
                    else
                        p[t] = model.AddVar(0.0, UBp[t], 0, GRB.INTEGER, "p_d^" + t); //@greg productionCapacity: is  this the maximum quantity we can produce per dayq

                    Period per = sol.periods[t];
                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route rt = per.periodRoutes[r];
                        e[r, t] = model.AddVar(0.0, excessSlack, unitCapPenalty, GRB.CONTINUOUS, "e_" + r + "^" + t);
                    }
                }

                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        if (isCont)
                            q[i, t] = model.AddVar(0.0, UBq[i, t], 0.0, GRB.CONTINUOUS, "q_" + i + "^" + t);
                        else
                            q[i, t] = model.AddVar(0.0, UBq[i, t], 0.0, GRB.INTEGER, "q_" + i + "^" + t);
                        I_cust[i, t] = model.AddVar(0.0, cust.stockMaximumLevel, cust.unitHoldingCost, GRB.CONTINUOUS, "I_" + i + " ^ " + t);
                        if (cust.visitSchedule[t]) // if is not route it cannot be removed 
                        {
                            addZ[i, t] = model.AddVar(0.0, 0.0, rtAddCost[i, t], GRB.BINARY, "addZ_" + i + " ^ " + t);
                            remZ[i, t] = model.AddVar(0.0, 1.0, rtRemCost[i, t], GRB.BINARY, "remZ_" + i + " ^ " + t);
                        }
                        else
                        {
                            addZ[i, t] = model.AddVar(0.0, 1.0, rtAddCost[i, t], GRB.BINARY, "addZ_" + i + " ^ " + t);
                            remZ[i, t] = model.AddVar(0.0, 0.0, rtRemCost[i, t], GRB.BINARY, "remZ_" + i + " ^ " + t);
                        }
                    }
                }

                // Objective
                model.ModelSense = GRB.MINIMIZE;

                // Constraints 
                // 1. Depot inventory flow 
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    GRBLinExpr lhs = 0.0;
                    lhs.AddTerm(1.0, I_depot[t]);
                    for (int i = 0; i < sol.customers.Count; i++)
                    {
                        lhs.AddTerm(1.0, q[i, t]);
                    }
                    if (t == 0)
                        model.AddConstr(lhs == sol.depot.startingInventory + p[t], "depot_inv_" + t);
                    else
                        model.AddConstr(lhs == I_depot[t - 1] + p[t], "depot_inv_" + t);
                }

                // 2. Customer inventory flow
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        GRBLinExpr lhs = 0.0;
                        lhs.AddTerm(1.0, I_cust[i, t]);
                        if (t == 0)
                            model.AddConstr(lhs == cust.startingInventory + q[i, t] - cust.productRate[t], "cust_inv" + i + " ^ " + t);
                        else
                            model.AddConstr(lhs == I_cust[i, t - 1] + q[i, t] - cust.productRate[t], "cust_inv" + i + " ^ " + t);
                    }
                }

                // 3. Customer delivery capacity
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        GRBLinExpr lhs = 0.0;
                        lhs.AddTerm(1.0, q[i, t]);
                        if (t == 0)
                            model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - cust.startingInventory, "cust_ml" + i + " ^ " + t);
                        else
                            model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - I_cust[i, t - 1], "cust_ml" + i + " ^ " + t);
                    }
                }

                // 5. Total production quantity
                GRBLinExpr lhsp = 0.0;
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    lhsp.AddTerm(1.0, p[t]);
                }
                model.AddConstr(lhsp == totalProdQuantity, "total_prod");

                // 4. Customer delivery vehicle capacity
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    Period per = sol.periods[t];

                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route route = per.periodRoutes[r];
                        GRBLinExpr lhs = 0.0;

                        //for (int i = 1; i < route.nodes.Count - 1; i++)
                        for (int i = 0; i < sol.customers.Count; i++)
                        {
                            Node cust = sol.customers[i];
                            if (route.nodes.Contains(cust) || Object.ReferenceEquals(rtAddCostRoute[i, t], route))
                            {
                                int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                lhs.AddTerm(1.0, q[ii, t]);
                            }
                        }
                        if (t==0 && sol.model.input.type == PRP_DATASET_VERSION.BOUDIA_FMT)
                            model.AddConstr(lhs <= actualCapacity, "cust_vehcap_" + r + " ^ " + t); //NOTE: effective capacity
                        else
                            model.AddConstr(lhs <= effectiveCapacity, "cust_vehcap_" + r + " ^ " + t); //NOTE: effective capacity
                    }
                }

                // 6. excessive slack
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    Period per = sol.periods[t];
                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route route = per.periodRoutes[r];
                        GRBLinExpr lhs = e[r, t];
                        //for (int i = 1; i < route.nodes.Count - 1; i++)
                        for (int i = 0; i < sol.customers.Count; i++)
                        {
                            Node cust = sol.customers[i];
                            if (route.nodes.Contains(cust) || Object.ReferenceEquals(rtAddCostRoute[i, t], route))
                            {
                                // if q{ii,t)is positive while not serviced it is surely in this route
                                int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                lhs.AddTerm(-1.0, q[ii, t]);
                            }
                        }
                        model.AddConstr(lhs >= -actualCapacity, "e" + r + " ^ " + t); //NOTE: effective capacity
                    }
                }

                // 7. addition constraints
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        if (!cust.visitSchedule[t])
                        {
                            GRBLinExpr lhs = q[i, t];
                            model.AddConstr(lhs <= (addZ[i, t]) * UBq[i, t], "add_visits" + i + " ^ " + t);
                        }
                    }
                }

                // 10. removal constraints
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        if (cust.visitSchedule[t])
                        {
                            GRBLinExpr lhs = q[i, t];
                            model.AddConstr(lhs <= (1 - remZ[i, t]) * UBq[i, t], "rem_visits" + i + " ^ " + t);
                        }
                    }
                }

                // 11. One removal per route
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    Period per = sol.periods[t];

                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route route = per.periodRoutes[r];
                        GRBLinExpr lhs = 0.0;

                        for (int i = 1; i < route.nodes.Count - 1; i++)
                        {
                            Node cust = route.nodes[i];
                            int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                            lhs.AddTerm(1.0, remZ[ii, t]);
                        }
                        model.AddConstr(lhs <= maxRouteRemovals, "one_removal" + r + " ^ " + t);
                    }
                }

                // 8. One addition per route (to be tuned)
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    Period per = sol.periods[t];

                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route route = per.periodRoutes[r];
                        GRBLinExpr lhs = 0.0;

                        //for (int i = 1; i < route.nodes.Count - 1; i++)
                        for (int i = 0; i < sol.customers.Count; i++)
                        {
                            Node cust = sol.customers[i];
                            if (Object.ReferenceEquals(rtAddCostRoute[i, t], route))
                            {
                                int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                lhs.AddTerm(1.0, addZ[ii, t]);
                            }
                        }
                        model.AddConstr(lhs <= maxRouteAdditions, "one_addition" + r + " ^ " + t);
                    }
                }

                // 12. Total addition and removals bounds 
                /*
                GRBLinExpr lhsa = 0.0;
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    for (int i = 0; i < sol.customers.Count; i++)
                    {
                        Node cust = sol.customers[i];
                        int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                        lhsa.AddTerm(1.0, addZ[ii, t]);
                    }
                }
                model.AddConstr(lhsa <= maxTotalAdditions, "totalAdditions");

                
                GRBLinExpr lhsb = 0.0;
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    for (int i = 0; i < sol.customers.Count; i++)
                    {
                        Node cust = sol.customers[i];
                        int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                        lhsa.AddTerm(1.0, remZ[ii, t]);
                    }
                }
                model.AddConstr(lhsa <= maxTotalRemovals, "totalRemovals");
                */


                // Optimize
                model.Optimize();
                int optimstatus = model.Status;

                switch (model.Status)
                {
                    case GRB.Status.OPTIMAL:
                        {
                            double objval = model.ObjVal;
                            int counter = 0;
                            int counter2 = 0;

                            if (existsViolatedQuantity(ref sol, q, p))
                            {
                                runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemoval(ref sol, false);
                                break;
                            } 
                            else 
                            {
                                // Update solution with new calculated quantities
                                for (int t = 0; t < numberOfPeriods; t++)
                                {
                                    sol.depot.deliveredQuantities[t] = 0;
                                    sol.depot.productionRates[t] = (int)Math.Round(p[t].X);
                                }

                                for (int i = 0; i < sol.customers.Count; i++)
                                {
                                    Node cust = sol.customers[i];

                                    for (int t = 0; t < numberOfPeriods; t++)
                                    {
                                        CustDelivery cd = cust.horizonDeliveryServices[t];
                                        if (cd.route != null)
                                            cd.route.load -= cd.quantity;

                                        cust.deliveredQuantities[t] = (int)Math.Round(q[i, t].X); //update delivery quantity
                                        cd.quantity = cust.deliveredQuantities[t];
                                        sol.depot.deliveredQuantities[t] -= cust.deliveredQuantities[t]; // OR +=;
                                        // update customer delivery and vehicle load
                                        if (cd.route != null && !GlobalUtils.IsEqual(addZ[i, t].X, 1.0) && !GlobalUtils.IsEqual(remZ[i, t].X, 1.0)) //remaining
                                        {
                                            cd.route.load += cd.quantity;
                                            cd.route.SetLoadAndCostLists(t, sol.model);
                                        }
                                        else if (cd.route != null && GlobalUtils.IsEqual(remZ[i, t].X, 1.0)) //removed
                                        {
                                            counter2 += 1;
                                            cust.visitSchedule[t] = false;
                                            Route old_rt = cd.route;
                                            for (int ii = 1; ii < old_rt.nodes.Count - 1; ii++)
                                            {
                                                Node me = old_rt.nodes[ii];
                                                if (me.uid == cust.uid)
                                                {
                                                    //Route removed from
                                                    old_rt.nodes.RemoveAt(ii);
                                                    old_rt.calculateRoutingCost(sol.model);
                                                    old_rt.SetLoadAndCostLists(t, sol.model);
                                                }
                                                // remove cust delivery
                                                cd.route = null;
                                            }
                                        }
                                        else if (cd.route == null && GlobalUtils.IsEqual(addZ[i, t].X, 1.0)) // added 
                                        {
                                            counter += 1;
                                            cust.visitSchedule[t] = true;
                                            cd.route = rtAddCostRoute[i, t];
                                            cd.route.load += cd.quantity;
                                            cd.route.nodes.Insert(1, cust);

                                            //fix route routing cost
                                            cd.route.calculateRoutingCost(sol.model);
                                            cd.route.SetLoadAndCostLists(t, sol.model);
                                        }
                                    }
                                    cust.CalculateInventoryLevels();
                                }
                                sol.depot.CalculateInventoryLevels();
                                GlobalUtils.writeToConsole("Total additions {0}", counter);
                                GlobalUtils.writeToConsole("Total deletions {0}", counter2);

                                //Fix solution objective
                                sol.routingCost = Solution.EvaluateRoutingObjectivefromScratch(sol);
                                sol.holdingCost = Solution.EvaluateInventoryObjectivefromScratch(sol);
                                //sol.totalUnitProductionCost = Solution.EvaluateUnitProductionObjectivefromScratch(sol);
                                //sol.setupProductionCost = Solution.EvaluateSetupProductionObjectivefromScratch(sol);
                                sol.totalObjective = sol.routingCost + sol.holdingCost + sol.totalUnitProductionCost + sol.setupProductionCost;

                                sol.status = sol.TestEverythingFromScratch();

                                if (!Solution.checkSolutionStatus(sol.status))
                                {
                                    //runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemoval(ref back_sol, false);
                                    //sol = new Solution(back_sol, 0.0);
                                    //TSP_GRB_Solver.SolveTSP(sol);
                                    //TODO: FIX PROBLEMS THIS SHOULD NOT RETURN
                                    //back_sol.SaveToFile("mip_failed_sol");
                                    Console.WriteLine("Lefteri ftiakse tis malakies sou sto production Mip with infeasibilities and insertion/removal");
                                    return false;
                                }

                                //Console.WriteLine("VIOLATIONS AFTER LP {0}", sol.calcVehicleLoadViolations());
                                TSP_GRB_Solver.SolveTSP(sol);
                                break;
                            }
                        }
                    case GRB.Status.INFEASIBLE:
                        {
                            Console.WriteLine("Model is infeasible");
                            // it could call itself with increased number of total insertions

                            // compute and write out IIS
                            //model.ComputeIIS();
                            //model.Write("model.ilp");
                            return false;
                        }
                    case GRB.Status.UNBOUNDED:
                        {
                            Console.WriteLine("Model is unbounded");
                            return false;
                        }
                    default:
                        {
                            Console.WriteLine("Optimization was stopped with status = "
                                              + optimstatus);
                            return false;
                        }

                }

                // Dispose of model
                model.Dispose();
                //gurobiEnv.Dispose();
            }
            catch (GRBException e)
            {
                Console.WriteLine("Error code: " + e.ErrorCode + ". " + e.Message);
            }

            return true;
        }


        //MIP formulation for simultaneously re-optimizing the delivery quantities of all customers and the production quantities 
        // as well allowing vehicle capacity to be violated. The violation is penalized and insertions and removals per route are allowed.
        //The production days may be altered as well
        public static bool runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemovalProdDays(ref Solution sol, bool isCont = true)
        {
            //Solution back_sol = new Solution(sol, 0.0);
            //if (!Solution.checkSolutionStatus(sol.TestEverythingFromScratch()))
            //throw new Exception("Production Mip with infeasibilities and insertion/removal - ERROR IN INPUT SOLUTION");

            int numberOfPeriods = sol.periods.Count;
            int numberOfCustomers = sol.customers.Count;

            int unitCapPenalty = 1000; //Note the coefficient for penalizing capacity violation (per unit)
            int maxRouteAdditions = 3; // number of max additions per route
            int maxTotalAdditions = 10;
            int maxRouteRemovals = 3; // number of max removals per route
            int maxTotalRemovals = 10;
            int prodMaxChanges = 4;
            int effectiveCapacity = sol.periods[0].periodRoutes[0].effectiveCapacity;
            int actualCapacity = sol.periods[0].periodRoutes[0].realCapacity;
            int excessSlack = effectiveCapacity - actualCapacity; // the max excess of capacity per route allowed

            int maxNumberVehUsed = 0;
            for (int t = 0; t < sol.periods.Count; t++)
            {
                Period per = sol.periods[t];
                maxNumberVehUsed = Math.Max(maxNumberVehUsed, per.periodRoutes.Count);
            }
            double[,] UBq = new double[numberOfCustomers, numberOfPeriods]; //upper bound of delivered quantities to customer
            double[] UBp = new double[numberOfPeriods]; //upper bound of delivered quantities to customer
            double totalProdQuantity = 0.0;
            double[,] rtAddCost = new double[numberOfCustomers, numberOfPeriods];
            Route[,] rtAddCostRoute = new Route[numberOfCustomers, numberOfPeriods];
            double[,] rtRemCost = new double[numberOfCustomers, numberOfPeriods];
            Route[,] rtRemCostRoute = new Route[numberOfCustomers, numberOfPeriods];

            //calculate UBq 
            for (int i = 0; i < sol.customers.Count; i++)
            {
                Node cust = sol.customers[i];
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    int aggrCap = cust.stockMaximumLevel + cust.productRate[t];
                    int totalDemandUntilEnd = 0;
                    for (int tt = t; tt < numberOfPeriods; ++tt)
                    {
                        totalDemandUntilEnd += cust.productRate[tt];
                    }
                    UBq[i, t] = (double)GlobalUtils.Min(aggrCap, actualCapacity, totalDemandUntilEnd);
                }
            }

            for (int t = 0; t < numberOfPeriods; ++t)
            {
                totalProdQuantity += sol.depot.productionRates[t];
                UBp[t] = sol.depot.productionCapacity;
            }

            for (int i = 0; i < sol.customers.Count; i++)
            {
                Node cust = sol.customers[i];
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    rtAddCost[i, t] = int.MaxValue;
                    rtAddCostRoute[i, t] = null;
                    rtRemCost[i, t] = 0;
                    rtRemCostRoute[i, t] = null;
                }
            }

            // calculate the possible savings
            for (int t = 0; t < sol.periods.Count; t++)
            {
                Period pr = sol.periods[t];
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    if (!cust.visitSchedule[t]) //if customer is not serviced then and only then can be added
                    {
                        for (int r = 0; r < pr.periodRoutes.Count; r++)
                        {
                            Route rt = pr.periodRoutes[r];
                            if (!rt.nodes.Contains(cust)) //always true
                            {
                                for (int ii = 0; ii < rt.nodes.Count - 1; ii++)
                                {
                                    Node prev = rt.nodes[ii];
                                    Node next = rt.nodes[ii + 1];

                                    double costRemoved = sol.model.distMatrix[prev.uid, next.uid];
                                    double costAdded = sol.model.distMatrix[prev.uid, cust.uid] + sol.model.distMatrix[cust.uid, next.uid];

                                    if (costAdded - costRemoved < rtAddCost[cust.uid - 1, t])
                                    {
                                        rtAddCost[cust.uid - 1, t] = costAdded - costRemoved;
                                        rtAddCostRoute[cust.uid - 1, t] = rt;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            for (int t = 0; t < sol.periods.Count; t++)
            {
                Period pr = sol.periods[t];
                for (int r = 0; r < pr.periodRoutes.Count; r++)
                {
                    Route rt = pr.periodRoutes[r];
                    for (int i = 1; i < rt.nodes.Count - 1; i++)
                    {
                        Node prev = rt.nodes[i - 1];
                        Node me = rt.nodes[i];
                        Node next = rt.nodes[i + 1];

                        double costRemoved = sol.model.distMatrix[prev.uid, me.uid] + sol.model.distMatrix[me.uid, next.uid];
                        double costAdded = sol.model.distMatrix[prev.uid, next.uid];

                        rtRemCost[me.uid - 1, t] = costAdded - costRemoved;
                        rtRemCostRoute[me.uid - 1, t] = rt;
                    }
                }
            }
            
            try
            {
                GRBEnv gurobiEnv = new GRBEnv();
                GRBModel model = new GRBModel(gurobiEnv);

                // Params
                model.ModelName = "allCustomersAndProductionQuantitiesLPVehCapInfeas" + sol.ellapsedMs;
                model.Parameters.OutputFlag = 0;
                model.Parameters.Threads = threadLimit;

                // Decision variables
                GRBVar[] I_depot = new GRBVar[numberOfPeriods]; // depot inventory
                GRBVar[] p = new GRBVar[numberOfPeriods]; // depot production
                GRBVar[] s = new GRBVar[numberOfPeriods]; // depot production
                GRBVar[,] I_cust = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer inventory
                GRBVar[,] q = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer delivery quantities
                GRBVar[,] e = new GRBVar[maxNumberVehUsed, numberOfPeriods]; // capacity violations
                GRBVar[,] addZ = new GRBVar[numberOfCustomers, numberOfPeriods]; // binary addition of visits
                GRBVar[,] remZ = new GRBVar[numberOfCustomers, numberOfPeriods]; // binary deletion of visits

                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    I_depot[t] = model.AddVar(0.0, sol.depot.stockMaximumLevel, sol.depot.unitHoldingCost, GRB.CONTINUOUS, "I_d^" + t);
                    if (isCont)
                        p[t] = model.AddVar(0.0, UBp[t], 0, GRB.CONTINUOUS, "p_d^" + t); //@greg productionCapacity: is  this the maximum quantity we can produce per dayq
                    else
                        p[t] = model.AddVar(0.0, UBp[t], 0, GRB.INTEGER, "p_d^" + t); //@greg productionCapacity: is  this the maximum quantity we can produce per dayq
                    s[t] = model.AddVar(0.0, 1.0, sol.setupProductionCost, GRB.BINARY, "s^ " + t);


                    Period per = sol.periods[t];
                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route rt = per.periodRoutes[r];
                        e[r, t] = model.AddVar(0.0, excessSlack, unitCapPenalty, GRB.CONTINUOUS, "e_" + r + "^" + t);
                    }
                }

                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        if (isCont)
                            q[i, t] = model.AddVar(0.0, UBq[i, t], 0.0, GRB.CONTINUOUS, "q_" + i + "^" + t);
                        else
                            q[i, t] = model.AddVar(0.0, UBq[i, t], 0.0, GRB.INTEGER, "q_" + i + "^" + t);
                        I_cust[i, t] = model.AddVar(0.0, cust.stockMaximumLevel, cust.unitHoldingCost, GRB.CONTINUOUS, "I_" + i + " ^ " + t);
                        if (cust.visitSchedule[t]) // if is not route it cannot be removed 
                        {
                            addZ[i, t] = model.AddVar(0.0, 0.0, rtAddCost[i, t], GRB.BINARY, "addZ_" + i + " ^ " + t);
                            remZ[i, t] = model.AddVar(0.0, 1.0, rtRemCost[i, t], GRB.BINARY, "remZ_" + i + " ^ " + t);
                        }
                        else
                        {
                            addZ[i, t] = model.AddVar(0.0, 1.0, rtAddCost[i, t], GRB.BINARY, "addZ_" + i + " ^ " + t);
                            remZ[i, t] = model.AddVar(0.0, 0.0, rtRemCost[i, t], GRB.BINARY, "remZ_" + i + " ^ " + t);
                        }
                    }
                }

                // Objective
                model.ModelSense = GRB.MINIMIZE;

                // Constraints 
                // 1. Depot inventory flow 
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    GRBLinExpr lhs = 0.0;
                    lhs.AddTerm(1.0, I_depot[t]);
                    for (int i = 0; i < sol.customers.Count; i++)
                    {
                        lhs.AddTerm(1.0, q[i, t]);
                    }
                    if (t == 0)
                        model.AddConstr(lhs == sol.depot.startingInventory + p[t], "depot_inv_" + t);
                    else
                        model.AddConstr(lhs == I_depot[t - 1] + p[t], "depot_inv_" + t);
                }

                // 1. Depot inventory flow 
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    GRBLinExpr lhs = p[t];
                    if ((t == 0) && sol.model.input.type == PRP_DATASET_VERSION.BOUDIA_FMT) //TODO: change hardcoded if boudia check
                        model.AddConstr(lhs == 0, "depot_setup_" + t);
                    else
                        model.AddConstr(lhs <= sol.depot.productionCapacity*s[t], "depot_setup_" + t);
                }

                // 1c. Depot prod changes
                
                GRBLinExpr lhsc = 0;
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    if (sol.depot.open[t])
                        lhsc += (1-s[t]);
                    else
                        lhsc += s[t];

                    model.AddConstr(lhsc >= 1, "prod chanches");
                }
                

                // 2. Customer inventory flow
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        GRBLinExpr lhs = 0.0;
                        lhs.AddTerm(1.0, I_cust[i, t]);
                        if (t == 0)
                            model.AddConstr(lhs == cust.startingInventory + q[i, t] - cust.productRate[t], "cust_inv" + i + " ^ " + t);
                        else
                            model.AddConstr(lhs == I_cust[i, t - 1] + q[i, t] - cust.productRate[t], "cust_inv" + i + " ^ " + t);
                    }
                }

                // 3. Customer delivery capacity
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        GRBLinExpr lhs = 0.0;
                        lhs.AddTerm(1.0, q[i, t]);
                        if (t == 0)
                            model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - cust.startingInventory, "cust_ml" + i + " ^ " + t);
                        else
                            model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - I_cust[i, t - 1], "cust_ml" + i + " ^ " + t);
                    }
                }

                // 5. Total production quantity
                GRBLinExpr lhsp = 0.0;
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    lhsp.AddTerm(1.0, p[t]);
                }
                model.AddConstr(lhsp == totalProdQuantity, "total_prod");

                // 4. Customer delivery vehicle capacity
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    Period per = sol.periods[t];

                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route route = per.periodRoutes[r];
                        GRBLinExpr lhs = 0.0;

                        //for (int i = 1; i < route.nodes.Count - 1; i++)
                        for (int i = 0; i < sol.customers.Count; i++)
                        {
                            Node cust = sol.customers[i];
                            if (route.nodes.Contains(cust) || Object.ReferenceEquals(rtAddCostRoute[i, t], route))
                            {
                                int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                lhs.AddTerm(1.0, q[ii, t]);
                            }
                        }
                        if (t == 0 && sol.model.input.type == PRP_DATASET_VERSION.BOUDIA_FMT)
                            model.AddConstr(lhs <= actualCapacity, "cust_vehcap_" + r + " ^ " + t); //NOTE: effective capacity
                        else
                            model.AddConstr(lhs <= effectiveCapacity, "cust_vehcap_" + r + " ^ " + t); //NOTE: effective capacity
                    }
                }

                // 6. excessive slack
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    Period per = sol.periods[t];
                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route route = per.periodRoutes[r];
                        GRBLinExpr lhs = e[r, t];
                        //for (int i = 1; i < route.nodes.Count - 1; i++)
                        for (int i = 0; i < sol.customers.Count; i++)
                        {
                            Node cust = sol.customers[i];
                            if (route.nodes.Contains(cust) || Object.ReferenceEquals(rtAddCostRoute[i, t], route))
                            {
                                // if q{ii,t)is positive while not serviced it is surely in this route
                                int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                lhs.AddTerm(-1.0, q[ii, t]);
                            }
                        }
                        model.AddConstr(lhs >= -actualCapacity, "e" + r + " ^ " + t); //NOTE: effective capacity
                    }
                }

                // 7. addition constraints
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        if (!cust.visitSchedule[t])
                        {
                            GRBLinExpr lhs = q[i, t];
                            model.AddConstr(lhs <= (addZ[i, t]) * UBq[i, t], "add_visits" + i + " ^ " + t);
                        }
                    }
                }

                // 10. removal constraints
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        if (cust.visitSchedule[t])
                        {
                            GRBLinExpr lhs = q[i, t];
                            model.AddConstr(lhs <= (1 - remZ[i, t]) * UBq[i, t], "rem_visits" + i + " ^ " + t);
                        }
                    }
                }

                // 11. One removal per route
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    Period per = sol.periods[t];

                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route route = per.periodRoutes[r];
                        GRBLinExpr lhs = 0.0;

                        for (int i = 1; i < route.nodes.Count - 1; i++)
                        {
                            Node cust = route.nodes[i];
                            int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                            lhs.AddTerm(1.0, remZ[ii, t]);
                        }
                        model.AddConstr(lhs <= maxRouteRemovals, "one_removal" + r + " ^ " + t);
                    }
                }

                // 8. One addition per route (to be tuned)
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    Period per = sol.periods[t];

                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route route = per.periodRoutes[r];
                        GRBLinExpr lhs = 0.0;

                        //for (int i = 1; i < route.nodes.Count - 1; i++)
                        for (int i = 0; i < sol.customers.Count; i++)
                        {
                            Node cust = sol.customers[i];
                            if (Object.ReferenceEquals(rtAddCostRoute[i, t], route))
                            {
                                int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                lhs.AddTerm(1.0, addZ[ii, t]);
                            }
                        }
                        model.AddConstr(lhs <= maxRouteAdditions, "one_addition" + r + " ^ " + t);
                    }
                }

                // 12. Total addition and removals bounds 
                /*
                GRBLinExpr lhsa = 0.0;
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    for (int i = 0; i < sol.customers.Count; i++)
                    {
                        Node cust = sol.customers[i];
                        int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                        lhsa.AddTerm(1.0, addZ[ii, t]);
                    }
                }
                model.AddConstr(lhsa <= maxTotalAdditions, "totalAdditions");

                
                GRBLinExpr lhsb = 0.0;
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    for (int i = 0; i < sol.customers.Count; i++)
                    {
                        Node cust = sol.customers[i];
                        int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                        lhsa.AddTerm(1.0, remZ[ii, t]);
                    }
                }
                model.AddConstr(lhsa <= maxTotalRemovals, "totalRemovals");
                */


                // Optimize
                model.Optimize();
                int optimstatus = model.Status;

                switch (model.Status)
                {
                    case GRB.Status.OPTIMAL:
                        {
                            double objval = model.ObjVal;
                            int counter = 0;
                            int counter2 = 0;
                            int counter3 = 0;
                            if (existsViolatedQuantity(ref sol, q, p))
                            {
                                runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemovalProdDays(ref sol, false);
                                break;
                            }
                            else
                            {
                                // Update solution with new calculated quantities
                                for (int t = 0; t < numberOfPeriods; t++)
                                {
                                    if ((!sol.depot.open[t] && (int)Math.Round(p[t].X) > 0) || (sol.depot.open[t] && (int)Math.Round(p[t].X) == 0))
                                        counter3 += 1;
                                    sol.depot.open[t] = false;
                                    sol.depot.deliveredQuantities[t] = 0;
                                    sol.depot.productionRates[t] = (int)Math.Round(p[t].X);
                                    if (sol.depot.productionRates[t] > 0)
                                        sol.depot.open[t] = true;
                                }

                                for (int i = 0; i < sol.customers.Count; i++)
                                {
                                    Node cust = sol.customers[i];

                                    for (int t = 0; t < numberOfPeriods; t++)
                                    {
                                        CustDelivery cd = cust.horizonDeliveryServices[t];
                                        if (cd.route != null)
                                            cd.route.load -= cd.quantity;

                                        cust.deliveredQuantities[t] = (int)Math.Round(q[i, t].X); //update delivery quantity
                                        cd.quantity = cust.deliveredQuantities[t];
                                        sol.depot.deliveredQuantities[t] -= cust.deliveredQuantities[t]; // OR +=;
                                        // update customer delivery and vehicle load
                                        if (cd.route != null && !GlobalUtils.IsEqual(addZ[i, t].X, 1.0) && !GlobalUtils.IsEqual(remZ[i, t].X, 1.0)) //remaining
                                        {
                                            cd.route.load += cd.quantity;
                                            cd.route.SetLoadAndCostLists(t, sol.model);
                                        }
                                        else if (cd.route != null && GlobalUtils.IsEqual(remZ[i, t].X, 1.0)) //removed
                                        {
                                            counter2 += 1;
                                            cust.visitSchedule[t] = false;
                                            Route old_rt = cd.route;
                                            for (int ii = 1; ii < old_rt.nodes.Count - 1; ii++)
                                            {
                                                Node me = old_rt.nodes[ii];
                                                if (me.uid == cust.uid)
                                                {
                                                    //Route removed from
                                                    old_rt.nodes.RemoveAt(ii);
                                                    old_rt.calculateRoutingCost(sol.model);
                                                    old_rt.SetLoadAndCostLists(t, sol.model);
                                                }
                                                // remove cust delivery
                                                cd.route = null;
                                            }
                                        }
                                        else if (cd.route == null && GlobalUtils.IsEqual(addZ[i, t].X, 1.0)) // added 
                                        {
                                            counter += 1;
                                            cust.visitSchedule[t] = true;
                                            cd.route = rtAddCostRoute[i, t];
                                            cd.route.load += cd.quantity;
                                            cd.route.nodes.Insert(1, cust);

                                            //fix route routing cost
                                            cd.route.calculateRoutingCost(sol.model);
                                            cd.route.SetLoadAndCostLists(t, sol.model);
                                        }
                                    }
                                    cust.CalculateInventoryLevels();
                                }
                                sol.depot.CalculateInventoryLevels();
                                GlobalUtils.writeToConsole("Total additions {0}", counter);
                                GlobalUtils.writeToConsole("Total deletions {0}", counter2);

                                //Fix solution objective
                                sol.routingCost = Solution.EvaluateRoutingObjectivefromScratch(sol);
                                sol.holdingCost = Solution.EvaluateInventoryObjectivefromScratch(sol);
                                //sol.totalUnitProductionCost = Solution.EvaluateUnitProductionObjectivefromScratch(sol);
                                sol.setupProductionCost = Solution.EvaluateSetupProductionObjectivefromScratch(sol);
                                sol.totalObjective = sol.routingCost + sol.holdingCost + sol.totalUnitProductionCost + sol.setupProductionCost;

                                sol.status = sol.TestEverythingFromScratch();

                                if (!Solution.checkSolutionStatus(sol.status))
                                {
                                    //runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemoval(ref back_sol, false);
                                    //sol = new Solution(back_sol, 0.0);
                                    //TSP_GRB_Solver.SolveTSP(sol);
                                    //TODO: FIX PROBLEMS THIS SHOULD NOT RETURN
                                    //back_sol.SaveToFile("mip_failed_sol");
                                    Console.WriteLine("Lefteri ftiakse tis malakies sou sto production Mip with infeasibilities and insertion/removal and production days");
                                    return false;
                                }

                                //Console.WriteLine("VIOLATIONS AFTER LP {0}", sol.calcVehicleLoadViolations());
                                TSP_GRB_Solver.SolveTSP(sol);
                                break;
                            }
                        }
                    case GRB.Status.INFEASIBLE:
                        {
                            Console.WriteLine("Model is infeasible");
                            // it could call itself with increased number of total insertions

                            // compute and write out IIS
                            //model.ComputeIIS();
                            //model.Write("model.ilp");
                            return false;
                        }
                    case GRB.Status.UNBOUNDED:
                        {
                            Console.WriteLine("Model is unbounded");
                            return false;
                        }
                    default:
                        {
                            Console.WriteLine("Optimization was stopped with status = "
                                              + optimstatus);
                            return false;
                        }

                }

                // Dispose of model
                model.Dispose();
                //gurobiEnv.Dispose();
            }
            catch (GRBException e)
            {
                Console.WriteLine("Error code: " + e.ErrorCode + ". " + e.Message);
            }

            return true;
        }


        //MIP formulation for simultaneously re-optimizing the delivery quantities of all customers and the production quantities 
        // as well allowing vehicle capacity to be violated. The violation is penalized and insertions and removals per route are allowed
        public static bool runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemovalSameDayRoutes(
            ref Solution sol, bool allowViol = false, bool isCont = true)
        {
            int numberOfPeriods = sol.periods.Count;
            int numberOfCustomers = sol.customers.Count;
            int unitCapPenalty = 1000; //Note the coefficient for penalizing capacity violation (per unit)
            int maxRouteAdditions = 2; // number of max additions per route
            int maxTotalAdditions = 10;
            int maxRouteRemovals = 2; // number of max removals per route
            int maxTotalRemovals = 10;
            int effectiveCapacity = sol.periods[0].periodRoutes[0].effectiveCapacity;
            int actualCapacity = sol.periods[0].periodRoutes[0].realCapacity;
            int excessSlack = effectiveCapacity - actualCapacity; // the max excess of capacity per route allowed
            int maxNumberVehUsed = 0;
            for (int t = 0; t < sol.periods.Count; t++)
            {
                Period per = sol.periods[t];
                maxNumberVehUsed = Math.Max(maxNumberVehUsed, per.periodRoutes.Count);
            }
            double[,] UBq = new double[numberOfCustomers, numberOfPeriods]; //upper bound of delivered quantities to customer
            double[] UBp = new double[numberOfPeriods]; //upper bound of delivered quantities to customer
            double totalProdQuantity = 0.0;
            double[,] rtAddCost = new double[numberOfCustomers, numberOfPeriods];
            Route[,] rtAddCostRoute = new Route[numberOfCustomers, numberOfPeriods];
            double[,] rtRemCost = new double[numberOfCustomers, numberOfPeriods];
            Route[,] rtRemCostRoute = new Route[numberOfCustomers, numberOfPeriods];

            //calculate upper customer delivery bound
            for (int i = 0; i < sol.customers.Count; i++)
            {
                Node cust = sol.customers[i];
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    int aggrCap = cust.stockMaximumLevel + cust.productRate[t];
                    int totalDemandUntilEnd = 0;
                    for (int tt = t; tt < numberOfPeriods; ++tt)
                    {
                        totalDemandUntilEnd += cust.productRate[tt];
                    }
                    UBq[i, t] = (double)GlobalUtils.Min(aggrCap, actualCapacity, totalDemandUntilEnd);
                }
            }

            //calculate upper production quantity bound
            for (int t = 0; t < numberOfPeriods; ++t)
            {
                totalProdQuantity += sol.depot.productionRates[t];
                if (!sol.depot.open[t])
                    UBp[t] = 0;
                else
                    UBp[t] = sol.depot.productionCapacity;
            }

            // calculate the possible savings and costs for inserting and removing customer
            for (int i = 0; i < sol.customers.Count; i++)
            {
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    rtAddCost[i, t] = int.MaxValue;
                    rtAddCostRoute[i, t] = null;
                    rtRemCost[i, t] = 0;
                    rtRemCostRoute[i, t] = null;
                }
            }

            //insertion and relocation costs
            for (int t = 0; t < sol.periods.Count; t++)
            {
                Period pr = sol.periods[t];
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int r = 0; r < pr.periodRoutes.Count; r++)
                    {
                        Route rt = pr.periodRoutes[r];
                        if (!rt.nodes.Contains(cust)) //if it is contained it cannot be relocated in the route
                        {
                            for (int ii = 0; ii < rt.nodes.Count - 1; ii++)
                            {
                                Node prev = rt.nodes[ii];
                                Node next = rt.nodes[ii + 1];
                                double costRemoved = sol.model.distMatrix[prev.uid, next.uid];
                                double costAdded = sol.model.distMatrix[prev.uid, cust.uid] + sol.model.distMatrix[cust.uid, next.uid];

                                if (costAdded - costRemoved < rtAddCost[cust.uid - 1, t])
                                {
                                    rtAddCost[cust.uid - 1, t] = costAdded - costRemoved;
                                    rtAddCostRoute[cust.uid - 1, t] = rt;
                                }
                            }
                        }
                    }
                }
            }

            // removal savings
            for (int t = 0; t < sol.periods.Count; t++)
            {
                Period pr = sol.periods[t];
                for (int r = 0; r < pr.periodRoutes.Count; r++)
                {
                    Route rt = pr.periodRoutes[r];
                    for (int i = 1; i < rt.nodes.Count - 1; i++)
                    {
                        Node prev = rt.nodes[i - 1];
                        Node me = rt.nodes[i];
                        Node next = rt.nodes[i + 1];

                        double costRemoved = sol.model.distMatrix[prev.uid, me.uid] + sol.model.distMatrix[me.uid, next.uid];
                        double costAdded = sol.model.distMatrix[prev.uid, next.uid];

                        rtRemCost[me.uid - 1, t] = costAdded - costRemoved;
                        rtRemCostRoute[me.uid - 1, t] = rt;
                    }
                }
            }

            try
            {
                GRBEnv gurobiEnv = new GRBEnv(); //TODO: fix this with init if possible
                GRBModel model = new GRBModel(gurobiEnv);

                // Params
                model.ModelName = "allCustomersAndProductionQuantitiesLPVehCapInfeasInsRemSameDays" + sol.ellapsedMs;
                model.Parameters.OutputFlag = (GlobalUtils.suppress_output) ? 0 : 1;
                model.Parameters.TimeLimit = 20;
                model.Parameters.MIPGap = 0.0005;
                model.Parameters.Threads = threadLimit;

                // Decision variables
                GRBVar[] I_depot = new GRBVar[numberOfPeriods]; // depot inventory
                GRBVar[] p = new GRBVar[numberOfPeriods]; // depot production
                GRBVar[,] I_cust = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer inventory
                GRBVar[,] q = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer delivery quantities
                GRBVar[,] qrlc = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer delivery quantities wwhen relocated
                GRBVar[,] addZ = new GRBVar[numberOfCustomers, numberOfPeriods]; // binary addition of visits
                GRBVar[,] remZ = new GRBVar[numberOfCustomers, numberOfPeriods]; // binary deletion of visits
                GRBVar[,] e = new GRBVar[maxNumberVehUsed, numberOfPeriods]; // capacity violations

                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    I_depot[t] = model.AddVar(0.0, sol.depot.stockMaximumLevel, sol.depot.unitHoldingCost, GRB.CONTINUOUS, "I_d^" + t);
                    if (isCont)
                        p[t] = model.AddVar(0.0, UBp[t], 0, GRB.CONTINUOUS, "p_d^" + t); //@greg productionCapacity: is  this the maximum quantity we can produce per dayq
                    else
                        p[t] = model.AddVar(0.0, UBp[t], 0, GRB.INTEGER, "p_d^" + t); //@greg productionCapacity: is  this the maximum quantity we can produce per dayq
                    
                    if (allowViol) // if violation is allow
                    {
                        Period per = sol.periods[t];
                        for (int r = 0; r < per.periodRoutes.Count; r++)
                        {
                            Route rt = per.periodRoutes[r];
                            e[r, t] = model.AddVar(0.0, excessSlack, unitCapPenalty, GRB.CONTINUOUS, "e_" + r + "^" + t);
                        }
                    }
                }

                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        if (isCont)
                            q[i, t] = model.AddVar(0.0, UBq[i, t], 0.0, GRB.CONTINUOUS, "q_" + i + "^" + t);
                        else
                            q[i, t] = model.AddVar(0.0, UBq[i, t], 0.0, GRB.INTEGER, "q_" + i + "^" + t);
                        I_cust[i, t] = model.AddVar(0.0, cust.stockMaximumLevel, cust.unitHoldingCost, GRB.CONTINUOUS, "I_" + i + " ^ " + t);
                        if (cust.visitSchedule[t]) // if is not route it cannot be removed 
                        {
                            if (isCont)
                                qrlc[i, t] = model.AddVar(0.0, UBq[i, t], 0.0, GRB.CONTINUOUS, "qrlc_" + i + "^" + t);
                            else
                                qrlc[i, t] = model.AddVar(0.0, UBq[i, t], 0.0, GRB.INTEGER, "qrlc_" + i + "^" + t);
                            addZ[i, t] = model.AddVar(0.0, 1.0, rtAddCost[i, t], GRB.BINARY, "addZ_" + i + " ^ " + t);
                            remZ[i, t] = model.AddVar(0.0, 1.0, rtRemCost[i, t], GRB.BINARY, "remZ_" + i + " ^ " + t);
                        }
                        else
                        {
                            addZ[i, t] = model.AddVar(0.0, 1.0, rtAddCost[i, t], GRB.BINARY, "addZ_" + i + " ^ " + t);
                            remZ[i, t] = model.AddVar(0.0, 0.0, rtRemCost[i, t], GRB.BINARY, "remZ_" + i + " ^ " + t);
                        }
                    }
                }

                // Objective
                model.ModelSense = GRB.MINIMIZE;

                // Constraints 
                // 1. Depot inventory flow 
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    GRBLinExpr lhs = I_depot[t];
                    for (int i = 0; i < sol.customers.Count; i++)
                    {
                        Node cust = sol.customers[i];
                        lhs.AddTerm(1.0, q[i, t]);
                        if (cust.visitSchedule[t]) // if the customer is in this period maybe is relocated
                            lhs.AddTerm(1.0, qrlc[i, t]);
                    }
                    if (t == 0)
                        model.AddConstr(lhs == sol.depot.startingInventory + p[t], "depot_inv_" + t);
                    else
                        model.AddConstr(lhs == I_depot[t - 1] + p[t], "depot_inv_" + t);
                }

                // 2. Customer inventory flow
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        GRBLinExpr lhs = I_cust[i, t];
                        if (t == 0)
                        {
                            if (cust.visitSchedule[t])
                                model.AddConstr(lhs == cust.startingInventory + q[i, t] + qrlc[i, t] - cust.productRate[t], "cust_inv" + i + " ^ " + t);
                            else
                                model.AddConstr(lhs == cust.startingInventory + q[i, t] - cust.productRate[t], "cust_inv" + i + " ^ " + t);
                        }
                        else
                        {
                            if (cust.visitSchedule[t])
                                model.AddConstr(lhs == I_cust[i, t - 1] + q[i, t] + qrlc[i, t] - cust.productRate[t], "cust_inv" + i + " ^ " + t);
                            else
                                model.AddConstr(lhs == I_cust[i, t - 1] + q[i, t] - cust.productRate[t], "cust_inv" + i + " ^ " + t);
                        }
                    }
                }

                // 3. Customer delivery capacity
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        GRBLinExpr lhs = q[i, t];
                        if (cust.visitSchedule[t]) 
                            lhs.AddTerm(1.0, qrlc[i, t]);
                        if (t == 0)
                            model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - cust.startingInventory, "cust_ml" + i + " ^ " + t);
                        else
                            model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - I_cust[i, t - 1], "cust_ml" + i + " ^ " + t);
                    }
                }

                // 5. Total production quantity
                GRBLinExpr lhsp = 0.0;
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    lhsp.AddTerm(1.0, p[t]);
                }
                model.AddConstr(lhsp == totalProdQuantity, "total_prod");

                // 4. Customer delivery vehicle capacity
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    Period per = sol.periods[t];
                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route route = per.periodRoutes[r];
                        GRBLinExpr lhs = 0.0;

                        for (int i = 0; i < sol.customers.Count; i++)
                        {
                            Node cust = sol.customers[i];
                            if (cust.visitSchedule[t]) //if the customer is contained in one of the routes of this day
                            {
                                if (route.nodes.Contains(cust))
                                {
                                    int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                    lhs.AddTerm(1.0, q[ii, t]);
                                }
                                else
                                {
                                    if (Object.ReferenceEquals(rtAddCostRoute[i, t], route))
                                    {
                                        int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                        lhs.AddTerm(1.0, qrlc[ii, t]);
                                    }
                                }
                            }
                            else //if not visited it can be regularly added
                            {
                                if (Object.ReferenceEquals(rtAddCostRoute[i, t], route))
                                {
                                    int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                    lhs.AddTerm(1.0, q[ii, t]);
                                }
                            }
                        }
                        if (t == 0 && sol.model.input.type == PRP_DATASET_VERSION.BOUDIA_FMT)
                            model.AddConstr(lhs <= actualCapacity, "cust_vehcap_" + r + " ^ " + t); //NOTE: effective capacity
                        else
                        {
                            if (allowViol)
                                model.AddConstr(lhs <= effectiveCapacity, "cust_vehcap_" + r + " ^ " + t); //NOTE: effective capacity
                            else
                                model.AddConstr(lhs <= actualCapacity, "cust_vehcap_" + r + " ^ " + t); //NOTE: actual capacity
                        }
                            
                    }
                }

                // 6. excessive slack
                if (allowViol)
                {
                    for (int t = 0; t < sol.periods.Count; t++)
                    {
                        Period per = sol.periods[t];
                        for (int r = 0; r < per.periodRoutes.Count; r++)
                        {
                            Route route = per.periodRoutes[r];
                            GRBLinExpr lhs = e[r, t];

                            for (int i = 0; i < sol.customers.Count; i++)
                            {
                                Node cust = sol.customers[i];
                                if (cust.visitSchedule[t]) //if the customer is contained in one of the routes of this day
                                {
                                    if (route.nodes.Contains(cust))
                                    {
                                        int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                        lhs.AddTerm(-1.0, q[ii, t]);
                                    }
                                    else
                                    {
                                        if (Object.ReferenceEquals(rtAddCostRoute[i, t], route))
                                        {
                                            int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                            lhs.AddTerm(-1.0, qrlc[ii, t]);
                                        }
                                    }
                                }
                                else //if not visited it can be regularly added
                                {
                                    if (Object.ReferenceEquals(rtAddCostRoute[i, t], route))
                                    {
                                        int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                        lhs.AddTerm(-1.0, q[ii, t]);
                                    }
                                }
                            }
                            model.AddConstr(lhs >= -actualCapacity, "e" + r + " ^ " + t); //NOTE: effective capacity
                        }
                    }
                }

                // 7. addition constraints
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        if (!cust.visitSchedule[t])
                        {
                            GRBLinExpr lhs = q[i, t];
                            model.AddConstr(lhs <= addZ[i, t] * UBq[i, t], "add_visits" + i + " ^ " + t);
                        }
                        else
                        {
                            model.AddConstr(addZ[i, t] <= remZ[i, t], "add_visits" + i + " ^ " + t);
                            model.AddConstr(qrlc[i, t] <= remZ[i, t] * UBq[i, t], "reloc_visits" + i + " ^ " + t);
                            model.AddConstr(qrlc[i, t] <= addZ[i, t] * UBq[i, t], "reloc_visits" + i + " ^ " + t);
                            //model.AddConstr(q[i, t] <= (1-remZ[i, t]) * UBq[i, t], "add_visits" + i + " ^ " + t);
                        }
                    }
                }

                // 10. removal constraints
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        if (cust.visitSchedule[t])
                        {
                            model.AddConstr(q[i, t] <= (1 - remZ[i, t]) * UBq[i, t], "rem_visits" + i + " ^ " + t);
                        }
                    }
                }

                // 11. Removals per route
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    Period per = sol.periods[t];
                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route route = per.periodRoutes[r];
                        GRBLinExpr lhs = 0.0;

                        for (int i = 1; i < route.nodes.Count - 1; i++)
                        {
                            Node cust = route.nodes[i];
                            int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                            lhs.AddTerm(1.0, remZ[ii, t]);
                        }
                        model.AddConstr(lhs <= maxRouteRemovals, "removal" + r + " ^ " + t);
                    }
                }

                // 8. One addition per route (to be tuned)
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    Period per = sol.periods[t];
                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route route = per.periodRoutes[r];
                        GRBLinExpr lhs = 0.0;

                        for (int i = 0; i < sol.customers.Count; i++)
                        {
                            Node cust = sol.customers[i];
                            if (Object.ReferenceEquals(rtAddCostRoute[i, t], route))
                            {
                                int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                lhs.AddTerm(1.0, addZ[ii, t]);
                            }
                        }
                        model.AddConstr(lhs <= maxRouteAdditions, "addition" + r + " ^ " + t);
                    }
                }

                // 12. Total addition and removals bounds 
                /*
                GRBLinExpr lhsa = 0.0;
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    for (int i = 0; i < sol.customers.Count; i++)
                    {
                        Node cust = sol.customers[i];
                        int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                        lhsa.AddTerm(1.0, addZ[ii, t]);
                    }
                }
                model.AddConstr(lhsa <= maxTotalAdditions, "totalAdditions");

                
                GRBLinExpr lhsb = 0.0;
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    for (int i = 0; i < sol.customers.Count; i++)
                    {
                        Node cust = sol.customers[i];
                        int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                        lhsa.AddTerm(1.0, remZ[ii, t]);
                    }
                }
                model.AddConstr(lhsa <= maxTotalRemovals, "totalRemovals");
                */

                // Optimize
                model.Optimize();
                int optimstatus = model.Status;

                switch (model.Status)
                {
                    case GRB.Status.OPTIMAL:
                    case GRB.Status.TIME_LIMIT:
                        {
                            int counter = 0;
                            int counter2 = 0;
                            int counter3 = 0;

                            if (existsViolatedQuantity(ref sol, q, qrlc, p))
                            {
                                model.Dispose();
                                return runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemovalSameDayRoutes(ref sol, allowViol, false);
                            }
                            else
                            { 
                                // Update solution with new calculated quantities
                                for (int t = 0; t < numberOfPeriods; t++)
                                {
                                    sol.depot.deliveredQuantities[t] = 0;
                                    sol.depot.productionRates[t] = (int)Math.Round(p[t].X);
                                }

                                for (int i = 0; i < sol.customers.Count; i++)
                                {
                                    Node cust = sol.customers[i];
                                    for (int t = 0; t < numberOfPeriods; t++)
                                    {
                                        CustDelivery cd = cust.horizonDeliveryServices[t];
                                        if (cd.route != null)
                                            cd.route.load -= cd.quantity;

                                        int alternativeQuanity = 0;
                                        if (cust.visitSchedule[t])
                                        {
                                            alternativeQuanity = (int)Math.Round(qrlc[i, t].X);
                                        }
                                        cust.deliveredQuantities[t] = Math.Max((int)Math.Round(q[i, t].X), alternativeQuanity); //update delivery quantity
                                        cd.quantity = cust.deliveredQuantities[t];
                                        sol.depot.deliveredQuantities[t] -= cust.deliveredQuantities[t]; // OR +=;
                                        // update customer delivery and vehicle load
                                        if (cd.route != null && !GlobalUtils.IsEqual(addZ[i, t].X, 1.0) && !GlobalUtils.IsEqual(remZ[i, t].X, 1.0)) //remaining
                                        {
                                            cd.route.load += cd.quantity;
                                            cd.route.SetLoadAndCostLists(t, sol.model);
                                        }
                                        else if (cd.route != null && GlobalUtils.IsEqual(addZ[i, t].X, 1.0) && GlobalUtils.IsEqual(remZ[i, t].X, 1.0)) //same period change route
                                        {
                                            counter3 += 1;
                                            Route old_rt = cd.route;
                                            for (int ii = 1; ii < old_rt.nodes.Count - 1; ii++)
                                            {
                                                Node me = old_rt.nodes[ii];
                                                if (me.uid == cust.uid)
                                                {
                                                    //Route removed from
                                                    old_rt.nodes.RemoveAt(ii);
                                                    old_rt.calculateRoutingCost(sol.model);
                                                    old_rt.SetLoadAndCostLists(t, sol.model);
                                                }
                                            }
                                            cd.route = rtAddCostRoute[i, t];
                                            cd.route.load += cd.quantity;
                                            cd.route.nodes.Insert(1, cust);
                                            //fix route routing cost
                                            cd.route.calculateRoutingCost(sol.model);
                                            cd.route.SetLoadAndCostLists(t, sol.model);
                                        }
                                        else if (cd.route != null && GlobalUtils.IsEqual(remZ[i, t].X, 1.0)) //removed
                                        {
                                            counter2 += 1;
                                            cust.visitSchedule[t] = false;
                                            Route old_rt = cd.route;
                                            for (int ii = 1; ii < old_rt.nodes.Count - 1; ii++)
                                            {
                                                Node me = old_rt.nodes[ii];
                                                if (me.uid == cust.uid)
                                                {
                                                    //Route removed from
                                                    old_rt.nodes.RemoveAt(ii);
                                                    old_rt.calculateRoutingCost(sol.model);
                                                    old_rt.SetLoadAndCostLists(t, sol.model);
                                                }
                                                // remove cust delivery
                                                cd.route = null;
                                            }
                                        }
                                        else if (cd.route == null && GlobalUtils.IsEqual(addZ[i, t].X, 1.0)) // added 
                                        {
                                            counter += 1;
                                            cust.visitSchedule[t] = true;
                                            cd.route = rtAddCostRoute[i, t];
                                            cd.route.load += cd.quantity;
                                            cd.route.nodes.Insert(1, cust);

                                            //fix route routing cost
                                            cd.route.calculateRoutingCost(sol.model);
                                            cd.route.SetLoadAndCostLists(t, sol.model);
                                        }
                                    }
                                    cust.CalculateInventoryLevels();
                                }
                                sol.depot.CalculateInventoryLevels();
                                GlobalUtils.writeToConsole("Total additions: {0} / Total deletions: {1} / Total same day relocations: {2}", counter, counter2, counter3);

                                //Fix solution objective
                                sol.routingCost = Solution.EvaluateRoutingObjectivefromScratch(sol);
                                sol.holdingCost = Solution.EvaluateInventoryObjectivefromScratch(sol);
                                //sol.totalUnitProductionCost = Solution.EvaluateUnitProductionObjectivefromScratch(sol);
                                //sol.setupProductionCost = Solution.EvaluateSetupProductionObjectivefromScratch(sol);
                                sol.totalObjective = sol.routingCost + sol.holdingCost + sol.totalUnitProductionCost + sol.setupProductionCost;

                                sol.status = sol.TestEverythingFromScratch();

                                if (!Solution.checkSolutionStatus(sol.status))
                                {
                                    Console.WriteLine("Effective Capacity {0} vs Real Capacity {1}",
                                        effectiveCapacity, actualCapacity);
                                    Console.WriteLine(sol.ToString());

                                    Console.WriteLine("Lefteri ftiakse tis malakies sou sto production Mip with infeasibilities and insertion/removal same days");
                                    return false;
                                }

                                //Console.WriteLine("VIOLATIONS AFTER LP {0}", sol.calcVehicleLoadViolations());
                                TSP_GRB_Solver.SolveTSP(sol);
                                break;
                            }
                        }
                    case GRB.Status.INFEASIBLE:
                        {
                            Console.WriteLine("Model is infeasible");
                            if (!allowViol)
                            {
                                Console.WriteLine("Rerunning to minimize violations");
                                runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemovalSameDayRoutes(ref sol, true, isCont);
                            }
                            else
                                return false;
                            // compute and write out IIS
                            //model.ComputeIIS();
                            //model.Write("model.ilp");
                            break;
                        }
                    case GRB.Status.UNBOUNDED:
                        {
                            Console.WriteLine("Model is unbounded");
                            return false;
                        }
                    default:
                        {
                            Console.WriteLine("Optimization was stopped with status = "
                                              + optimstatus);
                            return false;
                        }
                }

                // Dispose of model
                model.Dispose();
                //gurobiEnv.Dispose();
            }
            catch (GRBException e)
            {
                Console.WriteLine("Error code: " + e.ErrorCode + ". " + e.Message);
            }

            return true;
        }

        public static bool runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemovalSameDayRoutesOriginal(ref Solution sol, int maxRouteAdditions, int maxRouteRemovals, double gapLimit, int dirtyFix, bool isCont = true)
        {
            Solution back_sol = new Solution(sol, 0.0);
            //if (!Solution.checkSolutionStatus(sol.TestEverythingFromScratch()))
            //throw new Exception("Production Mip with infeasibilities and insertion/removal - ERROR IN INPUT SOLUTION");

            int numberOfPeriods = sol.periods.Count;
            int numberOfCustomers = sol.customers.Count;

            int unitCapPenalty = 10000; //Note the coefficient for penalizing capacity violation (per unit)
            //int maxTotalAdditions = 10;
            //int maxRouteRemovals = 20; // number of max removals per route
            //int maxTotalRemovals = 10;
            int effectiveCapacity = sol.periods[0].periodRoutes[0].effectiveCapacity;
            int actualCapacity = sol.periods[0].periodRoutes[0].realCapacity;
            int excessSlack = effectiveCapacity - actualCapacity; // the max excess of capacity per route allowed

            int maxNumberVehUsed = 0;
            for (int t = 0; t < sol.periods.Count; t++)
            {
                Period per = sol.periods[t];
                maxNumberVehUsed = Math.Max(maxNumberVehUsed, per.periodRoutes.Count);
            }
            double[,] UBq = new double[numberOfCustomers, numberOfPeriods]; //upper bound of delivered quantities to customer
            double[] UBp = new double[numberOfPeriods]; //upper bound of delivered quantities to customer
            double totalProdQuantity = 0.0;
            double[,] rtAddCost = new double[numberOfCustomers, numberOfPeriods];
            Route[,] rtAddCostRoute = new Route[numberOfCustomers, numberOfPeriods];
            double[,] rtRemCost = new double[numberOfCustomers, numberOfPeriods];
            Route[,] rtRemCostRoute = new Route[numberOfCustomers, numberOfPeriods];

            //calculate UBq 
            for (int i = 0; i < sol.customers.Count; i++)
            {
                Node cust = sol.customers[i];
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    int aggrCap = cust.stockMaximumLevel + cust.productRate[t];
                    int totalDemandUntilEnd = 0;
                    for (int tt = t; tt < numberOfPeriods; ++tt)
                    {
                        totalDemandUntilEnd += cust.productRate[tt];
                    }
                    //if (!cust.visitSchedule[t])
                    //    UBq[i, t] = 0;
                    //else
                    UBq[i, t] = (double)GlobalUtils.Min(aggrCap, actualCapacity, totalDemandUntilEnd);
                }
            }

            for (int t = 0; t < numberOfPeriods; ++t)
            {
                totalProdQuantity += sol.depot.productionRates[t];
                if (!sol.depot.open[t])
                    UBp[t] = 0;
                else
                    UBp[t] = Math.Min(sol.depot.productionCapacity, sol.depot.totalDemand);
            }

            for (int i = 0; i < sol.customers.Count; i++)
            {
                Node cust = sol.customers[i];
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    rtAddCost[i, t] = sol.totalObjective; //int.MaxValue;
                    rtAddCostRoute[i, t] = null;
                    rtRemCost[i, t] = 0;
                    rtRemCostRoute[i, t] = null;
                }
            }

            // calculate the possible savings
            for (int t = 0; t < sol.periods.Count; t++)
            {
                Period pr = sol.periods[t];
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    if (!cust.visitSchedule[t]) //if customer is not serviced then and only then can be added
                    {
                        for (int r = 0; r < pr.periodRoutes.Count; r++)
                        {
                            Route rt = pr.periodRoutes[r];
                            if (!rt.nodes.Contains(cust)) //always true
                            {
                                for (int ii = 0; ii < rt.nodes.Count - 1; ii++)
                                {
                                    Node prev = rt.nodes[ii];
                                    Node next = rt.nodes[ii + 1];

                                    double costRemoved = sol.model.distMatrix[prev.uid, next.uid];
                                    double costAdded = sol.model.distMatrix[prev.uid, cust.uid] + sol.model.distMatrix[cust.uid, next.uid];

                                    if (costAdded - costRemoved < rtAddCost[cust.uid - 1, t])
                                    {
                                        rtAddCost[cust.uid - 1, t] = costAdded - costRemoved;
                                        rtAddCostRoute[cust.uid - 1, t] = rt;
                                    }
                                }

                                //SOS: DIRTY FIX
                                if (rand.Next(0, 100) < dirtyFix)
                                {
                                    for (int rr = 0; rr < pr.periodRoutes.Count; rr++)
                                    {
                                        Route rrt = pr.periodRoutes[rr];
                                        if (rrt.load == 0)
                                        {
                                            rtAddCostRoute[cust.uid - 1, t] = rrt;
                                            Node previ = rrt.nodes[0];
                                            Node nexti = rrt.nodes[1];
                                            double costRemoved2 = sol.model.distMatrix[previ.uid, nexti.uid];
                                            double costAdded2 = sol.model.distMatrix[previ.uid, cust.uid] + sol.model.distMatrix[cust.uid, nexti.uid];
                                            rtAddCost[cust.uid - 1, t] = costAdded2 - costRemoved2;
                                            break;
                                        }
                                    }
                                }

                            }
                        }
                    }
                    else
                    {
                        for (int r = 0; r < pr.periodRoutes.Count; r++)
                        {
                            Route rt = pr.periodRoutes[r];
                            if (!rt.nodes.Contains(cust)) //always true
                            {
                                for (int ii = 0; ii < rt.nodes.Count - 1; ii++)
                                {
                                    Node prev = rt.nodes[ii];
                                    Node next = rt.nodes[ii + 1];

                                    double costRemoved = sol.model.distMatrix[prev.uid, next.uid];
                                    double costAdded = sol.model.distMatrix[prev.uid, cust.uid] + sol.model.distMatrix[cust.uid, next.uid];
                                    
                                    if (costAdded - costRemoved < rtAddCost[cust.uid - 1, t])
                                    {
                                        rtAddCost[cust.uid - 1, t] = costAdded - costRemoved;
                                        rtAddCostRoute[cust.uid - 1, t] = rt;
                                    }
                                }

                                //SOS: DIRTY FIX
                                if (rand.Next(0, 100) < dirtyFix)
                                {
                                    for (int rr = 0; rr < pr.periodRoutes.Count; rr++)
                                    {
                                        Route rrt = pr.periodRoutes[rr];
                                        if (rrt.load == 0)
                                        {
                                            rtAddCostRoute[cust.uid - 1, t] = rrt;
                                            Node previ = rrt.nodes[0];
                                            Node nexti = rrt.nodes[1];
                                            double costRemoved2 = sol.model.distMatrix[previ.uid, nexti.uid];
                                            double costAdded2 = sol.model.distMatrix[previ.uid, cust.uid] + sol.model.distMatrix[cust.uid, nexti.uid];
                                            rtAddCost[cust.uid - 1, t] = costAdded2 - costRemoved2;
                                            break;
                                        }
                                    }
                                }

                            }
                        }
                    }
                }
            }

            for (int t = 0; t < sol.periods.Count; t++)
            {
                Period pr = sol.periods[t];
                for (int r = 0; r < pr.periodRoutes.Count; r++)
                {
                    Route rt = pr.periodRoutes[r];
                    for (int i = 1; i < rt.nodes.Count - 1; i++)
                    {
                        Node prev = rt.nodes[i - 1];
                        Node me = rt.nodes[i];
                        Node next = rt.nodes[i + 1];

                        double costRemoved = sol.model.distMatrix[prev.uid, me.uid] + sol.model.distMatrix[me.uid, next.uid];
                        double costAdded = sol.model.distMatrix[prev.uid, next.uid];

                        rtRemCost[me.uid - 1, t] = costAdded - costRemoved;
                        rtRemCostRoute[me.uid - 1, t] = rt;
                    }
                }
            }

            try
            {
                //GRBEnv gurobiEnv = new GRBEnv();
                GRBModel model = new GRBModel(gurobiEnv);

                // Params
                model.ModelName = "allCustomersAndProductionQuantitiesLPVehCapInfeasInsRemSameDays" + sol.ellapsedMs;
                model.Parameters.OutputFlag = (GlobalUtils.suppress_output) ? 0 : 1;
                model.Parameters.MIPGap = gapLimit; //TODO: Expose that as well
                model.Parameters.Threads = threadLimit;
                model.Parameters.TimeLimit = 40;
                model.Parameters.MIPFocus = 1;

                // Decision variables
                GRBVar[] I_depot = new GRBVar[numberOfPeriods]; // depot inventory
                GRBVar[] p = new GRBVar[numberOfPeriods]; // depot production
                GRBVar[,] I_cust = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer inventory
                GRBVar[,] q = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer delivery quantities
                GRBVar[,] qrlc = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer delivery quantities
                GRBVar[,] e = new GRBVar[maxNumberVehUsed, numberOfPeriods]; // capacity violations
                GRBVar[,] addZ = new GRBVar[numberOfCustomers, numberOfPeriods]; // binary addition of visits
                GRBVar[,] remZ = new GRBVar[numberOfCustomers, numberOfPeriods]; // binary deletion of visits

                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    //I_depot[t] = model.AddVar(0.0, sol.depot.stockMaximumLevel, sol.depot.unitHoldingCost, GRB.CONTINUOUS, "I_d^" + t);
                    I_depot[t] = model.AddVar(0.0, sol.depot.totalDemand, sol.depot.unitHoldingCost, GRB.CONTINUOUS, "I_d^" + t);
                    if (isCont)
                        p[t] = model.AddVar(0.0, UBp[t], 0, GRB.CONTINUOUS, "p_d^" + t); //@greg productionCapacity: is  this the maximum quantity we can produce per dayq
                    else
                        p[t] = model.AddVar(0.0, UBp[t], 0, GRB.INTEGER, "p_d^" + t); //@greg productionCapacity: is  this the maximum quantity we can produce per dayq

                    Period per = sol.periods[t];
                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route rt = per.periodRoutes[r];
                        e[r, t] = model.AddVar(0.0, excessSlack, unitCapPenalty, GRB.CONTINUOUS, "e_" + r + "^" + t);
                    }
                }

                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        if (isCont)
                            q[i, t] = model.AddVar(0.0, UBq[i, t], 0.0, GRB.CONTINUOUS, "q_" + i + "^" + t);
                        else
                            q[i, t] = model.AddVar(0.0, UBq[i, t], 0.0, GRB.INTEGER, "q_" + i + "^" + t);
                        I_cust[i, t] = model.AddVar(0.0, cust.stockMaximumLevel, cust.unitHoldingCost, GRB.CONTINUOUS, "I_" + i + " ^ " + t);
                        if (cust.visitSchedule[t]) // if is not route it cannot be removed 
                        {
                            if (isCont)
                                qrlc[i, t] = model.AddVar(0.0, UBq[i, t], 0.0, GRB.CONTINUOUS, "qrlc_" + i + "^" + t);
                            else
                                qrlc[i, t] = model.AddVar(0.0, UBq[i, t], 0.0, GRB.INTEGER, "qrlc_" + i + "^" + t);
                            addZ[i, t] = model.AddVar(0.0, 1.0, rtAddCost[i, t], GRB.BINARY, "addZ_" + i + " ^ " + t);
                            remZ[i, t] = model.AddVar(0.0, 1.0, rtRemCost[i, t], GRB.BINARY, "remZ_" + i + " ^ " + t);
                        }
                        else
                        {
                            addZ[i, t] = model.AddVar(0.0, 1.0, rtAddCost[i, t], GRB.BINARY, "addZ_" + i + " ^ " + t);
                            remZ[i, t] = model.AddVar(0.0, 0.0, rtRemCost[i, t], GRB.BINARY, "remZ_" + i + " ^ " + t);
                        }
                    }
                }

                // Objective
                model.ModelSense = GRB.MINIMIZE;

                // Constraints 
                // 1. Depot inventory flow 
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    GRBLinExpr lhs = 0.0;
                    lhs.AddTerm(1.0, I_depot[t]);
                    for (int i = 0; i < sol.customers.Count; i++)
                    {
                        Node cust = sol.customers[i];

                        lhs.AddTerm(1.0, q[i, t]);
                        if (cust.visitSchedule[t])
                            lhs.AddTerm(1.0, qrlc[i, t]);
                    }
                    if (t == 0)
                        model.AddConstr(lhs == sol.depot.startingInventory + p[t], "depot_inv_" + t);
                    else
                        model.AddConstr(lhs == I_depot[t - 1] + p[t], "depot_inv_" + t);
                }

                // 2. Customer inventory flow
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        GRBLinExpr lhs = 0.0;
                        lhs.AddTerm(1.0, I_cust[i, t]);
                        if (t == 0)
                            if (cust.visitSchedule[t])
                            {
                                model.AddConstr(lhs == cust.startingInventory + q[i, t] + qrlc[i, t] - cust.productRate[t], "cust_inv" + i + " ^ " + t);
                            }
                            else
                            {
                                model.AddConstr(lhs == cust.startingInventory + q[i, t] - cust.productRate[t], "cust_inv" + i + " ^ " + t);
                            }
                        else
                        {
                            if (cust.visitSchedule[t])
                            {
                                model.AddConstr(lhs == I_cust[i, t - 1] + q[i, t] + qrlc[i, t] - cust.productRate[t], "cust_inv" + i + " ^ " + t);
                            }
                            else
                            {
                                model.AddConstr(lhs == I_cust[i, t - 1] + q[i, t] - cust.productRate[t], "cust_inv" + i + " ^ " + t);
                            }
                        }
                    }
                }

                // 3. Customer delivery capacity
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        GRBLinExpr lhs = 0.0;
                        lhs.AddTerm(1.0, q[i, t]);
                        if (cust.visitSchedule[t])
                            lhs.AddTerm(1.0, qrlc[i, t]);
                        if (t == 0)
                            model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - cust.startingInventory, "cust_ml" + i + " ^ " + t);
                        else
                            model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - I_cust[i, t - 1], "cust_ml" + i + " ^ " + t);
                    }
                }

                // 5. Total production quantity
                GRBLinExpr lhsp = 0.0;
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    lhsp.AddTerm(1.0, p[t]);
                }
                model.AddConstr(lhsp == totalProdQuantity, "total_prod");

                // 4. Customer delivery vehicle capacity
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    Period per = sol.periods[t];

                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route route = per.periodRoutes[r];
                        GRBLinExpr lhs = 0.0;

                        //for (int i = 1; i < route.nodes.Count - 1; i++)
                        for (int i = 0; i < sol.customers.Count; i++)
                        {
                            Node cust = sol.customers[i];
                            if (cust.visitSchedule[t]) //if the customer is contained in one of the routes of this day
                            {
                                if (route.nodes.Contains(cust))
                                {
                                    int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                    lhs.AddTerm(1.0, q[ii, t]);
                                }
                                else
                                {
                                    if (Object.ReferenceEquals(rtAddCostRoute[i, t], route))
                                    {
                                        int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                        lhs.AddTerm(1.0, qrlc[ii, t]);
                                    }
                                }
                            }
                            else //if not visited it can be regularly added
                            {
                                if (Object.ReferenceEquals(rtAddCostRoute[i, t], route))
                                {
                                    int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                    lhs.AddTerm(1.0, q[ii, t]);
                                }
                            }
                        }

                        if (t == 0 && sol.model.input.type == PRP_DATASET_VERSION.BOUDIA_FMT)
                            model.AddConstr(lhs <= actualCapacity, "cust_vehcap_" + r + " ^ " + t); //NOTE: effective capacity
                        else
                            model.AddConstr(lhs <= effectiveCapacity, "cust_vehcap_" + r + " ^ " + t); //NOTE: effective capacity
                    }
                }

                // 6. excessive slack
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    Period per = sol.periods[t];
                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route route = per.periodRoutes[r];
                        GRBLinExpr lhs = e[r, t];

                        //for (int i = 1; i < route.nodes.Count - 1; i++)
                        for (int i = 0; i < sol.customers.Count; i++)
                        {
                            Node cust = sol.customers[i];
                            if (cust.visitSchedule[t]) //if the customer is contained in one of the routes of this day
                            {
                                if (route.nodes.Contains(cust))
                                {
                                    int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                    lhs.AddTerm(-1.0, q[ii, t]);
                                }
                                else
                                {
                                    if (Object.ReferenceEquals(rtAddCostRoute[i, t], route))
                                    {
                                        int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                        lhs.AddTerm(-1.0, qrlc[ii, t]);
                                    }
                                }
                            }
                            else //if not visited it can be regularly added
                            {
                                if (Object.ReferenceEquals(rtAddCostRoute[i, t], route))
                                {
                                    int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                    lhs.AddTerm(-1.0, q[ii, t]);
                                }
                            }
                        }
                        model.AddConstr(lhs >= -actualCapacity, "e" + r + " ^ " + t); //NOTE: effective capacity
                    }
                }

                // 6. excessive slack
                /*
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    Period per = sol.periods[t];
                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route route = per.periodRoutes[r];
                        model.AddConstr(e[r,t] == 0, "e2" + r + " ^ " + t); //NOTE: effective capacity
                    }
                }
                */


                // 7. addition constraints
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        if (!cust.visitSchedule[t])
                        {
                            GRBLinExpr lhs = q[i, t];
                            model.AddConstr(lhs <= (addZ[i, t]) * UBq[i, t], "add_visits" + i + " ^ " + t);
                        }
                        else
                        {
                            model.AddConstr(addZ[i, t] <= remZ[i, t], "add_visits" + i + " ^ " + t);
                            model.AddConstr(qrlc[i, t] <= remZ[i, t] * UBq[i, t], "add_visits" + i + " ^ " + t);
                            model.AddConstr(qrlc[i, t] <= addZ[i, t] * UBq[i, t], "add_visits" + i + " ^ " + t);
                            //model.AddConstr(q[i, t] <= (1 - remZ[i, t]) * UBq[i, t], "add_visits" + i + " ^ " + t);
                        }
                    }
                }

                // 10. removal constraints
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        if (cust.visitSchedule[t])
                        {
                            GRBLinExpr lhs = q[i, t];
                            model.AddConstr(lhs <= (1 - remZ[i, t]) * UBq[i, t], "rem_visits" + i + " ^ " + t);
                            //model.AddConstr(lhs <= (1 + addZ[i, t] - remZ[i, t]) * UBq[i, t], "rem_visits" + i + " ^ " + t);
                        }
                    }
                }

                // 11. One removal per route
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    Period per = sol.periods[t];

                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route route = per.periodRoutes[r];
                        GRBLinExpr lhs = 0.0;

                        for (int i = 1; i < route.nodes.Count - 1; i++)
                        {
                            Node cust = route.nodes[i];
                            int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                            lhs.AddTerm(1.0, remZ[ii, t]);
                        }
                        for (int i = 0; i < sol.customers.Count; i++)
                        {
                            Node cust = sol.customers[i];
                            if (Object.ReferenceEquals(rtAddCostRoute[i, t], route))
                            {
                                int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                lhs.AddTerm(1.0, addZ[ii, t]);
                            }
                        }
                        // sad
                        model.AddConstr(lhs <= maxRouteAdditions + maxRouteRemovals, "removal" + r + " ^ " + t);
                    }
                }
                /*
                // 8. One addition per route (to be tuned)
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    Period per = sol.periods[t];

                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route route = per.periodRoutes[r];
                        GRBLinExpr lhs = 0.0;

                        //for (int i = 1; i < route.nodes.Count - 1; i++)
                        for (int i = 0; i < sol.customers.Count; i++)
                        {
                            Node cust = sol.customers[i];
                            if (Object.ReferenceEquals(rtAddCostRoute[i, t], route))
                            {
                                int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                lhs.AddTerm(1.0, addZ[ii, t]);
                            }
                        }
                        model.AddConstr(lhs <= maxRouteAdditions, "one_addition" + r + " ^ " + t);
                    }
                }
                */

                // 12. Total addition and removals bounds 
                /*
                GRBLinExpr lhsa = 0.0;
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    for (int i = 0; i < sol.customers.Count; i++)
                    {
                        Node cust = sol.customers[i];
                        int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                        lhsa.AddTerm(1.0, addZ[ii, t]);
                    }
                }
                model.AddConstr(lhsa <= maxTotalAdditions, "totalAdditions");

                
                GRBLinExpr lhsb = 0.0;
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    for (int i = 0; i < sol.customers.Count; i++)
                    {
                        Node cust = sol.customers[i];
                        int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                        lhsa.AddTerm(1.0, remZ[ii, t]);
                    }
                }
                model.AddConstr(lhsa <= maxTotalRemovals, "totalRemovals");
                */

                // Optimize
                model.Optimize();
                int optimstatus = model.Status;

                switch (model.Status)
                {
                    case GRB.Status.OPTIMAL:
                    case GRB.Status.TIME_LIMIT:
                        {
                            double objval = model.ObjVal;
                            int counter = 0;
                            int counter2 = 0;
                            int counter3 = 0;

                            if (existsViolatedQuantity(ref sol, q, qrlc, p))
                            {
                                model.Dispose();
                                return runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemovalSameDayRoutesOriginal(ref sol, maxRouteAdditions, maxRouteRemovals, gapLimit, dirtyFix, false);
                            }
                            else
                            {
                                // Update solution with new calculated quantities
                                for (int t = 0; t < numberOfPeriods; t++)
                                {
                                    sol.depot.deliveredQuantities[t] = 0;
                                    sol.depot.productionRates[t] = (int) Math.Round(p[t].X);
                                }

                                for (int i = 0; i < sol.customers.Count; i++)
                                {
                                    Node cust = sol.customers[i];

                                    for (int t = 0; t < numberOfPeriods; t++)
                                    {
                                        CustDelivery cd = cust.horizonDeliveryServices[t];
                                        if (cd.route != null)
                                            cd.route.load -= cd.quantity;

                                        int alternativeQuanity = 0;
                                        if (cust.visitSchedule[t])
                                        {
                                            alternativeQuanity = (int)Math.Round(qrlc[i, t].X);
                                        }
                                        cust.deliveredQuantities[t] = Math.Max((int)Math.Round(q[i, t].X), alternativeQuanity); //update delivery quantity
                                        cd.quantity = cust.deliveredQuantities[t];
                                        sol.depot.deliveredQuantities[t] -= cust.deliveredQuantities[t]; // OR +=;
                                        // update customer delivery and vehicle load
                                        if (cd.route != null && !GlobalUtils.IsEqual(addZ[i, t].X, 1.0, 1e-4) && !GlobalUtils.IsEqual(remZ[i, t].X, 1.0, 1e-4)) //remaining
                                        {
                                            cd.route.load += cd.quantity;
                                            cd.route.SetLoadAndCostLists(t, sol.model);
                                        }
                                        else if (cd.route != null && GlobalUtils.IsEqual(addZ[i, t].X, 1.0, 1e-4) && GlobalUtils.IsEqual(remZ[i, t].X, 1.0, 1e-4)) //same period change route
                                        {
                                            counter3 += 1;
                                            Route old_rt = cd.route;
                                            for (int ii = 1; ii < old_rt.nodes.Count - 1; ii++)
                                            {
                                                Node me = old_rt.nodes[ii];
                                                if (me.uid == cust.uid)
                                                {
                                                    //Route removed from
                                                    old_rt.nodes.RemoveAt(ii);
                                                    old_rt.calculateRoutingCost(sol.model);
                                                    old_rt.SetLoadAndCostLists(t, sol.model);
                                                }
                                            }
                                            cd.route = rtAddCostRoute[i, t];
                                            cd.route.load += cd.quantity;
                                            cd.route.nodes.Insert(1, cust);
                                            //fix route routing cost
                                            cd.route.calculateRoutingCost(sol.model);
                                            cd.route.SetLoadAndCostLists(t, sol.model);
                                        }
                                        else if (cd.route != null && GlobalUtils.IsEqual(remZ[i, t].X, 1.0, 1e-4)) //removed
                                        {
                                            counter2 += 1;
                                            cust.visitSchedule[t] = false;
                                            Route old_rt = cd.route;
                                            for (int ii = 1; ii < old_rt.nodes.Count - 1; ii++)
                                            {
                                                Node me = old_rt.nodes[ii];
                                                if (me.uid == cust.uid)
                                                {
                                                    //Route removed from
                                                    old_rt.nodes.RemoveAt(ii);
                                                    old_rt.calculateRoutingCost(sol.model);
                                                    old_rt.SetLoadAndCostLists(t, sol.model);
                                                }
                                                // remove cust delivery
                                                cd.route = null;
                                            }
                                        }
                                        else if (cd.route == null && GlobalUtils.IsEqual(addZ[i, t].X, 1.0, 1e-4)) // added 
                                        {
                                            counter += 1;
                                            cust.visitSchedule[t] = true;
                                            cd.route = rtAddCostRoute[i, t];
                                            cd.route.load += cd.quantity;
                                            cd.route.nodes.Insert(1, cust);

                                            //fix route routing cost
                                            cd.route.calculateRoutingCost(sol.model);
                                            cd.route.SetLoadAndCostLists(t, sol.model);
                                        }
                                    }
                                    cust.CalculateInventoryLevels();
                                }
                                sol.depot.CalculateInventoryLevels();
                                GlobalUtils.writeToConsole("Total additions {0}", counter);
                                GlobalUtils.writeToConsole("Total deletions {0}", counter2);
                                GlobalUtils.writeToConsole("Total change route same day {0}", counter3);


                                //Fix solution objective
                                sol.routingCost = Solution.EvaluateRoutingObjectivefromScratch(sol);
                                sol.holdingCost = Solution.EvaluateInventoryObjectivefromScratch(sol);
                                //sol.totalUnitProductionCost = Solution.EvaluateUnitProductionObjectivefromScratch(sol);
                                //sol.setupProductionCost = Solution.EvaluateSetupProductionObjectivefromScratch(sol);
                                sol.totalObjective = sol.routingCost + sol.holdingCost + sol.totalUnitProductionCost + sol.setupProductionCost;

                                sol.status = sol.TestEverythingFromScratch();

                                if (!Solution.checkSolutionStatus(sol.status))
                                {
                                    if (isCont)
                                    {
                                        runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemovalSameDayRoutesOriginal(ref back_sol, maxRouteAdditions, maxRouteRemovals, gapLimit, dirtyFix, false);
                                        sol = new Solution(back_sol, 0.0);
                                    }
                                    else
                                    {
                                        //
                                        //sol = new Solution(back_sol, 0.0);
                                        //TSP_GRB_Solver.SolveTSP(sol);
                                        //TODO: FIX PROBLEMS THIS SHOULD NOT RETURN
                                        //back_sol.SaveToFile("mip_failed_sol");
                                        Console.WriteLine("Lefteri ftiakse tis malakies sou sto production Mip with infeasibilities and insertion/removal same days");
                                        return false;
                                    }
                                }

                                //GlobalUtils.writeToConsole("VIOLATIONS AFTER LP {0}", sol.calcVehicleLoadViolations());
                                TSP_GRB_Solver.SolveTSP(sol);
                                break;
                            }
                        }
                    case GRB.Status.INFEASIBLE:
                        {
                            Console.WriteLine("Model is infeasible");
                            // it could call itself with increased number of total insertions
                            //runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemoval(ref sol);


                            // compute and write out IIS
                            //model.ComputeIIS();
                            //model.Write("model.ilp");
                            return false;
                        }
                    case GRB.Status.UNBOUNDED:
                        {
                            Console.WriteLine("Model is unbounded");
                            return false;
                        }
                    default:
                        {
                            Console.WriteLine("Optimization was stopped with status = "
                                              + optimstatus);
                            return false;
                        }

                }

                // Dispose of model
                model.Dispose();
                //gurobiEnv.Dispose();
            }
            catch (GRBException e)
            {
                Console.WriteLine("Error code: " + e.ErrorCode + ". " + e.Message);
            }

            return true;
        }

        
        public static bool runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemovalSameDayRoutesImprovedFormulation(ref Solution sol, int maxRouteAdditions, int maxRouteRemovals, double gapLimit, int dirtyFix, bool isCont = true)
        {
            Solution back_sol = new Solution(sol, 0.0);
            //if (!Solution.checkSolutionStatus(sol.TestEverythingFromScratch()))
            //throw new Exception("Production Mip with infeasibilities and insertion/removal - ERROR IN INPUT SOLUTION");

            int numberOfPeriods = sol.periods.Count;
            int numberOfCustomers = sol.customers.Count;

            int unitCapPenalty = 10000; //Note: the coefficient for penalizing capacity violation (per unit)
            //int maxTotalAdditions = 10;
            //int maxRouteRemovals = 20; // number of max removals per route
            //int maxTotalRemovals = 10;
            int effectiveCapacity = sol.periods[0].periodRoutes[0].effectiveCapacity;
            int actualCapacity = sol.periods[0].periodRoutes[0].realCapacity;
            int excessSlack = effectiveCapacity - actualCapacity; // the max excess of capacity per route allowed

            int maxNumberVehUsed = 0;
            for (int t = 0; t < sol.periods.Count; t++)
            {
                Period per = sol.periods[t];
                maxNumberVehUsed = Math.Max(maxNumberVehUsed, per.periodRoutes.Count);
            }
            double[,] UBq = new double[numberOfCustomers, numberOfPeriods]; //upper bound of delivered quantities to customer
            double[] UBp = new double[numberOfPeriods]; //upper bound of delivered quantities to customer
            double totalProdQuantity = 0.0;
            double[,] rtAddCost = new double[numberOfCustomers, numberOfPeriods];
            Route[,] rtAddCostRoute = new Route[numberOfCustomers, numberOfPeriods];
            double[,] rtRemCost = new double[numberOfCustomers, numberOfPeriods];
            Route[,] rtRemCostRoute = new Route[numberOfCustomers, numberOfPeriods];

            //calculate UBq 
            for (int i = 0; i < sol.customers.Count; i++)
            {
                Node cust = sol.customers[i];
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    int aggrCap = cust.stockMaximumLevel + cust.productRate[t];
                    int totalDemandUntilEnd = 0;
                    for (int tt = t; tt < numberOfPeriods; ++tt)
                    {
                        totalDemandUntilEnd += cust.productRate[tt];
                    }
                    //if (!cust.visitSchedule[t])
                    //    UBq[i, t] = 0;
                    //else
                    UBq[i, t] = (double)GlobalUtils.Min(aggrCap, actualCapacity, totalDemandUntilEnd);
                }
            }

            for (int t = 0; t < numberOfPeriods; ++t)
            {
                totalProdQuantity += sol.depot.productionRates[t];
                if (!sol.depot.open[t])
                    UBp[t] = 0;
                else
                    UBp[t] = Math.Min(sol.depot.productionCapacity, sol.depot.totalDemand);
            }

            for (int i = 0; i < sol.customers.Count; i++)
            {
                Node cust = sol.customers[i];
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    rtAddCost[i, t] = sol.totalObjective; //int.MaxValue;
                    rtAddCostRoute[i, t] = null;
                    rtRemCost[i, t] = 0;
                    rtRemCostRoute[i, t] = null;
                }
            }

            // calculate the possible savings
            for (int t = 0; t < sol.periods.Count; t++)
            {
                Period pr = sol.periods[t];
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    if (!cust.visitSchedule[t]) //if customer is not serviced then and only then can be added
                    {
                        for (int r = 0; r < pr.periodRoutes.Count; r++)
                        {
                            Route rt = pr.periodRoutes[r];
                            if (!rt.nodes.Contains(cust)) //always true
                            {
                                for (int ii = 0; ii < rt.nodes.Count - 1; ii++)
                                {
                                    Node prev = rt.nodes[ii];
                                    Node next = rt.nodes[ii + 1];

                                    double costRemoved = sol.model.distMatrix[prev.uid, next.uid];
                                    double costAdded = sol.model.distMatrix[prev.uid, cust.uid] + sol.model.distMatrix[cust.uid, next.uid];

                                    if (costAdded - costRemoved < rtAddCost[cust.uid - 1, t])
                                    {
                                        rtAddCost[cust.uid - 1, t] = costAdded - costRemoved;
                                        rtAddCostRoute[cust.uid - 1, t] = rt;
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        for (int r = 0; r < pr.periodRoutes.Count; r++)
                        {
                            Route rt = pr.periodRoutes[r];
                            if (!rt.nodes.Contains(cust)) //always true
                            {
                                for (int ii = 0; ii < rt.nodes.Count - 1; ii++)
                                {
                                    Node prev = rt.nodes[ii];
                                    Node next = rt.nodes[ii + 1];

                                    double costRemoved = sol.model.distMatrix[prev.uid, next.uid];
                                    double costAdded = sol.model.distMatrix[prev.uid, cust.uid] + sol.model.distMatrix[cust.uid, next.uid];

                                    if (costAdded - costRemoved < rtAddCost[cust.uid - 1, t])
                                    {
                                        rtAddCost[cust.uid - 1, t] = costAdded - costRemoved;
                                        rtAddCostRoute[cust.uid - 1, t] = rt;
                                    }

                                    //SOS: DIRTY FIX
                                    if (rand.Next(0, 100) < dirtyFix)
                                    {
                                        for (int rr = 0; rr < pr.periodRoutes.Count; rr++)
                                        {
                                            Route rrt = pr.periodRoutes[rr];
                                            if (rrt.load == 0)
                                            {
                                                rtAddCostRoute[cust.uid - 1, t] = rrt;
                                                Node previ = rrt.nodes[0];
                                                Node nexti = rrt.nodes[1];
                                                double costRemoved2 = sol.model.distMatrix[previ.uid, nexti.uid];
                                                double costAdded2 = sol.model.distMatrix[previ.uid, cust.uid] + sol.model.distMatrix[cust.uid, nexti.uid];
                                                rtAddCost[cust.uid - 1, t] = costAdded2 - costRemoved2;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            for (int t = 0; t < sol.periods.Count; t++)
            {
                Period pr = sol.periods[t];
                for (int r = 0; r < pr.periodRoutes.Count; r++)
                {
                    Route rt = pr.periodRoutes[r];
                    for (int i = 1; i < rt.nodes.Count - 1; i++)
                    {
                        Node prev = rt.nodes[i - 1];
                        Node me = rt.nodes[i];
                        Node next = rt.nodes[i + 1];

                        double costRemoved = sol.model.distMatrix[prev.uid, me.uid] + sol.model.distMatrix[me.uid, next.uid];
                        double costAdded = sol.model.distMatrix[prev.uid, next.uid];

                        rtRemCost[me.uid - 1, t] = costAdded - costRemoved;
                        rtRemCostRoute[me.uid - 1, t] = rt;
                    }
                }
            }

            try
            {
                //GRBEnv gurobiEnv = new GRBEnv();
                GRBModel model = new GRBModel(gurobiEnv);

                // Params
                model.ModelName = "allCustomersAndProductionQuantitiesLPVehCapInfeasInsRemSameDaysNew" + sol.ellapsedMs;
                model.Parameters.OutputFlag = (GlobalUtils.suppress_output) ? 0 : 1;
                model.Parameters.MIPGap = gapLimit; //TODO: Expose that as well
                model.Parameters.Threads = threadLimit;
                model.Parameters.TimeLimit = 40;
                model.Parameters.MIPFocus = 1;

                // Decision variables
                GRBVar[] I_depot = new GRBVar[numberOfPeriods]; // depot inventory
                GRBVar[] p = new GRBVar[numberOfPeriods]; // depot production
                GRBVar[,] I_cust = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer inventory
                GRBVar[,] q = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer delivery quantities
                //GRBVar[,] qrlc = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer delivery quantities
                GRBVar[,] e = new GRBVar[maxNumberVehUsed, numberOfPeriods]; // capacity violations
                GRBVar[,] addZ = new GRBVar[numberOfCustomers, numberOfPeriods]; // binary addition of visits
                GRBVar[,] remZ = new GRBVar[numberOfCustomers, numberOfPeriods]; // binary deletion of visits

                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    //I_depot[t] = model.AddVar(0.0, sol.depot.stockMaximumLevel, sol.depot.unitHoldingCost, GRB.CONTINUOUS, "I_d^" + t);
                    I_depot[t] = model.AddVar(0.0, sol.depot.totalDemand, sol.depot.unitHoldingCost, GRB.CONTINUOUS, "I_d^" + t);
                    if (isCont)
                        p[t] = model.AddVar(0.0, UBp[t], 0, GRB.CONTINUOUS, "p_d^" + t); //@greg productionCapacity: is  this the maximum quantity we can produce per dayq
                    else
                        p[t] = model.AddVar(0.0, UBp[t], 0, GRB.INTEGER, "p_d^" + t); //@greg productionCapacity: is  this the maximum quantity we can produce per dayq

                    Period per = sol.periods[t];
                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route rt = per.periodRoutes[r];
                        e[r, t] = model.AddVar(0.0, excessSlack, unitCapPenalty, GRB.CONTINUOUS, "e_" + r + "^" + t);
                    }
                }

                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        if (isCont)
                            q[i, t] = model.AddVar(0.0, UBq[i, t], 0.0, GRB.CONTINUOUS, "q_" + i + "^" + t);
                        else
                            q[i, t] = model.AddVar(0.0, UBq[i, t], 0.0, GRB.INTEGER, "q_" + i + "^" + t);
                        I_cust[i, t] = model.AddVar(0.0, cust.stockMaximumLevel, cust.unitHoldingCost, GRB.CONTINUOUS, "I_" + i + "^" + t);
                        if (cust.visitSchedule[t]) // if is not route it cannot be removed 
                        {
                            addZ[i, t] = model.AddVar(0.0, 1.0, rtAddCost[i, t], GRB.BINARY, "addZ_" + i + "^" + t);
                            remZ[i, t] = model.AddVar(0.0, 1.0, rtRemCost[i, t], GRB.BINARY, "remZ_" + i + "^" + t);
                        }
                        else
                        {
                            addZ[i, t] = model.AddVar(0.0, 1.0, rtAddCost[i, t], GRB.BINARY, "addZ_" + i + "^" + t);
                            remZ[i, t] = model.AddVar(0.0, 0.0, rtRemCost[i, t], GRB.BINARY, "remZ_" + i + "^" + t);
                        }
                    }
                }

                // Objective
                model.ModelSense = GRB.MINIMIZE;

                // Constraints 
                // 1. Depot inventory flow 
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    GRBLinExpr lhs = 0.0;
                    lhs.AddTerm(1.0, I_depot[t]);
                    for (int i = 0; i < sol.customers.Count; i++)
                    {
                        Node cust = sol.customers[i];

                        lhs.AddTerm(1.0, q[i, t]);
                    }
                    if (t == 0)
                        model.AddConstr(lhs == sol.depot.startingInventory + p[t], "depot_inv_" + t);
                    else
                        model.AddConstr(lhs == I_depot[t - 1] + p[t], "depot_inv_" + t);
                }

                // 2. Customer inventory flow
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        GRBLinExpr lhs = 0.0;
                        lhs.AddTerm(1.0, I_cust[i, t]);
                        if (t == 0)
                        {
                            model.AddConstr(lhs == cust.startingInventory + q[i, t] - cust.productRate[t], "cust_inv" + i + "^" + t);
                        }
                        else
                        {
                            
                            model.AddConstr(lhs == I_cust[i, t - 1] + q[i, t] - cust.productRate[t], "cust_inv" + i + "^" + t);
                            
                        }
                    }
                }

                // 3. Customer delivery capacity
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        GRBLinExpr lhs = 0.0;
                        lhs.AddTerm(1.0, q[i, t]);
                        if (t == 0)
                            model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - cust.startingInventory, "cust_ml" + i + "^" + t);
                        else
                            model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - I_cust[i, t - 1], "cust_ml" + i + "^" + t);
                    }
                }

                // 5. Total production quantity
                GRBLinExpr lhsp = 0.0;
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    lhsp.AddTerm(1.0, p[t]);
                }
                model.AddConstr(lhsp == totalProdQuantity, "total_prod");

                // 4. Customer delivery vehicle capacity
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    Period per = sol.periods[t];

                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route route = per.periodRoutes[r];
                        //GRBLinExpr lhs = 0.0;
                        GRBQuadExpr lhs = 0.0;


                        //for (int i = 1; i < route.nodes.Count - 1; i++)
                        for (int i = 0; i < sol.customers.Count; i++)
                        {
                            Node cust = sol.customers[i];
                            if (cust.visitSchedule[t]) //if the customer is contained in one of the routes of this day
                            {
                                if (route.nodes.Contains(cust))
                                {
                                    int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                    lhs.AddTerm(1.0, q[ii, t]);
                                    lhs.AddTerm(-1.0, remZ[ii, t], q[ii, t]);
                                }
                                else
                                {
                                    if (Object.ReferenceEquals(rtAddCostRoute[i, t], route))
                                    {
                                        int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                        lhs.AddTerm(1.0, addZ[ii, t], q[ii, t]);
                                    }
                                }
                            }
                            else //if not visited it can be regularly added
                            {
                                if (Object.ReferenceEquals(rtAddCostRoute[i, t], route))
                                {
                                    int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                    lhs.AddTerm(1.0, addZ[ii, t], q[ii, t]);
                                }
                            }
                        }

                        if (t == 0 && sol.model.input.type == PRP_DATASET_VERSION.BOUDIA_FMT)
                            model.AddQConstr(lhs <= actualCapacity, "cust_vehcap_" + r + "^" + t); //NOTE: effective capacity
                        else
                            model.AddQConstr(lhs <= effectiveCapacity, "cust_vehcap_" + r + "^" + t); //NOTE: effective capacity
                    }
                }

                // 6. excessive slack
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    Period per = sol.periods[t];
                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route route = per.periodRoutes[r];
                        //GRBLinExpr lhs = e[r, t];
                        GRBQuadExpr lhs = e[r, t];

                        //for (int i = 1; i < route.nodes.Count - 1; i++)
                        for (int i = 0; i < sol.customers.Count; i++)
                        {
                            Node cust = sol.customers[i];
                            if (cust.visitSchedule[t]) //if the customer is contained in one of the routes of this day
                            {
                                if (route.nodes.Contains(cust))
                                {
                                    int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                    lhs.AddTerm(-1.0, q[ii, t]);
                                    lhs.AddTerm(+1.0, remZ[ii, t], q[ii, t]);

                                    //lhs.AddTerm(-1.0, q[ii, t]);
                                }
                                else
                                {
                                    if (Object.ReferenceEquals(rtAddCostRoute[i, t], route))
                                    {
                                        int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                                                           //lhs.AddTerm(-1.0, q[ii, t]);
                                        lhs.AddTerm(-1.0, addZ[ii, t], q[ii, t]);

                                    }
                                }
                            }
                            else //if not visited it can be regularly added
                            {
                                if (Object.ReferenceEquals(rtAddCostRoute[i, t], route))
                                {
                                    int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                                                       //lhs.AddTerm(-1.0, q[ii, t]);
                                    lhs.AddTerm(-1.0, addZ[ii, t], q[ii, t]);

                                }
                            }
                        }
                        model.AddQConstr(lhs >= -actualCapacity, "e" + r + "^" + t); //NOTE: effective capacity
                    }
                }

                // 7. binary constraints+
                
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        if (!cust.visitSchedule[t])
                        {
                            model.AddConstr(q[i, t] <= (addZ[i, t] - remZ[i, t]) * UBq[i, t], "add_visits" + i + "^" + t);

                        }
                        else
                        {
                            model.AddConstr(addZ[i, t] <= remZ[i, t], "add_visits" + i + "^" + t);
                            model.AddConstr(q[i, t] <= (1 + addZ[i, t] - remZ[i, t]) * UBq[i, t], "add_visits" + i + "^" + t);
                        }
                    }
                }
                

                /*
                // 7. addition constraints
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        if (!cust.visitSchedule[t])
                        {
                            GRBLinExpr lhs = q[i, t];
                            model.AddConstr(lhs <= (addZ[i, t]) * UBq[i, t], "add_visits" + i + "^" + t);
                        }
                        else
                        {
                            model.AddConstr(addZ[i, t] <= remZ[i, t], "add_visits" + i + "^" + t);
                            model.AddConstr(qrlc[i, t] <= remZ[i, t] * UBq[i, t], "add_visits" + i + "^" + t);
                            model.AddConstr(qrlc[i, t] <= addZ[i, t] * UBq[i, t], "add_visits" + i + "^" + t);
                            //model.AddConstr(q[i, t] <= (1 - remZ[i, t]) * UBq[i, t], "add_visits" + i + "^" + t);
                        }
                    }
                }

                // 10. removal constraints
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        if (cust.visitSchedule[t])
                        {
                            GRBLinExpr lhs = q[i, t];
                            model.AddConstr(lhs <= (1 - remZ[i, t]) * UBq[i, t], "rem_visits" + i + "^" + t);
                            //model.AddConstr(lhs <= (1 + addZ[i, t] - remZ[i, t]) * UBq[i, t], "rem_visits" + i + "^" + t);
                        }
                    }
                }
                */

                // 11. One removal per route
                for (int t = 0; t < sol.periods.Count; t++)
                {
                    Period per = sol.periods[t];

                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route route = per.periodRoutes[r];
                        GRBLinExpr lhs = 0.0;

                        for (int i = 1; i < route.nodes.Count - 1; i++)
                        {
                            Node cust = route.nodes[i];
                            int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                            lhs.AddTerm(1.0, remZ[ii, t]);
                        }
                        for (int i = 0; i < sol.customers.Count; i++)
                        {
                            Node cust = sol.customers[i];
                            if (Object.ReferenceEquals(rtAddCostRoute[i, t], route))
                            {
                                int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                                lhs.AddTerm(1.0, addZ[ii, t]);
                            }
                        }
                        // sad
                        model.AddConstr(lhs <= maxRouteAdditions + maxRouteRemovals, "removal" + r + "^" + t);
                    }
                }
               


                // Optimize
                model.Optimize();
                int optimstatus = model.Status;

                switch (model.Status)
                {
                    case GRB.Status.OPTIMAL:
                    case GRB.Status.TIME_LIMIT:
                        {
                            double objval = model.ObjVal;
                            int counter = 0;
                            int counter2 = 0;
                            int counter3 = 0;

                            if (existsViolatedQuantity(ref sol, q, p))
                            {
                                model.Dispose();
                                return runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemovalSameDayRoutesImprovedFormulation(ref sol, maxRouteAdditions, maxRouteRemovals, gapLimit, dirtyFix, false);
                            }
                            else
                            {
                                // Update solution with new calculated quantities
                                for (int t = 0; t < numberOfPeriods; t++)
                                {
                                    sol.depot.deliveredQuantities[t] = 0;
                                    sol.depot.productionRates[t] = (int)Math.Round(p[t].X);
                                }

                                for (int i = 0; i < sol.customers.Count; i++)
                                {
                                    Node cust = sol.customers[i];

                                    for (int t = 0; t < numberOfPeriods; t++)
                                    {
                                        CustDelivery cd = cust.horizonDeliveryServices[t];
                                        if (cd.route != null)
                                            cd.route.load -= cd.quantity;

                                        int alternativeQuanity = 0;
                                        if (cust.visitSchedule[t])
                                        {
                                            alternativeQuanity = (int)Math.Round(q[i, t].X);
                                        }
                                        cust.deliveredQuantities[t] = Math.Max((int)Math.Round(q[i, t].X), alternativeQuanity); //update delivery quantity
                                        cd.quantity = cust.deliveredQuantities[t];
                                        sol.depot.deliveredQuantities[t] -= cust.deliveredQuantities[t]; // OR +=;
                                        // update customer delivery and vehicle load
                                        if (cd.route != null && !GlobalUtils.IsEqual(addZ[i, t].X, 1.0, 1e-4) && !GlobalUtils.IsEqual(remZ[i, t].X, 1.0, 1e-4)) //remaining
                                        {
                                            cd.route.load += cd.quantity;
                                            cd.route.SetLoadAndCostLists(t, sol.model);
                                        }
                                        else if (cd.route != null && GlobalUtils.IsEqual(addZ[i, t].X, 1.0, 1e-4) && GlobalUtils.IsEqual(remZ[i, t].X, 1.0, 1e-4)) //same period change route
                                        {
                                            counter3 += 1;
                                            Route old_rt = cd.route;
                                            for (int ii = 1; ii < old_rt.nodes.Count - 1; ii++)
                                            {
                                                Node me = old_rt.nodes[ii];
                                                if (me.uid == cust.uid)
                                                {
                                                    //Route removed from
                                                    old_rt.nodes.RemoveAt(ii);
                                                    old_rt.calculateRoutingCost(sol.model);
                                                    old_rt.SetLoadAndCostLists(t, sol.model);
                                                }
                                            }
                                            cd.route = rtAddCostRoute[i, t];
                                            cd.route.load += cd.quantity;
                                            cd.route.nodes.Insert(1, cust);
                                            //fix route routing cost
                                            cd.route.calculateRoutingCost(sol.model);
                                            cd.route.SetLoadAndCostLists(t, sol.model);
                                        }
                                        else if (cd.route != null && GlobalUtils.IsEqual(remZ[i, t].X, 1.0, 1e-4)) //removed
                                        {
                                            counter2 += 1;
                                            cust.visitSchedule[t] = false;
                                            Route old_rt = cd.route;
                                            for (int ii = 1; ii < old_rt.nodes.Count - 1; ii++)
                                            {
                                                Node me = old_rt.nodes[ii];
                                                if (me.uid == cust.uid)
                                                {
                                                    //Route removed from
                                                    old_rt.nodes.RemoveAt(ii);
                                                    old_rt.calculateRoutingCost(sol.model);
                                                    old_rt.SetLoadAndCostLists(t, sol.model);
                                                }
                                                // remove cust delivery
                                                cd.route = null;
                                            }
                                        }
                                        else if (cd.route == null && GlobalUtils.IsEqual(addZ[i, t].X, 1.0, 1e-4)) // added 
                                        {
                                            counter += 1;
                                            cust.visitSchedule[t] = true;
                                            cd.route = rtAddCostRoute[i, t];
                                            cd.route.load += cd.quantity;
                                            cd.route.nodes.Insert(1, cust);

                                            //fix route routing cost
                                            cd.route.calculateRoutingCost(sol.model);
                                            cd.route.SetLoadAndCostLists(t, sol.model);
                                        }
                                    }
                                    cust.CalculateInventoryLevels();
                                }
                                sol.depot.CalculateInventoryLevels();
                                GlobalUtils.writeToConsole("Total additions {0}", counter);
                                GlobalUtils.writeToConsole("Total deletions {0}", counter2);
                                GlobalUtils.writeToConsole("Total change route same day {0}", counter3);


                                //Fix solution objective
                                sol.routingCost = Solution.EvaluateRoutingObjectivefromScratch(sol);
                                sol.holdingCost = Solution.EvaluateInventoryObjectivefromScratch(sol);
                                //sol.totalUnitProductionCost = Solution.EvaluateUnitProductionObjectivefromScratch(sol);
                                //sol.setupProductionCost = Solution.EvaluateSetupProductionObjectivefromScratch(sol);
                                sol.totalObjective = sol.routingCost + sol.holdingCost + sol.totalUnitProductionCost + sol.setupProductionCost;

                                sol.status = sol.TestEverythingFromScratch();

                                if (!Solution.checkSolutionStatus(sol.status))
                                {
                                    if (isCont)
                                    {
                                        runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemovalSameDayRoutesImprovedFormulation(ref back_sol, maxRouteAdditions, maxRouteRemovals, gapLimit, dirtyFix, false);
                                        sol = new Solution(back_sol, 0.0);
                                    }
                                    else
                                    {
                                        //
                                        //sol = new Solution(back_sol, 0.0);
                                        //TSP_GRB_Solver.SolveTSP(sol);
                                        //TODO: FIX PROBLEMS THIS SHOULD NOT RETURN
                                        //back_sol.SaveToFile("mip_failed_sol");
                                        Console.WriteLine("Lefteri ftiakse tis malakies sou sto production Mip with infeasibilities and insertion/removal same days");
                                        return false;
                                    }
                                }

                                //GlobalUtils.writeToConsole("VIOLATIONS AFTER LP {0}", sol.calcVehicleLoadViolations());
                                TSP_GRB_Solver.SolveTSP(sol);
                                break;
                            }
                        }
                    case GRB.Status.INFEASIBLE:
                        {
                            Console.WriteLine("Model is infeasible");
                            // it could call itself with increased number of total insertions
                            //runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemoval(ref sol);


                            // compute and write out IIS
                            model.ComputeIIS();
                            model.Write("model.ilp");
                            return false;
                        }
                    case GRB.Status.UNBOUNDED:
                        {
                            Console.WriteLine("Model is unbounded");
                            return false;
                        }
                    default:
                        {
                            Console.WriteLine("Optimization was stopped with status = "
                                              + optimstatus);
                            return false;
                        }

                }

                // Dispose of model
                model.Dispose();
                //gurobiEnv.Dispose();
            }
            catch (GRBException e)
            {
                Console.WriteLine("Error code: " + e.ErrorCode + ". " + e.Message);
            }

            return true;
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
                throw new Exception("COuld not initialize gurobi environment");
        }

        public static void disposeEnv()
        {
            gurobiEnv.Dispose();
        }

    }
}
