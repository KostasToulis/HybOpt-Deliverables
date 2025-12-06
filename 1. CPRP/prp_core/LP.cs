using System;
using System.Diagnostics;
using System.Net.Http.Headers;
using System.Net.Sockets;
using System.Xml;
using Gurobi;

namespace PRP
{
    public class LP
    {
        public static GRBEnv gurobiEnv;
        //public static Stopwatch stopwatch = new Stopwatch();
        public static int threadLimit;

        /*
         * Finds the optimal quantities of a single customer with fixed the schedule of the rest
         */
        public static bool runLP(Solution sol, Node cust)
         {
            int numberOfPeriods = sol.periods.Count;
            double[] UBq = new double[numberOfPeriods]; //upper bound of delivered quantities to customer
            double[] otherq = new double[numberOfPeriods]; // the quantities delivered to other customers each period (except cust)
            double[] vehicle_remaining_cap = new double[numberOfPeriods];  //vehicle remaining capacity without cust

            //calculate UBq 
            for (int t = 0; t < numberOfPeriods; ++t)
            {
                int aggrCap = cust.stockMaximumLevel + cust.productRate[t]; //@greg cap + demand (OK)
                int vehCap = sol.model.capacity; //@greg veh capacity (OK)
                int totalDemandUntilEnd = 0;
                for (int tt = t; tt < numberOfPeriods; ++tt)
                {
                    totalDemandUntilEnd += cust.productRate[tt];  //@greg remaining demand until end of horizon (OK)
                }
                if (!cust.auxiliary_visitSchedule[t])
                    UBq[t] = 0; // (double)GlobalUtils.Min(0, aggrCap, vehCap, totalDemandUntilEnd)
                else
                    UBq[t] = (double)GlobalUtils.Min(aggrCap, vehCap, totalDemandUntilEnd);
            }

            // calculate quantities delivered to other customers
            for (int t = 0; t < numberOfPeriods; ++t)
            {
                otherq[t] = -sol.depot.deliveredQuantities[t] - cust.deliveredQuantities[t];
            }

            //  // calculate the remaining capacity of the truck serving the customer
            for (int t = 0; t < numberOfPeriods; ++t)
            {

                CustDelivery cd = cust.auxiliary_horizonDeliveryServices[t];

                vehicle_remaining_cap[t] = 0;
                if (cd.route != null)
                {
                    vehicle_remaining_cap[t] = cd.route.realCapacity - cd.route.load + cd.quantity;
                }
            }

            try
            {
                GRBModel model = new GRBModel(gurobiEnv);

                // Params
                model.ModelName = "customerLP";
                model.Parameters.OutputFlag = 0;
                model.Parameters.Threads = threadLimit;
                
                // Decision variables
                GRBVar[] I_depot = new GRBVar[numberOfPeriods]; // depot inventory
                GRBVar[] I_cust = new GRBVar[numberOfPeriods]; // customer inventory
                GRBVar[] q = new GRBVar[numberOfPeriods]; // customer delivery quantities

                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    I_depot[t] = model.AddVar(0.0, sol.depot.stockMaximumLevel, 0, GRB.CONTINUOUS, "I_d^" + t);
                    I_cust[t] = model.AddVar(0.0, cust.stockMaximumLevel, 0, GRB.CONTINUOUS, "I_c^" + t);
                }
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    q[t] = model.AddVar(0.0, UBq[t], 0.0, GRB.CONTINUOUS, "q_c^" + t);
                }

                GRBLinExpr objectiveFunction = new GRBLinExpr();
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    objectiveFunction.AddTerm(sol.depot.unitHoldingCost, I_depot[t]);
                    objectiveFunction.AddTerm(cust.unitHoldingCost, I_cust[t]);
                }
                model.SetObjective(objectiveFunction);

                // Objective
                model.ModelSense = GRB.MINIMIZE;
                
                // Constraints 
                // 1. Depot inventory flow 
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    GRBLinExpr lhs = 0.0;
                    lhs.AddTerm(1.0, I_depot[t]);
                    if (t == 0)
                    {
                        model.AddConstr(lhs == sol.depot.startingInventory + sol.depot.productionRates[t] - (q[t] + otherq[t]), "depot_inv_" + t);
                    }
                    else
                    {
                        model.AddConstr(lhs == I_depot[t - 1] + sol.depot.productionRates[t] - (q[t] + otherq[t]), "depot_inv_" + t);
                    }                      
                }

                // 2. Customer inventory flow
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    GRBLinExpr lhs = 0.0;
                    lhs.AddTerm(1.0, I_cust[t]);
                    if (t == 0)
                    {
                        lhs.AddTerm(-1.0, q[t]);
                        model.AddConstr(lhs == cust.startingInventory - cust.productRate[t], "cust_inv" + t);
                        //model.AddConstr(lhs == cust.startingInventory + q[t] - cust.productRate[t], "cust_inv" + t);
                    }
                    else
                    {
                        lhs.AddTerm(-1.0, q[t]);
                        lhs.AddTerm(-1.0, I_cust[t - 1]);
                        model.AddConstr(lhs == - cust.productRate[t], "cust_inv" + t);
                        //model.AddConstr(lhs == I_cust[t - 1] + q[t] - cust.productRate[t], "cust_inv" + t);
                    }                        
                }

                // 3. Customer delivery capacity
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    GRBLinExpr lhs = 0.0;
                    lhs.AddTerm(1.0, q[t]);
                    if (t == 0)
                        model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - cust.startingInventory, "cust_ml" + t);
                    else
                        model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - I_cust[t - 1], "cust_ml" + t);
                }

                // 4. Customer delivery vehicle capacity
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    GRBLinExpr lhs = 0.0;
                    lhs.AddTerm(1.0, q[t]);
                    model.AddConstr(lhs <= vehicle_remaining_cap[t], "cust_vehcap_" + t);
                }

                // Optimization
                model.Optimize();

                int optimstatus = model.Status;
                
                switch (model.Status)
                {
                    case GRB.Status.OPTIMAL:
                    {
                        double objval = model.ObjVal;
                    
                        Console.Write("{0,20}", "Cust Deliveries");
                        for (int i = 0; i < numberOfPeriods; i++)
                            Console.Write(" {0,8:0.0} ", (int)q[i].X);
                        Console.Write("\n");

                        Console.Write("{0,20}", "Cust Inv");
                        for (int i = 0; i < numberOfPeriods; i++)
                            Console.Write(" {0,8:0.0} ", (int)I_cust[i].X);
                        Console.Write("\n");
                    
                        //Compare with auxiliary deliveries
                        
                        for (int t = 0; t < numberOfPeriods; t++)
                        {
                            if (!GlobalUtils.IsEqual((int) q[t].X, cust.auxiliary_deliveries[t]) && !GlobalUtils.IsEqual(cust.unitHoldingCost, sol.depot.unitHoldingCost))
                            {
                                GlobalUtils.writeToConsole("LP vs Heuristic Mismatch");
                                cust.ReportInventory(true);
                        
                                GlobalUtils.writeToConsole("Optimal objective: " + objval);
                                //Print Day horizon
                                Console.Write("{0,20}", "Horizon");
                                for (int i = 0; i < numberOfPeriods; i++)
                                    Console.Write(" {0,8} ", i);
                                Console.Write("\n");
                        
                                //Print Start Day Inventory
                                Console.Write("{0,20}", "Depot Inv");
                                for (int i = 0; i < numberOfPeriods; i++)
                                    Console.Write(" {0,8:0.0} ", (int) I_depot[i].X);
                                Console.Write("\n");
                        
                                Console.Write("{0,20}", "Cust Deliveries");
                                for (int i = 0; i < numberOfPeriods; i++)
                                    Console.Write(" {0,8:0.0} ", (int) q[i].X);
                                Console.Write("\n");
                        
                                Console.Write("{0,20}", "Cust Inv");
                                for (int i = 0; i < numberOfPeriods; i++)
                                    Console.Write(" {0,8:0.0} ", (int) I_cust[i].X);
                                Console.Write("\n");


                                //cust.ApplySawMaxDeliveryLast(cust.auxiliary_visitSchedule, sol);
                                cust.ApplySawMaxDeliveryFirst(cust.auxiliary_visitSchedule, sol, false, -1);

                            }
                        }

                        break;
                    }
                    
                    case GRB.Status.INFEASIBLE:
                    {
                        GlobalUtils.writeToConsole("Model is infeasible");

                        // compute and write out IIS
                        //model.ComputeIIS();
                        //model.Write("model.ilp");
                        return false;
                    }
                    case GRB.Status.UNBOUNDED:
                    {
                        GlobalUtils.writeToConsole("Model is unbounded");
                        return false;
                    }
                    default:
                    {
                        GlobalUtils.writeToConsole("Optimization was stopped with status = "
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
                Console.WriteLine("Error code: " + e.ErrorCode + ". " + e.Message);
            }

            return true;
        }

        //==================================================== LPs ====================================================//

        //LP formulation for simultaneously re-optimizing the delivery quantities of all customers. 
        public static bool runSimultaneousDeliveryReoptimizationLP(Solution sol)
        {
            int numberOfPeriods = sol.periods.Count;
            int numberOfCustomers = sol.customers.Count;
            double[,] UBq = new double[numberOfCustomers, numberOfPeriods]; //upper bound of delivered quantities to customer

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
                        UBq[i,t] = 0;
                    else
                        UBq[i, t] = (double)GlobalUtils.Min(aggrCap, vehCap, totalDemandUntilEnd);
                }
            }

            try
            {
                GRBModel model = new GRBModel(gurobiEnv);

                // Params
                model.ModelName = "allCustomersQuantitiesLP";
                model.Parameters.OutputFlag = 0;
                model.Parameters.Threads = threadLimit;

                // Decision variables
                GRBVar[] I_depot = new GRBVar[numberOfPeriods]; // depot inventory
                GRBVar[,] I_cust = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer inventory
                GRBVar[,] q = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer delivery quantities
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    I_depot[t] = model.AddVar(0.0, sol.depot.stockMaximumLevel, sol.depot.unitHoldingCost, GRB.CONTINUOUS, "I_d^" + t);
                }
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        q[i, t] = model.AddVar(0.0, UBq[i, t], 0.0, GRB.CONTINUOUS, "q_" + i + "^" + t);
                        I_cust[i,t] = model.AddVar(0.0, cust.stockMaximumLevel, cust.unitHoldingCost, GRB.CONTINUOUS, "I_" + i + " ^ " + t);
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
                        model.AddConstr(lhs == sol.depot.startingInventory + sol.depot.productionRates[t], "depot_inv_" + t);
                    else
                        model.AddConstr(lhs == I_depot[t - 1] + sol.depot.productionRates[t], "depot_inv_" + t);
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
                        lhs.AddTerm(1.0, q[i,t]);
                        if (t == 0)
                            model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - cust.startingInventory, "cust_ml" + i + " ^ " + t);
                        else
                            model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - I_cust[i,t - 1], "cust_ml" + i + " ^ " + t);
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

                // 4. Customer delivery vehicle capacity
                /*for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i]

                    for (int t = 0; t < numberOfPeriods; ++t)
                    {

                        CustDelivery cd = cust.horizonDeliveryServices[t];

                        vehicle_remaining_cap[t] = 0;
                        if (cd != null)
                        {
                            vehicle_remaining_cap[t] = cd.route.capacity - cd.route.load + cd.quantity;
                        }

                    }
                }*/

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
                                    cust.deliveredQuantities[t] = (int)q[i, t].X; //update delivery quantity
                                    sol.depot.deliveredQuantities[t] -= cust.deliveredQuantities[t]; // OR +=;
                                    // update customer delivery and vehicle load
                                    if (cd.route != null)
                                    {
                                        cd.quantity = cust.deliveredQuantities[t];
                                        cd.route.load += cd.quantity;
                                        cd.route.SetLoadAndCostLists(t, sol.model);
                                    }
                                }
                                
                                cust.CalculateInventoryLevels();
                            }

                            sol.depot.CalculateInventoryLevels();

                            
                            //Fix solution objective
                            sol.totalObjective -= sol.holdingCost;
                            sol.totalObjective += objval;
                            sol.holdingCost = objval;


                            if (sol.TestEverythingFromScratch() != INFEASIBILITY_STATUS.FEASIBLE)
                                GlobalUtils.writeToConsole("Lefteri ftiakse tis malakies");
                            
                            break;
                        }
                    case GRB.Status.INFEASIBLE:
                        {
                            GlobalUtils.writeToConsole("Model is infeasible");

                            // compute and write out IIS
                            //model.ComputeIIS();
                            //model.Write("model.ilp");
                            return false;
                        }
                    case GRB.Status.UNBOUNDED:
                        {
                            GlobalUtils.writeToConsole("Model is unbounded");
                            return false;
                        }
                    default:
                        {
                            GlobalUtils.writeToConsole("Optimization was stopped with status = "
                                              + optimstatus);
                            return false;
                        }

                }

                // Dispose of model
                model.Dispose();
            }
            catch (GRBException e)
            {
                Console.WriteLine("Error code: " + e.ErrorCode + ". " + e.Message);
            }

            return true;
        }

        //LP formulation for simultaneously re-optimizing the delivery quantities of all customers and the production quantities as well. 
        public static bool runSimultaneousDeliveryProductionReoptimizationLP(ref Solution sol)
        {
            int numberOfPeriods = sol.periods.Count;
            int numberOfCustomers = sol.customers.Count;
            double[,] UBq = new double[numberOfCustomers, numberOfPeriods]; //upper bound of delivered quantities to customer
            double[] UBp = new double[numberOfPeriods]; //upper bound of delivered quantities to customer
            double totalProdQuantity = 0.0;
            int actualCapacity = sol.periods[0].periodRoutes[0].realCapacity;

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
                        totalDemandUntilEnd += cust.productRate[tt];  //@greg remaining demand until end of horizon (OK)
                    }
                    if (!cust.visitSchedule[t])
                        UBq[i, t] = 0;
                    else
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

            try
            {
                //gurobiEnv = new GRBEnv();
                GRBModel model = new GRBModel(gurobiEnv);

                // Params
                model.ModelName = "allCustomersAndProductionQuantitiesLP";
                model.Parameters.OutputFlag = 0;
                model.Parameters.Threads = threadLimit;

                // Decision variables
                GRBVar[] I_depot = new GRBVar[numberOfPeriods]; // depot inventory
                GRBVar[] p = new GRBVar[numberOfPeriods]; // depot production
                GRBVar[,] I_cust = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer inventory
                GRBVar[,] q = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer delivery quantities
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
                        q[i, t] = model.AddVar(0.0, UBq[i, t], 0.0, GRB.CONTINUOUS, "q_" + i + "^" + t);
                        I_cust[i, t] = model.AddVar(0.0, cust.stockMaximumLevel, cust.unitHoldingCost, GRB.CONTINUOUS, "I_" + i + " ^ " + t);
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
                        GRBLinExpr lhs = I_cust[i, t];
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
                        GRBLinExpr lhs = q[i, t];
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
                        model.AddConstr(lhs <= actualCapacity, "cust_vehcap_" + r + " ^ " + t);
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
                                sol.depot.productionRates[t] = (int)p[t].X;
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
                                    cust.deliveredQuantities[t] = (int)q[i, t].X; //update delivery quantity
                                    sol.depot.deliveredQuantities[t] -= cust.deliveredQuantities[t]; 
                                    // update customer delivery and vehicle load
                                    if (cd.route != null)
                                    {
                                        cd.quantity = cust.deliveredQuantities[t];
                                        cd.route.load += cd.quantity;
                                        cd.route.SetLoadAndCostLists(t, sol.model);
                                    }
                                }
                                cust.CalculateInventoryLevels();
                            }
                            sol.depot.CalculateInventoryLevels();
                            
                            //Fix solution objective
                            sol.totalObjective -= sol.holdingCost;
                            sol.totalObjective += objval;
                            sol.holdingCost = objval;

                            if (!Solution.checkSolutionStatus(sol.TestEverythingFromScratch()))
                                GlobalUtils.writeToConsole("Lefteri ftiakse tis malakies sou sto production lp");

                            break;
                        }
                    case GRB.Status.INFEASIBLE:
                        {
                            GlobalUtils.writeToConsole("Model is infeasible");

                            // compute and write out IIS
                            //model.ComputeIIS();
                            //model.Write("model.ilp");
                            return false;
                        }
                    case GRB.Status.UNBOUNDED:
                        {
                            GlobalUtils.writeToConsole("Model is unbounded");
                            return false;
                        }
                    default:
                        {
                            GlobalUtils.writeToConsole("Optimization was stopped with status = "
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

        //LP formulation for simultaneously re-optimizing the delivery quantities of all customers and the production quantities 
        // as well allowing vehicle capacity to be violated. 
        public static bool runSimultaneousDeliveryProductionReoptimizationLPVehCapInfeas(Solution sol)
        {
            int numberOfPeriods = sol.periods.Count;
            int numberOfCustomers = sol.customers.Count;

            int unitCapPenalty = 100; //Note the coefficient for penalizing capacity violation (per unit)
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
                    if (!cust.visitSchedule[t])
                        UBq[i, t] = 0;
                    else
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
                GRBVar[,] e = new GRBVar[maxNumberVehUsed, numberOfPeriods]; // customer delivery quantities

                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    I_depot[t] = model.AddVar(0.0, sol.depot.stockMaximumLevel, sol.depot.unitHoldingCost, GRB.CONTINUOUS, "I_d^" + t);
                    p[t] = model.AddVar(0.0, UBp[t], 0, GRB.CONTINUOUS, "p_d^" + t); //@greg productionCapacity: is  this the maximum quantity we can produce per dayq

                    Period per = sol.periods[t];
                    for (int r = 0; r < per.periodRoutes.Count; r++)
                    {
                        Route rt = per.periodRoutes[r];
                        e[r,t] = model.AddVar(0.0, excessSlack, unitCapPenalty, GRB.CONTINUOUS, "e_" + r + "^" + t);
                    }
                }
                
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        q[i, t] = model.AddVar(0.0, UBq[i, t], 0.0, GRB.CONTINUOUS, "q_" + i + "^" + t);
                        I_cust[i, t] = model.AddVar(0.0, cust.stockMaximumLevel, cust.unitHoldingCost, GRB.CONTINUOUS, "I_" + i + " ^ " + t);
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
                        GRBLinExpr lhs = e[r,t];
                        for (int i = 1; i < route.nodes.Count - 1; i++)
                        {
                            Node cust = route.nodes[i];
                            int ii = Int32.Parse(cust.ID) - 2; // Attention: id is set as input+1. So depot 0 is 1, Node 1 is 2, etc. For q we start the first customer (2) in index 0.
                            lhs.AddTerm(-1.0, q[ii, t]);
                        }
                        model.AddConstr(lhs >= -actualCapacity, "e" + r + " ^ " + t); //NOTE: effective capacity
                    }
                } 
                
                // 5. Total production quantity
                GRBLinExpr lhsp = 0.0;
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    lhsp.AddTerm(1.0, p[t]);
                }
                model.AddConstr(lhsp == totalProdQuantity, "total_prod");

                
                //GlobalUtils.writeToConsole("VIOLATIONS BEFORE LP {0}", sol.calcVehicleLoadViolations());
                
                // Optimize
                model.Optimize();
                int optimstatus = model.Status;

                switch (model.Status)
                {
                    case GRB.Status.OPTIMAL:
                        {
                            double objval = model.ObjVal;

                            // Update solution with new calculated quantities
                            for (int t = 0; t < numberOfPeriods; t++)
                            {
                                sol.depot.deliveredQuantities[t] = 0;
                                sol.depot.productionRates[t] = (int)p[t].X;
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
                                    cust.deliveredQuantities[t] = (int)q[i, t].X; //update delivery quantity
                                    sol.depot.deliveredQuantities[t] -= cust.deliveredQuantities[t]; // OR +=;
                                    // update customer delivery and vehicle load
                                    if (cd.route != null)
                                    {
                                        cd.quantity = cust.deliveredQuantities[t];
                                        cd.route.load += cd.quantity;
                                        cd.route.SetLoadAndCostLists(t, sol.model);
                                    }
                                }
                                cust.CalculateInventoryLevels();
                            }
                            sol.depot.CalculateInventoryLevels();
                            
                            //Fix solution objective
                            //sol.totalObjective -= sol.holdingCost;
                            //sol.totalObjective += objval;
                            //sol.holdingCost = objval;
                            //sol.routingCost = Solution.EvaluateRoutingObjectivefromScratch(sol);
                            sol.holdingCost = Solution.EvaluateInventoryObjectivefromScratch(sol);
                            //sol.totalUnitProductionCost = Solution.EvaluateUnitProductionObjectivefromScratch(sol);
                            //sol.setupProductionCost = Solution.EvaluateSetupProductionObjectivefromScratch(sol);
                            sol.totalObjective = sol.routingCost + sol.holdingCost + sol.totalUnitProductionCost + sol.setupProductionCost;

                            
                            
                            if (!Solution.checkSolutionStatus(sol.TestEverythingFromScratch()))
                                GlobalUtils.writeToConsole("Lefteri ftiakse tis malakies sou sto production lp");
                            
                            //GlobalUtils.writeToConsole("VIOLATIONS AFTER LP {0}", sol.calcVehicleLoadViolations());
                            break;
                        }
                    case GRB.Status.INFEASIBLE:
                        {
                            GlobalUtils.writeToConsole("Model is infeasible");

                            // compute and write out IIS
                            //model.ComputeIIS();
                            //model.Write("model.ilp");
                            return false;
                        }
                    case GRB.Status.UNBOUNDED:
                        {
                            GlobalUtils.writeToConsole("Model is unbounded");
                            return false;
                        }
                    default:
                        {
                            GlobalUtils.writeToConsole("Optimization was stopped with status = "
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

        //==================================================== Utilities ====================================================//
        //Gurobi Environment Management
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

        /* ATTEMPT TO KEEP MODEL 
// Create empty model without rhs 
public static GRBModel createEmptyCustomerModel(Solution sol, Node cust)
{
    int numberOfPeriods = sol.periods.Count;
    double[] UBq = new double[numberOfPeriods]; //upper bound of delivered quantities to customer
    double[] otherq = new double[numberOfPeriods]; // the quantities delivered to other customers each period (except cust)
    double[] vehicle_remaining_cap = new double[numberOfPeriods];  //vehicle remaining capacity without cust

    //calculate UBq 
    for (int t = 0; t < numberOfPeriods; ++t)
    {
        int aggrCap = cust.stockMaximumLevel + cust.productRate[t]; //@greg cap + demand (OK)
        int vehCap = sol.model.capacity; //@greg veh capacity (OK)
        int totalDemandUntilEnd = 0;
        for (int tt = t; tt < numberOfPeriods; ++tt)
        {
            totalDemandUntilEnd += cust.productRate[tt];  //@greg remaining demand until end of horizon (OK)
        }
        if (!cust.auxiliary_visitSchedule[t])
            UBq[t] = (double)GlobalUtils.Min(0, aggrCap, vehCap, totalDemandUntilEnd);
        else
            UBq[t] = (double)GlobalUtils.Min(aggrCap, vehCap, totalDemandUntilEnd);
    }

    // calculate quantities delivered to other customers
    for (int t = 0; t < numberOfPeriods; ++t)
    {
        otherq[t] = -sol.depot.deliveredQuantities[t] - cust.deliveredQuantities[t];
    }

    //  // calculate the remaining capacity of the truck serving the customer
    for (int t = 0; t < numberOfPeriods; ++t)
    {

        CustDelivery cd = cust.auxiliary_horizonDeliveryServices[t];

        vehicle_remaining_cap[t] = 0;
        if (cd != null)
        {
            vehicle_remaining_cap[t] = cd.route.capacity - cd.route.load + cd.quantity;
        }
    }

    GRBModel model = new GRBModel(gurobiEnv);
    // Params
    model.ModelName = "customerLP";
    model.Parameters.OutputFlag = 1;

    try
    {
        // Decision variables
        GRBVar[] I_depot = new GRBVar[numberOfPeriods]; // depot inventory
        GRBVar[] I_cust = new GRBVar[numberOfPeriods]; // customer inventory
        GRBVar[] q = new GRBVar[numberOfPeriods]; // customer delivery quantities

        for (int t = 0; t < numberOfPeriods; ++t)
        {
            I_depot[t] = model.AddVar(0.0, sol.depot.stockMaximumLevel, 0, GRB.CONTINUOUS, "I_d^" + t);
            I_cust[t] = model.AddVar(0.0, cust.stockMaximumLevel, 0, GRB.CONTINUOUS, "I_c^" + t);
        }
        for (int t = 0; t < numberOfPeriods; ++t)
        {
            q[t] = model.AddVar(0.0, UBq[t], 0.0, GRB.CONTINUOUS, "q_c^" + t);
        }

        GRBLinExpr objectiveFunction = new GRBLinExpr();
        for (int t = 0; t < numberOfPeriods; ++t)
        {
            objectiveFunction.AddTerm(sol.depot.unitHoldingCost, I_depot[t]);
            objectiveFunction.AddTerm(cust.unitHoldingCost, I_cust[t]);
        }
        model.SetObjective(objectiveFunction);

        // Objective
        model.ModelSense = GRB.MINIMIZE;

        // Constraints 
        // 1. Depot inventory flow 
        for (int t = 0; t < numberOfPeriods; ++t)
        {
            GRBLinExpr lhs = 0.0;
            lhs.AddTerm(1.0, I_depot[t]);
            if (t == 0)
            {
                model.AddConstr(lhs == sol.depot.startingInventory + sol.depot.productionRates[t] - (q[t] + otherq[t]), "depot_inv_" + t);
            }
            else
            {
                //lhs.AddTerm(-1.0, q[t]);
                //lhs.AddTerm(-1.0, I_cust[t - 1]);
                //model.AddConstr(lhs == -cust.productRate[t], "cust_inv" + t);
                model.AddConstr(lhs == I_depot[t - 1] + sol.depot.productionRates[t] - (q[t] + otherq[t]), "depot_inv_" + t);
            }
        }

        // 2. Customer inventory flow
        for (int t = 0; t < numberOfPeriods; ++t)
        {
            GRBLinExpr lhs = 0.0;
            lhs.AddTerm(1.0, I_cust[t]);
            if (t == 0)
            {
                lhs.AddTerm(-1.0, q[t]);
                model.AddConstr(lhs == cust.startingInventory - cust.productRate[t], "cust_inv" + t);
                //model.AddConstr(lhs == cust.startingInventory + q[t] - cust.productRate[t], "cust_inv" + t);
            }
            else
            {
                lhs.AddTerm(-1.0, q[t]);
                lhs.AddTerm(-1.0, I_cust[t - 1]);
                model.AddConstr(lhs == -cust.productRate[t], "cust_inv" + t);
                //model.AddConstr(lhs == I_cust[t - 1] + q[t] - cust.productRate[t], "cust_inv" + t);
            }
        }

        // 3. Customer delivery capacity
        for (int t = 0; t < numberOfPeriods; ++t)
        {
            GRBLinExpr lhs = 0.0;
            lhs.AddTerm(1.0, q[t]);
            if (t == 0)
                model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - cust.startingInventory, "cust_ml" + t);
            else
                model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - I_cust[t - 1], "cust_ml" + t);
        }

        // 4. Customer delivery vehicle capacity
        for (int t = 0; t < numberOfPeriods; ++t)
        {
            GRBLinExpr lhs = 0.0;
            lhs.AddTerm(1.0, q[t]);
            model.AddConstr(lhs <= vehicle_remaining_cap[t], "cust_vehcap_" + t);
        }

        // Optimization
        model.Optimize();

        Console.Write("\nInitial model \n{0,20}", "Cust Deliveries");
        for (int i = 0; i < numberOfPeriods; i++)
            Console.Write(" {0,8:0.0} ", (int)q[i].X);
        Console.Write("\n");

        Console.Write("{0,20}", "Cust Inv");
        for (int i = 0; i < numberOfPeriods; i++)
            Console.Write(" {0,8:0.0} ", (int)I_cust[i].X);
        Console.Write("\n");
    }
    catch (GRBException e)
    {
        GlobalUtils.writeToConsole("Error code: " + e.ErrorCode + ". " + e.Message);
    }
    return model;
}


//LP formulation for optimizing the customer cust quantities given a solution sol.
public static bool runLP(Solution sol, Node cust, GRBModel model)
{
    int numberOfPeriods = sol.periods.Count;
    double[] UBq = new double[numberOfPeriods]; //upper bound of delivered quantities to customer
    double[] otherq = new double[numberOfPeriods]; // the quantities delivered to other customers each period (except cust)
    double[] vehicle_remaining_cap = new double[numberOfPeriods];  //vehicle remaining capacity without cust

    //calculate UBq 
    for (int t = 0; t < numberOfPeriods; ++t)
    {
        int aggrCap = cust.stockMaximumLevel + cust.productRate[t]; //@greg cap + demand (OK)
        int vehCap = sol.model.capacity; //@greg veh capacity (OK)
        int totalDemandUntilEnd = 0;
        for (int tt = t; tt < numberOfPeriods; ++tt)
        {
            totalDemandUntilEnd += cust.productRate[tt];  //@greg remaining demand until end of horizon (OK)
        }
        if (!cust.auxiliary_visitSchedule[t])
            UBq[t] = (double)GlobalUtils.Min(0, aggrCap, vehCap, totalDemandUntilEnd);
        else
            UBq[t] = (double)GlobalUtils.Min(aggrCap, vehCap, totalDemandUntilEnd);
    }

    // calculate quantities delivered to other customers
    for (int t = 0; t < numberOfPeriods; ++t)
    {
        otherq[t] = -sol.depot.deliveredQuantities[t] - cust.deliveredQuantities[t];
    }

    //  // calculate the remaining capacity of the truck serving the customer
    for (int t = 0; t < numberOfPeriods; ++t)
    {

        CustDelivery cd = cust.auxiliary_horizonDeliveryServices[t];

        vehicle_remaining_cap[t] = 0;
        if (cd != null)
        {
            vehicle_remaining_cap[t] = cd.route.capacity - cd.route.load + cd.quantity;
        }
    }

    try
    {
        // Decision variables
        GRBVar[] I_depot = new GRBVar[numberOfPeriods]; // depot inventory
        GRBVar[] I_cust = new GRBVar[numberOfPeriods]; // customer inventory
        GRBVar[] q = new GRBVar[numberOfPeriods]; // customer delivery quantities

        for (int t = 0; t < numberOfPeriods; ++t)
        {
            I_depot[t] = model.AddVar(0.0, sol.depot.stockMaximumLevel, 0, GRB.CONTINUOUS, "I_d^" + t);
            I_cust[t] = model.AddVar(0.0, cust.stockMaximumLevel, 0, GRB.CONTINUOUS, "I_c^" + t);
        }
        for (int t = 0; t < numberOfPeriods; ++t)
        {
            q[t] = model.AddVar(0.0, UBq[t], 0.0, GRB.CONTINUOUS, "q_c^" + t);
        }

        GRBLinExpr objectiveFunction = new GRBLinExpr();
        for (int t = 0; t < numberOfPeriods; ++t)
        {
            objectiveFunction.AddTerm(sol.depot.unitHoldingCost, I_depot[t]);
            objectiveFunction.AddTerm(cust.unitHoldingCost, I_cust[t]);
        }
        model.SetObjective(objectiveFunction);

        // Objective
        model.ModelSense = GRB.MINIMIZE;

        // Constraints 
        // 1. Depot inventory flow 
        for (int t = 0; t < numberOfPeriods; ++t)
        {
            GRBLinExpr lhs = 0.0;
            lhs.AddTerm(1.0, I_depot[t]);
            if (t == 0)
            {
                model.AddConstr(lhs == sol.depot.startingInventory + sol.depot.productionRates[t] - (q[t] + otherq[t]), "depot_inv_" + t);
            }
            else
            {
                //lhs.AddTerm(-1.0, q[t]);
                //lhs.AddTerm(-1.0, I_cust[t - 1]);
                //model.AddConstr(lhs == -cust.productRate[t], "cust_inv" + t);
                model.AddConstr(lhs == I_depot[t - 1] + sol.depot.productionRates[t] - (q[t] + otherq[t]), "depot_inv_" + t);
            }                      
        }

        // 2. Customer inventory flow
        for (int t = 0; t < numberOfPeriods; ++t)
        {
            GRBLinExpr lhs = 0.0;
            lhs.AddTerm(1.0, I_cust[t]);
            if (t == 0)
            {
                lhs.AddTerm(-1.0, q[t]);
                model.AddConstr(lhs == cust.startingInventory - cust.productRate[t], "cust_inv" + t);
                //model.AddConstr(lhs == cust.startingInventory + q[t] - cust.productRate[t], "cust_inv" + t);
            }
            else
            {
                lhs.AddTerm(-1.0, q[t]);
                lhs.AddTerm(-1.0, I_cust[t - 1]);
                model.AddConstr(lhs == - cust.productRate[t], "cust_inv" + t);
                //model.AddConstr(lhs == I_cust[t - 1] + q[t] - cust.productRate[t], "cust_inv" + t);
            }                        
        }

        // 3. Customer delivery capacity
        for (int t = 0; t < numberOfPeriods; ++t)
        {
            GRBLinExpr lhs = 0.0;
            lhs.AddTerm(1.0, q[t]);
            if (t == 0)
                model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - cust.startingInventory, "cust_ml" + t);
            else
                model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - I_cust[t - 1], "cust_ml" + t);
        }

        // 4. Customer delivery vehicle capacity
        for (int t = 0; t < numberOfPeriods; ++t)
        {
            GRBLinExpr lhs = 0.0;
            lhs.AddTerm(1.0, q[t]);
            model.AddConstr(lhs <= vehicle_remaining_cap[t], "cust_vehcap_" + t);
        }

        // Optimization
        model.Optimize();


        //change coef 
        //model.ChgCoeff();
        //GRBExpr obj = model.GetObjective();
        //model.GetVarByName("q").Obj.(0);
        //obj.coeffs[0] = 7;
        //Console.Write(obj);
        //model.ChgCoeff(objectiveFunction, I_depot[t], -sol.depot.unitHoldingCost);
        //model.ChgCoeff(objectiveFunction, I_cust[t], -cust.unitHoldingCost);
        //I_depot[t].Set(GRB_DoubleAttr_Obj, 0.0);
        //I_depot[t].Set(GRB.DoubleAttr, --cust.unitHoldingCost);
        //I_cust[t].Set(GRB.DoubleAttr, --cust.unitHoldingCost);
        //a1[6].set(GRB_DoubleAttr_Obj, 2.0);
        GRBLinExpr objectiveFunction2 = new GRBLinExpr();
        for (int t = 0; t < numberOfPeriods; ++t)
        {
            objectiveFunction2.AddTerm(sol.depot.unitHoldingCost, I_depot[t]);
            objectiveFunction2.AddTerm(cust.unitHoldingCost, I_cust[t]);
        }

        model.SetObjective(objectiveFunction2);
        for (int t = 0; t < numberOfPeriods; ++t)
        {
            if (t == 0)
                model.GetConstrByName("cust_inv"+t).Set(GRB.DoubleAttr.RHS, cust.startingInventory - cust.productRate[t]);
            else
                model.GetConstrByName("cust_inv" + t).Set(GRB.DoubleAttr.RHS, -cust.productRate[t]);


            // 1. Depot inventory flow 
            if (t == 0)
                model.GetConstrByName("depot_inv_" + t).Set(GRB.DoubleAttr.RHS, sol.depot.startingInventory + sol.depot.productionRates[t] - otherq[t]);
            else
                model.GetConstrByName("depot_inv_" + t).Set(GRB.DoubleAttr.RHS, sol.depot.productionRates[t] - otherq[t]);
            if (t == 0)
                model.AddConstr(lhs == sol.depot.startingInventory + sol.depot.productionRates[t] - (q[t] + otherq[t]), "depot_inv_" + t);
            else
                model.AddConstr(lhs == I_depot[t - 1] + sol.depot.productionRates[t] - (q[t] + otherq[t]), "depot_inv_" + t);


            // 2. Customer inventory flow
            for (int t = 0; t < numberOfPeriods; ++t)
            {
                GRBLinExpr lhs = 0.0;
                lhs.AddTerm(1.0, I_cust[t]);
                if (t == 0)
                {
                    //lhs.AddTerm(-1.0, q[t]);
                    //model.AddConstr(lhs == cust.startingInventory - cust.productRate[t], "cust_inv" + t);
                    model.AddConstr(lhs == cust.startingInventory + q[t] - cust.productRate[t], "cust_inv" + t);
                }
                else
                {
                    //lhs.AddTerm(-1.0, q[t]);
                    //lhs.AddTerm(-1.0, I_cust[t - 1]);
                    //model.AddConstr(lhs == - cust.productRate[t], "cust_inv" + t);
                    model.AddConstr(lhs == I_cust[t - 1] + q[t] - cust.productRate[t], "cust_inv" + t);
                }
            }

            // 3. Customer delivery capacity
            for (int t = 0; t < numberOfPeriods; ++t)
            {
                GRBLinExpr lhs = 0.0;
                lhs.AddTerm(1.0, q[t]);
                if (t == 0)
                    model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - cust.startingInventory, "cust_ml" + t);
                else
                    model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - I_cust[t - 1], "cust_ml" + t);
            }

            // 4. Customer delivery vehicle capacity
            for (int t = 0; t < numberOfPeriods; ++t)
            {
                GRBLinExpr lhs = 0.0;
                lhs.AddTerm(1.0, q[t]);
                model.AddConstr(lhs <= vehicle_remaining_cap[t], "cust_vehcap_" + t);
            }

        }


        model.Update();
        model.Optimize();

        Console.Write("\nModel after \n{0,20}", "Cust Deliveries");
        for (int i = 0; i < numberOfPeriods; i++)
            Console.Write(" {0,8:0.0} ", (int)q[i].X);
        Console.Write("\n");

        Console.Write("{0,20}", "Cust Inv");
        for (int i = 0; i < numberOfPeriods; i++)
            Console.Write(" {0,8:0.0} ", (int)I_cust[i].X);
        Console.Write("\n");

        int optimstatus = model.Status;

        if (optimstatus == GRB.Status.OPTIMAL)
        {
            double objval = model.ObjVal;


            //Compare with auxiliary deliveries
            /*
            for (int t = 0; t < numberOfPeriods; t++)
            {
                if (!GlobalUtils.IsEqual((int) q[t].X, cust.auxiliary_deliveries[t]) && !GlobalUtils.IsEqual(cust.unitHoldingCost, sol.depot.unitHoldingCost))
                {
                    GlobalUtils.writeToConsole("LP vs Heuristic Mismatch");
                    cust.ReportInventory(true);


                    GlobalUtils.writeToConsole("Optimal objective: " + objval);
                    //Print Day horizon
                    Console.Write("{0,20}", "Horizon");
                    for (int i = 0; i < numberOfPeriods; i++)
                        Console.Write(" {0,8} ", i);
                    Console.Write("\n");

                    //Print Start Day Inventory
                    Console.Write("{0,20}", "Depot Inv");
                    for (int i = 0; i < numberOfPeriods; i++)
                        Console.Write(" {0,8:0.0} ", (int) I_depot[i].X);
                    Console.Write("\n");

                    Console.Write("{0,20}", "Cust Deliveries");
                    for (int i = 0; i < numberOfPeriods; i++)
                        Console.Write(" {0,8:0.0} ", (int) q[i].X);
                    Console.Write("\n");

                    Console.Write("{0,20}", "Cust Inv");
                    for (int i = 0; i < numberOfPeriods; i++)
                        Console.Write(" {0,8:0.0} ", (int) I_cust[i].X);
                    Console.Write("\n");


                    //cust.ApplySawMaxDeliveryLast(cust.auxiliary_visitSchedule, sol);
                    cust.ApplySawMaxDeliveryFirst(cust.auxiliary_visitSchedule, sol);


                }
            }



        }
        else if (optimstatus == GRB.Status.INFEASIBLE)
        {
            GlobalUtils.writeToConsole("Model is infeasible");

            // compute and write out IIS
            //model.ComputeIIS();
            //model.Write("model.ilp");
            return false;
        }
        else if (optimstatus == GRB.Status.UNBOUNDED)
        {
            GlobalUtils.writeToConsole("Model is unbounded");
            return false;
        }
        else
        {
            GlobalUtils.writeToConsole("Optimization was stopped with status = "
                                + optimstatus);
            return false;
        }

        // Dispose of model
        //model.Dispose();
    }
    catch (GRBException e)
    {
        GlobalUtils.writeToConsole("Error code: " + e.ErrorCode + ". " + e.Message);
    }

    return true;
}

*/
    }
}
