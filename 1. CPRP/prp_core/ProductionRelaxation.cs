using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Net.Http.Headers;
using System.Net.Sockets;
using System.Xml;
using System.Linq;
using Gurobi;

namespace PRP
{
    public class ProductionRelaxation
    {
        public static GRBEnv gurobiEnv;
        public static Stopwatch stopwatch = new Stopwatch();
        public static int threadLimit;

        //==================================================== MIPS ====================================================//
        
        public static bool runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemovalSameDayRoutesOriginalProdDays(
            ref Solution sol, List<Solution> previousSols, bool forceDiffSchedule, bool preserveProdDaysNum, int maxRouteAdditions, int maxRouteRemovals, double discountFactor, bool isCont = false)
        {
            Solution back_sol = new Solution(sol, 0.0);
            //if (!Solution.checkSolutionStatus(sol.TestEverythingFromScratch()))
            //throw new Exception("Production Mip with infeasibilities and insertion/removal - ERROR IN INPUT SOLUTION");

            int numberOfPeriods = sol.periods.Count;
            int numberOfCustomers = sol.customers.Count;
            int effectiveCapacity = sol.periods[0].periodRoutes[0].effectiveCapacity;
            int actualCapacity = sol.periods[0].periodRoutes[0].realCapacity;

            double[,] UBq = new double[numberOfCustomers, numberOfPeriods]; //upper bound of delivered quantities to customer
            double[] UBp = new double[numberOfPeriods]; //upper bound of delivered quantities to customer
            double totalProdQuantity = 0.0;
            int totalProdDays = 0;

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
                if (sol.depot.open[t])
                    totalProdDays += 1;
                UBp[t] = sol.depot.productionCapacity;
            }

            // find the routes in the solution and create the solution pool
            List<Route> routesPool = new List<Route>();
            routesPool = findRoutes(routesPool, ref sol);
            routesPool = appendDirectRoutes(routesPool, ref sol);

            double[] routeCost = new double[routesPool.Count]; //upper bound of delivered quantities to customer
            int[,] routeCustomer= new int[sol.customers.Count, routesPool.Count]; //upper bound of delivered quantities to customer
            routeCost = calculateRouteCostCustomers(routesPool, routeCost, routeCustomer, ref sol, discountFactor);

            try
            {
                GRBEnv gurobiEnv = new GRBEnv(); //TODO: global env?
                GRBModel model = new GRBModel(gurobiEnv);

                // Params
                model.ModelName = "allCustomersAndProductionQuantitiesLPVehCapInfeasInsRemSameDaysProdDays" + sol.ellapsedMs;
                model.Parameters.OutputFlag = (GlobalUtils.suppress_output) ? 0 : 1;
                model.Parameters.MIPGap = 0.001;
                model.Parameters.TimeLimit = 120;

                // Decision variables
                GRBVar[] I_depot = new GRBVar[numberOfPeriods]; // depot inventory
                GRBVar[] p = new GRBVar[numberOfPeriods]; // depot production
                GRBVar[] y = new GRBVar[numberOfPeriods]; // depot production setup
                GRBVar[,] I_cust = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer inventory
                GRBVar[,] q = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer delivery quantities
                GRBVar[,] z = new GRBVar[numberOfCustomers, numberOfPeriods];
                GRBVar[,] x = new GRBVar[routesPool.Count, numberOfPeriods];

                for (int r = 0; r < routesPool.Count; ++r)
                {
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        x[r, t] = model.AddVar(0.0, 1.0, routeCost[r], GRB.BINARY, "x^" + t);
                    }
                }

                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    I_depot[t] = model.AddVar(0.0, sol.depot.stockMaximumLevel, sol.depot.unitHoldingCost, GRB.CONTINUOUS, "I_d^" + t);
                    y[t] = model.AddVar(0.0, 1.0, sol.depot.productionSetupCost, GRB.BINARY, "y^" + t);
                    if (isCont)
                        p[t] = model.AddVar(0.0, UBp[t], sol.depot.unitProductionCost, GRB.CONTINUOUS, "p_d^" + t); //@greg productionCapacity: is  this the maximum quantity we can produce per dayq
                    else
                        p[t] = model.AddVar(0.0, UBp[t], sol.depot.unitProductionCost, GRB.INTEGER, "p_d^" + t); //@greg productionCapacity: is  this the maximum quantity we can produce per dayq
                }

                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        z[i, t] = model.AddVar(0.0, 1.0, 0.0, GRB.BINARY, "z" + i + "^" + t);
                        if (isCont)
                            q[i, t] = model.AddVar(0.0, UBq[i, t], 0.0, GRB.CONTINUOUS, "q_" + i + "^" + t);
                        else
                            q[i, t] = model.AddVar(0.0, UBq[i, t], 0.0, GRB.INTEGER, "q_" + i + "^" + t);
                        I_cust[i, t] = model.AddVar(0.0, cust.stockMaximumLevel, cust.unitHoldingCost, GRB.CONTINUOUS, "I_" + i + "^" + t);
                    }
                }

                //Branching Priority of production setup
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    y[t].BranchPriority = 2;
                }

                // Objective
                model.ModelSense = GRB.MINIMIZE;

                // Constraints 
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    GRBLinExpr lhs = p[t];
                    if ((t == 0) && sol.model.input.type == PRP_DATASET_VERSION.BOUDIA_FMT) 
                        model.AddConstr(lhs == 0, "boudia_no_prod");
                    else
                        model.AddConstr(lhs <= sol.depot.productionCapacity * y[t], "depot_setup_" + t);
                }

                if (preserveProdDaysNum)
                { 
                    GRBLinExpr lhs2 = 0;
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        lhs2.AddTerm(1.0, y[t]);
                    }
                    model.AddConstr(lhs2 <= totalProdDays, "totalprodDays");
                }     
                
                //@constraint(model, prod_cona[t = indices.T],
                //    p[t] - y[t] * (sum(sum(pd.demands[i][t_1] for t_1 = t:length(indices.T)) for i = indices.V_cus )) <= 0)
                //@constraint(model, prod_conc[t = indices.T],
                //    p[t] - (sum(sum(pd.demands[i][t_1] for t_1 = t:length(indices.T)) -I[i, t - 1] for i = indices.V_cus ) -I[1, t - 1]) <= 0)

                // disallow same schedule
                if (forceDiffSchedule)
                {
                    GRBLinExpr lhste = 0.0;
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        if (sol.depot.open[t])
                        {
                            lhste += 1;
                            lhste.AddTerm(-1.0, y[t]);
                        }
                        else
                        {
                            lhste.AddTerm(1.0, y[t]);
                        }
                    }
                    model.AddConstr(lhste >= 1, "different_schedule");
                }


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
                /*
                GRBLinExpr lhsp = 0.0;
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    lhsp.AddTerm(1.0, p[t]);
                }
                model.AddConstr(lhsp == totalProdQuantity, "total_prod");
                */

                // Total quantity delivered to customer per time period 
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    GRBLinExpr lhs = 0.0;
                    for (int i = 0; i < sol.customers.Count; i++)
                    {
                        Node cust = sol.customers[i];
                        lhs.AddTerm(1.0, q[i, t]);
                    }
                    model.AddConstr(lhs <= sol.model.input.availableVehicles * actualCapacity, "cap^" + t);
                }

                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        GRBLinExpr lhs = q[i, t];
                        model.AddConstr(lhs <= z[i, t] * UBq[i, t], "cust_zq" + i + "^" + t);
                    }
                }

                
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        GRBLinExpr rhs = 0.0;
                        for (int r = 0; r < routesPool.Count; r++)
                        {
                            rhs.AddTerm(routeCustomer[i, r], x[r, t]);
                        }
                        model.AddConstr(z[i, t] <= rhs, "xz_" + i + "_" + "^" + t);
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


                            Console.Write("\nOld schedule: ");
                            for (int t = 0; t < numberOfPeriods; t++)
                            {
                                if (sol.depot.open[t])
                                    Console.Write("_1");
                                else 
                                    Console.Write("_0");
                            }

                            // Update production
                            for (int t = 0; t < numberOfPeriods; t++)
                            {
                                sol.depot.open[t] = false;
                                sol.depot.deliveredQuantities[t] = 0;
                                sol.depot.productionRates[t] = (int)Math.Round(p[t].X);
                                if (sol.depot.productionRates[t] > 0)
                                    sol.depot.open[t] = true;
                            }
                            Console.Write("\nNew schedule: ");
                            for (int t = 0; t < numberOfPeriods; t++)
                            {
                                if (sol.depot.open[t])
                                    Console.Write("_1");
                                else
                                    Console.Write("_0");
                            }
                            GlobalUtils.writeToConsole("");

                            // update customer deliveries 
                            for (int i = 0; i < sol.customers.Count; i++)
                            {
                                Node cust = sol.customers[i];

                                for (int t = 0; t < numberOfPeriods; t++)
                                {
                                    CustDelivery cd = cust.horizonDeliveryServices[t];
                                    if (cd.route != null)
                                    {
                                        cd.route.load -= cd.quantity;
                                        cd.route = null;
                                    }

                                    cust.deliveredQuantities[t] = (int)Math.Round(q[i, t].X); //update delivery quantity
                                    cd.quantity = cust.deliveredQuantities[t];
                                    sol.depot.deliveredQuantities[t] -= cust.deliveredQuantities[t];
                                    if (cd.quantity > 0)
                                        cust.visitSchedule[t] = true;
                                    else
                                        cust.visitSchedule[t] = false;
                                }
                                cust.CalculateInventoryLevels();
                            }
                            sol.depot.CalculateInventoryLevels();


                            // update customer routes 
                            // erase routes routes
                            for (int t = 0; t < sol.periods.Count; t++)
                            {
                                Period per = sol.periods[t];
                                per.periodRoutes.Clear();
                                for (int k = 0; k < sol.model.input.availableVehicles; k++)
                                {
                                    Route rt = new Route(actualCapacity);
                                    rt.effectiveCapacity = effectiveCapacity;
                                    rt.initialize(sol.depot);
                                    per.periodRoutes.Add(rt);
                                }
                            }


                            bool[,] introducedCustomer = new bool[numberOfCustomers,numberOfPeriods];
                            for (int t = 0; t < numberOfPeriods; ++t)
                            {
                                Period per = sol.periods[t];

                                for (int r = 0; r < routesPool.Count; r++)
                                {
                                    if (GlobalUtils.IsEqual(x[r, t].X, 1.0))
                                    {
                                        GlobalUtils.writeToConsole("t={0} --> route: {1}", t, routesPool[r]);
                                        String used = "used: ";
                                        String skipped = "skipped: ";
                                        for (int ii = 1; ii < routesPool[r].nodes.Count - 1; ii++)
                                        {
                                            Node cust2 = routesPool[r].nodes[ii];
                                            int idx = Int32.Parse(cust2.ID) - 2;
                                            if (GlobalUtils.IsEqual(z[idx, t].X, 1.0))
                                                used = $"{used},{cust2.uid}";
                                            else
                                                skipped = $"{skipped},{cust2.uid}";
                                        }
                                        GlobalUtils.writeToConsole("{0} vs {1}", used, skipped);

                                        for (int ii=1; ii < routesPool[r].nodes.Count-1; ii++)
                                        {
                                            Node cust2 = routesPool[r].nodes[ii];
                                            int idx = Int32.Parse(cust2.ID) - 2;
                                            Node cust = sol.customers[idx];
                                            CustDelivery cd = cust.horizonDeliveryServices[t];

                                            if (!introducedCustomer[idx,t] && cd.quantity > 0) //GlobalUtils.IsEqual(z[idx, t].X, 1.0) && 
                                            {
                                                for (int rr = 0; rr < per.periodRoutes.Count; rr++)
                                                {
                                                    Route rt = per.periodRoutes[rr];
                                                    //CustDelivery cds = cust.horizonDeliveryServices[t];

                                                    if (rt.load + cd.quantity < effectiveCapacity)
                                                    {
                                                        cd.route = rt;
                                                        cd.route.load += cd.quantity;
                                                        cd.route.nodes.Insert(1, cust);
                                                        cd.route.calculateRoutingCost(sol.model);
                                                        cd.route.SetLoadAndCostLists(t, sol.model);
                                                        introducedCustomer[idx, t] = true;
                                                        break;
                                                    }
                                                    else
                                                    {
                                                        if (rr == per.periodRoutes.Count - 1)
                                                        {
                                                            Console.Write("\n never routed cust {0}, period {1} with quantity {2} cap:{3} -->  ", cust.uid,
                                                                t, cd.quantity, effectiveCapacity);
                                                            for (int rrr = 0; rrr < per.periodRoutes.Count; rrr++)
                                                            {
                                                                Route rrt = per.periodRoutes[rrr];
                                                                Console.Write("{0}/", rrt.load);
                                                            }
                                                            GlobalUtils.writeToConsole("");

                                                            Route rtToAdd = findLessViolatedRt(per , cd);
                                                            cd.route = rtToAdd;
                                                            cd.route.load += cd.quantity;
                                                            cd.route.nodes.Insert(1, cust);
                                                            cd.route.calculateRoutingCost(sol.model);
                                                            cd.route.SetLoadAndCostLists(t, sol.model);
                                                            introducedCustomer[idx, t] = true;
                                                            break;
                                                        }

                                                    }
                                                }
                                                //introducedCustomer[idx,t] = addCustomerToRoute(sol, cust, t, effectiveCapacity);
                                            }
                                        }
                                    }
                                }
                            }

                            //Fix solution objective
                            sol.routingCost = Solution.EvaluateRoutingObjectivefromScratch(sol);
                            sol.holdingCost = Solution.EvaluateInventoryObjectivefromScratch(sol);
                            sol.totalUnitProductionCost = Solution.EvaluateUnitProductionObjectivefromScratch(sol);
                            sol.setupProductionCost = Solution.EvaluateSetupProductionObjectivefromScratch(sol);
                            sol.totalObjective = sol.routingCost + sol.holdingCost + sol.totalUnitProductionCost + sol.setupProductionCost;

                            // fix the effective capacity to make sol feasible
                            sol.setVehicleCapacityCoeff(1.20f);
                            sol.findMaxVehicleCapacityCoeff(true);

                            sol.status = sol.TestEverythingFromScratch();

                            if (!Solution.checkSolutionStatus(sol.status))
                            {
                                //
                                //sol = new Solution(back_sol, 0.0);
                                //TSP_GRB_Solver.SolveTSP(sol);
                                //TODO: FIX PROBLEMS THIS SHOULD NOT RETURN
                                //back_sol.SaveToFile("mip_failed_sol");
                                GlobalUtils.writeToConsole("Lefteri ftiakse tis malakies sou sto production Mip with infeasibilities and insertion/removal same days prod days");
                                return false;
                            }

                            //GlobalUtils.writeToConsole("VIOLATIONS AFTER LP {0}", sol.calcVehicleLoadViolations());
                            TSP_GRB_Solver.SolveTSP(sol);
                            MIP.runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemovalSameDayRoutesOriginal(
                                ref sol, maxRouteAdditions, maxRouteRemovals, 0.02, 0, isCont = true);
                            break;
                        }
                    case GRB.Status.INFEASIBLE:
                        {
                             GlobalUtils.writeToConsole("Model is infeasible");
                            // compute and write out IIS
                            model.ComputeIIS();
                            model.Write($"{model.ModelName}.ilp");
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


        // Using the info from a solution pool of the previous restart it produces a relaxation of the problem with the routing approximations.
        // It may be infeasible NOT FUNCTIONAL YET
        public static bool solveProductionRelaxRoutingApprox(
            List<Solution> previousSols, ref Solution sol, bool isCont = false)
        {
            int numberOfPeriods = sol.periods.Count;
            int numberOfCustomers = sol.customers.Count;
            int effectiveCapacity = sol.periods[0].periodRoutes[0].effectiveCapacity;
            int actualCapacity = sol.periods[0].periodRoutes[0].realCapacity;
            int nonVisitConstantRouteCost = 1000;
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
            double[,] insertionCostApprox = new double[numberOfCustomers, numberOfPeriods];
            double[,] avgVisitCost = new double[numberOfCustomers, numberOfPeriods];
            //double[,] rtAddCost = new double[numberOfCustomers, numberOfPeriods];
            //Route[,] rtAddCostRoute = new Route[numberOfCustomers, numberOfPeriods];
            //double[,] rtRemCost = new double[numberOfCustomers, numberOfPeriods];
            //Route[,] rtRemCostRoute = new Route[numberOfCustomers, numberOfPeriods];

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
                    insertionCostApprox[i, t] = 0;

                    for (int ii = 0; ii < sol.model.distMatrix.GetLength(1); ii++)
                    {
                        avgVisitCost[i, t] += sol.model.distMatrix[cust.uid, ii];
                    }
                    avgVisitCost[i, t] = avgVisitCost[i, t] / sol.model.distMatrix.GetLength(1);
                }
            }

            // calculate the possible insertion costs
            foreach (Solution prevSol in previousSols)
            {
                for (int t = 0; t < prevSol.periods.Count; t++)
                {
                    Period pr = prevSol.periods[t];
                    for (int r = 0; r < pr.periodRoutes.Count; r++)
                    {
                        Route rt = pr.periodRoutes[r];
                        for (int ii = 1; ii < rt.nodes.Count - 1; ii++)
                        {
                            Node prev = rt.nodes[ii - 1];
                            Node me = rt.nodes[ii];
                            Node next = rt.nodes[ii + 1];
                            insertionCostApprox[me.uid - 1, t] += prevSol.model.distMatrix[prev.uid, me.uid] + prevSol.model.distMatrix[me.uid, next.uid];
                        }
                    }
                }
                for (int t = 0; t < prevSol.periods.Count; t++)
                {
                    Period pr = prevSol.periods[t];
                    for (int i = 0; i < prevSol.customers.Count; i++)
                    {
                        Node cust = prevSol.customers[i];
                        if (!cust.visitSchedule[t]) //if customer is not serviced then and only then can be added
                        {
                            insertionCostApprox[cust.uid - 1, t] += nonVisitConstantRouteCost; //1.2*avgVisitCost[i,t];
                                //nonVisitConstantRouteCost;
                        }
                    }
                }
            }
            for (int t = 0; t < sol.periods.Count; t++)
            {
                Period pr = sol.periods[t];
                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    insertionCostApprox[cust.uid - 1, t] /= previousSols.Count;
                }
            }


            try
            {
                GRBEnv gurobiEnv = new GRBEnv(); //TODO: I dont kwow what we do with the environment
                GRBModel model = new GRBModel(gurobiEnv);

                // Params
                model.ModelName = "ProductionRelaxRoutingApprox" + sol.ellapsedMs;
                model.Parameters.OutputFlag = (GlobalUtils.suppress_output) ? 0 : 1;
                model.Parameters.TimeLimit = 60;
                model.Parameters.MIPGap = 0.01;
                model.Parameters.Threads = threadLimit;

                // Decision variables
                GRBVar[] I_depot = new GRBVar[numberOfPeriods]; // depot inventory
                GRBVar[] p = new GRBVar[numberOfPeriods]; // depot production
                GRBVar[] y = new GRBVar[numberOfPeriods]; // depot production
                GRBVar[,] I_cust = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer inventory
                GRBVar[,] q = new GRBVar[numberOfCustomers, numberOfPeriods]; // customer delivery quantities
                GRBVar[,] z = new GRBVar[numberOfCustomers, numberOfPeriods]; // binary addition of visits

                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    I_depot[t] = model.AddVar(0.0, sol.depot.stockMaximumLevel, sol.depot.unitHoldingCost, GRB.CONTINUOUS, "I_d^" + t);
                    y[t] = model.AddVar(0.0, 1.0, sol.depot.productionSetupCost, GRB.BINARY, "y^" + t);
                    if (isCont)
                        p[t] = model.AddVar(0.0, UBp[t], 0, GRB.CONTINUOUS, "p_d^" + t); //@greg productionCapacity: is  this the maximum quantity we can produce per dayq
                    else
                        p[t] = model.AddVar(0.0, UBp[t], 0, GRB.INTEGER, "p_d^" + t); //@greg productionCapacity: is  this the maximum quantity we can produce per dayq
                }

                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        z[i, t] = model.AddVar(0.0, 1.0, insertionCostApprox[i, t], GRB.BINARY, "z_" + i + "^" + t);
                        if (isCont)
                            q[i, t] = model.AddVar(0.0, UBq[i, t], 0.0, GRB.CONTINUOUS, "q_" + i + "^" + t);
                        else
                            q[i, t] = model.AddVar(0.0, UBq[i, t], 0.0, GRB.INTEGER, "q_" + i + "^" + t);
                        I_cust[i, t] = model.AddVar(0.0, cust.stockMaximumLevel, cust.unitHoldingCost, GRB.CONTINUOUS, "I_" + i + "^" + t);
                    }
                }

                // Objective
                model.ModelSense = GRB.MINIMIZE;

                // Constraints 

                //Production Constraints
                if (sol.model.input.type == PRP_DATASET_VERSION.BOUDIA_FMT)
                {
                    model.AddConstr(y[0] == 0, "boudia no prod");
                }
                // 5. Total production quantity
                GRBLinExpr lhsp = 0.0;
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    lhsp.AddTerm(1.0, p[t]);
                }
                model.AddConstr(lhsp == totalProdQuantity, "total_prod");

                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    GRBLinExpr lhs = p[t];
                    model.AddConstr(lhs <= sol.depot.productionCapacity*y[t], "prod" + t);
                }
                //@constraint(model, prod_cona[t = indices.T],
                //    p[t] - y[t] * (sum(sum(pd.demands[i][t_1] for t_1 = t:length(indices.T)) for i = indices.V_cus )) <= 0)
                //@constraint(model, prod_conc[t = indices.T],
                //    p[t] - (sum(sum(pd.demands[i][t_1] for t_1 = t:length(indices.T)) -I[i, t - 1] for i = indices.V_cus ) -I[1, t - 1]) <= 0)

                // disallow same schedule
                
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    GRBLinExpr lhs = 0.0;
                    if (sol.depot.open[t])
                    {
                        lhs += 1;
                        lhs.AddTerm(-1.0, y[t]);
                    }
                    else
                    {
                        lhs.AddTerm(1.0, y[t]);
                    }
                }

                // 1. Depot inventory flow 
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    GRBLinExpr lhs = I_depot[t];
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
                        GRBLinExpr lhs = I_cust[i, t];
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
                        GRBLinExpr lhs = q[i, t];
                        if (t == 0)
                            model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - cust.startingInventory, "cust_ml" + i + "^" + t);
                        else
                            model.AddConstr(lhs <= cust.stockMaximumLevel + cust.productRate[t] - I_cust[i, t - 1], "cust_ml" + i + "^" + t);
                    }
                }

                // Total quantity delivered to customer per time period 
                for (int t = 0; t < numberOfPeriods; ++t)
                {
                    GRBLinExpr lhs = 0.0;
                    for (int i = 0; i < sol.customers.Count; i++)
                    {
                        Node cust = sol.customers[i];
                        lhs.AddTerm(1, q[i, t]);
                    }
                    model.AddConstr(lhs <= sol.model.input.availableVehicles*actualCapacity, "cap^" + t);
                }

                for (int i = 0; i < sol.customers.Count; i++)
                {
                    Node cust = sol.customers[i];
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        GRBLinExpr lhs = q[i, t];
                        model.AddConstr(lhs <= z[i,t]*Math.Min(cust.stockMaximumLevel + cust.productRate[t], actualCapacity), "cust_zq" + i + "^" + t);
                    }
                }

                // 4. Customer delivery vehicle capacity
                /*
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
                            model.AddConstr(lhs <= actualCapacity, "cust_vehcap_" + r + "^" + t); //NOTE: effective capacity
                        else
                        {
                            if (allowViol)
                                model.AddConstr(lhs <= effectiveCapacity, "cust_vehcap_" + r + "^" + t); //NOTE: effective capacity
                            else
                                model.AddConstr(lhs <= actualCapacity, "cust_vehcap_" + r + "^" + t); //NOTE: actual capacity
                        }

                    }
                }
                */

                //Extra constraints for handling previous solutions
                for(int ss = 0; ss < previousSols.Count; ss++)
                {
                    Solution prevSol = previousSols[ss];
                    GRBLinExpr lhste = 0.0;
                    for (int t = 0; t < numberOfPeriods; ++t)
                    {
                        if (prevSol.depot.open[t])
                        {
                            lhste += 1;
                            lhste.AddTerm(-1.0, y[t]);
                        }
                        else
                        {
                            lhste.AddTerm(1.0, y[t]);
                        }
                    }
                    model.AddConstr(lhste >= 1, "different_schedule_sol_" + ss);
                }


                // Optimize
                model.Optimize();
                int optimstatus = model.Status;

                switch (model.Status)
                {
                    case GRB.Status.OPTIMAL:
                    case GRB.Status.TIME_LIMIT:
                        {
                            if (existsViolatedQuantity(ref sol, q, p))
                            {
                                model.Dispose();
                                return solveProductionRelaxRoutingApprox(previousSols, ref sol, false);
                            }
                            else
                            {
                                // Update solution with new calculated quantities
                                for (int t = 0; t < numberOfPeriods; t++)
                                {
                                    sol.depot.open[t] = false;
                                    sol.depot.deliveredQuantities[t] = 0;
                                    sol.depot.productionRates[t] = (int)Math.Round(p[t].X);
                                    if (sol.depot.productionRates[t] > 0.5)
                                        sol.depot.open[t] = true;
                                }
                                // update customer deliveries and routes
                                for (int t = 0; t < sol.periods.Count; t++)
                                {
                                    Period per = sol.periods[t];
                                    per.periodRoutes.Clear();
                                    for (int k = 0; k < sol.model.input.availableVehicles; k++)
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

                                        for (int k = 0; k < sol.model.input.availableVehicles; k++)
                                        {
                                            if (GlobalUtils.IsEqual(z[i, t].X, 1.0))
                                            {
                                                cust.visitSchedule[t] = true;
                                                Period per = sol.periods[t];
                                                Route rt = per.periodRoutes[k];
                                                rt.nodes.Insert(1, cust);
                                                cd.route = rt;

                                                cust.deliveredQuantities[t] = (int)Math.Round(q[i,t].X); //update delivery quantity
                                                sol.depot.deliveredQuantities[t] -= cust.deliveredQuantities[t]; // OR +=;
                                                cd.quantity = cust.deliveredQuantities[t];
                                                cd.route.load += cd.quantity;
                                                cd.route.SetLoadAndCostLists(t, sol.model);
                                                break;
                                            }
                                        }
                                    }
                                }

                                // calculate new routes costs
                                for (int t = 0; t < sol.periods.Count; t++)
                                {
                                    Period per = sol.periods[t];

                                    for (int k = 0; k < sol.model.input.availableVehicles; k++)
                                    {
                                        Route rt = per.periodRoutes[k];
                                        rt.totalRoutingCost = 0;
                                        rt.load = 0;

                                        for (int ii = 0; ii < rt.nodes.Count - 1; ii++)
                                        {
                                            Node me = rt.nodes[ii];
                                            Node next = rt.nodes[ii + 1];
                                            rt.totalRoutingCost += sol.model.distMatrix[me.uid, next.uid];
                                            if (ii > 0)
                                                rt.load += me.deliveredQuantities[t];

                                        }
                                    }
                                }

                                // update inventories
                                for (int i = 0; i < sol.customers.Count; i++)
                                {
                                    Node cust = sol.customers[i];
                                    cust.CalculateInventoryLevels();
                                }
                                sol.depot.CalculateInventoryLevels();


                                //Fix solution objective
                                sol.routingCost = Solution.EvaluateRoutingObjectivefromScratch(sol);
                                sol.holdingCost = Solution.EvaluateInventoryObjectivefromScratch(sol);
                                sol.totalUnitProductionCost = Solution.EvaluateUnitProductionObjectivefromScratch(sol);
                                sol.setupProductionCost = Solution.EvaluateSetupProductionObjectivefromScratch(sol);
                                sol.totalObjective = sol.routingCost + sol.holdingCost + sol.totalUnitProductionCost + sol.setupProductionCost;

                                sol.status = sol.TestEverythingFromScratch();

                                if (!Solution.checkSolutionStatus(sol.status))
                                {
                                    GlobalUtils.writeToConsole("Lefteri ftiakse tis malakies sou sto production Production relaxation");
                                    return false;
                                }

                                //GlobalUtils.writeToConsole("VIOLATIONS AFTER LP {0}", sol.calcVehicleLoadViolations());
                                TSP_GRB_Solver.SolveTSP(sol);
                                break;
                            }
                        }
                    case GRB.Status.INFEASIBLE:
                        {
                            return false;
                            // compute and write out IIS
                            //model.ComputeIIS();
                            //model.Write("model.ilp");
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


        //==================================================== Helper ====================================================//
        private static List<Route> findRoutes(List<Route> routesPool, ref Solution sol)
        {
            for (int t = 0; t < sol.periods.Count; t++)
            {
                Period per = sol.periods[t];
                for (int r = 0; r < per.periodRoutes.Count; r++)
                {
                    Route route = new Route(per.periodRoutes[r]);
                    if (route.nodes.Count > 2)
                        routesPool.Add(route);
                }
            }

            return routesPool;
        }

        private static List<Route> appendDirectRoutes(List<Route> routesPool, ref Solution sol)
        {
            for (int i = 0; i < sol.customers.Count; i++)
            {
                Node cust = sol.customers[i];
                Route rt = new Route(sol.periods[0].periodRoutes[0].effectiveCapacity);
                rt.initialize(sol.depot);
                rt.nodes.Insert(1, cust);
                rt.totalRoutingCost = 0;
                for (int ii = 0; ii < sol.customers.Count; ii++)
                {
                    if (i != ii)
                    {
                        Node other = sol.customers[ii];
                        rt.totalRoutingCost += sol.model.distMatrix[other.uid, cust.uid];
                    }
                }
                rt.totalRoutingCost = (int)rt.totalRoutingCost/(sol.customers.Count-1);

                routesPool.Add(rt);
            }
            return routesPool;
        }

        private static double[] calculateRouteCostCustomers(List<Route> routesPool, double[] routeCost, int[,] routeCustomer, ref Solution sol, double discountFactor)
        {
            for (int r = 0; r < routesPool.Count; r++)
            {
                Route rt = routesPool[r];
                routeCost[r] = (int)(discountFactor*rt.totalRoutingCost);
                for (int i = 1; i < rt.nodes.Count-1; i++)
                {
                    Node cust = rt.nodes[i];
                    routeCustomer[Int32.Parse(cust.ID)-2, r] = 1;
                    //else
                       //routeCustomer[i, r] = false;
                }
                GlobalUtils.writeToConsole("route {0}: {1}", rt, routeCost[r]);
            }

            return routeCost;
        }


        private static bool addCustomerToRoute(Solution sol, Node cust, int t, int effectiveCapacity)
        {
            Period per = sol.periods[t];
            for (int r = 0; r < per.periodRoutes.Count; r++)
            {
                Route rt = per.periodRoutes[r];
                CustDelivery cd = cust.horizonDeliveryServices[t];

                if (rt.load + cd.quantity < effectiveCapacity)
                {
                    cd.route = rt;
                    cd.route.load += cd.quantity;
                    cd.route.nodes.Insert(1, cust);
                    cd.route.calculateRoutingCost(sol.model);
                    cd.route.SetLoadAndCostLists(t, sol.model);
                    return true;
                }
            }
            return false;
        }

        private static Route findLessViolatedRt(Period per, CustDelivery cd)
        {
            int rr = 0;
            double totalLoad = int.MaxValue;
            for (int r = 0; r < per.periodRoutes.Count; r++)
            {
                Route rt = per.periodRoutes[r];
                if (rt.load + cd.quantity < totalLoad)
                {
                    totalLoad = rt.load + cd.quantity;
                    rr = r;
                }
            }
            return per.periodRoutes[rr];
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
                        GlobalUtils.writeToConsole("{0} vs {1} {2},{3}", Math.Round(q[i,t].X), q[i,t].X, i, t);
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
                    GlobalUtils.writeToConsole("{0} vs {1}", Math.Round(p[t].X), p[t].X);
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
                    GlobalUtils.writeToConsole("{0} vs {1}", deliveries, cust.totalDemand - cust.startingInventory);
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
                    GlobalUtils.writeToConsole("{0} vs {1}", Math.Round(p[t].X), p[t].X);
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
                    GlobalUtils.writeToConsole("{0} vs {1}", deliveries, cust.totalDemand - cust.startingInventory);
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
