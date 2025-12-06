using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Runtime.InteropServices.ComTypes;
using System.Security.Permissions;
using System.Text;
using System.Threading.Tasks;
using System.Transactions;



namespace PRP
{
    public class RCLElement
    {
        public InsertionPosition pos;
        public Node cust;
        public double cost;
        public bool feasible;
    }
    
    
    public static class ConstructionHeuristic
    {
        static PRP model; //Keep a local copy of the PRP model
        static Solution solution; //Keep a local copy of the WIP solution
        static Random r; //Default Generator
        static int default_seed = 2; //Default random Seed


        
        
        public static Solution GenerateInitialSolution(PRP irpModel, int seed)
        {
            model = irpModel; //Set Model
            r = new Random(seed); //Reset Random Generator

            return GenerateInitialSolution();
        }

        //Use default seed
        public static Solution GenerateInitialSolution(PRP irpModel)
        {
            model = irpModel; //Set Model
            r = new Random(default_seed); //Reset Random Generator

            return GenerateInitialSolution();
        }

        public static Solution GenerateInitialSolution(){

            solution = new Solution(model);
            
            solution.InitializeEmptySolutionDetails(); //Init empty solution

            //Use production/visit info from the input relaxed solution
            if (!BuildInitialSolutionBasedOnRelaxedSolution(ref solution))
            {
                Console.WriteLine("Could not generate a feasible solution based on the relaxed solution");
                throw new Exception("Could not generate a feasible solution based on the relaxed solution");
                return null;
            }

            //Greedily generate a new solution
            //BuildInitialSolutionGreedily(ref solution);
            
            //solution.SaveToFile("Test_lp_solution");
            return solution;
        }
        
        private static List<E> ShuffleList<E>(List<E> inputList)
        {
             List<E> randomList = new List<E>();
        
#if DEBUG
             Random r = new Random(1);
#else
             Random r = new Random();
#endif
             int randomIndex = 0;
             while (inputList.Count > 0)
             {
                  randomIndex = r.Next(0, inputList.Count); //Choose a random object in the list
                  randomList.Add(inputList[randomIndex]); //add it to the new, random list
                  inputList.RemoveAt(randomIndex); //remove to avoid duplicates
             }
        
             return randomList; //return the new random list
        }

        private static bool BuildInitialSolutionBasedOnRelaxedSolution(ref Solution sol)
        {
            if (sol.model.input.type == PRP_DATASET_VERSION.BOUDIA_FMT)
            {
                return BuildInitialInfeasibleSolutionBasedOnRelaxedSolution(ref sol);
                //return BuildInitialSolutionBasedOnRelaxedSolution_withRouting(ref sol); 
            }
                
            return BuildInitialSolutionBasedOnRelaxedSolution_withoutRouting(ref sol);
            
        }

        private static bool BuildInitialSolutionBasedOnRelaxedSolution_withoutRouting(ref Solution sol)
        {
            //Sorted the customer list based on their maximum delivery as set in the production schedule
            List<Node> sortedCustomers = new List<Node>();
            for (int i = 0; i < sol.customers.Count; i++)
                sortedCustomers.Add(sol.customers[i]);

            //Shuffle customer list
            sortedCustomers = ShuffleList(sortedCustomers);
            
            //Sort customers by max demand
            
            /*
            sortedCustomers.Sort(delegate (Node node1, Node node2) 
            {
                int max_q1 = 0;
                int max_q2 = 0;

                for (int i = 0; i < model.horizonDays; i++)
                {
                    max_q1 = (int) Math.Max(model.production_input.customerDeliveries[i, node1.uid], max_q1);
                    max_q2 = (int) Math.Max(model.production_input.customerDeliveries[i, node2.uid], max_q2);
                } 

                return max_q2.CompareTo(max_q1);
            });
            */
            
            List<Node> custList = new List<Node>();
            List<RCLElement> RCL = new List<RCLElement>();

            int RCL_LIST_LENGTH = 50;
            
            for (int pr_i = 0; pr_i < model.horizonDays; pr_i++)
            {
                Period pr = sol.periods[pr_i];
                
                //Clear lists
                custList.Clear();
                
                //Gather customers with delivery quantities onthe first day 

                for (int j = 0; j < sol.customers.Count; j++)
                {
                    Node cust = sol.customers[j];
                    int deliveredQuantity = model.production_input.customerDeliveries[pr_i, cust.uid];

                    if (deliveredQuantity > 0)
                    {
                        custList.Add(cust);
                    }
                }
                
                //For all candidate customers assemble an RCL list with the associated routing cost

                while (custList.Count > 0)
                {
                    RCL.Clear();
                    
                    for (int j = 0; j < custList.Count; j++)
                    {
                        Node cust = custList[j];

                        int deliveredQuantity = model.production_input.customerDeliveries[pr_i, cust.uid];
                        
                        //Iterate in period routes
                        bool tested_empty = false;
                        for (int k = 0; k < pr.periodRoutes.Count; k++)
                        {
                            Route rt = pr.periodRoutes[k];

                            //Check routing feasibility 
                            
                            if (rt.load + deliveredQuantity > rt.effectiveCapacity)
                                continue;
                            
                            if (tested_empty)
                                break;
                            
                            if (rt.nodes.Count == 0)
                                tested_empty = true;
                            
                            for (int ip = 1; ip < rt.nodes.Count; ip++)
                            {
                                Node prev = rt.nodes[ip - 1];
                                Node next = rt.nodes[ip];
                                double rt_cost = sol.model.distMatrix[prev.uid, cust.uid] +
                                                 sol.model.distMatrix[cust.uid, next.uid] -
                                                 sol.model.distMatrix[prev.uid, next.uid];
                                
                                InsertionPosition pos = new InsertionPosition(pr_i , k, ip-1);
                                
                                RCLElement elem = new RCLElement();
                                elem.cost = rt_cost;
                                elem.cust = cust;
                                elem.pos = pos;
                                elem.feasible = (rt.load + deliveredQuantity > rt.realCapacity);

                                //Try to insert the element to the RCL list
                                int insertpos = 0;
                                for (int ii = 0; ii < RCL.Count; ii++)
                                {
                                    if (elem.cost < RCL[ii].cost && (elem.feasible == true || RCL[ii].feasible == elem.feasible))
                                    {
                                        insertpos = ii;
                                        break;
                                    }
                                    insertpos++;
                                }
                                RCL.Insert(insertpos, elem);
                                
                                while(RCL.Count > RCL_LIST_LENGTH)
                                    RCL.RemoveAt(RCL.Count - 1);
                                
                            }
                        }
                    }

                    if (RCL.Count == 0)
                    {
                        Console.WriteLine("exei paixtei malakia. no RCL candidate");
                        continue;
                    }
                        
                
                    //Select one Customer at random out of the RCL list

                    RCLElement selected_elem = RCL[r.Next() % (int) Math.Max(RCL.Count / 5, 1)];
                    //Apply the insertion move
                            
                    //Assign if not assigned
                    int final_deliveredQuantity = model.production_input.customerDeliveries[pr_i, selected_elem.cust.uid];
                    InsertionPosition final_ip = selected_elem.pos;
                    final_ip.deliveryQuantity = final_deliveredQuantity;
                    final_ip.totalHoldingCostChange = CalculateHoldingCostChange(sol.depot, selected_elem.cust, final_deliveredQuantity, pr_i);
                    //final_ip.totalRoutingCostChange = CalculateRoutingCostChangeForInsertingAfterPosition(selected_elem.cust, rt, rt.nodes.Count - 2);
                    final_ip.totalRoutingCostChange = selected_elem.cost;
                    final_ip.totalObjectiveChange = final_ip.totalHoldingCostChange + final_ip.totalRoutingCostChange;  
                                
                    sol.depot.CalculateInventoryLevels();
                    selected_elem.cust.CalculateInventoryLevels();
                                
                    ApplyInsertion(ref sol, selected_elem.cust, final_ip);
                    
                    //Console.WriteLine("Customer {0} inserted on day {1} on route {2}",
                    //    cust.uid, pr_i, routeIndex);
                                
                    sol.depot.CalculateInventoryLevels();
                    selected_elem.cust.CalculateInventoryLevels();
                    
                    
                    //Remove customer from custList
                    custList.Remove(selected_elem.cust);
                    
                }
                
                
                
            }
            
            
            CheckEverything(ref sol, true);

            //SolutionManager.ConstructAndTestEverythingFromScratch(sol, sol.model);
            bool feasible = Solution.checkSolutionStatus(sol.TestEverythingFromScratch()); 
            
            if (!feasible)
                Console.WriteLine(sol.ToString());
            
            //Fix vehicles without checking for vehicle feasibility
            sol.fixVehicles();
            
            return feasible;
        }
        
        private static bool BuildInitialSolutionBasedOnRelaxedSolution_withRouting(ref Solution sol)
        {
            //Sorted the customer list based on their maximum delivery as set in the production schedule
            List<Node> sortedCustomers = new List<Node>();
            for (int i = 0; i < sol.customers.Count; i++)
                sortedCustomers.Add(sol.customers[i]);

            //Shuffle customer list
            //sortedCustomers = ShuffleList(sortedCustomers);
            
            //Sort customers by max demand
            
            /*
            sortedCustomers.Sort(delegate (Node node1, Node node2) 
            {
                int max_q1 = 0;
                int max_q2 = 0;

                for (int i = 0; i < model.horizonDays; i++)
                {
                    max_q1 = (int) Math.Max(model.production_input.customerDeliveries[i, node1.uid], max_q1);
                    max_q2 = (int) Math.Max(model.production_input.customerDeliveries[i, node2.uid], max_q2);
                } 

                return max_q2.CompareTo(max_q1);
            });
            */
            

            for (int i = 0; i < sortedCustomers.Count; i++)
            {
                Node cust = sortedCustomers[i];
                cust.CalculateInventoryLevels();

                //Load default visit schedule from the production input
                bool[] visitSchedule = new bool[model.horizonDays];
                
                for (int pr_i = 0; pr_i < model.horizonDays; pr_i++)
                {
                    Period pr = sol.periods[pr_i];
                    visitSchedule[pr_i] = false;
                    
                    int deliveredQuantity = model.production_input.customerDeliveries[pr_i, cust.uid];
                    
                    if (deliveredQuantity > 0)
                    {
                        //Add delivery to customer
                        visitSchedule[pr_i] = true;
                        bool node_inserted_to_day = false;
                        int routeIndex = 0;
                        
                        //Try to insert customer to route
                        
                        //Load route index from relaxation solution
                        routeIndex = model.production_input.customerRouteAssignment[pr_i, cust.uid];
                        
                        Route rt = pr.periodRoutes[routeIndex];
                            
                        //TODO : Try to find the best possible route assignment and not the first available
                            
                        //Check if route fits the customer
                        if (rt.effectiveCapacity < rt.load + deliveredQuantity)
                        {
                            Console.WriteLine("Unable to insert customer {0} on route {1} on day {2}",
                                cust.uid, routeIndex, pr_i);
                        }

                        
                        //Assign if not assigned
                        InsertionPosition ip = new InsertionPosition(pr_i , routeIndex, rt.nodes.Count - 2);
                        ip.deliveryQuantity = deliveredQuantity;
                        ip.totalHoldingCostChange = CalculateHoldingCostChange(sol.depot, cust, deliveredQuantity, pr_i);
                        ip.totalRoutingCostChange = CalculateRoutingCostChangeForInsertingAfterPosition(cust, rt, rt.nodes.Count - 2);
                        ip.totalObjectiveChange = ip.totalHoldingCostChange + ip.totalRoutingCostChange;  
                            
                        sol.depot.CalculateInventoryLevels();
                        cust.CalculateInventoryLevels();
                            
                        ApplyInsertion(ref sol, cust, ip);
                            
                        //Console.WriteLine("Customer {0} inserted on day {1} on route {2}",
                        //    cust.uid, pr_i, routeIndex);
                            
                        sol.depot.CalculateInventoryLevels();
                        cust.CalculateInventoryLevels();
                        
                    }
                }
                
            }
                
            
            CheckEverything(ref sol, true);

            //SolutionManager.ConstructAndTestEverythingFromScratch(sol, sol.model);
            if (!Solution.checkSolutionStatus(sol.TestEverythingFromScratch()))
            {
                Console.WriteLine(sol.ToString());
                throw new Exception("|MALAKIES");
            }
             
            //Fix vehicles without checking for vehicle feasibility
            sol.fixVehicles();
            
            return true;
        }
        
        
        private static bool BuildInitialInfeasibleSolutionBasedOnRelaxedSolution(ref Solution sol)
        {
            //Immediately increase capacity on routes
            float vehCapacityCoeff = 1.08f;
            sol.setVehicleCapacityCoeff(vehCapacityCoeff);
            int constructIterLimit = 10;
            int constructIterCounter = 0;
            
            //Sorted the customer list based on their maximum delivery as set in the production schedule
            List<Node> sortedCustomers = new List<Node>();
            for (int i = 0; i < sol.customers.Count; i++)
                sortedCustomers.Add(sol.customers[i]);

            //Shuffle customer list
            sortedCustomers = ShuffleList(sortedCustomers);
            
            //Sort customers by max demand
            
            /*
            sortedCustomers.Sort(delegate (Node node1, Node node2) 
            {
                int max_q1 = 0;
                int max_q2 = 0;

                for (int i = 0; i < model.horizonDays; i++)
                {
                    max_q1 = (int) Math.Max(model.production_input.customerDeliveries[i, node1.uid], max_q1);
                    max_q2 = (int) Math.Max(model.production_input.customerDeliveries[i, node2.uid], max_q2);
                } 

                return max_q2.CompareTo(max_q1);
            });
            */
            
            List<Node> custList = new List<Node>();
            List<RCLElement> RCL = new List<RCLElement>();

            int RCL_LIST_LENGTH = 50;
            
            for (int pr_i = 0; pr_i < model.horizonDays; pr_i++)
            {
                Period pr = sol.periods[pr_i];
                
                //Clear lists
                custList.Clear();
                
                //Gather customers with delivery quantities onthe first day 

                for (int j = 0; j < sol.customers.Count; j++)
                {
                    Node cust = sol.customers[j];
                    int deliveredQuantity = model.production_input.customerDeliveries[pr_i, cust.uid];
                    //Console.WriteLine("Customer {0} should receive {1} in period {2}", cust.uid, deliveredQuantity, pr_i);
                    if (deliveredQuantity > 0)
                    {
                        custList.Add(cust);
                    }
                }
                
                //For all candidate customers assemble an RCL list with the associated routing cost

                while (custList.Count > 0)
                {
                    RCL.Clear();
                    
                    for (int j = 0; j < custList.Count; j++)
                    {
                        Node cust = custList[j];

                        int deliveredQuantity = model.production_input.customerDeliveries[pr_i, cust.uid];
                        
                        //Iterate in period routes
                        bool tested_empty = false;
                        for (int k = 0; k < pr.periodRoutes.Count; k++)
                        {
                            Route rt = pr.periodRoutes[k];

                            //Check routing feasibility 
                            if (pr_i == 0)
                            {
                                if (rt.load + deliveredQuantity > rt.realCapacity)
                                    continue;
                            } else
                            {
                                if (rt.load + deliveredQuantity > rt.effectiveCapacity)
                                    continue;
                            }
                            
                            
                            if (tested_empty)
                                break;
                            
                            if (rt.nodes.Count == 0)
                                tested_empty = true;
                            
                            for (int ip = 1; ip < rt.nodes.Count; ip++)
                            {
                                Node prev = rt.nodes[ip - 1];
                                Node next = rt.nodes[ip];
                                double rt_cost = sol.model.distMatrix[prev.uid, cust.uid] +
                                                 sol.model.distMatrix[cust.uid, next.uid] -
                                                 sol.model.distMatrix[prev.uid, next.uid];
                                
                                InsertionPosition pos = new InsertionPosition(pr_i , k, ip-1);
                                
                                RCLElement elem = new RCLElement();
                                elem.cost = rt_cost;
                                elem.cust = cust;
                                elem.pos = pos;
                                elem.feasible = (rt.load + deliveredQuantity < rt.realCapacity);
                                
                                //Try to insert the element to the RCL list
                                int insertpos = 0;
                                for (int ii = 0; ii < RCL.Count; ii++)
                                {
                                    if (elem.cost < RCL[ii].cost && (elem.feasible == true || RCL[ii].feasible == elem.feasible))
                                    {
                                        insertpos = ii;
                                        break;
                                    }
                                    insertpos++;
                                }
                                RCL.Insert(insertpos, elem);
                                
                                while(RCL.Count > RCL_LIST_LENGTH)
                                    RCL.RemoveAt(RCL.Count - 1);
                                
                            }
                        }
                    }

                    if (RCL.Count == 0)
                    {
                        if (constructIterCounter == constructIterLimit)
                            throw new Exception("exei paixtei malakia. NO RCL CANDIDATE");
                        Console.WriteLine("Unable to insert {0} customers. Increasing infeasibility coeff", custList.Count);
                        vehCapacityCoeff += 0.02f;
                        sol.setVehicleCapacityCoeff(vehCapacityCoeff);
                        constructIterCounter++;
                        continue;
                    }
                        
                    //Select one Customer at random out of the RCL list

                    RCLElement selected_elem = RCL[r.Next() % (int) Math.Max(RCL.Count / 5, 1)];
                    //Apply the insertion move
                            
                    //Assign if not assigned
                    int final_deliveredQuantity = model.production_input.customerDeliveries[pr_i, selected_elem.cust.uid];
                    InsertionPosition final_ip = selected_elem.pos;
                    final_ip.deliveryQuantity = final_deliveredQuantity;
                    final_ip.totalHoldingCostChange = CalculateHoldingCostChange(sol.depot, selected_elem.cust, final_deliveredQuantity, pr_i);
                    //final_ip.totalRoutingCostChange = CalculateRoutingCostChangeForInsertingAfterPosition(selected_elem.cust, rt, rt.nodes.Count - 2);
                    final_ip.totalRoutingCostChange = selected_elem.cost;
                    final_ip.totalObjectiveChange = final_ip.totalHoldingCostChange + final_ip.totalRoutingCostChange;  
                                
                    sol.depot.CalculateInventoryLevels();
                    selected_elem.cust.CalculateInventoryLevels();
                                
                    ApplyInsertion(ref sol, selected_elem.cust, final_ip);
                    
                    //Console.WriteLine("Customer {0} inserted on day {1} on route {2}",
                    //    selected_elem.cust.uid, pr_i, selected_elem.pos.routeindex);
                                
                    sol.depot.CalculateInventoryLevels();
                    selected_elem.cust.CalculateInventoryLevels();
                    
                    
                    //Remove customer from custList
                    custList.Remove(selected_elem.cust);
                    
                }
                
            }

            sol.status = sol.TestEverythingFromScratch();


            //CALL MIP TO TRY TO RESOLVE INFEASIBILITIES
            bool lpstatus = MIP.runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemovalSameDayRoutesOriginal(ref sol, 20, 20, 0.02, 0);

            if (!lpstatus)
                Console.WriteLine("LP FAILED DURING SOLUTION CONSTRUCTION");

            CheckEverything(ref sol, true);
            
            //SolutionManager.ConstructAndTestEverythingFromScratch(sol, sol.model);
            sol.status = sol.TestEverythingFromScratch();
            bool feasible = Solution.checkSolutionStatus(sol.status);

            if (!feasible)
            {
                Console.WriteLine(sol.ToString());
                throw new Exception("Solution Not feasible");
            }


            //BOUDIA CHECK: Check for infeasibilities in the first period
            if (sol.model.input.type == PRP_DATASET_VERSION.BOUDIA_FMT)
            {
                foreach (Route rt in sol.periods[0].periodRoutes)
                {
                    if (rt.load > rt.realCapacity)
                        throw new Exception("Period 0 Route violation. This should not happen");
                }
            }
            
            //sol.resetVehicleCapacityCoeff();
            //Fix vehicles without checking for vehicle feasibility
            sol.fixVehicles();

            //Bound the infeasibility coefficient
            sol.findMaxVehicleCapacityCoeff(true);
            
            return feasible;
        }
        
        private static void BuildInitialSolutionGreedily(ref Solution sol)
        {
            for (int day = 0; day < model.horizonDays; day++)
            {
                PushCustomersIntoPeriodRoutes(ref sol, day);
            }
            
            CheckEverything(ref sol, true);
        }

        private static void PushCustomersIntoPeriodRoutes(ref Solution sol, int day)
        {
            Period p = solution.periods[day];
            List<Node> priorityQueue = CreatePriorityQueueForDay(ref sol, day);

            int i = 0;
            while (i < priorityQueue.Count)
            {
                Node toBeInserted = priorityQueue[i];
                bool fix_depot_inventory = false;
                double aux_stock = 0;
                
                //ConditionstoBeinserted
                InsertionPosition ip = new InsertionPosition(-1, -1, -1);
                int search_status = FindBestFeasibleInsertionPointInDay(ref sol, toBeInserted, ip, day);
                switch (search_status)
                {
                    case 0:
                    {
                        ApplyInsertion(ref sol, toBeInserted, ip);
                        GlobalUtils.writeToConsole("Inserting customer " + toBeInserted.uid + " in day " + day + " on route " + ip.routeindex);
                        i++; //Proceed to next customer
                        break;
                    }
                    case 1:
                    {
                        Console.WriteLine("Cannot host customer " + toBeInserted.uid + " in day " + day + ". Maxed out vehicles. Proceed to next day");
                        return;
                    }
                    case 2:
                    {
                        Console.WriteLine("Cannot host customer " + toBeInserted.uid + " in day " + day+ ". Insufficient Depot Inventory. Fixing...");
                        fix_depot_inventory = true;
                        aux_stock = toBeInserted.stockMaximumLevel - toBeInserted.startDayInventory[day];
                        //Report Depot and Customers
                        //sol.depot.ReportInventory();
                        //toBeInserted.ReportInventory();
                        break;
                    }
                }
                
                if (fix_depot_inventory)
                {
                    //Increase depot start inventory to serve all customers
                    sol.depot.CalculateInventoryLevels();
                    //sol.depot.ReportInventory();
                    
                    //Recalculate holding cost of the depot
                    double oldDepotHoldingCost = sol.depot.totalHoldingCost;
                    sol.depot.totalHoldingCost = sol.depot.CalculateHoldingCost(0);
                    double holdingCostChange = sol.depot.totalHoldingCost - oldDepotHoldingCost;  
                    //Update solution objective
                    sol.holdingCost += holdingCostChange;
                    sol.totalObjective += holdingCostChange;
                    
                    CheckEverything(ref sol, false);
                }
                
            }
            
        }

        private static void ApplyInsertion(ref Solution sol, Node toBeInserted, InsertionPosition ip)
        {
            Period p = solution.periods[ip.day];
            Route rt = p.periodRoutes[ip.routeindex];

            //Report toBeInserted node and depot before the move
            //sol.depot.ReportInventory();
            //toBeInserted.ReportInventory();
            //toBeInserted.ReportStockoutDays();
            
            
            //Arrange Route Presence
            rt.nodes.Insert(ip.positionIndex + 1, toBeInserted);
            //Arrange delivery Quantities
            rt.quantDelivered.Insert(ip.positionIndex + 1, ip.deliveryQuantity);
            
            //TODO loadTillMe rt.loadTillMe[].Insert(ip.positionIndex + 1, ip.deliveryQuantity);
            rt.load += ip.deliveryQuantity;
            rt.totalRoutingCost += ip.totalRoutingCostChange;

            double customerHoldingCostChange = CalculateHoldingCostChangeDeliveryAtDay_Node(toBeInserted, ip.deliveryQuantity, ip.day);
            toBeInserted.totalHoldingCost += customerHoldingCostChange;

            double depotHoldingCostChange = CalculateHoldingCostChangeDeliveryAtDay_Node(sol.depot, -ip.deliveryQuantity, ip.day);
            sol.depot.totalHoldingCost += depotHoldingCostChange;

            toBeInserted.deliveredQuantities[ip.day] = ip.deliveryQuantity;
            toBeInserted.horizonDeliveryServices[ip.day] = new CustDelivery(ip.deliveryQuantity, rt);
            toBeInserted.visitSchedule[ip.day] = true;
            sol.depot.deliveredQuantities[ip.day] -= ip.deliveryQuantity;

            //Reconstruct LoadTillMe Array
            rt.SetLoadAndCostLists(ip.day, sol.model);


            toBeInserted.CalculateVisitDays(toBeInserted.visitSchedule);
            
            
            ArrangeNodeStockInfoDeliveryAtDay(sol.depot, -ip.deliveryQuantity, ip.day);
            ArrangeNodeStockInfoDeliveryAtDay(toBeInserted, ip.deliveryQuantity, ip.day);
            
            solution.routingCost += ip.totalRoutingCostChange;
            solution.holdingCost += ip.totalHoldingCostChange;
            solution.totalObjective += ip.totalObjectiveChange;
    
            
            //sol.depot.ReportInventory();
            //toBeInserted.ReportInventory();
            //toBeInserted.ReportStockoutDays();
            
            CheckEverything(ref sol, false);
        }

        private static void CheckEverything(ref Solution sol, bool completeSolution)
        {
            double rtObjective = Solution.EvaluateRoutingObjectivefromScratch(sol);
            double inventoryObjective = Solution.EvaluateInventoryObjectivefromScratch(sol);

            if (completeSolution)
            {
                if (!GlobalUtils.IsEqual(rtObjective, sol.routingCost))
                    Console.WriteLine("Objective cost mismatch");
                
                if (!GlobalUtils.IsEqual(inventoryObjective, sol.holdingCost))
                    Console.WriteLine("Objective cost mismatch");
            }
            
            if (!GlobalUtils.IsEqual(solution.totalObjective - sol.totalUnitProductionCost - sol.setupProductionCost, rtObjective + inventoryObjective ))
            {
                Console.WriteLine("Check Total Objective. Cached " + solution.totalObjective + " vs Calculated " + (rtObjective + inventoryObjective));
            }
        }

        
        private static void ArrangeNodeStockInfoDeliveryAtDay(Node node, int quant, int day)
        {
            node.endDayInventory[day] += quant;
            for (int i = day + 1; i < model.horizonDays; i++)
            {
                node.endDayInventory[i] += quant;
                node.startDayInventory[i] += quant;
            }
            
            node.ArrangeDaysToStockOut();
        }

        private static int FindBestFeasibleInsertionPointInDay(ref Solution sol, Node node, InsertionPosition ip, int dayIndex)
        {
            //This method returns values 0: if a move has been found, 1: if its infeasible due to limited vehicle capacity
            //2, if its infeasible due to limited depot inventory
            int search_status = 1;
            
            bool feasibleInsertion = false;
            Period period = solution.periods[dayIndex];
            double lowestInsertionCost = double.MaxValue;

            //Feasibility for entire period -> depot can provide
            //Calculate delivery Quantity based on the OU model
            int deliveryQuantity = node.stockMaximumLevel - node.startDayInventory[dayIndex];
            
            //Calculate delivery Quantities based on the SAW model
            
            //Make sure that the depot has enough quantity to provide
            if (sol.depot.endDayInventory[dayIndex] < deliveryQuantity)
                return 2;
            
            double holdingCostChange = CalculateHoldingCostChange(sol.depot, node, deliveryQuantity, dayIndex);

            for (int i = 0; i < period.periodRoutes.Count; i++)
            {
                Route rt = period.periodRoutes[i];
                
                if (!FeasibilityInRoute(node, rt, dayIndex, deliveryQuantity))
                    continue;
                
                for (int j = 0; j < rt.nodes.Count -1; j++)
                {
                    double routingCostChange = CalculateRoutingCostChangeForInsertingAfterPosition(node, rt, j);
                    RecordBestMove(ref lowestInsertionCost, deliveryQuantity, holdingCostChange, routingCostChange, ip, dayIndex, i, j);
                    search_status = 0;
                }
            }

            return search_status;
        }

        private static void RecordBestMove(ref double lowestInsertionCost, int delQuant, double holdCostChange, double rtCostChange, 
            InsertionPosition ip, int dayIndex, int rtIndex, int nodeIndex)
        {
            double totalChange = holdCostChange + rtCostChange;

            //if (lowestInsertionCost - GlobalUtils.doublePrecision > totalChange)
            if (totalChange - lowestInsertionCost < GlobalUtils.doublePrecision)
            {
                lowestInsertionCost = totalChange;
                ip.day = dayIndex;
                ip.routeindex = rtIndex;
                ip.positionIndex = nodeIndex;
                ip.totalObjectiveChange = totalChange;
                ip.totalHoldingCostChange = holdCostChange;
                ip.totalRoutingCostChange = rtCostChange;
                ip.deliveryQuantity = delQuant;
            }
        }

        private static bool FeasibilityInRoute(Node node, Route rt, int dayIndex, double deliveryQuantity)
        {
            if (deliveryQuantity + rt.load > rt.effectiveCapacity)
            {
                return false;
            }
            return true;
        }

        private static List<Node> CreatePriorityQueueForDay(ref Solution sol, int day)
        {
            List<Node> priority = sol.customers.Where(x => x.endDayInventory.Last() < 0).
                OrderBy(x => x.daysTillStockOut[day]).ThenByDescending(x => x.stockMaximumLevel).ToList();

            priority = RandomizeTopPercentage(0.25, priority);

            return priority;

        }

        private static List<Node> RandomizeTopPercentage(double v, List<Node> priority)
        {
            if (priority.Count == 0)
            {
                return priority;
            }

            List<Node> copy = new List<Node>(priority);
            List<Node> randomized = new List<Node>();

            int indexOflastInRandomizedRange = (int) (copy.Count * v); 

            for (int i = 0; i <= indexOflastInRandomizedRange; i++)
            {
                int index = r.Next(0, indexOflastInRandomizedRange + 1 - i);
                randomized.Add(copy[index]);
                copy.RemoveAt(index);
            }
            randomized.AddRange(copy);
            return randomized;
        }

        /*
        private static void BuildInitialSolution()
        {
            bool solutionNotFound = true;
            int k = 0;

            while (solutionNotFound == true && k < model.customers.Count -1)
            {
                //new Solution
                bool isSolutionFeasible = true;

                if (k == 1)
                {
                    k++;
                }

                //Iteratively Construct all day schedule
                for (int p = 0; p < model.horizonDays; p++)
                {
                    List<Node> compulsoryRetailers = RetailersMustBeServed(p);
                    //Extend the list according to k

                    bool isFeasible = true;
                    Solution s = runClarkeAndWright(ref isFeasible, compulsoryRetailers, p);

                    if (isFeasible == false)
                    {
                        isSolutionFeasible = false;
                        break;
                    }
                }

                if (isSolutionFeasible)
                {
                    solutionNotFound = true;
                    //Arrange solution characteristics
                }
                else
                {

                }
            }
        }
        */

        #region HoldingCostCalculationMethods

        //Calculates the holding cost change per node
        public static double CalculateHoldingCostChangeDeliveryAtDay_Node(Node n, double deliveryQuantity, int dayIndex)
        {
            double holdCostBefore = 0;
            double holdCostAfter = 0;

            holdCostBefore = n.CalculateHoldingCost(0,  dayIndex);
            holdCostAfter = n.CalculateHoldingCost(deliveryQuantity,  dayIndex);

            return holdCostAfter - holdCostBefore;
        }

        //Combines the change on the holding cost at the node and the depot combined
        public static double CalculateHoldingCostChange(Node depot, Node n, double deliveryQuantity, int dayIndex)
        {
            double holdCostChangeDepot = CalculateHoldingCostChangeDeliveryAtDay_Node(depot, -deliveryQuantity, dayIndex);
            double holdCostChangeNode = CalculateHoldingCostChangeDeliveryAtDay_Node(n, deliveryQuantity, dayIndex);

            return holdCostChangeDepot + holdCostChangeNode;
        }

        //Ok!
        public static double CalculateRoutingCostChangeForInsertingAfterPosition(Node node, Route rt, int j)
        {
            Node prev = rt.nodes[j];
            Node succ = rt.nodes[j + 1];

            double costAdded = model.distMatrix[prev.uid, node.uid] + model.distMatrix[node.uid, succ.uid];
            double costRemoved = model.distMatrix[prev.uid, succ.uid];

            return costAdded - costRemoved;
        }

#endregion


        
    }
}
