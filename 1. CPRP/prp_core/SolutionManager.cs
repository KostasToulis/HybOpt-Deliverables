using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;

namespace PRP
{
    public static class SolutionManager
    {
        public static Solution CloneSolution(Solution source, PRP model, double constantFirstPeriodInvCost)
        {
            Solution copy = new Solution(model);
            for (int p = 0; p < source.periods.Count; p++)
            {
                Period per = source.periods[p];
                copy.periods[p].totalOutboundProductFlow = per.totalOutboundProductFlow;

                for (int r = 0; r < per.periodRoutes.Count; r++)
                {
                    Route rt = per.periodRoutes[r];
                    
                    for (int n = 0; n < rt.nodes.Count; n++)
                    {
                        Node node = rt.nodes[n];
                        copy.periods[p].periodRoutes[r].nodes.Add(node);
                    }
                    copy.periods[p].periodRoutes[r].totalRoutingCost = rt.totalRoutingCost;
                    copy.periods[p].periodRoutes[r].load = rt.load;
                }
            }
            copy.totalObjective = source.totalObjective;
            copy.holdingCost = source.holdingCost;
            copy.routingCost = source.routingCost;
            copy.totalObjectiveIncFirstPeriodInventory = source.totalObjective + constantFirstPeriodInvCost;
            copy.ellapsedMs = source.ellapsedMs;
            copy.totalTimeRepairing = source.totalTimeRepairing;
            copy.restartElapsedMs = source.restartElapsedMs;
            copy.totalRepairs = source.totalRepairs;
            copy.feasibleSpaceIters = source.feasibleSpaceIters;
            copy.infeasibleSpaceIters = source.infeasibleSpaceIters;

            return copy;
        }

        internal static void SaveSolutionFile(Solution sol, PRP model)
        {
            StreamWriter SW;
            
            if (File.Exists("Sol_" + model.instanceName) == false)
            {
                SW = File.CreateText("Sol_" + model.instanceName);
            }
            else
            {
                File.Delete("Sol_" + model.instanceName);
                SW = File.CreateText("Sol_" + model.instanceName);
            }

            SW.WriteLine("Total_Objective: " + sol.totalObjective);
            SW.WriteLine("Total_Routing_Objective: " + sol.routingCost);
            SW.WriteLine("Total_Inventory_Objective: " + sol.holdingCost);

            SW.WriteLine();

            SW.WriteLine("Total periods: " + sol.periods.Count);
            SW.WriteLine("Total routes per period: " + sol.periods[0].periodRoutes.Count);

            SW.WriteLine();

            for (int i = 0; i < sol.periods.Count; i++)
            {
                SW.WriteLine("Period_" + i +  "_Routes:");
                Period per = sol.periods[i];

                for (int r  = 0; r < per.periodRoutes.Count; r++)
                {
                    SW.WriteLine("-Route_" + r +":");
                    Route rt = per.periodRoutes[r];
                    for (int n = 0; n < rt.nodes.Count; n++)
                    {
                        Node nd = rt.nodes[n];
                        SW.Write(nd.ID + " ");
                    }
                    SW.WriteLine();
                }
            }
            SW.Close();  
        }

        internal static void SaveAnalyticalSolutionFile(Solution sol)
        {
            PRP model = sol.model; //Keep things as is for now
            
            //ConstructAndTestEverythingFromScratchGivenSolution(sol, model);
            sol.TestEverythingFromScratch();

            sol.SaveToFile("Sol_" + model.instanceName);
            
        }

        internal static void SaveAnalyticalMIPSolutionFile(Solution sol, Dictionary<string, double> dict)
        {
            PRP model = sol.model; //Keep things as is for now

            //ConstructAndTestEverythingFromScratchGivenSolution(sol, model);
            INFEASIBILITY_STATUS solStatus;
            if (MathematicalProgramming.exactParams.periodicity == "periodic")
            {
                solStatus = sol.TestEverythingFromScratchPeriodic();
            } 
            else if (MathematicalProgramming.exactParams.periodicity == "cyclic")
            {
                solStatus = sol.TestEverythingFromScratchCyclic();
            }
            else
            {
                solStatus = sol.TestEverythingFromScratch();
            }

            if (solStatus == INFEASIBILITY_STATUS.FEASIBLE)
            {
                dict["Solution validity"] = 1.0;
            } else
            {
                dict["Solution validity"] = 0.0;
            }

            //sol.SaveToFile("MIPSol_" + model.instanceName);
            String filename = "";
            String modelInstanceName = model.instanceName.Remove(0,7);
            modelInstanceName = modelInstanceName.Substring(0, modelInstanceName.Length - 4 - 2*model.input.horizonDays)+ ".txt";
            if (MathematicalProgramming.exactParams.periodicity == "periodic")
            {
                filename = "PPRP_MIPSol_" + modelInstanceName;
                if (MathematicalProgramming.exactParams.limitedVehicles)
                {
                    filename = "PPRPLim_MIPSol_" + modelInstanceName;
                }
            } 
            else if (MathematicalProgramming.exactParams.periodicity == "cyclic")
            {
                filename = "CPRP_MIPSol_" + modelInstanceName;
                if (MathematicalProgramming.exactParams.limitedVehicles)
                {
                    filename = "CPRPLim_MIPSol_" + modelInstanceName;
                }
            }
            else if (MathematicalProgramming.exactParams.periodicity == "basic")
            {
                filename = "PRP_MIPSol_" + modelInstanceName;
                if (MathematicalProgramming.exactParams.limitedVehicles)
                {
                    filename = "PRPLim_MIPSol_" + modelInstanceName;
                }
            }

            SaveToFileMIP(sol, filename, dict);
        }

        internal static void SaveToFileMIP(Solution sol, string filename, Dictionary<string, double> dict)
        {
            string output = "";
            output += "NumVars: " + dict["NumVars"] + "\n";
            output += "NumConstrs: " + dict["NumConstrs"] + "\n";
            output += "NumBinVars: " + dict["NumBinVars"] + "\n";
            output += "Status: " + dict["Status"] + "\n";
            output += "Solution validity: " + dict["Solution validity"] + "\n";
            output += "\n";

            output += "NodeCount: " + dict["NodeCount"] + "\n";
            output += "Objective: " + dict["ObjVal"] + "\n";
            output += "ObjBound: " + dict["ObjBound"] + "\n";
            output += "MIPGap: " + dict["MIPGap"] + "\n";
            output += "Runtime: " + dict["Runtime"] + "\n";
            output += "\n";

            output += "Total_Objective: " + sol.totalObjective + "(" + dict["totalObjectiveFromMIP"] + ")" + "\n";
            output += "Total_Routing_Objective: " + sol.routingCost + "(" + dict["routingCostFromMIP"] + ")" + "\n";
            output += "Total_Inventory_Objective: " + sol.holdingCost + "(" + dict["holdingCostFromMIP"] + ")" + "\n";
            output += "Total_Unit_Production_Objective: " + sol.totalUnitProductionCost + "(" + dict["totalUnitProductionCostFromMIP"] + ")" + "\n";
            output += "Total_Production_Setup_Objective: " + sol.setupProductionCost + "(" + dict["setupProductionCostFromMIP"] + ")" + "\n";
            //output += "Total_Inventory_Objective_(incl.firstDayInvCost): " + sol.totalObjectiveIncFirstPeriodInventory + "\n";
            output += "\n";

            output += "Total periods: " + sol.periods.Count + "\n";
            output += "Total routes per period: " + sol.periods[0].periodRoutes.Count + "\n";
            output += "\n";

            for (int i = 0; i < sol.periods.Count; i++)
            {
                output += "Period_" + (i + 1) + "_Routes:" + "\n";
                Period per = sol.periods[i];

                bool empty = false;
                for (int r = 0; r < per.periodRoutes.Count; r++)
                {
                    if (!empty)
                    {
                        output += "-Route_" + r + " : ";
                        Route rt = per.periodRoutes[r];
                        for (int n = 0; n < rt.nodes.Count; n++)
                        {
                            Node nd = rt.nodes[n];
                            output += nd.ID + " ";
                        }
                        output += "\n";
                        if (rt.nodes.Count < 2)
                        {
                            empty = true;
                        }
                    }
                }

                output += "\n";
            }

            output += "\n\n";

            //Export production quantities
            for (int i = 0; i < sol.periods.Count; i++)
            {
                output += "Period_" + (i + 1) + "_production_quantity:\n";
                output += sol.depot.productionRates[i] + "\n";
            }

            output += "\n\n";

            //Export Delivered quantities
            for (int i = 0; i < sol.periods.Count; i++)
            {
                output += "Period_" + (i + 1) + "_delivered_quantity:\n";
                string node_s = "";
                string node_q_s = "";

                for (int j = 1; j < sol.nodes.Count; j++)
                {
                    Node n = sol.nodes[j];
                    if (n.deliveredQuantities[i] > 0)
                    {
                        node_s += (j + 1) + " ";
                        node_q_s += n.deliveredQuantities[i] + " ";
                    }
                }

                //Set default values if there are no deliveries
                if (node_s == "" || node_q_s == "")
                {
                    node_s = "-1";
                    node_q_s = "0.0";
                }
                output += node_s + "\n" + node_q_s + "\n\n";
            }

            // For cyclic this is the initial inventories of the depot and the customers
            output += "Initial Inventories (depot, customers):\n";
            for (int i = 0; i < sol.nodes.Count; i++)
            {
                Node n = sol.nodes[i];

                output += n.startingInventory + " ";
            }
            output += "\n\n";


            StreamWriter SW;

            if (File.Exists(filename) == false)
            {
                SW = File.CreateText(filename);
            }
            else
            {
                File.Delete(filename);
                SW = File.CreateText(filename);
            }

            SW.Write(output);

            SW.Close();
        }

        public static Solution ImportCustomSolution(PRP model, String filename)
        {
            Solution sol = new Solution(model);
            sol.InitializeEmptySolutionDetails();

            FileInfo src = new FileInfo(filename);
            TextReader reader = src.OpenText();
            String str;
            char[] seperator = new char[2] { ' ', '\t' };
            List<String> data;


            //Skip objectives, they will be re-evaluated later
            //Also skip periods and routes per period (solution should have been properly initialized
            for (int i = 0; i < 10; i++)
                reader.ReadLine();

            //Parse routes
            for (int i = 0; i < sol.periods.Count; i++)
            {
                Period pr = sol.periods[i];
                reader.ReadLine();
                while (true)
                {
                    str = reader.ReadLine();
                    if (str == "")
                        break;
                    List<String> routeData = GlobalUtils.SeperateStringIntoSubstrings(':', str);
                    //Add route
                    int route_id = pr.vehicleNum;
                    pr.appendRoute(model, sol.depot);
                    Route rt = pr.periodRoutes[route_id];

                    rt.effectiveCapacity = (int)(1.00 * rt.realCapacity);

                    List<String> nodeData = GlobalUtils.SeperateStringIntoSubstrings(' ', routeData[1]);
                    //Add nodes    
                    for (int k = 1; k < nodeData.Count - 1; k++)
                    {
                        int cust_id = Int32.Parse(nodeData[k]) - 1;
                        Node cust = sol.nodes[cust_id];
                        cust.visitSchedule[i] = true;
                        cust.horizonDeliveryServices[i] = new CustDelivery(0, rt);
                        rt.nodes.Insert(rt.nodes.Count - 1, cust);
                    }

                    //Update route cost
                    for (int k = 1; k < rt.nodes.Count; k++)
                        rt.totalRoutingCost += model.distMatrix[rt.nodes[k - 1].uid, rt.nodes[k].uid];
                }
            }

            reader.ReadLine();
            reader.ReadLine();

            //Parse Production
            for (int i = 0; i < model.horizonDays; i++)
            {
                reader.ReadLine(); //Skip first line
                str = reader.ReadLine();
                data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
                int q = (int)Math.Round(double.Parse(data.Last()));
                sol.depot.productionRates[i] = 0;
                sol.depot.open[i] = false;

                if (q > 0.0)
                    sol.depot.open[i] = true;
                sol.depot.productionRates[i] = q;
            }

            reader.ReadLine();
            reader.ReadLine();

            for (int i = 0; i < model.horizonDays; i++)
            {
                //Parse Delivered quantities
                reader.ReadLine();
                str = reader.ReadLine();
                List<String> customerData = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
                str = reader.ReadLine();
                data = GlobalUtils.SeperateStringIntoSubstrings(seperator, str);
                str = reader.ReadLine(); //Skip last empty line

                for (int j = 0; j < data.Count; j++)
                {
                    int custId = int.Parse(customerData[j]) - 1; //Load ID
                    if (custId > 0)
                    {
                        Node cust = sol.nodes[custId];
                        int q = (int)Math.Round(double.Parse(data[j])); //Load quantity
                        cust.deliveredQuantities[i] = q;
                        cust.horizonDeliveryServices[i].quantity = q;
                        cust.horizonDeliveryServices[i].route.load += q;
                        sol.depot.deliveredQuantities[i] -= q;
                    }
                }
            }

            //Recalculate inventory levels for the depot and the customers
            sol.depot.CalculateInventoryLevels();

            for (int i = 0; i < sol.customers.Count; i++)
                sol.customers[i].CalculateInventoryLevels();

            //Recalculate loads for all routes
            for (int i = 0; i < model.horizonDays; i++)
            {
                Period pr = sol.periods[i];
                for (int j = 0; j < pr.periodRoutes.Count; j++)
                    pr.periodRoutes[j].SetLoadAndCostLists(i, model);
            }

            //Re-evaluate objectives
            sol.routingCost = Solution.EvaluateRoutingObjectivefromScratch(sol);
            sol.holdingCost = Solution.EvaluateInventoryObjectivefromScratch(sol);
            sol.totalUnitProductionCost = Solution.EvaluateUnitProductionObjectivefromScratch(sol);
            sol.setupProductionCost = Solution.EvaluateSetupProductionObjectivefromScratch(sol);

            sol.totalObjective = sol.routingCost + sol.holdingCost + sol.totalUnitProductionCost +
                                        sol.setupProductionCost;


            if (!Solution.checkSolutionStatus(sol.TestEverythingFromScratch()))
            {
                GlobalUtils.writeToConsole("Solution failed to import properly");
            }


            return sol;        
        }

        // The functions are not used for the time being. Solution's testEverythingFromScratch() is used instead.
        /*
        public static void ConstructAndTestEverythingFromScratch(Solution sol, PRP model)
        {
            //Reinitialize delivered quantities
            for (int i = 0; i < sol.customers.Count; i++)
            {
                for (int p = 0; p < model.horizonDays; p++)
                {
                    sol.customers[i].deliveredQuantities[p] = 0;
                }
            }
            
            //Calculate Production Cost (unit and setup) from Scratch
            double unitProductionCost = 0;
            double setupProductionCost = 0;
            for (int p = 0; p < sol.periods.Count; p++)
            {
                unitProductionCost += sol.depot.productionRates[p] * sol.depot.unitProductionCost;
                if (sol.depot.open[p])
                    setupProductionCost += sol.depot.productionSetupCost;
            }
            if (!GlobalUtils.IsEqual(sol.setupProductionCost, setupProductionCost))
            {
                GlobalUtils.writeToConsole(String.Format("Mismatch of production setup cost: Solution {0} diff. calculated from scratch {1}", sol.setupProductionCost, setupProductionCost));
            }

            if (!GlobalUtils.IsEqual(sol.totalUnitProductionCost, unitProductionCost))
            {
                GlobalUtils.writeToConsole(String.Format("Mismatch of production unit cost: Solution {0} diff. calculated from scratch {1}", sol.totalUnitProductionCost, unitProductionCost));
            }


            //Routing Issues
            double totalRoutingCost = 0;
            for (int p = 0; p < sol.periods.Count; p++)
            {
                Period per = sol.periods[p];
                for (int r = 0; r < per.periodRoutes.Count; r++)
                {
                    Route rt = per.periodRoutes[r];
                    double rtCost = 0;
                    for (int n = 0; n < rt.nodes.Count - 1; n++)
                    {
                        Node a = rt.nodes[n];
                        Node b = rt.nodes[n + 1];
                        rtCost += model.distMatrix[a.uid, b.uid];
                    }
                    if (!GlobalUtils.IsEqual(rtCost, rt.totalRoutingCost))
                    {
                        GlobalUtils.writeToConsole(String.Format("Mismatch of routing cost of route {0}", rt.ToString()));
                    }
                    if (rt.nodes[0].uid != 0 || rt.nodes[rt.nodes.Count - 1].uid != 0)
                    {
                        GlobalUtils.writeToConsole(String.Format("Not depot in start or end of route {0}", rt.ToString()));
                    }
                    for (int n = 1; n < rt.nodes.Count - 1; n++)
                    {
                        Node cus = rt.nodes[n];
                        cus.deliveredQuantities[p] = int.MaxValue; //initialize deliveries quantities of route to max 
                        if (cus.uid == 0)
                        {
                            GlobalUtils.writeToConsole("Depot in the middle");
                        }
                    }
                    totalRoutingCost += rtCost;
                }
            }
            if (!GlobalUtils.IsEqual(totalRoutingCost, sol.routingCost))
            {
                GlobalUtils.writeToConsole(String.Format("Mismatch of routing cost: Solution {0} diff. calculated from scratch {1}", sol.routingCost, totalRoutingCost));
            }

            //TODO Add a check to validate the visit schedule
            //what kind of check? it is checked through inventory and quantity checks.

            double AllCustomersInventoryCost = 0;
            for (int i = 0; i < sol.customers.Count; i++)
            {
                double customerInventoryCost = 0;
                Node customer = sol.customers[i];
                
                //Calculate quantities with saw
                bool feasible = customer.ApplySaw(customer.visitSchedule);
                customer.auxiliary_deliveries.CopyTo(customer.deliveredQuantities, 0);
                customer.auxiliary_endDayInventory.CopyTo(customer.endDayInventory, 0);
                customer.CalculateInventoryLevels();
                
                double stock = customer.startingInventory;
                for (int p = 0; p < model.horizonDays; p++)
                {
                    customerInventoryCost += (customer.endDayInventory[p] * customer.unitHoldingCost);
                }
                customer.totalHoldingCost = customerInventoryCost;

                AllCustomersInventoryCost += customer.totalHoldingCost;
                if (!feasible)
                {
                    GlobalUtils.writeToConsole(String.Format("Saw mechanism is invalid for customer {0}", customer));
                }
            }

            double depotInventoryObjective = 0;
            double depotStock = sol.depot.startingInventory;
            for (int p = 0; p < sol.periods.Count; p++)
            {
                depotStock = depotStock + sol.depot.productionRates[p];

                double outboundFromCustomers = totalOutboundFlowOfPeriodFromCustomers(model, sol, p);
                double outboundFromRoutes = totalOutboundFlowOfPeriodFromRoutes(model, sol, p);
                if (!GlobalUtils.IsEqual(outboundFromRoutes, outboundFromCustomers))
                {
                    GlobalUtils.writeToConsole("Outbound Flow Mismatch on period " + p);
                }

                depotStock = depotStock - outboundFromCustomers;
                depotInventoryObjective += (depotStock * sol.depot.unitHoldingCost);
            }
            sol.depot.totalHoldingCost = depotInventoryObjective;

            double solutionInventoryObjective = depotInventoryObjective + AllCustomersInventoryCost;

            if (!GlobalUtils.IsEqual(solutionInventoryObjective, sol.holdingCost))
            {
                GlobalUtils.writeToConsole(String.Format("Mismatch of total inventory cost: Solution {0} diff. calculated from scratch {1}", sol.holdingCost, solutionInventoryObjective));
            }

            //Make a rough check of the saw results
            //When there is a delivery (apart from the first delivert) the starting inventory should be 0  
            for (int i = 0; i < sol.customers.Count; i++)
            {
                Node customer = sol.customers[i];

                bool firstDelivery = false;
                for (int p = 0; p < model.horizonDays; p++)
                {
                    if (customer.deliveredQuantities[p] > 0)
                    {
                        if (!firstDelivery)
                        {
                            firstDelivery = true;
                            continue;
                        }

                        if (!GlobalUtils.IsEqual(customer.startDayInventory[p], 0))
                        {
                            customer.ReportInventory();
                            GlobalUtils.writeToConsole("According to SAW the start inventory should be 0");
                        }
                            
                    }
                    
                    if (customer.endDayInventory[p] < 0)
                    {
                        GlobalUtils.writeToConsole("Customer Stock Out");
                    }
                }
            }

            if (!GlobalUtils.IsEqual(solutionInventoryObjective + totalRoutingCost + setupProductionCost + unitProductionCost, sol.totalObjective))
            {
                GlobalUtils.writeToConsole("Outbound Total Objective Cost");
            }
        }

        private static void ConstructAndTestEverythingFromScratchGivenSolution(Solution sol, PRP model)
        {
            for (int i = 0; i < sol.customers.Count; i++)
            {
                for (int p = 0; p < model.horizonDays; p++)
                {
                    sol.customers[i].deliveredQuantities[p] = 0;
                }
            }

            //Routing Issues
            double totalRoutingCost = 0;
            for (int p = 0; p < sol.periods.Count; p++)
            {
                Period per = sol.periods[p];
                for (int r = 0; r < per.periodRoutes.Count; r++)
                {
                    Route rt = per.periodRoutes[r];
                    double rtCost = 0;
                    for (int n = 0; n < rt.nodes.Count - 1; n++)
                    {
                        Node a = rt.nodes[n];
                        Node b = rt.nodes[n + 1];
                        rtCost += model.distMatrix[a.uid, b.uid];
                    }
                    if (!GlobalUtils.IsEqual(rtCost, rt.totalRoutingCost))
                    {
                        GlobalUtils.writeToConsole("Routing Cost");
                    }
                    if (rt.nodes[0].uid != 0 || rt.nodes[rt.nodes.Count - 1].uid != 0)
                    {
                        GlobalUtils.writeToConsole("Not depot in start end");
                    }
                    for (int n = 1; n < rt.nodes.Count - 1; n++)
                    {
                        Node cus = rt.nodes[n];
                        cus.deliveredQuantities[p] = int.MaxValue;
                        if (cus.uid == 0)
                        {
                            GlobalUtils.writeToConsole("Depot in the middle");
                        }
                    }
                    totalRoutingCost += rtCost;
                }
            }

            if (!GlobalUtils.IsEqual(totalRoutingCost, sol.routingCost))
            {
                GlobalUtils.writeToConsole("Routing Cost");
            }

            double AllCustomersInventoryCost = 0;
            for (int i = 0; i < sol.customers.Count; i++)
            {
                double customerInventoryCost = 0;
                Node customer = sol.customers[i];
                double stock = customer.startingInventory;
                for (int p = 0; p < model.horizonDays; p++)
                {
                    stock = stock - customer.productRate;

                    if (customer.deliveredQuantities[p] > 0)
                    {
                        customer.deliveredQuantities[p] = customer.stockMaximumLevel - stock;
                    }

                    stock = stock + customer.deliveredQuantities[p];

                    customer.endDayInventory[p] = stock;

                    customerInventoryCost += (stock * customer.unitHoldingCost);
                }
                customer.totalHoldingCost = customerInventoryCost;

                AllCustomersInventoryCost += customer.totalHoldingCost;
            }

            for (int p = 0; p < sol.periods.Count; p++)
            {
                Period per = sol.periods[p];
                for (int r = 0; r < per.periodRoutes.Count; r++)
                {
                    Route rt = per.periodRoutes[r];
                    rt.load = 0;
                    for (int n = 1; n < rt.nodes.Count - 1; n++)
                    {
                        Node nd = rt.nodes[n];
                        rt.load += nd.deliveredQuantities[p];
                    }
                }
            }

            double depotInventoryObjective = 0;
            double depotStock = sol.depot.startingInventory;
            for (int p = 0; p < sol.periods.Count; p++)
            {
                depotStock = depotStock + sol.depot.productRate;

                double outboundFromCustomers = totalOutboundFlowOfPeriodFromCustomers(model, sol, p);
                double outboundFromRoutes = totalOutboundFlowOfPeriodFromRoutes(model, sol, p);
                if (!GlobalUtils.IsEqual(outboundFromRoutes, outboundFromCustomers))
                {
                    GlobalUtils.writeToConsole("Outbound Cost");
                }

                depotStock = depotStock - outboundFromCustomers;
                depotInventoryObjective += (depotStock * sol.depot.unitHoldingCost);
            }
            sol.depot.totalHoldingCost = depotInventoryObjective;

            double solutionInventoryObjective = depotInventoryObjective + AllCustomersInventoryCost;

            for (int i = 0; i < sol.customers.Count; i++)
            {
                Node customer = sol.customers[i];

                for (int p = 0; p < model.horizonDays; p++)
                {
                    if (customer.deliveredQuantities[p] > 0)
                    {
                        if (customer.endDayInventory[p] != customer.stockMaximumLevel)
                        {
                            GlobalUtils.writeToConsole("Maximum Level");
                        }
                    }
                    if (customer.endDayInventory[p] < 0)
                    {
                        GlobalUtils.writeToConsole("Stock Out");
                    }
                }
            }

            if (!GlobalUtils.IsEqual(solutionInventoryObjective + totalRoutingCost, sol.totalObjective))
            {
                GlobalUtils.writeToConsole("Outbound Cost");
            }
        }
        */
    }
}
