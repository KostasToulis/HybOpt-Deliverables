using System;
using System.Collections.Generic;
using System.Data;
using System.IO;
using System.Linq;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using System.Reflection.PortableExecutable;
using System.Text;
using System.Threading.Tasks;

namespace PRP
{
    public enum INFEASIBILITY_STATUS
    {
        FEASIBLE = 0x0,
        CUSTOMER_STOCKOUT,
        MISCALC_CUSTOMER_INVENTORY,
        DEPOT_STOCKOUT,
        CUSTOMER_MAXSTOCK,
        DEPOT_MAXSTOCK,
        CUSTOMER_INCONSISTENT_VISITSCHEDULE,
        CUSTOMER_INCONSISTENT_DELIVERY,
        ROUTE_GENERIC_VIOLATION,
        ROUTE_LOAD_HARDVIOLATION,
        ROUTE_LOAD_SOFTVIOLATION,
        FLOW_MISMATCH,
        MISCALC_OBJ_GENERIC,
        MISCALC_OBJ_HOLDING_COST,
        MISCALC_OBJ_ROUTING_COST,
        MISCALC_OBJ_PRODUCTION_COST,
        PERIODICITY_VIOLATION,
        INFEASIBLE //General status
    }
    
    public class Solution
    {
        public List<Period> periods;
        public INFEASIBILITY_STATUS status;
        public double totalObjective;
        public double holdingCost;
        public double routingCost;
        public double setupProductionCost;
        public double totalUnitProductionCost;
        public double violationCost;
        public double totalObjectiveIncFirstPeriodInventory; //unused
        public double ellapsedMs;
        public double restartElapsedMs;
        public double totalTimeRepairing;
        public int feasibleSpaceIters;
        public int infeasibleSpaceIters;
        public int totalRepairs;
        public float infeasibilityCoeff;
        public List<Node> nodes;
        public List<Node> augNodes;
        public List<Node> customers;
        public Depot depot;

        public PRP model; //Model reference
        
        public Solution(PRP m)
        {
            model = m; //Setup solution model
            periods = new List<Period>(model.horizonDays);
            infeasibilityCoeff = 1.0f;

            //Make a unique copy of the nodes from the model
            nodes = new List<Node>();
            augNodes = new List<Node>();
            customers = new List<Node>();

            for (int i = 0; i < m.input.nodes.Count; i++)
            {

                if (i > 0)
                {
                    Node nNode = new Node(m.input.nodes[i]);
                    nodes.Add(nNode);
                    augNodes.Add(nNode);
                    customers.Add(nNode);
                }
                else
                {
                    Depot nNode = new Depot(m.input.nodes[i] as Depot);
                    nodes.Add(nNode);
                    augNodes.Add(nNode);
                    depot = nNode;
                    depot.auxiliary_SignedFlowDiffs_Depot = new double[model.horizonDays];
                }
            }

            Depot nNode2 = new Depot(m.input.nodes[0] as Depot);
            augNodes.Add(nNode2);

            for (int i = 0; i < model.horizonDays; i++)
            {
                Period nPeriod = new Period(model.vehicles, model.capacity);
                nPeriod.periodIndex = i;
                periods.Add(nPeriod);
            }
        }

        public Solution(Solution sol, double constantFirstPeriodInvCost)
        {
            //Copy solution metrics
            totalObjective = sol.totalObjective;
            status = sol.status;
            infeasibilityCoeff = sol.infeasibilityCoeff;
            holdingCost = sol.holdingCost;
            routingCost = sol.routingCost;
            violationCost = sol.violationCost;
            setupProductionCost = sol.setupProductionCost;
            totalUnitProductionCost = sol.totalUnitProductionCost;
            totalObjectiveIncFirstPeriodInventory = sol.totalObjective + constantFirstPeriodInvCost;
            ellapsedMs = sol.ellapsedMs;
            restartElapsedMs = sol.restartElapsedMs;
            totalTimeRepairing = sol.totalTimeRepairing;
            feasibleSpaceIters = sol.feasibleSpaceIters;
            infeasibleSpaceIters = sol.infeasibleSpaceIters;
            totalRepairs = sol.totalRepairs;
            model = sol.model;
            periods = new List<Period>(model.horizonDays);
            nodes = new List<Node>();
            customers = new List<Node>();

            //Copy Nodes and customers
            for (int i = 0; i < sol.nodes.Count; i++)
            {
                if (i > 0)
                {
                    Node nNode = new Node(sol.nodes[i]);
                    nodes.Add(nNode);
                    customers.Add(nNode);
                }
                else
                {
                    Depot nNode = new Depot(sol.nodes[i] as Depot);
                    nodes.Add(nNode);
                    depot = nNode;
                    depot.auxiliary_SignedFlowDiffs_Depot = new double[model.horizonDays];
                }
            }

            //Copy periods
            for (int p = 0; p < sol.periods.Count; p++)
            {
                Period nPeriod = new Period(sol.periods[p]);
                Period sol_nPeriod = sol.periods[p];

                //GlobalUtils.writeToConsole(Object.ReferenceEquals(nPeriod, sol_nPeriod));

                //Iterate in Routes and make sure to replace the nodes
                for (int r = 0; r < nPeriod.periodRoutes.Count; r++)
                {
                    Route rt = nPeriod.periodRoutes[r];
                    Route sol_rt = sol_nPeriod.periodRoutes[r];

                    //GlobalUtils.writeToConsole(Object.ReferenceEquals(rt, sol_rt));

                    rt.nodes.Clear();
                    //Replace Node references
                    for (int i = 0; i < sol_rt.nodes.Count; i++)
                    {
                        Node sol_nd = sol_rt.nodes[i];
                        Node nd = nodes[sol_nd.uid];
                        rt.nodes.Add(nd);

                        if (sol_nd.horizonDeliveryServices[p] != null)
                            nd.horizonDeliveryServices[p] =
                                new CustDelivery(sol_nd.horizonDeliveryServices[p].quantity, rt);
                    }

                }

                //nPeriod.Report();
                periods.Add(nPeriod);
            }
        }

        public bool isBetterThan(Solution sol)
        {
            if (this.totalObjective < sol.totalObjective)
                return true;
            return false;
        }

        public void InitializeEmptySolutionDetails()
        {
            InitializeInventoryAspects();
            InitializeProductionAspects();
            InitializeRoutingAspects();
            
            totalObjective = routingCost + holdingCost + setupProductionCost + totalUnitProductionCost;
        }

        public void InitializeProductionAspects()
        {
            setupProductionCost = 0;
            totalUnitProductionCost = 0;

            //We can directly calculate the production costs since they will not change throughout the search
            
            for (int i = 0; i < model.horizonDays; i++)
            {
                totalUnitProductionCost += model.production_input.productionQuantities[i]* depot.unitProductionCost; //TODO ensure that depot is nodes[0]
                if (model.production_input.productionQuantities[i] > 0)
                {
                    setupProductionCost += depot.productionSetupCost;
                }
            }
        }

        public void InitializeInventoryAspects()
        {
            InitializeDepotProductionInfo();
            InitializeInventoryLevelsForAllNodes();
            InitializeMinimumRequiredVisitsForAllNodes();
            ArrangeDaysToStockOutForAllCustomers();
            InitializeInventoryCostsForAllNodes();
            InitializeHorizonDeliveries();
        }

        private void InitializeHorizonDeliveries()
        {
            foreach (var nd in nodes)
            {
                //Initialize Horizon Deliveries for every customer to 0
                for (int i = 0; i < model.horizonDays; i++)
                {
                    nd.horizonDeliveryServices[i].reset();
                    nd.deliveredQuantities[i] = 0;
                }
            }
        }

        private void InitializeDepotProductionInfo()
        {
            model.production_input.plantOpen.CopyTo(depot.open, 0);
            model.production_input.productionQuantities.CopyTo(depot.productionRates, 0);
        }

        private void InitializeInventoryLevelsForAllNodes()
        {
            //Init inventory levels including the depot
            for (int i = 0; i < nodes.Count; i++)
            {
                nodes[i].InitializeInventoryLevels();
            }
        }

        private void InitializeMinimumRequiredVisitsForAllNodes()
        {
            //Init inventory levels including the depot
            for (int i = 0; i < nodes.Count; i++)
            {
                nodes[i].calculateMinimumRequiredVisits(model.capacity);
            }
        }       


        private void InitializeInventoryCostsForAllNodes()
        {
            holdingCost = 0;
            for (int i = 0; i < nodes.Count; i++)
            {
                Node n = nodes[i];
                //During init this is probably just the remaining stock at day 0 * the holding cost. Can we get rid of this call?
                n.totalHoldingCost = n.CalculateHoldingCost(0, 0);
                holdingCost += n.totalHoldingCost;
            }
        }

        private void InitializeRoutingAspects()
        {
            foreach (var p in periods)
            {
                
                foreach (var rt in p.periodRoutes)
                {
                    rt.initialize(depot);
                }
            }

            routingCost = 0;
        }

        private void ArrangeDaysToStockOutForAllCustomers()
        {
            for (int i = 0; i < customers.Count; i++)
            {
                Node n = customers[i];
                n.ArrangeDaysToStockOut();
            }
        }


        public bool DiagnoseInfeasibility()
        {
            for (int i = 0; i < customers.Count; i++)
            {
                Node customer = customers[i];

                if (customer.stockMaximumLevel - customer.startingInventory > model.capacity)
                {
                    return false;
                }
            }

            return true;
        }

        public static bool checkSolutionStatus(INFEASIBILITY_STATUS status)
        {
            switch (status)
            {
                //Soft route load violations are allowed
                case INFEASIBILITY_STATUS.ROUTE_LOAD_SOFTVIOLATION:
                case INFEASIBILITY_STATUS.FEASIBLE:
                    return true;
                //Any other solution status is considered as an infeasibility
                default:
                    return false;
            }
        }

        private static int occurencesInRoutesOfPeriod(Node customer, Period period)
        {
            int occ = 0;
            for (int r = 0; r < period.periodRoutes.Count; r++)
            {
                Route rt = period.periodRoutes[r];
                for (int n = 1; n < rt.nodes.Count - 1; n++)
                {
                    if (rt.nodes[n] == customer)
                    {
                        occ++;
                    }
                }
            }

            return occ;
        }


        public bool checkVehicleFeasibility()
        {
            foreach (Period pr in periods)
            {
                if (pr.periodRoutes.Count > model.input.availableVehicles)
                    return false;
            }

            return true;
        }

        public void setVehicleCapacityCoeff(float coeff)
        {
            infeasibilityCoeff = coeff;
            foreach (Period p in periods)
            {
                foreach (Route rt in p.periodRoutes)
                {
                    rt.effectiveCapacity = (int) (coeff * rt.realCapacity);
                }
            }
        }

        public void findMaxVehicleCapacityCoeff(bool fullcap = false)
        {
            float old_val = infeasibilityCoeff;
            float step = 0.005f;
            float total_move = 0.0f;


            while (Solution.checkSolutionStatus(TestEverythingFromScratch()) && (total_move <= 0.01f || fullcap))
            {
                old_val -= step;
                setVehicleCapacityCoeff(old_val);
                total_move += step;
            }

            GlobalUtils.writeToConsole("Setting Min possible infeasibility coeff {0}", old_val + step);
            setVehicleCapacityCoeff((float) Math.Round(old_val + step, 3)); //This should be the last working value

            if (!Solution.checkSolutionStatus(TestEverythingFromScratch()))
                throw new Exception("Do something");
            
        }

        public void resetVehicleCapacityCoeff()
        {
            infeasibilityCoeff = 1.0f;
            foreach (Period p in periods)
            {
                foreach (Route rt in p.periodRoutes)
                {
                    rt.effectiveCapacity = rt.realCapacity;
                }
            }
        }
        
        
        public int calcVehicleLoadViolations()
        {
            int viol = 0;
            
            foreach (Period pr in periods)
            {
                viol += calcVehicleLoadViolationOnPeriod(pr.periodIndex);
            }   
            
            return viol;
        }

        public int calcVehicleLoadViolationOnPeriod(int day)
        {
            int viol = 0;
            Period pr = periods[day];
            foreach (Route rt in pr.periodRoutes)
                viol += Math.Max(0, rt.load - rt.realCapacity);
            return viol;
        }

        public void reportVehicleLoadViolations()
        {
            foreach (Period pr in periods)
            {
                for (int i=0; i< pr.periodRoutes.Count; i++)
                {
                    Route rt = pr.periodRoutes[i];

                    if (rt.load > rt.realCapacity)
                        GlobalUtils.writeToConsole("Day {0} Route {1} Capacity Violation {2}", 
                            pr.periodIndex, i, rt.load - rt.realCapacity);
                }
            }
        }


        public void fixVehicles()
        {
            foreach (Period pr in periods)
            {
                int emptyRouteNum = 0;
                bool hasemptyroute = false;
                int routeIndex = 0;

                bool alreadyinfeasible = (pr.periodRoutes.Count > model.input.availableVehicles);
                
                
                while (routeIndex < pr.periodRoutes.Count)
                {
                    Route rt = pr.periodRoutes[routeIndex];

                    if (rt.nodes.Count == 2)
                    {
                        if (hasemptyroute)
                        {
                            pr.removeRoute(routeIndex);
                            continue;
                        } else
                            hasemptyroute = true;
                    }
                    
                    routeIndex++;
                }

                //Add an empty vehicle if necessary
                if ((pr.periodRoutes.Count < model.input.availableVehicles) && !hasemptyroute)
                    pr.appendRoute(model, depot);
            }
        }
        
        public void ApplySaw()
        {
            //This method respects the minimum production quantity for the schedule as well as the routing of the problem
            //Given this information, it calculates the delivered quantities for all the customers using the saw mechanism
            
            //SAW MECHANISMS PRINCIPLES:
            //1 : Expect for the first delivery day that can be performed even with positive inventory,
            // for all other deliveries the starting inventory quantity of the delivery day for this customer should be equal to 0.
            // In other words, the delivered quantity is just enough until the next delivery period (1 delivered excepted)
            //2 : The total sum of the delivered quantity to any node should match the sum of delivered quantities +
            //the start inventory
            
            for (int i = 0; i < customers.Count; i++)
            {
                Node cust = customers[i];
                
                //Construct visit schedule based on the nodes horizon delivery service
                bool[] visitSchedule = new bool[model.horizonDays];
                for (int j = 0; j < model.horizonDays; j++)
                {
                    visitSchedule[j] = false;
                    if (cust.horizonDeliveryServices[j] != null)
                        visitSchedule[j] = true;
                }
                
                cust.ApplySaw(visitSchedule);
            }
        }
        
        //Objective evaluators
        public static double EvaluateUnitProductionObjectivefromScratch(Solution sol)
        {
            double totalUnitProductionCost = 0;
            
            for (int i = 0; i < sol.model.horizonDays; i++)
                totalUnitProductionCost += sol.depot.productionRates[i]* sol.depot.unitProductionCost; 
            
            return totalUnitProductionCost;
        }
        
        public static double EvaluateSetupProductionObjectivefromScratch(Solution sol)
        {
            double setupProductionCost = 0;
            
            for (int i = 0; i < sol.model.horizonDays; i++)
            {
                if (sol.depot.productionRates[i] > 0)
                    setupProductionCost += sol.depot.productionSetupCost;
            }

            return setupProductionCost;
        }
        
        public static double EvaluateRoutingObjectivefromScratch(Solution sol)
        {
            double totalRoutingObjective = 0;

            for (int i = 0; i < sol.periods.Count; i++)
            {
                Period p = sol.periods[i];

                for (int j = 0; j < p.periodRoutes.Count; j++)
                {
                    Route rt = p.periodRoutes[j];
                    double rtCost = rt.totalRoutingCost;
                    double rtCostFromScratch = 0;

                    for (int k = 0; k < rt.nodes.Count -1; k++)
                    {
                        rtCostFromScratch += sol.model.distMatrix[rt.nodes[k].uid, rt.nodes[k + 1].uid];
                    }

                    totalRoutingObjective += rtCost;
                }
            }
            
            return totalRoutingObjective;
        }
        
        public static double EvaluateInventoryObjectivefromScratch(Solution sol)
        {
            double invObjective = 0;
            
            for (int i = 0; i < sol.nodes.Count; i++)
            {
                Node n = sol.nodes[i];

                double nodeInvObj = 0;
                double stock = n.startingInventory;
                
                for (int p = 0; p < sol.model.horizonDays;p++)
                {
                    if (n is Depot nDepot)
                        stock += nDepot.productionRates[p];
                    else
                        stock += n.productRateSigned[p];
                    
                    stock += n.deliveredQuantities[p];

                    double actualStock = Math.Max(0, stock);

                    nodeInvObj += (n.unitHoldingCost * actualStock);
                }
                
                invObjective += nodeInvObj;
            }
            
            return invObjective;
        }

        public INFEASIBILITY_STATUS TestEverythingFromScratchCyclic()
        {
            //WARNING: this method does not re-initialize the delivered quantities now.

            // 1. Calculate Production Cost (unit and setup) from Scratch
            double unitProductionCostLocal = 0;
            double setupProductionCostLocal = 0;
            for (int p = 0; p < periods.Count; p++)
            {
                unitProductionCostLocal += depot.productionRates[p] * depot.unitProductionCost;
                if (depot.open[p])
                    setupProductionCostLocal += depot.productionSetupCost;
            }
            if (!GlobalUtils.IsEqual(setupProductionCost, setupProductionCostLocal))
            {
                GlobalUtils.writeToConsole(String.Format("Mismatch of production setup cost: Solution {0} diff. calculated from scratch {1}", setupProductionCost, setupProductionCostLocal));
                return INFEASIBILITY_STATUS.MISCALC_OBJ_PRODUCTION_COST;
            }
            if (!GlobalUtils.IsEqual(totalUnitProductionCost, unitProductionCostLocal))
            {
                GlobalUtils.writeToConsole(String.Format("Mismatch of production unit cost: Solution {0} diff. calculated from scratch {1}", totalUnitProductionCost, unitProductionCostLocal));
                return INFEASIBILITY_STATUS.MISCALC_OBJ_PRODUCTION_COST;
            }

            // 2. Routing Issues
            double totalRoutingCostLocal = 0;
            for (int p = 0; p < periods.Count; p++)
            {
                Period per = periods[p];

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
                        GlobalUtils.writeToConsole(String.Format("Mismatch of routing cost of route {0} between solution {1} and calculated {2}",
                            rt.ToString(), rt.totalRoutingCost, rtCost));
                        return INFEASIBILITY_STATUS.MISCALC_OBJ_ROUTING_COST;
                    }
                    if (rt.nodes.Count != 0)
                    {
                        if (rt.nodes[0].uid != 0 || rt.nodes[rt.nodes.Count - 1].uid != 0)
                        {
                            GlobalUtils.writeToConsole(String.Format("Not depot in start or end of route {0}", rt.ToString()));
                            return INFEASIBILITY_STATUS.INFEASIBLE;
                        }
                        // check only non empty routes
                        for (int n = 1; n < rt.nodes.Count - 1; n++)
                        {
                            Node cus = rt.nodes[n];
                            if (cus.uid == 0)
                            {
                                GlobalUtils.writeToConsole("Depot in the middle");
                                return INFEASIBILITY_STATUS.INFEASIBLE;
                            }
                        }
                    }
                    totalRoutingCostLocal += rtCost;
                }
            }
            if (!GlobalUtils.IsEqual(totalRoutingCostLocal, routingCost))
            {
                GlobalUtils.writeToConsole(String.Format("Mismatch of routing cost: Solution {0} diff. calculated from scratch {1}", routingCost, totalRoutingCostLocal));
                return INFEASIBILITY_STATUS.MISCALC_OBJ_ROUTING_COST;
            }


            //Check the visit schedule and the consistency of the delivery services
            for (int i = 0; i < customers.Count; i++)
            {
                Node cust = customers[i];

                for (int p = 0; p < model.horizonDays; p++)
                {
                    if (cust.horizonDeliveryServices[p].route != null && cust.visitSchedule[p] == false)
                    {
                        GlobalUtils.writeToConsole("Inconsistent visit schedule and horizon deliveries of customer {0} (routed but no service is scheduled)",
                            cust.uid);
                        return INFEASIBILITY_STATUS.CUSTOMER_INCONSISTENT_VISITSCHEDULE;
                    }

                    if (cust.horizonDeliveryServices[p].quantity > 0 && cust.visitSchedule[p] == false)
                    {
                        GlobalUtils.writeToConsole("Inconsistent visit schedule and horizon delivery quantity of customer {0} (quantity delivery but no service is scheduled)",
                            cust.uid);
                        return INFEASIBILITY_STATUS.CUSTOMER_INCONSISTENT_DELIVERY;
                    }

                    if (cust.visitSchedule[p])
                    {
                        if (cust.horizonDeliveryServices[p] == null)
                        {
                            GlobalUtils.writeToConsole("Active visit day but null delivery service on customer {0}", cust.uid);
                            return INFEASIBILITY_STATUS.CUSTOMER_INCONSISTENT_VISITSCHEDULE;
                        }

                        if (cust.horizonDeliveryServices[p].route == null)
                        {
                            GlobalUtils.writeToConsole(
                                "Active visit day, valid delivery service but null route on delivery service");
                            return INFEASIBILITY_STATUS.CUSTOMER_INCONSISTENT_VISITSCHEDULE;
                        }

                        if (cust.horizonDeliveryServices[p].quantity != cust.deliveredQuantities[p])
                        {
                            GlobalUtils.writeToConsole(
                                "Quantity mismatch between the delivery service and the cached delivered quantity. Customer {0}, Period {1}",
                                cust.uid, p);
                            return INFEASIBILITY_STATUS.CUSTOMER_INCONSISTENT_DELIVERY;
                        }
                        if (cust.horizonDeliveryServices[p].quantity == 0 || cust.deliveredQuantities[p] == 0)
                        {
                            GlobalUtils.writeToConsole(
                                "Customer {0}, Period {1}: Warning Scheduled delivery of zero quantity",
                                cust.uid, p);
                            //return INFEASIBILITY_STATUS.CUSTOMER_INCONSISTENT_DELIVERY;
                        }
                    }
                }
            }

            //For the days that horizon delivery is zero, make sure the customer is not present in this day routes
            //Note: this should be allowed for parsing the production schedules
            for (int i = 0; i < customers.Count; i++)
            {
                Node customer = customers[i];
                for (int p = 0; p < periods.Count; p++)
                {
                    int countsInRoutes = occurencesInRoutesOfPeriod(customer, periods[p]);

                    if (countsInRoutes > 1)
                    {
                        GlobalUtils.writeToConsole("Customer {0} is visited multiple times on the same period {1}", customer.ID, p);
                        return INFEASIBILITY_STATUS.INFEASIBLE;
                    }

                    if (customer.horizonDeliveryServices[p].route != null)
                    {
                        if (countsInRoutes != 1 || !customer.visitSchedule[p])
                        {
                            GlobalUtils.writeToConsole(String.Format("Customer {0} has been delivered quantity {1} without visit in period {2}",
                                customer.ID, customer.deliveredQuantities[p], p));
                            customer.ReportInventory();
                            return INFEASIBILITY_STATUS.INFEASIBLE;
                        }
                    }
                }
            }

            // 3. Customer inventory holding costs
            double AllCustomersInventoryCost = 0;
            for (int i = 0; i < customers.Count; i++)
            {
                double customerInventoryCost = 0;
                Node customer = customers[i];

                customer.CalculateInventoryLevels();

                double stock = customer.startingInventory;
                for (int p = 0; p < model.horizonDays; p++)
                {
                    customerInventoryCost += (customer.endDayInventory[p] * customer.unitHoldingCost);
                    if (customer.deliveredQuantities[p] < 0)
                    {
                        GlobalUtils.writeToConsole(String.Format("Non positive delivery for period {0} and customer {1}", p, customer.ID));
                        return INFEASIBILITY_STATUS.INFEASIBLE;
                    }
                }
                customer.totalHoldingCost = customerInventoryCost;

                AllCustomersInventoryCost += customer.totalHoldingCost;

            }

            //remaining stock for each customer
            for (int i = 0; i < customers.Count; i++)
            {
                Node customer = customers[i];
                double stockEndOfDay = customer.startingInventory;
                for (int p = 0; p < periods.Count; p++)
                {
                    stockEndOfDay -= customer.productRate[p];
                    stockEndOfDay += customer.deliveredQuantities[p];

                    if (!GlobalUtils.IsEqual(stockEndOfDay, customer.endDayInventory[p]))
                    {
                        GlobalUtils.writeToConsole(String.Format("Mismatch of solution stock {0} and calculated from scratch {1} for customer {2}",
                            customer.endDayInventory[p], stockEndOfDay, customer.ID));
                        return INFEASIBILITY_STATUS.MISCALC_CUSTOMER_INVENTORY;
                    }
                    if (stockEndOfDay < 0 && Math.Abs(stockEndOfDay) > 1e-5)
                    {
                        GlobalUtils.writeToConsole(String.Format("Customer stock out for period {0} and customer {1}", p, customer.ID));
                        return INFEASIBILITY_STATUS.CUSTOMER_STOCKOUT;
                    }
                }
            }

            // 6. Inventory minimum and maximum
            for (int i = 0; i < nodes.Count; i++)
            {
                Node node = nodes[i];
                for (int p = 0; p < model.horizonDays; p++)
                {
                    if (node.endDayInventory[p] > node.stockMaximumLevel)
                    {
                        GlobalUtils.writeToConsole(String.Format("Node {1} maximum stock exceeded for period {0}", p, node.ID));
                        return INFEASIBILITY_STATUS.CUSTOMER_MAXSTOCK;
                    }
                    if (node.endDayInventory[p] < 0.0)
                    {
                        GlobalUtils.writeToConsole(String.Format("Node {1} stock out for period {0}", p, node.ID));
                        return INFEASIBILITY_STATUS.CUSTOMER_STOCKOUT;
                    }
                }
            }

            // 4. Depot inventory holding costs
            double depotInventoryObjective = 0;
            int depotStock = depot.startingInventory;
            depot.startingInventory = depotStock;
            for (int p = 0; p < periods.Count; p++)
            {
                depot.startDayInventory[p] = depotStock;
                depotStock += depot.productionRates[p];

                int outboundFromCustomers = totalOutboundFlowOfPeriodFromCustomers(p);
                int outboundFromRoutes = totalOutboundFlowOfPeriodFromRoutes(p);
                if (outboundFromCustomers != outboundFromRoutes)
                {
                    GlobalUtils.writeToConsole(String.Format("Outbound Flow Mismatch on period {0}: calculated from routes {1}, " +
                        "calculated from customers {2}", p, outboundFromRoutes, outboundFromCustomers));
                    return INFEASIBILITY_STATUS.FLOW_MISMATCH;
                }

                depotStock = depotStock - outboundFromCustomers;
                depot.endDayInventory[p] = depotStock;
                depotInventoryObjective += (depotStock * depot.unitHoldingCost);
            }
            depot.totalHoldingCost = depotInventoryObjective;

            double solutionInventoryObjective = depotInventoryObjective + AllCustomersInventoryCost;
            if (!GlobalUtils.IsEqual(solutionInventoryObjective, holdingCost))
            {
                GlobalUtils.writeToConsole(String.Format("Mismatch of total inventory cost: Solution {0} diff. calculated from scratch {1}", holdingCost, solutionInventoryObjective));
                return INFEASIBILITY_STATUS.MISCALC_OBJ_HOLDING_COST;
            }

            // 8. Periodicity check for depot and customers
            for (int i = 0; i < nodes.Count; i++)
            {
                Node node = nodes[i];
                if (node.startingInventory != node.endDayInventory[node.endDayInventory.Length - 1])
                {
                    GlobalUtils.writeToConsole(String.Format("Node {0}: end day inventory  {1} != starting inventory {2}", node.uid, node.endDayInventory[node.endDayInventory.Length - 1], node.startingInventory));
                    return INFEASIBILITY_STATUS.PERIODICITY_VIOLATION;
                }
            }


            // 7. Total objective 
            double totalObjectiveLocal = solutionInventoryObjective + totalRoutingCostLocal + setupProductionCost + unitProductionCostLocal;
            if (!GlobalUtils.IsEqual(totalObjectiveLocal, totalObjective))
            {
                GlobalUtils.writeToConsole(String.Format("Mismatch of total objective cost: Solution {0} diff. calculated from scratch {1}", totalObjective, totalObjectiveLocal));
                return INFEASIBILITY_STATUS.MISCALC_OBJ_GENERIC;
            }



            //test load After me list
            for (int p = 0; p < periods.Count; p++)
            {
                Period per = periods[p];

                for (int r = 0; r < per.periodRoutes.Count; r++)
                {
                    Route rt = per.periodRoutes[r];
                    double rtLoadTillMe = 0;
                    for (int n = 0; n < rt.nodes.Count; n++)
                    {
                        Node node = rt.nodes[n];
                        if (node.uid != 0)
                        {
                            rtLoadTillMe += node.deliveredQuantities[p];
                        }

                        if (!GlobalUtils.IsEqual(rtLoadTillMe, rt.loadTillMe[n]))
                        {
                            GlobalUtils.writeToConsole(String.Format("Mismatch of load till me {0} and calculated from scratch {1} for customer {2} at period {3} at route {4}",
                                rt.loadTillMe[n], rtLoadTillMe, n, p, r));
                            return INFEASIBILITY_STATUS.ROUTE_GENERIC_VIOLATION;
                        }
                    }
                }
            }

            //calculate total flows for routes
            for (int p = 0; p < periods.Count; p++)
            {
                Period per = periods[p];
                per.totalOutboundProductFlow = 0;
                for (int r = 0; r < per.periodRoutes.Count; r++)
                {
                    Route rt = per.periodRoutes[r];
                    double rtLoad = 0;
                    for (int n = 1; n < rt.nodes.Count - 1; n++)
                    {
                        rtLoad += rt.nodes[n].deliveredQuantities[p];
                    }

                    if (!GlobalUtils.IsEqual(rtLoad, rt.load))
                    {
                        GlobalUtils.writeToConsole(String.Format("Mismatch of load for route {0}: solution {1} diff. calculated {2}", rt, rt.load, rtLoad));
                        return INFEASIBILITY_STATUS.ROUTE_GENERIC_VIOLATION;
                    }

                    if (rt.load > rt.effectiveCapacity)
                    {
                        GlobalUtils.writeToConsole(String.Format("Hard Vehicle capacity violation for period {0} in route {1} with load {2} and capacity {3}",
                            p, rt, rt.load, rt.effectiveCapacity));
                        return INFEASIBILITY_STATUS.ROUTE_LOAD_HARDVIOLATION;
                    }
                    else if (rt.load > rt.realCapacity)
                    {
                        //GlobalUtils.writeToConsole(String.Format("Soft Vehicle capacity violation for period {0} in route {1} with load {2} and capacity {3}",
                        //    p, rt, rt.load, rt.realCapacity));
                        return INFEASIBILITY_STATUS.ROUTE_LOAD_SOFTVIOLATION;
                    }
                    per.totalOutboundProductFlow += rtLoad;
                }
            }

            //check that the total outbound quantity per period can be satisfied by depot production
            for (int p = 0; p < periods.Count; p++)
            {
                //Note: this could be allowed if feasbile solution are not possible
                if (periods[p].totalOutboundProductFlow > depot.startDayInventory[p] + depot.productionRates[p])
                {
                    GlobalUtils.writeToConsole("Production is not sufficient for period " + p);
                    GlobalUtils.writeToConsole("{0} vs {1}", periods[p].totalOutboundProductFlow,
                        depot.startDayInventory[p] + depot.productionRates[p]);
                    depot.ReportInventory();
                    return INFEASIBILITY_STATUS.INFEASIBLE;
                }
            }

            return INFEASIBILITY_STATUS.FEASIBLE;
        }


        public INFEASIBILITY_STATUS TestEverythingFromScratchPeriodic()
        {
            //WARNING: this method does not re-initialize the delivered quantities now.

            // 1. Calculate Production Cost (unit and setup) from Scratch
            double unitProductionCostLocal = 0;
            double setupProductionCostLocal = 0;
            for (int p = 0; p < periods.Count; p++)
            {
                unitProductionCostLocal += depot.productionRates[p] * depot.unitProductionCost;
                if (depot.open[p])
                    setupProductionCostLocal += depot.productionSetupCost;
            }
            if (!GlobalUtils.IsEqual(setupProductionCost, setupProductionCostLocal))
            {
                GlobalUtils.writeToConsole(String.Format("Mismatch of production setup cost: Solution {0} diff. calculated from scratch {1}", setupProductionCost, setupProductionCostLocal));
                return INFEASIBILITY_STATUS.MISCALC_OBJ_PRODUCTION_COST;
            }
            if (!GlobalUtils.IsEqual(totalUnitProductionCost, unitProductionCostLocal))
            {
                GlobalUtils.writeToConsole(String.Format("Mismatch of production unit cost: Solution {0} diff. calculated from scratch {1}", totalUnitProductionCost, unitProductionCostLocal));
                return INFEASIBILITY_STATUS.MISCALC_OBJ_PRODUCTION_COST;
            }

            // 2. Routing Issues
            double totalRoutingCostLocal = 0;
            for (int p = 0; p < periods.Count; p++)
            {
                Period per = periods[p];

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
                        GlobalUtils.writeToConsole(String.Format("Mismatch of routing cost of route {0} between solution {1} and calculated {2}",
                            rt.ToString(), rt.totalRoutingCost, rtCost));
                        return INFEASIBILITY_STATUS.MISCALC_OBJ_ROUTING_COST;
                    }
                    if (rt.nodes.Count != 0)
                    {
                        if (rt.nodes[0].uid != 0 || rt.nodes[rt.nodes.Count - 1].uid != 0)
                        {
                            GlobalUtils.writeToConsole(String.Format("Not depot in start or end of route {0}", rt.ToString()));
                            return INFEASIBILITY_STATUS.INFEASIBLE;
                        }
                        // check only non empty routes
                        for (int n = 1; n < rt.nodes.Count - 1; n++)
                        {
                            Node cus = rt.nodes[n];
                            if (cus.uid == 0)
                            {
                                GlobalUtils.writeToConsole("Depot in the middle");
                                return INFEASIBILITY_STATUS.INFEASIBLE;
                            }
                        }
                    }
                    totalRoutingCostLocal += rtCost;
                }
            }
            if (!GlobalUtils.IsEqual(totalRoutingCostLocal, routingCost))
            {
                GlobalUtils.writeToConsole(String.Format("Mismatch of routing cost: Solution {0} diff. calculated from scratch {1}", routingCost, totalRoutingCostLocal));
                return INFEASIBILITY_STATUS.MISCALC_OBJ_ROUTING_COST;
            }


            //Check the visit schedule and the consistency of the delivery services
            for (int i = 0; i < customers.Count; i++)
            {
                Node cust = customers[i];

                for (int p = 0; p < model.horizonDays; p++)
                {
                    if (cust.horizonDeliveryServices[p].route != null && cust.visitSchedule[p] == false)
                    {
                        GlobalUtils.writeToConsole("Inconsistent visit schedule and horizon deliveries of customer {0} (routed but no service is scheduled)",
                            cust.uid);
                        return INFEASIBILITY_STATUS.CUSTOMER_INCONSISTENT_VISITSCHEDULE;
                    }

                    if (cust.horizonDeliveryServices[p].quantity > 0 && cust.visitSchedule[p] == false)
                    {
                        GlobalUtils.writeToConsole("Inconsistent visit schedule and horizon delivery quantity of customer {0} (quantity delivery but no service is scheduled)",
                            cust.uid);
                        return INFEASIBILITY_STATUS.CUSTOMER_INCONSISTENT_DELIVERY;
                    }

                    if (cust.visitSchedule[p])
                    {
                        if (cust.horizonDeliveryServices[p] == null)
                        {
                            GlobalUtils.writeToConsole("Active visit day but null delivery service on customer {0}", cust.uid);
                            return INFEASIBILITY_STATUS.CUSTOMER_INCONSISTENT_VISITSCHEDULE;
                        }

                        if (cust.horizonDeliveryServices[p].route == null)
                        {
                            GlobalUtils.writeToConsole(
                                "Active visit day, valid delivery service but null route on delivery service");
                            return INFEASIBILITY_STATUS.CUSTOMER_INCONSISTENT_VISITSCHEDULE;
                        }

                        if (cust.horizonDeliveryServices[p].quantity != cust.deliveredQuantities[p])
                        {
                            GlobalUtils.writeToConsole(
                                "Quantity mismatch between the delivery service and the cached delivered quantity. Customer {0}, Period {1}",
                                cust.uid, p);
                            return INFEASIBILITY_STATUS.CUSTOMER_INCONSISTENT_DELIVERY;
                        }
                        if (cust.horizonDeliveryServices[p].quantity == 0 || cust.deliveredQuantities[p]==0)
                        {
                            GlobalUtils.writeToConsole(
                                "Customer {0}, Period {1}: Warning Scheduled delivery of zero quantity",
                                cust.uid, p);
                            //return INFEASIBILITY_STATUS.CUSTOMER_INCONSISTENT_DELIVERY;
                        }
                    }
                }
            }

            //For the days that horizon delivery is zero, make sure the customer is not present in this day routes
            //Note: this should be allowed for parsing the production schedules
            for (int i = 0; i < customers.Count; i++)
            {
                Node customer = customers[i];
                for (int p = 0; p < periods.Count; p++)
                {
                    int countsInRoutes = occurencesInRoutesOfPeriod(customer, periods[p]);

                    if (countsInRoutes > 1)
                    {
                        GlobalUtils.writeToConsole("Customer {0} is visited multiple times on the same period {1}",customer.ID, p);
                        return INFEASIBILITY_STATUS.INFEASIBLE;
                    }

                    if (customer.horizonDeliveryServices[p].route != null)
                    {
                        if (countsInRoutes != 1 || !customer.visitSchedule[p])
                        {
                            GlobalUtils.writeToConsole(String.Format("Customer {0} has been delivered quantity {1} without visit in period {2}",
                                customer.ID, customer.deliveredQuantities[p], p));
                            customer.ReportInventory();
                            return INFEASIBILITY_STATUS.INFEASIBLE;
                        }
                    }
                }
            }

            // 3. Customer inventory holding costs
            double AllCustomersInventoryCost = 0;
            for (int i = 0; i < customers.Count; i++)
            {
                double customerInventoryCost = 0;
                Node customer = customers[i];

                customer.CalculateInventoryLevels();

                double stock = customer.startingInventory;
                for (int p = 0; p < model.horizonDays; p++)
                {
                    customerInventoryCost += (customer.endDayInventory[p] * customer.unitHoldingCost);
                    if (customer.deliveredQuantities[p] < 0)
                    {
                        GlobalUtils.writeToConsole(String.Format("Non positive delivery for period {0} and customer {1}", p, customer.ID));
                        return INFEASIBILITY_STATUS.INFEASIBLE;
                    }
                }
                customer.totalHoldingCost = customerInventoryCost;

                AllCustomersInventoryCost += customer.totalHoldingCost;

            }

            //remaining stock for each customer
            for (int i = 0; i < customers.Count; i++)
            {
                Node customer = customers[i];
                double stockEndOfDay = customer.startingInventory;
                for (int p = 0; p < periods.Count; p++)
                {
                    stockEndOfDay -= customer.productRate[p];
                    stockEndOfDay += customer.deliveredQuantities[p];

                    if (!GlobalUtils.IsEqual(stockEndOfDay, customer.endDayInventory[p]))
                    {
                        GlobalUtils.writeToConsole(String.Format("Mismatch of solution stock {0} and calculated from scratch {1} for customer {2}",
                            customer.endDayInventory[p], stockEndOfDay, customer.ID));
                        return INFEASIBILITY_STATUS.MISCALC_CUSTOMER_INVENTORY;
                    }
                    if (stockEndOfDay < 0 && Math.Abs(stockEndOfDay) > 1e-5)
                    {
                        GlobalUtils.writeToConsole(String.Format("Customer stock out for period {0} and customer {1}", p, customer.ID));
                        return INFEASIBILITY_STATUS.CUSTOMER_STOCKOUT;
                    }
                }
            }

            // 6. Inventory minimum and maximum
            for (int i = 0; i < nodes.Count; i++)
            {
                Node node = nodes[i];
                for (int p = 0; p < model.horizonDays; p++)
                {
                    if (node.endDayInventory[p] > node.stockMaximumLevel)
                    {
                        GlobalUtils.writeToConsole(String.Format("Node {1} maximum stock exceeded for period {0}", p, node.ID));
                        return INFEASIBILITY_STATUS.CUSTOMER_MAXSTOCK;
                    }
                    if (node.endDayInventory[p] < 0.0)
                    {
                        GlobalUtils.writeToConsole(String.Format("Node {1} stock out for period {0}", p, node.ID));
                        return INFEASIBILITY_STATUS.CUSTOMER_STOCKOUT;
                    }
                }
            }

            // 4. Depot inventory holding costs
            double depotInventoryObjective = 0;
            int depotStock = depot.startingInventory;
            depot.startingInventory = depotStock;
            for (int p = 0; p < periods.Count; p++)
            {
                depot.startDayInventory[p] = depotStock;
                depotStock += depot.productionRates[p];

                int outboundFromCustomers = totalOutboundFlowOfPeriodFromCustomers(p);
                int outboundFromRoutes = totalOutboundFlowOfPeriodFromRoutes(p);
                if (outboundFromCustomers != outboundFromRoutes)
                {
                    GlobalUtils.writeToConsole(String.Format("Outbound Flow Mismatch on period {0}: calculated from routes {1}, " +
                        "calculated from customers {2}", p, outboundFromRoutes, outboundFromCustomers));
                    return INFEASIBILITY_STATUS.FLOW_MISMATCH;
                }

                depotStock = depotStock - outboundFromCustomers;
                depot.endDayInventory[p] = depotStock;
                depotInventoryObjective += (depotStock * depot.unitHoldingCost);
            }
            depot.totalHoldingCost = depotInventoryObjective;

            double solutionInventoryObjective = depotInventoryObjective + AllCustomersInventoryCost;
            if (!GlobalUtils.IsEqual(solutionInventoryObjective, holdingCost))
            {
                GlobalUtils.writeToConsole(String.Format("Mismatch of total inventory cost: Solution {0} diff. calculated from scratch {1}", holdingCost, solutionInventoryObjective));
                return INFEASIBILITY_STATUS.MISCALC_OBJ_HOLDING_COST;
            }

            // 8. Periodicity check for depot and customers
            for (int i = 0; i < nodes.Count; i++)
            {
                Node node = nodes[i];
                if (node.startingInventory != node.endDayInventory[node.endDayInventory.Length-1])
                {
                    GlobalUtils.writeToConsole(String.Format("Node {0}: end day inventory  {1} != starting inventory {2}", node.uid, node.endDayInventory[node.endDayInventory.Length - 1], node.startingInventory));
                    return INFEASIBILITY_STATUS.PERIODICITY_VIOLATION;
                }
            }


            // 7. Total objective 
            double totalObjectiveLocal = solutionInventoryObjective + totalRoutingCostLocal + setupProductionCost + unitProductionCostLocal;
            if (!GlobalUtils.IsEqual(totalObjectiveLocal, totalObjective))
            {
                GlobalUtils.writeToConsole(String.Format("Mismatch of total objective cost: Solution {0} diff. calculated from scratch {1}", totalObjective, totalObjectiveLocal));
                return INFEASIBILITY_STATUS.MISCALC_OBJ_GENERIC;
            }



            //test load After me list
            for (int p = 0; p < periods.Count; p++)
            {
                Period per = periods[p];

                for (int r = 0; r < per.periodRoutes.Count; r++)
                {
                    Route rt = per.periodRoutes[r];
                    double rtLoadTillMe = 0;
                    for (int n = 0; n < rt.nodes.Count; n++)
                    {
                        Node node = rt.nodes[n];
                        if (node.uid != 0)
                        {
                            rtLoadTillMe += node.deliveredQuantities[p];
                        }

                        if (!GlobalUtils.IsEqual(rtLoadTillMe, rt.loadTillMe[n]))
                        {
                            GlobalUtils.writeToConsole(String.Format("Mismatch of load till me {0} and calculated from scratch {1} for customer {2} at period {3} at route {4}",
                                rt.loadTillMe[n], rtLoadTillMe, n, p, r));
                            return INFEASIBILITY_STATUS.ROUTE_GENERIC_VIOLATION;
                        }
                    }
                }
            }

            //calculate total flows for routes
            for (int p = 0; p < periods.Count; p++)
            {
                Period per = periods[p];
                per.totalOutboundProductFlow = 0;
                for (int r = 0; r < per.periodRoutes.Count; r++)
                {
                    Route rt = per.periodRoutes[r];
                    double rtLoad = 0;
                    for (int n = 1; n < rt.nodes.Count - 1; n++)
                    {
                        rtLoad += rt.nodes[n].deliveredQuantities[p];
                    }

                    if (!GlobalUtils.IsEqual(rtLoad, rt.load))
                    {
                        GlobalUtils.writeToConsole(String.Format("Mismatch of load for route {0}: solution {1} diff. calculated {2}", rt, rt.load, rtLoad));
                        return INFEASIBILITY_STATUS.ROUTE_GENERIC_VIOLATION;
                    }

                    if (rt.load > rt.effectiveCapacity)
                    {
                        GlobalUtils.writeToConsole(String.Format("Hard Vehicle capacity violation for period {0} in route {1} with load {2} and capacity {3}",
                            p, rt, rt.load, rt.effectiveCapacity));
                        return INFEASIBILITY_STATUS.ROUTE_LOAD_HARDVIOLATION;
                    }
                    else if (rt.load > rt.realCapacity)
                    {
                        //GlobalUtils.writeToConsole(String.Format("Soft Vehicle capacity violation for period {0} in route {1} with load {2} and capacity {3}",
                        //    p, rt, rt.load, rt.realCapacity));
                        return INFEASIBILITY_STATUS.ROUTE_LOAD_SOFTVIOLATION;
                    }
                    per.totalOutboundProductFlow += rtLoad;
                }
            }

            //check that the total outbound quantity per period can be satisfied by depot production
            for (int p = 0; p < periods.Count; p++)
            {
                //Note: this could be allowed if feasbile solution are not possible
                if (periods[p].totalOutboundProductFlow > depot.startDayInventory[p] + depot.productionRates[p])
                {
                    GlobalUtils.writeToConsole("Production is not sufficient for period " + p);
                    GlobalUtils.writeToConsole("{0} vs {1}", periods[p].totalOutboundProductFlow,
                        depot.startDayInventory[p] + depot.productionRates[p]);
                    depot.ReportInventory();
                    return INFEASIBILITY_STATUS.INFEASIBLE;
                }
            }

            return INFEASIBILITY_STATUS.FEASIBLE;
        }

        public INFEASIBILITY_STATUS TestEverythingFromScratch()
        {
            //WARNING: this method does not re-initialize the delivered quantities now.
            
            // 1. Calculate Production Cost (unit and setup) from Scratch
            double unitProductionCostLocal = 0;
            double setupProductionCostLocal = 0;
            for (int p = 0; p < periods.Count; p++)
            {
                unitProductionCostLocal += depot.productionRates[p] * depot.unitProductionCost;
                if (depot.open[p])
                    setupProductionCostLocal += depot.productionSetupCost;
            }
            if (!GlobalUtils.IsEqual(setupProductionCost, setupProductionCostLocal))
            {
                GlobalUtils.writeToConsole(String.Format("Mismatch of production setup cost: Solution {0} diff. calculated from scratch {1}", setupProductionCost, setupProductionCostLocal));
                return INFEASIBILITY_STATUS.MISCALC_OBJ_PRODUCTION_COST;
            }
            if (!GlobalUtils.IsEqual(totalUnitProductionCost, unitProductionCostLocal))
            {
                GlobalUtils.writeToConsole(String.Format("Mismatch of production unit cost: Solution {0} diff. calculated from scratch {1}", totalUnitProductionCost, unitProductionCostLocal));
                return INFEASIBILITY_STATUS.MISCALC_OBJ_PRODUCTION_COST;
            }

            // 2. Routing Issues
            double totalRoutingCostLocal = 0;
            for (int p = 0; p < periods.Count; p++)
            {
                Period per = periods[p];
                
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
                        GlobalUtils.writeToConsole(String.Format("Mismatch of routing cost of route {0} between solution {1} and calculated {2}",
                            rt.ToString(), rt.totalRoutingCost, rtCost));
                        return INFEASIBILITY_STATUS.MISCALC_OBJ_ROUTING_COST;
                    }
                    if (rt.nodes.Count != 0)
                    {

                        if (rt.nodes[0].uid != 0 || rt.nodes[rt.nodes.Count - 1].uid != 0)
                        {
                            GlobalUtils.writeToConsole(String.Format("Not depot in start or end of route {0}", rt.ToString()));
                            return INFEASIBILITY_STATUS.INFEASIBLE;
                        }
                        for (int n = 1; n < rt.nodes.Count - 1; n++)
                        {
                            Node cus = rt.nodes[n];
                            if (cus.uid == 0)
                            {
                                GlobalUtils.writeToConsole("Depot in the middle");
                                return INFEASIBILITY_STATUS.INFEASIBLE;
                            }
                        }
                    }
                    totalRoutingCostLocal += rtCost;
                }
            }
            if (!GlobalUtils.IsEqual(totalRoutingCostLocal, routingCost))
            {
                GlobalUtils.writeToConsole(String.Format("Mismatch of routing cost: Solution {0} diff. calculated from scratch {1}", routingCost, totalRoutingCostLocal));
                return INFEASIBILITY_STATUS.MISCALC_OBJ_ROUTING_COST;
            }

            
            //Check the visit schedule and the consistency of the delivery services
            for (int i = 0; i < customers.Count; i++)
            {
                Node cust = customers[i];

                for (int p = 0; p < model.horizonDays; p++)
                {
                    if (cust.horizonDeliveryServices[p].route != null && cust.visitSchedule[p] == false)
                    {
                        GlobalUtils.writeToConsole("Inconsistent visit schedule and horizon deliveries of customer {0}",
                            cust.uid);
                        return INFEASIBILITY_STATUS.CUSTOMER_INCONSISTENT_VISITSCHEDULE;
                    }

                    if (cust.horizonDeliveryServices[p].quantity > 0 && cust.visitSchedule[p] == false)
                    {
                        GlobalUtils.writeToConsole("Inconsistent visit schedule and horizon delivery quantity of customer {0}",
                            cust.uid);
                        return INFEASIBILITY_STATUS.CUSTOMER_INCONSISTENT_DELIVERY;
                    }

                    if (cust.visitSchedule[p])
                    {
                        if (cust.horizonDeliveryServices[p] == null)
                        {
                            GlobalUtils.writeToConsole("Active visit day but null delivery service on customer {0}", cust.uid);
                            return INFEASIBILITY_STATUS.CUSTOMER_INCONSISTENT_VISITSCHEDULE;
                        }

                        if (cust.horizonDeliveryServices[p].route == null)
                        {
                            GlobalUtils.writeToConsole(
                                "Active visit day, valid delivery service but null route on delivery service");
                            return INFEASIBILITY_STATUS.CUSTOMER_INCONSISTENT_VISITSCHEDULE;
                        }

                        if (cust.horizonDeliveryServices[p].quantity != cust.deliveredQuantities[p])
                        {
                            GlobalUtils.writeToConsole(
                                "Quantity mismatch between the delivery service and the cached delivered quantity. Customer {0}, Period {1}",
                                cust.uid, p);
                            return INFEASIBILITY_STATUS.CUSTOMER_INCONSISTENT_DELIVERY;
                        }

                    }
                }
            }

            //For the days that horizon delivery is zero, make sure the customer is not present in this day routes
            //Note: this should be allowed for parsing the production schedules
            for (int i = 0; i < customers.Count; i++)
            {
                Node customer = customers[i];
                for (int p = 0; p < periods.Count; p++)
                {
                    int countsInRoutes = occurencesInRoutesOfPeriod(customer, periods[p]);

                    if (countsInRoutes > 1)
                    {
                        GlobalUtils.writeToConsole("Customer is visited multiple times on the same period");
                        return INFEASIBILITY_STATUS.INFEASIBLE;
                    }
                        
                    if (customer.horizonDeliveryServices[p].route != null)
                    {
                        if (countsInRoutes != 1 || !customer.visitSchedule[p])
                        {
                            GlobalUtils.writeToConsole(String.Format("Customer {0} has been delivered quantity {1} without visit in period {2}",
                                customer.ID, customer.deliveredQuantities[p], p));
                            customer.ReportInventory();
                            return INFEASIBILITY_STATUS.INFEASIBLE;
                        }
                    }
                }
            }

            // 3. Customer inventory holding costs
            double AllCustomersInventoryCost = 0;
            for (int i = 0; i < customers.Count; i++)
            {
                double customerInventoryCost = 0;
                Node customer = customers[i];
                
                customer.CalculateInventoryLevels();
                
                double stock = customer.startingInventory;
                for (int p = 0; p < model.horizonDays; p++)
                {
                    customerInventoryCost += (customer.endDayInventory[p] * customer.unitHoldingCost);
                    if (customer.deliveredQuantities[p] < 0)
                    {
                        GlobalUtils.writeToConsole(String.Format("Non positive delivery for period {0} and customer {1}", p, customer.ID));
                        return INFEASIBILITY_STATUS.INFEASIBLE;
                    }
                }
                customer.totalHoldingCost = customerInventoryCost;

                AllCustomersInventoryCost += customer.totalHoldingCost;
                
            }

            //remaining stock for each customer
            for (int i = 0; i < customers.Count; i++)
            {
                Node customer = customers[i];
                double stockEndOfDay = customer.startingInventory;
                for (int p = 0; p < periods.Count; p++)
                {
                    stockEndOfDay -= customer.productRate[p];
                    stockEndOfDay += customer.deliveredQuantities[p];
                    
                    if (!GlobalUtils.IsEqual(stockEndOfDay, customer.endDayInventory[p]))
                    {
                        GlobalUtils.writeToConsole(String.Format("Mismatch of solution stock {0} and calculated from scratch {1} for customer {2}",
                            customer.endDayInventory[p], stockEndOfDay, customer.ID));
                        return INFEASIBILITY_STATUS.MISCALC_CUSTOMER_INVENTORY;
                    }
                    if (stockEndOfDay < 0 && Math.Abs(stockEndOfDay) > 1e-5)
                    {
                        GlobalUtils.writeToConsole(String.Format("Customer stock out for period {0} and customer {1}", p, customer.ID));
                        return INFEASIBILITY_STATUS.CUSTOMER_STOCKOUT;
                    }
                }
            }

            // 6. Inventory minimum and maximum
            for (int i = 0; i < nodes.Count; i++)
            {
                Node node = nodes[i];
                for (int p = 0; p < model.horizonDays; p++)
                {
                    if (node.endDayInventory[p] > node.stockMaximumLevel)
                    {
                        GlobalUtils.writeToConsole(String.Format("Node {1} maximum stock exceeded for period {0}", p, node.ID));
                        return INFEASIBILITY_STATUS.CUSTOMER_MAXSTOCK;
                    }
                    if (node.endDayInventory[p] < 0.0)
                    {
                        GlobalUtils.writeToConsole(String.Format("Node {1} stock out for period {0}", p, node.ID));
                        return INFEASIBILITY_STATUS.CUSTOMER_STOCKOUT;
                    }
                }
            }

            // 4. Depot inventory holding costs
            double depotInventoryObjective = 0;
            int depotStock = depot.startingInventory;
            depot.startingInventory = depotStock;
            for (int p = 0; p < periods.Count; p++)
            {
                depot.startDayInventory[p] = depotStock;
                depotStock += depot.productionRates[p];

                int outboundFromCustomers = totalOutboundFlowOfPeriodFromCustomers(p);
                int outboundFromRoutes = totalOutboundFlowOfPeriodFromRoutes(p);
                if (outboundFromCustomers != outboundFromRoutes)
                {
                    GlobalUtils.writeToConsole(String.Format("Outbound Flow Mismatch on period {0}: calculated from routes {1}, " +
                        "calculated from customers {2}", p, outboundFromRoutes, outboundFromCustomers));
                    return INFEASIBILITY_STATUS.FLOW_MISMATCH;
                }

                depotStock = depotStock - outboundFromCustomers;
                depot.endDayInventory[p] = depotStock;
                depotInventoryObjective += (depotStock * depot.unitHoldingCost);
            }
            depot.totalHoldingCost = depotInventoryObjective;

            double solutionInventoryObjective = depotInventoryObjective + AllCustomersInventoryCost;
            if (!GlobalUtils.IsEqual(solutionInventoryObjective, holdingCost))
            {
                GlobalUtils.writeToConsole(String.Format("Mismatch of total inventory cost: Solution {0} diff. calculated from scratch {1}", holdingCost, solutionInventoryObjective));
                return INFEASIBILITY_STATUS.MISCALC_OBJ_HOLDING_COST;
            }

            
            
            // 7. Total objective 
            double totalObjectiveLocal = solutionInventoryObjective + totalRoutingCostLocal + setupProductionCost + unitProductionCostLocal;
            if (!GlobalUtils.IsEqual(totalObjectiveLocal, totalObjective))
            {
                GlobalUtils.writeToConsole(String.Format("Mismatch of total objective cost: Solution {0} diff. calculated from scratch {1}", totalObjective, totalObjectiveLocal));
                return INFEASIBILITY_STATUS.MISCALC_OBJ_GENERIC;
            }

            

            //test load After me list
            for (int p = 0; p < periods.Count; p++)
            {
                Period per = periods[p];

                for (int r = 0; r < per.periodRoutes.Count; r++)
                {
                    Route rt = per.periodRoutes[r];
                    double rtLoadTillMe = 0;
                    for (int n = 0; n < rt.nodes.Count; n++)
                    {
                        Node node = rt.nodes[n];
                        if (node.uid != 0)
                        {
                            rtLoadTillMe += node.deliveredQuantities[p];
                        }

                        if (!GlobalUtils.IsEqual(rtLoadTillMe, rt.loadTillMe[n]))
                        {
                            GlobalUtils.writeToConsole(String.Format("Mismatch of load till me {0} and calculated from scratch {1} for customer {2} at period {3} at route {4}",
                                rt.loadTillMe[n], rtLoadTillMe, n, p, r));
                            return INFEASIBILITY_STATUS.ROUTE_GENERIC_VIOLATION;
                        }
                    }
                }
            }          
            
            //calculate total flows for routes
            for (int p = 0; p < periods.Count; p++)
            {
                Period per = periods[p];
                per.totalOutboundProductFlow = 0;
                for (int r = 0; r < per.periodRoutes.Count; r++)
                {
                    Route rt = per.periodRoutes[r];
                    double rtLoad = 0;
                    for (int n = 1; n < rt.nodes.Count - 1; n++)
                    {
                        rtLoad += rt.nodes[n].deliveredQuantities[p];
                    }

                    if (!GlobalUtils.IsEqual(rtLoad, rt.load))
                    {
                        GlobalUtils.writeToConsole(String.Format("Mismatch of load for route {0}: solution {1} diff. calculated {2}", rt, rt.load, rtLoad));
                        return INFEASIBILITY_STATUS.ROUTE_GENERIC_VIOLATION;
                    }

                    if (rt.load > rt.effectiveCapacity)
                    {
                        GlobalUtils.writeToConsole(String.Format("Hard Vehicle capacity violation for period {0} in route {1} with load {2} and capacity {3}",
                            p, rt, rt.load, rt.effectiveCapacity));
                        return INFEASIBILITY_STATUS.ROUTE_LOAD_HARDVIOLATION;
                    } else if (rt.load > rt.realCapacity)
                    {
                        //GlobalUtils.writeToConsole(String.Format("Soft Vehicle capacity violation for period {0} in route {1} with load {2} and capacity {3}",
                        //    p, rt, rt.load, rt.realCapacity));
                        return INFEASIBILITY_STATUS.ROUTE_LOAD_SOFTVIOLATION;
                    }
                    per.totalOutboundProductFlow += rtLoad;
                }
            }

            //check that the total outbound quantity per period can be satisfied by depot production
            for (int p = 0; p < periods.Count; p++)
            {
                //Note: this could be allowed if feasbile solution are not possible
                if (periods[p].totalOutboundProductFlow > depot.startDayInventory[p] + depot.productionRates[p])
                {
                    GlobalUtils.writeToConsole("Production is not sufficient for period " + p);
                    GlobalUtils.writeToConsole("{0} vs {1}", periods[p].totalOutboundProductFlow,
                        depot.startDayInventory[p] + depot.productionRates[p]);
                    depot.ReportInventory();
                    return INFEASIBILITY_STATUS.INFEASIBLE;
                }
            }

            return INFEASIBILITY_STATUS.FEASIBLE;
        }
        
        public int totalOutboundFlowOfPeriodFromRoutes(int p)
        {
            int tot = 0;
            for (int i = 0; i < periods[p].periodRoutes.Count; i++)
            {
                tot += periods[p].periodRoutes[i].load;
            }
            return tot;
        }

        public int totalOutboundFlowOfPeriodFromCustomers(int p)
        {
            int tot = 0;
            for (int i = 0; i < customers.Count; i++)
            {
                Node cust = customers[i];
                tot += cust.deliveredQuantities[p];
            }
            return tot;
        }

        
        public override string ToString()
        {
            string output = "";
            output += "Total_Objective: " + totalObjective + "\n";
            output += "Total_Routing_Objective: " + routingCost+ "\n";
            output += "Total_Inventory_Objective: " + holdingCost+ "\n";
            output += "Total_Production_Objective: " + totalUnitProductionCost + "\n";
            output += "Total_Production_Setup_Objective: " + setupProductionCost + "\n";
            output += "Total_Inventory_Objective_(incl.firstDayInvCost): " + totalObjectiveIncFirstPeriodInventory+ "\n";
            output += "\n";

            output += "Total periods: " + periods.Count+ "\n";
            output += "Total routes per period: " + periods[0].periodRoutes.Count+ "\n";
            output += "\n";

            for (int i = 0; i < periods.Count; i++)
            {
                output += "Period_" + (i+1) + "_Routes:"+ "\n";
                Period per = periods[i];

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
            for (int i = 0; i < periods.Count; i++)
            {
                output += "Period_" + (i+1) + "_production_quantity:\n";
                output += depot.productionRates[i] + "\n";
            }
            
            output += "\n\n";
            
            //Export Delivered quantities
            
            for (int i = 0; i < periods.Count; i++)
            {
                output += "Period_" + (i+1) + "_delivered_quantity:\n";
                string node_s = "";
                string node_q_s = "";
                
                for (int j = 1; j < nodes.Count; j++)
                {
                    Node n = nodes[j];
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
            
            return output;
        }

        public string OldToString()
        {
            string output = "";
            output += "Total_Objective: " + totalObjective + "\n";
            output += "Total_Routing_Objective: " + routingCost + "\n";
            output += "Total_Inventory_Objective: " + holdingCost + "\n";
            output += "Total_Production_Objective: " + totalUnitProductionCost + "\n";
            output += "Total_Production_Setup_Objective: " + setupProductionCost + "\n";
            output += "Total_Inventory_Objective_(incl.firstDayInvCost): " + totalObjectiveIncFirstPeriodInventory + "\n";
            output += "\n";

            output += "Total periods: " + periods.Count + "\n";
            output += "Total routes per period: " + periods[0].periodRoutes.Count + "\n";
            output += "\n";

            for (int i = 0; i < periods.Count; i++)
            {
                output += "Period_" + (i + 1) + "_Routes:" + "\n";
                Period per = periods[i];

                for (int r = 0; r < per.periodRoutes.Count; r++)
                {
                    output += "-Route_" + r + " : ";
                    Route rt = per.periodRoutes[r];
                    for (int n = 0; n < rt.nodes.Count; n++)
                    {
                        Node nd = rt.nodes[n];
                        output += nd.ID + " ";
                    }

                    output += "\n";
                }

                output += "\n";
            }

            output += "\n\n";

            //Export production quantities
            for (int i = 0; i < periods.Count; i++)
            {
                output += "Period_" + (i + 1) + "_production_quantity:\n";
                output += depot.productionRates[i] + "\n";
            }

            output += "\n\n";

            //Export Delivered quantities

            for (int i = 0; i < periods.Count; i++)
            {
                output += "Period_" + (i + 1) + "_delivered_quantity:\n";
                string node_s = "";
                string node_q_s = "";

                for (int j = 1; j < nodes.Count; j++)
                {
                    Node n = nodes[j];
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

            return output;
        }


        public void SaveToFile(string filename)
        {
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
            
            SW.Write(this.ToString());
            SW.Close();
        }


        /*
         * Russell
         */
        public void ImportFromFile(string filename, bool allow_infeasible = false)
        {
            FileInfo src = new FileInfo(filename);
            TextReader reader = src.OpenText();
            String str;
            char[] seperator = new char[2] { ' ', '\t' };
			List<String> data;
            
            
            //Skip objectives, they will be re-evaluated later
            //Also skip periods and routes per period (solution should have been properly initialized
            for (int i=0;i<10;i++)
                reader.ReadLine();
            
            //Clear customer delivered quantities and visit Schedule
            for (int i = 0; i < customers.Count; i++)
            {
                Node cust = customers[i];
                for (int j = 0; j < model.horizonDays; j++)
                {
                    depot.deliveredQuantities[j] = 0;
                    cust.visitSchedule[j] = false;
                    cust.deliveredQuantities[j] = 0;
                    cust.horizonDeliveryServices[j] = new CustDelivery(0, null);
                }
            }
            
            //Parse routes
            for (int i = 0; i < periods.Count; i++)
            {
                Period pr = periods[i];
                pr.periodRoutes.Clear();
                pr.vehicleNum = 0;
                reader.ReadLine();
                while(true)
                {
                    str = reader.ReadLine();
                    if (str == "")
                        break;
                    List<String> routeData = GlobalUtils.SeperateStringIntoSubstrings(':', str);
                    //Add route
                    int route_id = pr.vehicleNum;
                    pr.appendRoute(model, depot);
                    Route rt = pr.periodRoutes[route_id];

                    if (allow_infeasible)
                        rt.effectiveCapacity = (int) (1.03 * rt.realCapacity);
                    
                    List<String> nodeData = GlobalUtils.SeperateStringIntoSubstrings(' ', routeData[1]);
                    //Add nodes    
                    for (int k = 1; k < nodeData.Count - 1; k++)
                    {
                        int cust_id = Int32.Parse(nodeData[k]) - 1;
                        Node cust = nodes[cust_id];
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
                int q = (int) Math.Round(double.Parse(data.Last()));
                depot.productionRates[i] = 0;
                depot.open[i] = false;
                
                if (q > 0.0)
                    depot.open[i] = true;
                depot.productionRates[i] = q;
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
                        Node cust = nodes[custId];
                        int q = (int) Math.Round(double.Parse(data[j])); //Load quantity
                        cust.deliveredQuantities[i] = q;
                        cust.horizonDeliveryServices[i].quantity = q;
                        cust.horizonDeliveryServices[i].route.load += q;
                        depot.deliveredQuantities[i] -= q;
                    }
                }
            }
            
            //Recalculate inventory levels for the depot and the customers
            depot.CalculateInventoryLevels();
            
            for (int i=0;i<customers.Count;i++)
                customers[i].CalculateInventoryLevels();

            //Recalculate loads for all routes
            for (int i = 0; i < model.horizonDays; i++)
            {
                Period pr = periods[i];
                for (int j = 0; j < pr.periodRoutes.Count; j++)
                    pr.periodRoutes[j].SetLoadAndCostLists(i, model);
            }
            
            //Re-evaluate objectives
            routingCost = Solution.EvaluateRoutingObjectivefromScratch(this);
            holdingCost = Solution.EvaluateInventoryObjectivefromScratch(this);
            totalUnitProductionCost = Solution.EvaluateUnitProductionObjectivefromScratch(this);
            setupProductionCost = Solution.EvaluateSetupProductionObjectivefromScratch(this);

            totalObjective = routingCost + holdingCost + totalUnitProductionCost +
                                      setupProductionCost;


            if (!Solution.checkSolutionStatus(TestEverythingFromScratch()))
            {
                GlobalUtils.writeToConsole("Solution failed to import properly");
            }

            this.SaveToFile("russel_exported_from_us.txt");

        }
        
    }
}
