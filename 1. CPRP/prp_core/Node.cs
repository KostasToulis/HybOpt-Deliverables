using System;
using System.Collections.Generic;
using System.Data;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Transactions;
using System.Windows.Markup;

namespace PRP
{
    public class Node
    {
        //Input
        public double unitHoldingCost;
        public int[] productRate;
        public int[] productRateSigned;
        public int totalDemand;
        public int stockMinimumLevel;
        public int stockMaximumLevel;
        public int startingInventory;
        public int minimumRequiredVisits; // the lower bound of the number of visits required to service the customer
        public double x_coord;
        public double y_coord;
        public string ID;
        public int uid;
        public readonly PRP model;
        private int horizonDays;
        public bool sawmode = false;

        
        //Model Parameters
        public int[] startDayInventory; //Old stockRemBefDelivery
        public int[] endDayInventory; //Old remainingStock
        public int[] deliveredQuantities; //Old horizonDeliveries
        public int[] demandedQuantities;

        public double cyclicStartInv;


        public List<int> daysTillStockOut;
        public bool stocksOut;
        public double totalHoldingCost;

        public CustDelivery[] horizonDeliveryServices;
        public CustDelivery[] auxiliary_horizonDeliveryServices;
        public bool[] visitSchedule;
        public double[] minimumStockTillNextDelivery_StartInc;
        public double[,] minimumStockDaysAhead_StartInc;
        public int[] nextDeliveryDay;
        public int[] prevDeliveryDay;
        public int[] daysTillNextDeliveryOrEnd;
        public int[] daysTillPreviousDeliveryOrStart;
        public bool[] auxiliary_visitSchedule;
        public double[] auxiliary_RemainingStock;
        public int[] auxiliary_deliveries;
        public double[] auxiliary_SignedFlowDiffs_Depot;
        public int[] auxiliary_endDayInventory;
        public int[] auxiliary_startDayInventory;

        
        public Node (int dayHorizon)
        {
            horizonDays = dayHorizon; //Keep a local copy of the horizonDays
            ListReset();
        }

        public Node()
        {
           
        }

        public override string ToString()
        {
            return string.Format("{0}", uid);
        }

        public void ListReset()
        {
            startDayInventory = new int[horizonDays];
            endDayInventory = new int[horizonDays];
            deliveredQuantities = new int[horizonDays];
            demandedQuantities = new int[horizonDays];
            productRate = new int[horizonDays];
            productRateSigned = new int[horizonDays];

            nextDeliveryDay = new int[horizonDays];
            prevDeliveryDay = new int[horizonDays];
            visitSchedule = new bool[horizonDays];
            auxiliary_visitSchedule = new bool[horizonDays];
            
            horizonDeliveryServices = new CustDelivery[horizonDays];
            auxiliary_horizonDeliveryServices = new CustDelivery[horizonDays];
            auxiliary_RemainingStock = new double[horizonDays];
            auxiliary_deliveries = new int[horizonDays];
            auxiliary_endDayInventory = new int[horizonDays];
            auxiliary_startDayInventory = new int[horizonDays];
            
            minimumStockDaysAhead_StartInc = new double[horizonDays, horizonDays];
            minimumStockTillNextDelivery_StartInc = new double[horizonDays];
            daysTillNextDeliveryOrEnd = new int[horizonDays];
            daysTillPreviousDeliveryOrStart = new int[horizonDays];
            
            daysTillStockOut = new List<int>(horizonDays);
            stocksOut = false;
            
            //Fill horizonDeliveryServices
            for (int i = 0; i < horizonDays; i++)
            {
                horizonDeliveryServices[i] = new CustDelivery();
                auxiliary_horizonDeliveryServices[i] = new CustDelivery();
            }
        }

        public void saveTempState()
        {
            auxiliary_deliveries.CopyTo(deliveredQuantities, 0);
            auxiliary_visitSchedule.CopyTo(visitSchedule, 0);
            auxiliary_endDayInventory.CopyTo(endDayInventory, 0);
            auxiliary_startDayInventory.CopyTo(startDayInventory,0 );
            copyHorizonDeliveries(auxiliary_horizonDeliveryServices, horizonDeliveryServices, horizonDays);
            
            //Check state
            for (int i = 0; i < horizonDays; i++)
            {
                if (auxiliary_horizonDeliveryServices[i] != null && horizonDeliveryServices[i] != null)
                {
                    if (auxiliary_horizonDeliveryServices[i].route != horizonDeliveryServices[i].route)
                        GlobalUtils.writeToConsole("Route copy error");
                    
                    if (auxiliary_horizonDeliveryServices[i].quantity != horizonDeliveryServices[i].quantity)
                        GlobalUtils.writeToConsole("Route quantity copy error");
                    
                    if (horizonDeliveryServices[i].quantity != deliveredQuantities[i])
                        GlobalUtils.writeToConsole("Quantity consistency error");
                }
            }
            
        }

        public Node(Node nd)
        {
            unitHoldingCost = nd.unitHoldingCost;
            totalDemand = nd.totalDemand;
            stockMinimumLevel = nd.stockMinimumLevel;
            stockMaximumLevel = nd.stockMaximumLevel;
            startingInventory = nd.startingInventory;
            x_coord = nd.x_coord;
            y_coord = nd.y_coord;
            ID = nd.ID;
            uid = nd.uid;
            sawmode = nd.sawmode;
            horizonDays = nd.horizonDays;


            cyclicStartInv = nd.cyclicStartInv;

            //Model Parameters
            ListReset();
            
            nd.startDayInventory.CopyTo(startDayInventory, 0);
            nd.endDayInventory.CopyTo(endDayInventory, 0);
            nd.deliveredQuantities.CopyTo(deliveredQuantities, 0);
            nd.demandedQuantities.CopyTo(demandedQuantities, 0);
            
            nd.nextDeliveryDay.CopyTo(nextDeliveryDay, 0);
            nd.prevDeliveryDay.CopyTo(prevDeliveryDay, 0);
            nd.visitSchedule.CopyTo(visitSchedule, 0);
            
            //Are these copied all over or they are just the same refs?
            nd.auxiliary_RemainingStock.CopyTo(auxiliary_RemainingStock, 0);
            nd.auxiliary_deliveries.CopyTo(auxiliary_deliveries, 0);
            nd.auxiliary_endDayInventory.CopyTo(auxiliary_endDayInventory, 0);
            nd.auxiliary_startDayInventory.CopyTo(auxiliary_startDayInventory, 0);

            minimumStockDaysAhead_StartInc = nd.minimumStockDaysAhead_StartInc.Clone() as double[,];
            nd.minimumStockTillNextDelivery_StartInc.CopyTo(minimumStockTillNextDelivery_StartInc, 0);
            nd.daysTillNextDeliveryOrEnd.CopyTo(daysTillNextDeliveryOrEnd, 0);
            nd.daysTillPreviousDeliveryOrStart.CopyTo(daysTillPreviousDeliveryOrStart, 0);
            
            nd.productRate.CopyTo(productRate, 0);
            nd.productRateSigned.CopyTo(productRateSigned, 0);
            
            daysTillStockOut = new List<int>(nd.daysTillStockOut);
            stocksOut = nd.stocksOut;
            totalHoldingCost = nd.totalHoldingCost;


        //Manually handle horizonDelivery Services
        copyHorizonDeliveries(nd.horizonDeliveryServices, horizonDeliveryServices, horizonDays);
        }

        public static void copyHorizonDeliveries(CustDelivery[] from, CustDelivery[] to, int N)
        {
            for (int i = 0; i < N; i++)
            {
                to[i].quantity = from[i].quantity;
                to[i].route = from[i].route;
            }
        }
        
        public virtual void InitializeInventoryLevels()
        {
            deliveredQuantities = new int[horizonDays]; //Reset Deliveries

            CalculateInventoryLevels();
        }

        public virtual void CalculateInventoryLevels()
        {
            endDayInventory = new int[horizonDays];
            
            int stock = startingInventory; //Init to start inventory
            for (int i = 0; i < horizonDays; i++) 
            {
                startDayInventory[i] = stock;
                stock -= productRate[i]; //Subtracts production
                stock += deliveredQuantities[i]; //Adds delivery
                endDayInventory[i] = stock;
            }
        }

        public double CalculateHoldingCost(int dayIndex)
        {
            return CalculateHoldingCost(0, dayIndex);
        }
        
        public double CalculateHoldingCost(double deliveredQuantity, int dayIndex)
        {
            double holdCost = 0;
            for (int d = dayIndex; d < horizonDays; d++)
            {
                double stock = endDayInventory[d] + deliveredQuantity;
                double actualStock = Math.Max(stock, 0); //Ceil to 0 in case of a stock out
                holdCost += (actualStock * unitHoldingCost);
            }
            
            return holdCost;
        }

        public void CalculateTotalDemand()
        {
            totalDemand = 0;
            for (int i=0; i < horizonDays; i++)
            {
                totalDemand += productRate[i];
            }
        }


        public bool[] getVisitSchedule()
        {
            //TODO: Return cached visitSchedule in the future
            bool[] nVisitSchedule = new bool[horizonDays];

            for (int i = 0; i < horizonDays; i++)
                nVisitSchedule[i] = horizonDeliveryServices[i].route != null;
            return nVisitSchedule;
        }

        public void CalculateVisitDays(bool[] visitSchedule)
        {
            //Set visit days to node
            //TODO: Find a better way to do this
            int old_pos = 0;
            //Init next/prev delivery days
            for (int j = 0; j < horizonDays; j++)
            {
                nextDeliveryDay[j] = -1;
                prevDeliveryDay[j] = -1;
            }
                    
            for (int j = 0; j < horizonDays; j++)
            {
                if (horizonDeliveryServices[j].route != null)
                {
                    for (int k = old_pos; k < j; k++)
                    {
                        nextDeliveryDay[k] = j;
                        old_pos++;
                    }                                
                }
            }

            old_pos = horizonDays - 1;
            for (int j = horizonDays -1; j >=0; j--)
            {
                if (horizonDeliveryServices[j].route != null)
                {
                    for (int k = old_pos; k > j; k--)
                    {
                        prevDeliveryDay[k] = j;
                        old_pos--;
                    }                                
                }
            }
        
        }

        public bool ApplySaw(bool[] visitSchedule)
        {
            //New calculated delivery and end quantities are saved in auxiliary arrays
            
            int stock = startingInventory;
            bool feasible = true;

            for (int j = 0; j < horizonDays; j++)
            {
                int days_until_next_delivery = 0;
                //Findout the necessary delivered quantity to cover the demand until the next delivery
                if (visitSchedule[j])
                {
                    days_until_next_delivery += 1; //Add current day
                    //TODO: we can use an ordered list of visits to avoid for in order to find next visit
                    for (int k = j + 1; k < horizonDays; k++) 
                        if (visitSchedule[k])
                            break;
                        else
                            days_until_next_delivery++;
                }
                else
                    days_until_next_delivery = 0;
                    
                //Calculate delivery quantity
                int deliveredQuantity = 0;
                for (int k = j; k < j + days_until_next_delivery; k++)
                    deliveredQuantity += productRate[k];
                deliveredQuantity = Math.Max(deliveredQuantity - stock, 0);
                auxiliary_deliveries[j] = deliveredQuantity;

                //Make sure that calculated delivered quantities where a visit is issued are not eliminated
                if (visitSchedule[j] && (GlobalUtils.IsEqual(deliveredQuantity, 0)))
                    return false;
                
                //we should check if the stock respects the maximum capacity level and 
                //if strict version is solved if the the quantity delivered respects the warehouse capacity
                //Check max capacity violation on previous visit day
#if RESPECT_CUSTOMER_CAPACITY
                if (auxiliary_deliveries[j] > stockMaximumLevel - stock)
#else
                if (auxiliary_deliveries[j] > stockMaximumLevel - stock + productRate[j])
#endif
                {
                    //GlobalUtils.writeToConsole("Maximum Stock violated on delivery {0} vs {1}", auxiliary_deliveries[j], stockMaximumLevel - stock + productRate);
                    return false;
                }

                stock += deliveredQuantity - productRate[j];
                auxiliary_endDayInventory[j] = stock;
                
                //Check for stockouts
                if (auxiliary_endDayInventory[j] < -0.5)
                    return false;
            }
            
            
            //ReportInventory();
            
            return true;
        }
        
        public bool ApplyNewSaw(bool[] visitSchedule, Solution s, bool vehicleChanged, int changeDay)
        {
            //return ApplySawMaxDeliveryLast(visitSchedule, s);
            
            //if (s.depot.unitHoldingCost > unitHoldingCost)
            if (sawmode)
            {
                //as early as possible
                //GlobalUtils.writeToConsole("As early as possible");
                return ApplySawMaxDeliveryFirst(visitSchedule, s, vehicleChanged, changeDay);
            }
            else
            {
                //as late as possible
                //GlobalUtils.writeToConsole("As late as possible");
                return ApplySawMaxDeliveryLast(visitSchedule, s, vehicleChanged, changeDay); 
            }
        }

        public bool ApplyNewSaw(bool[] visitSchedule, Solution s)
        {
            return ApplyNewSaw(visitSchedule, s, false, -1);
        }
        
        
        public bool ApplySawMaxDeliveryLast(bool[] visitSchedule, Solution s, bool vehicleChanged, int changeDay) {
            //Export solution as ref solution
            //s.SaveToFile("ref_solution");
            
            //New calculated delivery and end quantities are saved in auxiliary arrays
            //TODO: save this as a class member
            int totalDemand = -startingInventory;
            for (int j = 0; j < horizonDays; j++)
                totalDemand += productRate[j];

            //Calculate depot slack
            int[] depot_slack = new int[horizonDays];
            depot_slack[0] = deliveredQuantities[0]; 
            for (int j = 1; j < horizonDays; j++)
                depot_slack[j] = depot_slack[j-1] + deliveredQuantities[j];
            
            for (int j = 0; j < horizonDays; j++)
                depot_slack[j] += s.depot.endDayInventory[j];
            
            for (int j = horizonDays - 1; j >= 0; j--)
            {
                int oldDeliveredQuantity = deliveredQuantities[j];
                int deliveredQuantity = 0;

                int extra_vehicle_slack = 0;
                if (vehicleChanged && changeDay == j)
                    extra_vehicle_slack = 0;
                else
                    extra_vehicle_slack = oldDeliveredQuantity;
                
                if (j == horizonDays - 1)
                    auxiliary_endDayInventory[j] = 0;
                else
                    auxiliary_endDayInventory[j] = auxiliary_startDayInventory[j+1];

                
                //we should check if the stock respects the maximum capacity level and 
                //if strict version is solved if the the quantity delivered respects the warehouse capacity
                //Check max capacity violation on previous visit day
#if RESPECT_CUSTOMER_CAPACITY
                //TODO Fix that if required
                if (auxiliary_deliveries[j] > stockMaximumLevel - stock)
#else
                if (auxiliary_endDayInventory[j] > stockMaximumLevel)
#endif
                {
                    //GlobalUtils.writeToConsole("Maximum Stock violated on delivery {0} vs {1}", auxiliary_deliveries[j], stockMaximumLevel - stock + productRate);
                    return false;
                }

                //Check for stockouts
                if (auxiliary_endDayInventory[j] < -0.5)
                    return false;
                
                
                //Findout the necessary delivered quantity to cover the demand until the next delivery
                if (visitSchedule[j])
                {
                    //Calculate vehicle slack for day
                    int vehicle_slack = auxiliary_horizonDeliveryServices[j].route.effectiveCapacity - auxiliary_horizonDeliveryServices[j].route.load 
                                                                                        +extra_vehicle_slack;
                    //Calculate max delivery
                    int maxdeliveredQuantity = Math.Min(auxiliary_endDayInventory[j] + productRate[j], totalDemand);
                    
                    //Calculate delivered Quantity
                    deliveredQuantity = GlobalUtils.Min(vehicle_slack, depot_slack[j], stockMaximumLevel + productRate[j], maxdeliveredQuantity);
                    deliveredQuantity = Math.Max(deliveredQuantity, 0);
                    
                    //Fix depot slack
                    for (int t = j; t < horizonDays; t++)
                        depot_slack[t] -= deliveredQuantity; 
                    
                    //Fix total delivery demand
                    totalDemand -= deliveredQuantity;
                }
                else
                {
                    //Do nothing?!?!
                }

                auxiliary_deliveries[j] = deliveredQuantity;
                auxiliary_startDayInventory[j] = auxiliary_endDayInventory[j] - deliveredQuantity + productRate[j];
                
            }
            
            //Check if the customer has received all teh required quantity
            if (totalDemand > 0)
                return false;
            
            //Check depot feasibility
            for (int t = 0; t < horizonDays; t++)
            {
                if (depot_slack[t] < 0 || depot_slack[t] > s.depot.stockMaximumLevel)
                {
                    //ReportInventory(true);
                    return false;
                }
            }
                
            //ReportInventory(true);
            return true;
        }
        
        public bool ApplySawMaxDeliveryFirst(bool[] visitSchedule, Solution s, bool vehicleChanged, int changeDay) {
            //Export solution as ref solution
            //s.SaveToFile("ref_solution");
            
            //New calculated delivery and end quantities are saved in auxiliary arrays
            int depot_diff = 0;
            bool feasible = true;
            //TODO: save this as a class member
            int totalDemand = -startingInventory;
            for (int j = 0; j < horizonDays; j++)
                totalDemand += productRate[j];
            
            //Calculate depot slack
            int[] depot_slack_hor = new int[horizonDays];
            depot_slack_hor[0] = deliveredQuantities[0]; 
            for (int j = 1; j < horizonDays; j++)
                depot_slack_hor[j] = depot_slack_hor[j-1] + deliveredQuantities[j];
            
            for (int j = 0; j < horizonDays; j++)
                depot_slack_hor[j] += s.depot.endDayInventory[j];
            
            for (int j = 0; j < horizonDays; j++)
            {
                int vehicle_slack = 0;
                int depot_slack = 0;
                int demandTillNextVisit = 0;
                int oldDeliveredQuantity = deliveredQuantities[j];
                int extra_vehicle_slack = 0;
                if (vehicleChanged && changeDay == j)
                    extra_vehicle_slack = 0;
                else
                    extra_vehicle_slack = oldDeliveredQuantity;
                    
                
                int deliveredQuantity = 0;
                
                if (j > 0)
                    auxiliary_startDayInventory[j] = auxiliary_endDayInventory[j - 1];
                else
                    auxiliary_startDayInventory[j] = startingInventory;

                //Findout the necessary delivered quantity to cover the demand until the next delivery
                if (visitSchedule[j])
                {
                    //Calculate vehicle slack for day
                    vehicle_slack = auxiliary_horizonDeliveryServices[j].route.effectiveCapacity - auxiliary_horizonDeliveryServices[j].route.load 
                                                                                        + extra_vehicle_slack;
                    //Calculate depot slack for day\
                    depot_slack = depot_slack_hor[j];
                    for (int i = j + 1; i < horizonDays; i++)
                        depot_slack = Math.Min(depot_slack, depot_slack_hor[i]);
                    
                    //Calculate delivered Quantity
                    deliveredQuantity = GlobalUtils.Min(vehicle_slack, depot_slack, stockMaximumLevel - auxiliary_startDayInventory[j] + productRate[j], totalDemand);
                    deliveredQuantity = Math.Max(deliveredQuantity, 0);
                    
                    //Fix depot stock
                    depot_diff +=  oldDeliveredQuantity - deliveredQuantity;
                    
                    //Fix depot slack
                    for (int t = j; t < horizonDays; t++)
                        depot_slack_hor[t] -= deliveredQuantity;

                    //Fix total delivery demand
                    totalDemand -= deliveredQuantity;
                }
                else
                {
                    //Do nothing?!?!
                }

                auxiliary_deliveries[j] = deliveredQuantity;
                
                
                //we should check if the stock respects the maximum capacity level and 
                //if strict version is solved if the the quantity delivered respects the warehouse capacity
                //Check max capacity violation on previous visit day
#if RESPECT_CUSTOMER_CAPACITY
                if (auxiliary_deliveries[j] > stockMaximumLevel - stock)
#else
                if (auxiliary_deliveries[j] > stockMaximumLevel - auxiliary_startDayInventory[j] + productRate[j])
#endif
                {
                    //GlobalUtils.writeToConsole("Maximum Stock violated on delivery {0} vs {1}", auxiliary_deliveries[j], stockMaximumLevel - stock + productRate);
                    return false;
                }

                auxiliary_endDayInventory[j] = auxiliary_startDayInventory[j] + deliveredQuantity - productRate[j];
                
                //Check for stockouts
                if (auxiliary_endDayInventory[j] < -0.5)
                    return false;
            }
            
            //ReportInventory(true);
            
            return true;
        }
        
        public void ArrangeDaysToStockOut()
        {
            //Question : I don't understand the point of keeping multiple stock out dates
            //if we are always working on feasible space.

            daysTillStockOut = new List<int>();
            stocksOut = false;

            for (int i = 0; i < horizonDays; i++)
            {
                bool filledIn = false;
                for (int j = i; j < horizonDays; j++)
                {
                    double stock = endDayInventory[j];

                    if (stock < -0.5)
                    {
                        daysTillStockOut.Add(j - i);
                        stocksOut = true; //set stockout flag
                        filledIn = true;
                        break;
                    }
                }
                
                if (!filledIn)
                {
                    double stock = endDayInventory[i];
                    int days = 0;
                    double temp_stock = stock;
                    int temp_i = i;
                    while (temp_stock > 0.0 && temp_i < horizonDays)
                    {
                        days++;
                        temp_stock -= productRate[temp_i];
                        temp_i++;
                    }
                    daysTillStockOut.Add(days);
                }
            }
        }
        
        
#region Reports/Validators

        public void calculateMinimumRequiredVisits(int vehicleCapacity)
        {
            //calculate lower bound
            int totalDemandServiced = productRate.Sum();
            int maxDemand = productRate.Max();
            int denominator = Math.Min(vehicleCapacity, stockMaximumLevel + maxDemand);

            totalDemandServiced -= startingInventory;
           
            minimumRequiredVisits = (int)Math.Ceiling((double)totalDemandServiced / denominator); 
        }

        #region Reports/Validators


        public void ReportVisitSchedule()
        {
            Console.Write("{0,20}", "Periods: ");
            for (int i = 0; i < horizonDays; i++)
                Console.Write("{0,8} ", i);
            Console.Write("\n");
            
            Console.Write("{0,20}", "Visits: ");
            for (int i = 0; i < horizonDays; i++)
                Console.Write("{0,8} ", visitSchedule[i]);
            Console.Write("\n");
            
            Console.Write("{0,20}", "Prev: ");
            for (int i = 0; i < horizonDays; i++)
                Console.Write("{0,8} ", prevDeliveryDay[i]);
            Console.Write("\n");
            
            Console.Write("{0,20}", "Next: ");
            for (int i = 0; i < horizonDays; i++)
                Console.Write("{0,8} ", nextDeliveryDay[i]);
            Console.Write("\n");
        }


        public void ReportInventory(bool aux)
        {
            if (!aux)
                ReportInventory();
            else
                ReportInventoryAux();
        }
        
        public void ReportInventory()
        {
            GlobalUtils.writeToConsole("Node {0:d} Inventory Levels", this.uid);
            //Print Day horizon
            Console.Write("{0,20}", "Horizon");
            for (int i = 0; i < horizonDays; i++)
                Console.Write(" {0,8} ", i);
            Console.Write("\n");
            //Print Start Day Inventory
            Console.Write("{0,20}", "Start Inventory");
            for (int i = 0; i < horizonDays; i++)
                Console.Write(" {0,8:0.0} ", startDayInventory[i]);
            Console.Write("  (Ignore if cyclic) \n");
            //Print Delivered Quantities
            Console.Write("{0,20}", "Deliveries");
            for (int i = 0; i < horizonDays; i++)
                Console.Write(" {0,8:0.0} ", deliveredQuantities[i]);
            Console.Write("\n");
            //Print Demanded Quantities
            Console.Write("{0,20}", "Demand");
            for (int i = 0; i < horizonDays; i++)
                Console.Write(" {0,8:0.0} ", productRate[i]);
            Console.Write("\n");
            //Print End Quantities
            Console.Write("{0,20}", "End Inventory");
            for (int i = 0; i < horizonDays; i++)
                Console.Write(" {0,8:0.0} (Ignore if cyclic)", endDayInventory[i]);
            Console.Write("  Same as start inventory in cyclic \n");
        }

        public void ReportInventoryAux()
        {
            GlobalUtils.writeToConsole("Node {0:d} Inventory Levels", this.uid);
            //Print Day horizon
            Console.Write("{0,20}", "Horizon");
            for (int i = 0; i < horizonDays; i++)
                Console.Write(" {0,8} ", i);
            Console.Write("\n");
            //Print Start Day Inventory
            Console.Write("{0,20}", "Start Inventory");
            for (int i = 0; i < horizonDays; i++)
                Console.Write(" {0,8:0.0} ", auxiliary_startDayInventory[i]);
            Console.Write("\n");
            //Print Delivered Quantities
            Console.Write("{0,20}", "Deliveries");
            for (int i = 0; i < horizonDays; i++)
                Console.Write(" {0,8:0.0} ", auxiliary_deliveries[i]);
            Console.Write("\n");
            //Print Demanded Quantities
            Console.Write("{0,20}", "Demand");
            for (int i = 0; i < horizonDays; i++)
                Console.Write(" {0,8:0.0} ", productRate[i]);
            Console.Write("\n");
            //Print End Quantities
            Console.Write("{0,20}", "End Inventory");
            for (int i = 0; i < horizonDays; i++)
                Console.Write(" {0,8:0.0} ", auxiliary_endDayInventory[i]);
            Console.Write("\n");
        }
        
        public void ReportStockoutDays()
        {
            GlobalUtils.writeToConsole("Node {0:d} Stockout Info", this.uid);
            //Print Day horizon
            Console.Write("{0,20}", "Horizon");
            for (int i = 0; i < horizonDays; i++)
                Console.Write(" {0,5} ", i);
            Console.Write("\n");
            //Print Start Day Inventory
            Console.Write("{0,20}", "Days Until Stockout");
            for (int i = 0; i < horizonDays; i++)
                Console.Write(" {0,5} ", daysTillStockOut[i]);
            Console.Write("\n");
            
        }
        
        
        //Node validator methods
        public virtual bool Validate()
        {
            //Check inventory equilibrium   
            for (int i = 0; i < horizonDays; i++)
            {
                if (!GlobalUtils.IsEqual(startDayInventory[i] + deliveredQuantities[i] - productRate[i], endDayInventory[i]))
                    return false;
                
                //TODO: Check holding costs
            
                if (deliveredQuantities[i] > 0 || horizonDeliveryServices[i].quantity > 0)
                {
                    if (horizonDeliveryServices[i] == null)
                        return false;

                    if (!GlobalUtils.IsEqual(horizonDeliveryServices[i].quantity, deliveredQuantities[i]))
                        return false;
                }
                
            }

            return true;
        }
        
        
#endregion
    }

    
    //TODO: Implement a Depot Class for cleaner usage
    public class Depot : Node
    {
        public bool[] open;
        public int[] productionRates;
        public double unitProductionCost;
        public double productionSetupCost;
        public int productionCapacity;
        private int horizonDays;

        public Depot(int horizonDays) : base(horizonDays)
        {
            this.horizonDays = horizonDays;
            open = new bool[horizonDays];
            productionRates = new int[horizonDays];
        }
        
        public Depot(Depot nd) : base(nd)
        {
            horizonDays = nd.horizonDays;
            unitProductionCost = nd.unitProductionCost;
            productionSetupCost = nd.productionSetupCost;
            productionCapacity = nd.productionCapacity; 
            
            //Init Arrays
            open = new bool[horizonDays];
            productionRates = new int[horizonDays];
            
            //Copy input data
            nd.open.CopyTo(open, 0);
            nd.productionRates.CopyTo(productionRates, 0);
        }
        
        public override void InitializeInventoryLevels()
        {
            deliveredQuantities = new int[horizonDays]; //Reset Deliveries

            CalculateInventoryLevels();
        }
        
        public override void CalculateInventoryLevels()
        {
            endDayInventory = new int[horizonDays];
            
            int stock = startingInventory; //Init to start inventory
            for (int i = 0; i < horizonDays; i++) 
            {
                startDayInventory[i] = stock;
                stock += productionRates[i]; //Adds production
                stock += deliveredQuantities[i]; //Delivered quantities are negative for the depot
                endDayInventory[i] = stock;
            }
        }
        
        public new void ReportInventory()
        {
            GlobalUtils.writeToConsole("Node {0:d} Inventory Levels", this.uid);
            //Print Day horizon
            Console.Write("{0,20}", "Horizon");
            for (int i = 0; i < horizonDays; i++)
                Console.Write(" {0,8} ", i);
            Console.Write("\n");
            //Print Start Day Inventory
            Console.Write("{0,20}", "Start Inventory");
            for (int i = 0; i < horizonDays; i++)
                Console.Write(" {0,8:0.0} ", startDayInventory[i]);
            Console.Write("\n");
            //Print Delivered Quantities
            Console.Write("{0,20}", "Deliveries");
            for (int i = 0; i < horizonDays; i++)
                Console.Write(" {0,8:0.0} ", deliveredQuantities[i]);
            Console.Write("\n");
            //Print Demanded Quantities
            Console.Write("{0,20}", "Production");
            for (int i = 0; i < horizonDays; i++)
                Console.Write(" {0,8:0.0} ", productionRates[i]);
            Console.Write("\n");
            //Print End Quantities
            Console.Write("{0,20}", "End Inventory");
            for (int i = 0; i < horizonDays; i++)
                Console.Write(" {0,8:0.0} ", endDayInventory[i]);
            Console.Write("\n");
        }
        
        public override bool Validate()
        {

            for (int i = 0; i < horizonDays; i++)
            {
                if (open[i] && productionRates[i] <= GlobalUtils.doublePrecision)
                    return false;
            }
            return base.Validate();
        }
       
    }
}

#endregion