using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace PRP
{
    class InfeasibleSolutionManager
    {
        public Solution bestObjTotal;
        public Solution bestObjRouting;
        public Solution bestObjInventory;
        public Solution bestViolation;
        public Solution current;


        //Counters
        int cBestObjTotal = 0;
        int cBestObjRouting = 0;
        int cBestObjInventory = 0;
        int cBestViolation = 0;
        int cCurrent = 0;

        public InfeasibleSolutionManager()
        {
            
        }

        public void applyMIP()
        {
            MIP.runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemovalSameDayRoutesOriginal(ref bestObjTotal, 3, 3, 0.02, 2);
            MIP.runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemovalSameDayRoutesOriginal(ref bestObjRouting, 3, 3, 0.02, 2);
            MIP.runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemovalSameDayRoutesOriginal(ref bestObjInventory, 3, 3, 0.02, 2);
            MIP.runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemovalSameDayRoutesOriginal(ref bestViolation, 3, 3, 0.02, 2);
            MIP.runSimultaneousDeliveryProductionReoptimizationMIPVehCapInfeaswithCustInsertionRemovalSameDayRoutesOriginal(ref current, 3, 3, 0.02, 2);
        }

        public void setCurrent(Solution sol)
        {
            current = sol;
        }

        public void saveToFile()
        {
            StreamWriter streawriter = File.AppendText("Infeasmanager_report");
            streawriter.WriteLine("Best Selections: Best Obj {0} Best Inv {1} Best Routing {2} Best Violation {3} Current {4}",
                cBestObjTotal, cBestObjInventory, cBestObjRouting, cBestViolation, cCurrent);
            streawriter.Flush();
            streawriter.Close();
        }

        public Solution getBest()
        {
            List<Solution> sol_list = new List<Solution>();
            sol_list.Add(bestObjTotal);
            sol_list.Add(bestObjInventory);
            sol_list.Add(bestObjRouting);
            sol_list.Add(bestViolation);
            sol_list.Add(current);


            //Find best 
            Solution b = bestObjTotal;
            int selected_index = 0;

            for (int i = 1; i < 5; i++)
            {
                if (sol_list[i].isBetterThan(b))
                {
                    selected_index = i;
                    b = sol_list[i];
                }
            }

            switch (selected_index)
            {
                case 0:
                    cBestObjTotal++;
                    break;
                case 1:
                    cBestObjInventory++;
                    break;
                case 2:
                    cBestObjRouting++;
                    break;
                case 3:
                    cBestViolation++;
                    break;
                case 4:
                    cCurrent++;
                    break;
            }
                
            return b;
        }


        public void saveSolution(Solution sol)
        {
            if (bestObjTotal == null || sol.totalObjective < bestObjTotal.totalObjective)
                bestObjTotal = new Solution(sol, 0.0);
            
            if (bestObjInventory == null || sol.holdingCost < bestObjInventory.holdingCost)
                bestObjInventory = new Solution(sol, 0.0);
            
            if (bestObjRouting == null || sol.routingCost < bestObjRouting.routingCost)
                bestObjRouting = new Solution(sol, 0.0);
            
            if (bestViolation == null || sol.violationCost < bestViolation.violationCost)
                bestViolation = new Solution(sol, 0.0);
            
        }


        public void report()
        {
            GlobalUtils.writeToConsole("Best Obj Cost {0} Best Inv {1} Best Routing {2} Best Violation {3}",
                bestObjTotal?.totalObjective, bestObjInventory?.holdingCost, bestObjRouting?.routingCost, bestViolation?.violationCost);
            GlobalUtils.writeToConsole("Best Selections: Best Obj {0} Best Inv {1} Best Routing {2} Best Violation {3} Current {4}",
                cBestObjTotal, cBestObjInventory, cBestObjRouting, cBestViolation, cCurrent);
        }


        public void clearCounters()
        {
            cBestObjInventory = 0;
            cBestObjRouting = 0;
            cBestObjTotal = 0;
            cBestViolation = 0;
            cCurrent = 0;
        }

        public void clearSolutions()
        {
            bestObjTotal = null;
            bestObjInventory = null;
            bestObjRouting = null;
            bestViolation = null;
        }

    }
}
