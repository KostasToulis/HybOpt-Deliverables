using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PRP
{
    public class DataInput
    {
        public List<Node> nodes;
        public int dayVehicleCapacity;
        public int availableVehicles;
        public int horizonDays;
        public int customerNum;
        public double distanceCoeff;
        public PRP_DATASET_VERSION type;
        public bool customers_zero_inv_cost;

        public DataInput()
        {
            nodes = new List<Node>();
        }

        public void report()
        {
            GlobalUtils.writeToConsole("Nodes: {0} Vehicle Capacity {1}, Vehicle Num {2}, HorizonDays {3}, CustomerNum {4}, distanceCoeff {5}, customers_zero_inv_cost {6}", 
                nodes.Count, dayVehicleCapacity, availableVehicles, horizonDays, customerNum, distanceCoeff, customers_zero_inv_cost);
        }
    }
    
    public class ProductionDataInput
    {
        public string ID;
        public int[] productionQuantities;
        public bool[] plantOpen;
        public int horizonDays;
        public int[,] customerDeliveries;
        public int[,] customerRouteAssignment;
        public double relaxedObjective;
        
        public ProductionDataInput()
        {
            
        }
    }
}
