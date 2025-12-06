using System.Collections.Generic;

namespace PRP
{
    internal class RelocationMove
    {
        public MOVES operatorType;
        public int day;
        public int originRoutePosition;
        public int originNodePosition;

        public int targetRoutePosition;
        public int targetNodePosition;

        public double originRouteObjectiveChange;
        public double targetRouteObjectiveChange;
        

        public double totalObjectiveChange;
        public double violationChange;
        
        public int[] arcsDeleted;

        public RelocationMove()
        {
            operatorType = MOVES.RELOCATE;
            totalObjectiveChange = double.MaxValue;
            arcsDeleted = new int[10*3 + 1]; //Store 10 arcs at most
        }
    }
    
    internal class RelocationWithInventoryMove
    {
        public MOVES operatorType;
        public int day;
        public int originRoutePosition;
        public int originNodePosition;

        public int targetRoutePosition;
        public int targetNodePosition;

        public double originRouteObjectiveChange;
        public double targetRouteObjectiveChange;
        public double extraRoutingCostChange;

        public double inventoryCustomerObjectiveChange;
        public double inventoryDepotObjectiveChange;

        public double totalObjectiveChange;
        public double violationChange;
        
        public int[] arcsDeleted;

        public RelocationWithInventoryMove()
        {
            operatorType = MOVES.RELOCATEWITHINVENTORY;
            totalObjectiveChange = double.MaxValue;
            arcsDeleted = new int[10*3 + 1];
        }
    }
}