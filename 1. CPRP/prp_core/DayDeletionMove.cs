using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PRP
{
    class DayDeletionMove
    {
        public MOVES operatorType;
        public int removedDay;
        public int removedRoutePosition;
        public int removedNodePosition;

        public double removedRouteObjectiveChange;

        public double inventoryCustomerObjectiveChange;
        public double inventoryDepotObjectiveChange;

        public double totalObjectiveChange;
        public double violationCostChange;
        
        public DayDeletionMove()
        {
            operatorType = MOVES.DELIVERYDAYDELETION;
            totalObjectiveChange = double.MaxValue;
        }
    }
}
