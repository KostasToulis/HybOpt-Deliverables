using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PRP
{
    class DayRelocationMove
    {
        public MOVES operatorType;
        public int removedDay;
        public int removedRoutePosition;
        public int removedNodePosition;

        public int insertedDay;
        public int insertedRoutePosition;
        public int insertedNodePosition;

        public double removedRouteObjectiveChange;
        public double insertedRouteObjectiveChange;

        public double inventoryCustomerObjectiveChange;
        public double inventoryDepotObjectiveChange;

        public double totalObjectiveChange;
        public double violationCostChange;
        
        public DayRelocationMove()
        {
            operatorType = MOVES.DELIVERYDAYRELOCATION;
            totalObjectiveChange = double.MaxValue;
        }
    }
}
