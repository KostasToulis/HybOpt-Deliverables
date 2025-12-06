using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PRP
{
    class DayInsertionMove
    {
        public MOVES operatorType;
        public int insertedCustomer;

        public int insertedDay;
        public int insertedRoutePosition;
        public int insertedNodePosition;

        public double insertedRouteObjectiveChange;

        public double inventoryCustomerObjectiveChange;
        public double inventoryDepotObjectiveChange;

        public double totalObjectiveChange;
        public double violationCostChange;
        
        public DayInsertionMove()
        {
            operatorType = MOVES.DELIVERYDAYINSERTION;
            totalObjectiveChange = double.MaxValue;
        }
    }
}
