using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PRP
{
    public class InsertionPosition
    {
        public int day;
        public int routeindex;
        public int positionIndex;
        public int deliveryQuantity;
        public double totalObjectiveChange;
        public double totalHoldingCostChange;
        public double totalRoutingCostChange;


        public InsertionPosition(int d, int r, int p)
        {
            day = d;
            routeindex = r;
            positionIndex = p;
        }
       
    }
}
