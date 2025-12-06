using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PRP
{
    public class CustDelivery
    {
        public int quantity;
        public Route route;
        
        public CustDelivery()
        {
            quantity = 0;
            route = null;
        }
        
        public CustDelivery(int quant, Route rt)
        {
            quantity = quant;
            route = rt;
        }

        public void reset()
        {
            quantity = 0;
            route = null;
        }
    }
}
