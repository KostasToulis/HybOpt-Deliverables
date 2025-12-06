using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PRP
{
    public class Period
    {
        public int periodIndex;
        public int vehicleNum;
        public int routeCapacity;
        public List<Route> periodRoutes;
        public double periodRoutingCost;
        public double totalOutboundProductFlow;
        public Period (int vehs, int capacity)
        {
            vehicleNum = vehs;
            routeCapacity = capacity;

            periodRoutes = new List<Route>();

            for (int i = 0; i < vehicleNum; i++)
            {
                periodRoutes.Add(new Route(routeCapacity));
            }
        }

        public Period(Period p){
            periodIndex = p.periodIndex;
            vehicleNum = p.vehicleNum;
            routeCapacity = p.routeCapacity;
            totalOutboundProductFlow = p.totalOutboundProductFlow;

            periodRoutes = new List<Route>();

            //Copy Routes
            for (int i = 0; i < vehicleNum; i++)
            {
                periodRoutes.Add(new Route(p.periodRoutes[i]));
            }   
        }

        public void appendRoute(PRP model, Node depot)
        {
            vehicleNum++;
            Route rt = new Route(model.input.dayVehicleCapacity);
            rt.initialize(depot);
            
            periodRoutes.Add(rt);
        }

        public void removeRoute(int index)
        {
            periodRoutes.RemoveAt(index);
            vehicleNum--;
        }

        public void Report()
        {
            Console.WriteLine("Period ", periodIndex);
            Console.WriteLine("\tRoutes : ");
            for (int i = 0; i < periodRoutes.Count; i++)
            {
                Route pr = periodRoutes[i];
                Console.WriteLine("\t\tRoute: ");

                for (int j = 1; j < pr.nodes.Count-1; j++)
                {
                    Node nd = pr.nodes[j];
                    bool isNodeValid = false;
                    if (nd.deliveredQuantities[periodIndex] > 0 && nd.horizonDeliveryServices[periodIndex] != null)
                    {
                        if (nd.horizonDeliveryServices[periodIndex].quantity == nd.deliveredQuantities[periodIndex] &&
                            object.ReferenceEquals(nd.horizonDeliveryServices[periodIndex].route, pr))
                            isNodeValid = true;
                    }
                    
                    Console.WriteLine("\t\t\t {0:D} ({1:D}) ", nd.uid, isNodeValid);    
                }
            }
        }

    }
}
