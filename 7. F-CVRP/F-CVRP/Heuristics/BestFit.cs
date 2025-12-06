using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;

namespace F_CVRP.Heuristics
{
    public class BestFit
    {
        public int binNumber { get; set; }
        public List<Route> bins { get; set; }
        public int capacity { get; set; }

        public BestFit(int vehicles, int cap, Node depot)
        {
            binNumber = vehicles;            
            capacity = cap;
            bins = InitializeRoutes(depot);         
        }

        public class Slack 
        { 
            public int binId { get; set; }
            public float value { get; set; }

            public Slack(int id, float val)
            {
                binId = id;
                value = val;
            }
        }


        private List<Route> InitializeRoutes(Node depot)
        {
            List<Route> routes = new List<Route>();
            for (int i = 0; i < binNumber; i++)
            {
                routes.Add(new Route(i, new List<Node> { depot, depot }, capacity, 0, 0)) ;
            }
            return routes;
        }


        public Solution SolveWithBestFit(List<Node> selectedCustomers)
        {
            Solution sol = new Solution();

            selectedCustomers = selectedCustomers.OrderByDescending(n => n.demand).ToList();

            foreach (Node customer in selectedCustomers)
            {
                Route rt = FindBestBin(customer);

                rt.sequenceOfNodes.Insert(1, customer);
                rt.CalculateRouteCostDemand();

            }

            sol.routes = bins;
            sol.UpdateCost();

            return sol;

        }

        private Route FindBestBin (Node customer)
        {
            Route bin = bins.Where(b => b.capacity - b.load - customer.demand >= 0).OrderBy(b => b.capacity - b.load - customer.demand).First();
            return bin;
        }
    }
}
