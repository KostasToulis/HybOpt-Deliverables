//using System;
//using System.Collections.Generic;
//using System.Text;
//using System.Linq;

//namespace F_CVRP
//{
    

//    class ClarknWright : Solver
//    {
//        public ClarknWright(Model model, Solver s) : base(model) 
//        {
//            this.numNodes = model.numNodes;
//            this.numFam = model.numFam;
//            this.numReq = model.numReq;
//            this.capacity = model.capacity;
//            this.vehicles = model.vehicles;
//            this.famMembers = model.famMembers;
//            this.famReq = model.famReq;
//            this.famDemand = model.famDemand;
//            this.costMatrix = model.costMatrix;
//            this.depot = model.depot;
//            this.nodes = model.nodes;
//            this.customers = model.customers;
//            this.families = model.families;
//            this.sol = s.sol;
//            this.selectedCustomers = s.selectedCustomers;
//        }

//        public class Saving
//        {
//            public Saving(Node n1, Node n2, float s)
//            {
//                this.n1 = n1;
//                this.n2 = n2;
//                this.score = s;
//            }

//            public Node n1;
//            public Node n2;
//            public float score;
//        }

//        private float CalculateTotalCost(Solution sol)
//        {
//            float c = 0;
//            for (int i = 0; i < sol.routes.Count; i++)
//            {
//                Route rt = sol.routes[i];
//                for (int j = 0; j < rt.sequenceOfNodes.Count-1; j++)
//                {
//                    Node a = rt.sequenceOfNodes[j];
//                    Node b = rt.sequenceOfNodes[j + 1];
//                    c += this.costMatrix[a.id][b.id];
//                }
//            }
//            return c;
//        }

//        private void UpdateRouteCostAndLoad(Route rt)
//        {
//            float tc = 0;
//            float tl = 0;
//            for (int i = 0; i < rt.sequenceOfNodes.Count - 1; i++)
//            {
//                Node a = rt.sequenceOfNodes[i];
//                Node b = rt.sequenceOfNodes[i+1];
//                tc += this.costMatrix[a.id][b.id];
//                tl += a.demand;
//            }
//            rt.load = tl;
//            rt.cost = tc;
//        }

//        private List<Saving> CalculateSavings()
//        {
//            List<Saving> savings = new List<Saving>();
//            for (int i = 0; i < this.selectedCustomers.Count; i++)
//            {
//                Node n1 = this.selectedCustomers[i];
//                for (int j = 0; j < this.selectedCustomers.Count; j++)
//                {
//                    if (i == j)
//                    {
//                        continue;
//                    }
//                    Node n2 = this.selectedCustomers[j];

//                    float score = this.costMatrix[n1.id][this.depot.id] + this.costMatrix[this.depot.id][n2.id];
//                    score -= this.costMatrix[n1.id][n2.id];
//                    Saving sav = new Saving(n1, n2, score);
//                    savings.Add(sav);
//                }
//            }
//            return savings;
//        }

//        private Solution CreateInitialRoutes() 
//        {
//            Solution s = new Solution();
//            for (int i = 0; i < this.selectedCustomers.Count; i++)
//            {
//                Node n = this.selectedCustomers[i];
//                Route rt = new Route(i, null, this.capacity, 0, 0);
//                n.route = rt;
//                n.positionInRoute = 1;
//                n.isRouted = true;
//                rt.sequenceOfNodes = new List<Node> { this.depot, n, this.depot };
//                rt.load = n.demand;
//                rt.cost = this.costMatrix[this.depot.id][n.id] + this.costMatrix[n.id][this.depot.id];
//                s.routes.Add(rt);
//                s.cost += rt.cost;
//            }
//            return s;
//        }

//        private bool NotFirstOrLast(Route rt, Node n) 
//        {
//            if ( n.positionInRoute != 1 && n.positionInRoute != rt.sequenceOfNodes.Count - 2 ) 
//            {
//                return true;
//            }
//            return false;
//        }

//        private void MergeRoutes(Node n1, Node n2) 
//        {
//            Route rt1 = n1.route;
//            Route rt2 = n2.route;

//            if (n1.positionInRoute == 1 && n2.positionInRoute == rt2.sequenceOfNodes.Count - 2)
//            {
//                for (int i = rt2.sequenceOfNodes.Count - 2; i > 0; i--)
//                {
//                    Node n = rt2.sequenceOfNodes[i];
//                    rt1.sequenceOfNodes.Insert(1, n);
//                }
//            }
//            else if (n1.positionInRoute == 1 && n2.positionInRoute == 1)
//            {
//                for (int i = 1; i < rt2.sequenceOfNodes.Count - 1; i++)
//                {
//                    Node n = rt2.sequenceOfNodes[i];
//                    rt1.sequenceOfNodes.Insert(1, n);
//                }
//            }
//            else if (n1.positionInRoute == rt1.sequenceOfNodes.Count - 2 && n2.positionInRoute == 1)
//            {
//                for (int i = 1; i < rt2.sequenceOfNodes.Count - 1; i++)
//                {
//                    Node n = rt2.sequenceOfNodes[i];
//                    rt1.sequenceOfNodes.Insert(rt1.sequenceOfNodes.Count - 1, n);
//                }
//            }
//            else if (n1.positionInRoute == rt1.sequenceOfNodes.Count - 2 && n2.positionInRoute == rt2.sequenceOfNodes.Count - 2)
//            {
//                for (int i = rt2.sequenceOfNodes.Count - 2; i > 0; i--)
//                {
//                    Node n = rt2.sequenceOfNodes[i];
//                    rt1.sequenceOfNodes.Insert(rt1.sequenceOfNodes.Count - 1, n);
//                }
//            }
//            rt1.load += rt2.load;
//            this.sol.routes.Remove(rt2);
//            UpdateRouteCustomers(rt1);
//            UpdateRouteCostAndLoad(rt1);
//        }

//        private void UpdateRouteCustomers(Route rt)
//        {
//            for (int i = 1; i < rt.sequenceOfNodes.Count - 1; i++)
//            {
//                Node n = rt.sequenceOfNodes[i];
//                n.route = rt;
//                n.positionInRoute = i;
//            }
//        }

//        public Solution ClarkWrightSolution()
//        {
//            this.sol = CreateInitialRoutes();
//            List<Saving> savings = CalculateSavings();
//            savings = savings.OrderByDescending(s => s.score).ToList();

//            for (int i = 0; i < savings.Count; i++)
//            {
//                Saving sav = savings[i];
//                Node n1 = sav.n1;
//                Node n2 = sav.n2;
//                Route rt1 = n1.route;
//                Route rt2 = n2.route;

//                if (n1.route == n2.route)
//                {
//                    continue;
//                }
//                if (NotFirstOrLast(rt1, n1) || NotFirstOrLast(rt2, n2))
//                {
//                    continue;
//                }
//                if (rt1.load + rt2.load > this.capacity)
//                {
//                    continue;
//                }
//                MergeRoutes(n1, n2);
//                this.sol.cost -= sav.score;

//            }
//            sol.cost = CalculateTotalCost(sol);
//            return sol;
//        }

//    }
//}
