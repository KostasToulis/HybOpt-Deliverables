using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using static F_CVRP.Setup;
using F_CVRP.Heuristics;

namespace F_CVRP
{
    class ConstructiveHeuristic : Model
    {
        //public new List<Node> unservedCustomers
        //{
        //    get { return base.unservedCustomers; }
        //    set { base.unservedCustomers = value; }
        //}

        public Solution sol { get; set; }
        public List<Node> unservedCustomers { get; set; }
        public ConstructiveHeuristic(Model model) 
        {
            this.numNodes = model.numNodes;
            this.numFam = model.numFam;
            this.numReq = model.numReq;
            this.capacity = model.capacity;
            this.vehicles = model.vehicles;
            this.famMembers = model.famMembers;
            this.famReq = model.famReq;
            this.famDemand = model.famDemand;
            this.costMatrix = model.costMatrix;
            this.depot = model.depot;
            this.nodes = model.nodes;
            this.customers = model.customers;
            this.families = model.families;
            //this.sol = s.sol;
            //this.unservedCustomers = s.unservedCustomers;
        }

        public (Solution, List<Node>) Solve(List<Node> selectedCustomers)
        {
            customers.ForEach(x => { x.isRouted = false; });

            List<Node> unservedCustomers = new List<Node>();

            Solution sol = new Solution();

            (sol, unservedCustomers) = MinimumInsertionSolution(selectedCustomers);

            if (unservedCustomers != null && unservedCustomers.Count > 0)
            {
                (sol, unservedCustomers) = SolveWithBestFit(selectedCustomers);
            }

            return (sol, unservedCustomers);
        }

        public (Solution, List<Node>) MinimumInsertionSolution(List<Node> selectedCustomers)
        {

            //Stopwatch sw = new Stopwatch();
            //sw.Start();

            customers.ForEach(x => { x.isRouted = false; });

            List<Route> routes = new List<Route>();
            for (int i = 0; i < this.vehicles; i++)
            {
                List<Node> sequenceOfNodes = new List<Node> { depot, depot };
                Route rt = new Route(i, sequenceOfNodes, this.capacity, 0, 0);
                routes.Add(rt);
            }

            int insertedCustomers = 0;

            while (insertedCustomers < selectedCustomers.Count)
            {
                int insertChange = insertedCustomers;
                foreach (Route rt in routes)
                {
                    (Node insertedCustomer, int pos, float insertionCost) = SelectBestInsertion(rt, selectedCustomers);

                    if (pos > 0)
                    {
                        rt.sequenceOfNodes.Insert(pos, insertedCustomer);
                        insertedCustomer.isRouted = true;
                        rt.cost += insertionCost;
                        rt.load += insertedCustomer.demand;

                        insertedCustomers++;
                    }   
                }

                if (insertedCustomers - insertChange == 0)
                {
                    this.unservedCustomers = selectedCustomers.Where(x => x.isRouted == false).ToList();
                    
                    break;
                }


            }

            routes.ForEach(rt => rt.sequenceOfNodes.ForEach( n => n.route = rt));


            Solution sol = new Solution();
            sol.routes = routes;
            sol.cost = routes.Sum(rt => rt.cost);

            sol.UpdateCost();

            return (sol, unservedCustomers);

            //if (unservedCustomers != null && unservedCustomers.Count > 0)
            //{
            //    //ShuffleSolution(sol, selectedCustomers, unservedCustomers);


            //    List<Node> remainingCustomers = new List<Node>();
            //    (sol, remainingCustomers) = SolveWithBestFit(selectedCustomers);

            //    return sol;
            //}

            ////sw.Stop();

            ////Console.WriteLine("Minimum insertion ms: " + sw.ElapsedMilliseconds);
            //sol.UpdateCost();
            //return sol;
        }

        public (Node, int, float) SelectBestInsertion(Route rt, List<Node> selectedCustomers)
        {
            Node minNode = this.depot;
            float minCost = 100000;
            int minPos = -1;
            foreach (Node n in selectedCustomers)
            {
                if (n.isRouted || n.demand + rt.load > rt.capacity)
                {
                    continue;
                }

                for (int i = 1; i < rt.sequenceOfNodes.Count; i++)
                {
                    float costAdded = this.costMatrix[rt.sequenceOfNodes[i - 1].id][n.id] + this.costMatrix[n.id][rt.sequenceOfNodes[i].id];
                    float costRemoved = 0;
                    if (rt.sequenceOfNodes[i - 1] != rt.sequenceOfNodes[i])
                    {
                        costRemoved = this.costMatrix[rt.sequenceOfNodes[i - 1].id][rt.sequenceOfNodes[i].id];
                    }
                    
                    float insertionCost = costAdded - costRemoved;

                    if (insertionCost < minCost)
                    {
                        minCost = insertionCost;
                        minPos = i;
                        minNode = n;
                    }
                }
            }
            return (minNode, minPos, minCost);
        }


        private void ShuffleSolution(Solution sol, List<Node> selectedCustomers, List<Node> unserved)
        {
            //Solution newSol = sol.DeepCopy();     

            Node bestNode;
            Route bestRoute;

            while(unserved.Count > 0)
            {
                bool relocSuccess = ApplyRelocation(sol);

                if (!relocSuccess)
                {
                    ApplySwap(sol);
                }

                float maxRemainingLoad = sol.routes.Max(rt => capacity - rt.load);
                float minDemand = unserved.Min(u => u.demand);

                if (minDemand <= maxRemainingLoad)
                {
                    Route minRoute = sol.routes.Where(rt => capacity - rt.load == maxRemainingLoad).First();
                    Node candidateUnserved = unserved.Where(u => u.demand <= maxRemainingLoad).OrderByDescending(u => u.demand).First();

                    (int pos, float cost) = FindBestInsertion(minRoute, candidateUnserved);

                    if (pos > -1)
                    {
                        minRoute.sequenceOfNodes.Insert(pos, candidateUnserved);
                        minRoute.cost += cost;
                        minRoute.load += candidateUnserved.demand;

                        unserved.Remove(candidateUnserved);
                    }
                }
            }
        }

        private bool ApplyRelocation(Solution sol)
        {

            float maxRemainingLoad = sol.routes.Max(rt => capacity - rt.load);

            Route minRoute = sol.routes.Where(rt => capacity - rt.load == maxRemainingLoad).First();

            float secondMaxLoad = sol.routes.Where(rt => rt != minRoute).Max(rt => capacity - rt.load);

            minRoute.sequenceOfNodes.ForEach(n => { if (n.demand <= secondMaxLoad && n != depot) { n.candidateForRelocation = true; } });

            
            foreach (Node n in minRoute.sequenceOfNodes)
            {
                if (n.candidateForRelocation)
                {

                    Route candidateRoute = sol.routes.Where(r => r.load + n.demand <= capacity && r != minRoute).ToList().OrderByDescending(r => r.load).FirstOrDefault();

                    if (candidateRoute != null)
                    {
                        (int pos, float cost) = FindBestInsertion(candidateRoute, n);

                        if (pos > -1)
                        {
                            minRoute.sequenceOfNodes.Remove(n);
                            minRoute.CalculateRouteCostDemand();

                            candidateRoute.sequenceOfNodes.Insert(pos, n);
                            candidateRoute.cost += cost;
                            candidateRoute.load += n.demand;
                            n.route = candidateRoute;

                            minRoute.sequenceOfNodes.ForEach(no => no.candidateForRelocation = false);
                            return true;
                        }
                    }
                }                
            }
            minRoute.sequenceOfNodes.ForEach(no => no.candidateForRelocation = false);
            return false;
        }

        private void ApplySwap (Solution sol)
        {
            float maxSlack = 0;
            Node originNode = new Node(-1, null, null, 0);
            Node targetNode = new Node(-1, null, null, 0);
            Route originRoute = sol.routes.OrderBy(rt => rt.load).First();
            Route targetRoute = new Route(-1, null, 0, 0, 0);

            
            Route rt1 = originRoute;
            for (int secondRouteIndex = 0; secondRouteIndex < sol.routes.Count; secondRouteIndex++)
            {
                Route rt2 = sol.routes[secondRouteIndex];

                if (rt1 == rt2)
                {
                    continue;
                }

                for (int firstNodeIndex = 1; firstNodeIndex < rt1.sequenceOfNodes.Count - 1; firstNodeIndex++)
                {
                    int startOfSecondNodeIndex = 1;
                   
                    for (int secondNodeIndex = startOfSecondNodeIndex; secondNodeIndex < rt2.sequenceOfNodes.Count - 1; secondNodeIndex++)
                    {

                        Node A = rt1.sequenceOfNodes[firstNodeIndex];
                        Node B = rt2.sequenceOfNodes[secondNodeIndex];

                        if ((rt1.load - A.demand + B.demand > this.capacity) || (rt2.load - B.demand + A.demand > this.capacity))
                        {
                            continue;
                        }

                        float slackCreated = A.demand - B.demand;

                        if (slackCreated > maxSlack)
                        {
                            maxSlack = slackCreated;
                            originNode = A;
                            targetNode = B;
                            targetRoute = rt2;
                        }

                    }
                }
                
            }

            if (targetNode.id > -1)
            {
                SwapNodes(originNode, targetNode, originRoute, targetRoute);
            }
            else
            {
                FindBestTwoSwap(sol, originRoute, new List<Route>(), null, 0);
            }
        }

        private void FindBestTwoSwap(Solution sol, Route minRoute, List<Route> excludedRoutes, Node excludedNode, float slack)
        {
            List<Route> candidateRoutes = sol.routes.Where(r => r != minRoute && !excludedRoutes.Contains(r)).ToList();
            bool swapFound = false;
            List<Node> nodesToSwap = minRoute.sequenceOfNodes.Where(n => n != depot && n != excludedNode).OrderBy(n => n.demand).ToList();
            foreach (Node n in nodesToSwap)
            {
               
                Node candidateNode = nodes.Where(c => c != depot && c.isRouted && !minRoute.sequenceOfNodes.Contains(c) && c.demand < n.demand && n.demand - c.demand >= slack).OrderByDescending(c => c.demand).FirstOrDefault();
                
                if (candidateNode != null)
                {
                    if (minRoute.load - n.demand + candidateNode.demand <= capacity && candidateNode.route.load + n.demand - candidateNode.demand <= capacity)
                    {
                        SwapNodes(n, candidateNode, minRoute, candidateNode.route);
                        return;
                    }
                    else
                    {
                        float slackCreated = n.demand - candidateNode.demand;
                        excludedRoutes.Add(minRoute);
                        excludedNode = candidateNode;
                        FindBestTwoSwap(sol, candidateNode.route, excludedRoutes, candidateNode, slackCreated);
                        SwapNodes(n, candidateNode, minRoute, candidateNode.route);
                        return;
                    }

                }
                
            }

        }


        private void SwapNodes(Node n1, Node n2, Route rt1, Route rt2)
        {
            rt1.sequenceOfNodes.Remove(n1);
            rt2.sequenceOfNodes.Remove(n2);

            (int pos, float cost) = FindBestInsertion(rt1, n2);
            rt1.sequenceOfNodes.Insert(pos, n2);
            rt1.CalculateRouteCostDemand();
            n2.route = rt1;

            (pos, cost) = FindBestInsertion(rt2, n1);
            rt2.sequenceOfNodes.Insert(pos, n1);
            rt2.CalculateRouteCostDemand();
            n1.route = rt2;
        }

        public (int, float) FindBestInsertion(Route rt, Node n)
        {
            int minPos = -1;
            float minCost = 1000000;
            for (int i = 1; i < rt.sequenceOfNodes.Count; i++)
            {
                float costAdded = this.costMatrix[rt.sequenceOfNodes[i - 1].id][n.id] + this.costMatrix[n.id][rt.sequenceOfNodes[i].id];
                float costRemoved = this.costMatrix[rt.sequenceOfNodes[i - 1].id][rt.sequenceOfNodes[i].id];

                float insertionCost = costAdded - costRemoved;

                if (insertionCost < minCost)
                {
                    minCost = insertionCost;
                    minPos = i;
                }
            }

            return (minPos, minCost);
        }

        private (Solution, List<Node>) SolveWithBestFit (List<Node> selectedCustomers)
        {
            selectedCustomers.ForEach(x => { x.isRouted = false; });

            List<Route> routes = new List<Route>();
            for (int i = 0; i < this.vehicles; i++)
            {
                List<Node> sequenceOfNodes = new List<Node> { depot, depot };
                Route rt = new Route(i, sequenceOfNodes, this.capacity, 0, 0);
                routes.Add(rt);
            }

            Solution sol = new Solution();

            selectedCustomers = selectedCustomers.OrderByDescending(c => c.demand).ToList();
            List<Node> remainingCustomers = new List<Node>(selectedCustomers);

            foreach (Node c in selectedCustomers)
            {
                Route bestFitRoute = null;
                float minRemainingCapacity = float.MaxValue;

                foreach (Route rt in routes)
                {
                    if (rt.capacity - rt.load >= c.demand && rt.capacity - rt.load - c.demand < minRemainingCapacity)
                    {
                        bestFitRoute = rt;
                        minRemainingCapacity = rt.capacity - rt.load - c.demand;
                    }
                }

                if (bestFitRoute != null)
                {
                    (int pos, float cost) = FindBestInsertion(bestFitRoute, c);
                    bestFitRoute.sequenceOfNodes.Insert(pos, c);
                    //bestFitRoute.cost += cost;
                    //bestFitRoute.load += c.demand;
                    bestFitRoute.CalculateRouteCostDemand();

                    remainingCustomers.Remove(c);
                }
            }

            sol.routes = routes;
            sol.UpdateCost();

            foreach(Route rt in sol.routes)
            {
                rt.sequenceOfNodes.ForEach(n => n.isRouted = true);
            }

            return (sol, remainingCustomers);
        }
    }
}
