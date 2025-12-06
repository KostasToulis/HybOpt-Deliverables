using System;
using System.Collections.Generic;
using System.Text;
using static F_CVRP.Setup;

namespace F_CVRP
{
    public class Model
    {
        public int numNodes { get; set; }
        public int numFam { get; set; }
        public int numReq { get; set; }
        public int capacity { get; set; }
        public int vehicles { get; set; }
        public List<int> famMembers { get; set; }
        public List<int> famReq { get; set; }
        public List<float> famDemand { get; set; }
        public List<List<float>> costMatrix { get; set; }
        public Node depot { get; set; }
        public List<Node> nodes { get; set; }
        public List<Node> customers { get; set; }
        public List<Family> families { get; set; }
        public Run_config runData { get; set; }

        public bool CheckSolutionFeasibility(Solution sol)
        {
            //ReportSolution(sol);
            return CheckFamilyDemands(sol) && CheckRouteCapacity(sol) && CheckCustomerUniqueness(sol) && CheckVehicles(sol);
        }

        public bool CheckFamilyDemands(Solution sol)
        {
            List<int> visitedPerFamily = new List<int>(new int[this.numFam]);

            foreach (Route rt in sol.routes)
            {
                if (rt.load > this.capacity)
                {
                    return false;
                }
                for (int i = 1; i < rt.sequenceOfNodes.Count - 1; i++)
                {
                    visitedPerFamily[rt.sequenceOfNodes[i].family.id] += 1;
                }
            }

            for (int i = 0; i < visitedPerFamily.Count; i++)
            {
                if (visitedPerFamily[i] != this.famReq[i])
                {
                    Console.WriteLine("Family demand violation for family: " + i);
                    return false;
                }
            }
            return true;
        }

        public bool CheckRouteCapacity(Solution sol)
        {
            foreach (Route rt in sol.routes)
            {
                float rtCost = 0;
                foreach (Node n in rt.sequenceOfNodes)
                {
                    rtCost += n.demand;
                }
                if (rtCost > rt.capacity)
                {
                    Console.WriteLine("Capacity violation in route: " + rt.id);
                    return false;
                }
            }
            return true;
        }


        public bool CheckCustomerUniqueness(Solution sol)
        {
            List<Node> customers = new List<Node>();
            foreach (Route rt in sol.routes)
            {
                for (int i = 1; i < rt.sequenceOfNodes.Count - 1; i++)
                {
                    if (customers.Contains(rt.sequenceOfNodes[i]))
                    {
                        Console.WriteLine("Customer uniqueness violation for customer: " + rt.sequenceOfNodes[i].id);
                        return false;
                    }
                    customers.Add(rt.sequenceOfNodes[i]);
                }
            }
            return true;
        }


        public bool CheckVehicles(Solution sol)
        {
            if (sol.routes.Count != this.vehicles)
            {
                return false;
            }
            return true;
        }


        public void ReportSolution(Solution sol)
        {

            List<Node> newCustomers = new List<Node>();
            Console.WriteLine();
            //Console.WriteLine("Reported solution cost: "+ sol.cost);
            float solCost = 0;
            foreach (Route rt in sol.routes)
            {
                float rtCost = 0;
                List<int> nodeIds = new List<int>();

                //Console.WriteLine("Reported route cost: "+ rt.cost);  
                for (int i = 0; i < rt.sequenceOfNodes.Count - 1; i++)
                {
                    nodeIds.Add(rt.sequenceOfNodes[i].id);
                    rtCost += rt.sequenceOfNodes[i].costs[rt.sequenceOfNodes[i + 1].id];
                    newCustomers.Add(rt.sequenceOfNodes[i]);
                }
                nodeIds.Add(0);

                Console.WriteLine("Route " + rt.id);
                foreach (int id in nodeIds)
                {
                    Console.Write(id);
                    Console.Write("-");
                }
                //nodeIds.ForEach(Console.Write);
                Console.WriteLine();
                solCost += rtCost;

            }
            Console.WriteLine();
            Console.WriteLine("Solution cost: " + solCost);
            Console.WriteLine();
            Console.WriteLine("///////////////////////////////////////////////////////////////////////////////////////////");

            //this.selectedCustomers = newCustomers;
        }
    }

    public class Node
    {
        public Node(int i, Family family, List<float> costs, float demand)
        {
            this.id = i;
            this.family = family;
            this.costs = costs;
            this.demand = demand;
            this.timesRemoved = 0;
            this.isRouted = false;
        }

        public int id { get; set; }
        public bool isRouted { get; set; }
        public bool isDepot = false;
        public Family family { get; set; }
        public List<float> costs { get; set; }
        public float demand { get; set; }
        public Route route { get; set; }
        public int positionInRoute { get; set; }
        public float sumCost { get; set; }
        public int timesInserted { get; set; }
        public int timesRemoved { get; set; }
        public int timesInFinalSol { get; set; }
        public int timesInNewSol { get; set; }
        public int timesInBestSol { get; set; }

        public bool selectedForRemoval = false;

        public bool candidateForRelocation = false;
    }

    public class Family
    {
        public Family(int i, List<Node> nodes, float v1, int v2)
        {
            this.id = i;
            this.nodes = nodes;
            this.demand = v1;
            this.required = v2;
        }

        public int id { get; set; }
        public List<Node> nodes { get; set; }
        public float demand { get; set; }
        public int required { get; set; }
    }

    [Serializable]
    public class Solution
    {
        public Solution()
        {
            this.routes = new List<Route>();
        }

        public float cost { get; set; }
        public List<Route> routes { get; set; }
        public bool feasible = true;
        public string origin = "Initial";

        public Solution DeepCopy()
        {
            var newSolution = new Solution();
            newSolution.cost = this.cost;
            newSolution.routes = new List<Route>();
            newSolution.feasible = this.feasible;
            newSolution.origin = this.origin;
            foreach (var route in this.routes)
            {
                int id = route.id;
                List<Node> nodes = new List<Node>(route.sequenceOfNodes);
                int capacity = route.capacity;
                float cost = route.cost;
                float load = route.load;
                newSolution.routes.Add(new Route(id, nodes, capacity, cost, load));
            }

            return newSolution;
        }

        public void UpdateCost()
        {
            float c = 0;
            foreach(Route rt in this.routes)
            {
                c += rt.cost;
            }
            this.cost = c;
        }
    }

    [Serializable]
    public class Route
    {
        public Route(int id, List<Node> nodes, int cap, float cost, float load)
        {
            this.id = id;
            this.sequenceOfNodes = nodes;
            this.capacity = cap;
            this.cost = cost;
            this.load = load;
        }

        public int id { get; set; }
        public List<Node> sequenceOfNodes { get; set; }
        public float load { get; set; }
        public float cost { get; set; }
        public int capacity { get; set; }

        public void CalculateRouteCostDemand()
        {
            float tc = 0;
            float tl = 0;
            for (int i = 0; i < this.sequenceOfNodes.Count - 1; i++)
            {
                Node A = this.sequenceOfNodes[i];
                Node B = this.sequenceOfNodes[i + 1];
                tc += this.sequenceOfNodes[i].costs[this.sequenceOfNodes[i + 1].id];
                tl += B.demand;
            }
            this.load = tl;
            this.cost = tc;
        }

        public void CalculateRouteDemand()
        {
            float demand = 0;
            foreach (Node node in this.sequenceOfNodes)
            {
                demand = demand + node.demand;
            }

            this.load = demand;
        }
    }

    public class Flow
    {

        public Flow(Node n1, Node n2, float c)
        {
            firstNode = n1;
            secondNode = n2;
            cost = c;
        }

        Node firstNode { get; set; }
        Node secondNode { get; set; }
        float cost { get; set; }

        private void Reverse()
        {
            cost = secondNode.costs[firstNode.id];
            (firstNode, secondNode) = (secondNode, firstNode);
        }

    }

}

