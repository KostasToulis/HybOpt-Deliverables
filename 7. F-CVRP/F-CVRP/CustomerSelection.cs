using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;
using System.Diagnostics;
using F_CVRP;
using System.Reflection;

namespace F_CVRP
{
    public class CustomerSelection 
    {
        public Model m { get; set; }
        public int[,] adjMatrix { get; set; }
        public List<List<int>> perturbationMatrix { get; set; }
        public double a { get; set; }


        public CustomerSelection(Solver s)
        {
            m = s.m;
            adjMatrix = new int[s.m.numNodes+1, s.m.numNodes+1];
            perturbationMatrix = new List<List<int>>();

            for (int i = 0; i <= m.customers.Count; i++)
            {
                List<int> row = new List<int>(new int[m.customers.Count+1]);
                perturbationMatrix.Add(row);
            }
        }


        public List<Node> InitialSelection()
        {
            foreach (Node customer in this.m.customers)
            {
                customer.sumCost = customer.costs.Sum();
            }

            List<Node> customers = m.customers.OrderBy(c => c.sumCost).ToList();

            //Random random = new Random();
            //customers = customers.OrderBy(c => random.Next()).ToList();

            List<int> familyReqs = new List<int>(new int[this.m.numFam]);
            List<Node> candidateCustomers = new List<Node>();

            foreach (Node customer in customers)
            {
                if (familyReqs[customer.family.id] >= this.m.families[customer.family.id].required)
                {
                    continue;
                }
                candidateCustomers.Add(customer);
                familyReqs[customer.family.id] += 1;
            }

            //UpdatePerturbationMatrix(candidateCustomers);
            return candidateCustomers;
        }

        public List<Node> SubsetSelection()
        {
            List<Node> selectedSubset = new List<Node>();

            foreach (Family l in m.families)
            {
                l.nodes.OrderByDescending(n => n.timesInBestSol);
                int numToAdd = (int)Math.Floor(0.75 * l.required);
                List<Node> nodesToAdd = l.nodes.Take(numToAdd).ToList();
                selectedSubset.AddRange(nodesToAdd);
            }

            return selectedSubset;
        }

        public List<Node> RandomSelection()
        {
            Random rand = new Random();
            List<Node> customers = m.customers.OrderBy(c => rand.Next()).ToList();

            List<int> familyReqs = new List<int>(new int[this.m.numFam]);
            List<Node> candidateCustomers = new List<Node>();

            foreach (Node customer in customers)
            {
                if (familyReqs[customer.family.id] >= this.m.families[customer.family.id].required)
                {
                    continue;
                }
                candidateCustomers.Add(customer);
                familyReqs[customer.family.id] += 1;
            }

            return candidateCustomers;
        }


        //public Solution NewRandomSolution(Solution s)
        //{
        //    Solution newSol = s.DeepCopy();
        //    foreach (Route rt in newSol.routes)
        //    {
        //        for (int i = 1; i < rt.sequenceOfNodes.Count() - 1; i++)
        //        {
        //            double removeChance = this.random.NextDouble();
        //            if (removeChance < (double)this.a)
        //            {

        //            }
        //        }
        //    }
        //    return newSol;
        //}

        public Solution NewPerturbationSolution(Solution s)
        {
            //Stopwatch stopwatch = new Stopwatch();
            //stopwatch.Start();

            UpdatePerturbationMatrix(s);
            //ReportSolution(s);

            UpdateAdjMatrix(s);

            this.m.customers.ForEach(customer => {customer.isRouted = false; });

            s.routes.ForEach(
                rt =>
                {
                    rt.sequenceOfNodes.ForEach(
                        n => { n.isRouted = true; }
                    );
                }
            );

            List<Node> removedCustomers = SelectRemovedCustomers(s.routes, 0.5);

            removedCustomers.ForEach(n => { n.timesRemoved++; });
            //removedCustomers.ForEach(n => { n.isRouted = false; });


            List<Node> insertedCustomers = SelectInsertedCustomers(removedCustomers); //GetRandomSubsetWithFamilyDistribution(removedCustomers);

            //insertedCustomers.ForEach(n => { n.isRouted = true; });


            Solution newSol = InsertNewSelections(s, removedCustomers, insertedCustomers);


            newSol.cost = 0;

            UpdateNewSolCost(newSol);


            this.m.customers.ForEach(customer => { customer.isRouted = false; });
            newSol.routes.ForEach(
                rt =>
                {
                    rt.sequenceOfNodes.ForEach(
                        n => { 
                            n.timesInNewSol++; 
                            n.isRouted = true;
                        }
                    );
                }
            );



            //CheckSolutionFeasibility(newSol);
            Console.WriteLine("New Solution Cost: " + newSol.cost);
            return newSol;
        }

        private void UpdateAdjMatrix(Solution sol)
        {
            foreach (Route rt in sol.routes)
            {
                for (int i = 0; i < rt.sequenceOfNodes.Count - 1; i++)
                {
                    this.adjMatrix[rt.sequenceOfNodes[i].id, rt.sequenceOfNodes[i + 1].id]++;
                }
            }
        }

        private void UpdateNewSolCost(Solution newSol)
        {
            foreach (Route rt in newSol.routes)
            {
                float tc = 0;
                float tl = 0;
                for (int i = 0; i < rt.sequenceOfNodes.Count - 1; i++)
                {
                    Node A = rt.sequenceOfNodes[i];
                    Node B = rt.sequenceOfNodes[i + 1];
                    tc += this.m.costMatrix[A.id][B.id];
                    tl += A.demand;
                }
                rt.load = tl;
                rt.cost = tc;
                newSol.cost += tc;
            }
        }

        public List<Node> SelectRemovedCustomers(List<Route> routes, double fraction)
        {

            //List<Node> customers = new List<Node>();

            //foreach (Route rt in routes)
            //{
            //    for (int i = 1; i < rt.sequenceOfNodes.Count - 1; i++)
            //    {
            //        customers.Add(rt.sequenceOfNodes[i]);
            //    }
            //}

            List<Node> removedSelection = new List<Node>();

            foreach (Family f in this.m.families)
            {
                int numRouted = f.nodes.Where(n => n.isRouted).Count();
                int numNotRouted = f.nodes.Where(n => !n.isRouted).Count();

                //List<Node> famCandidates = f.nodes.Where(n => n.isRouted).ToList().Take(numNotRouted).ToList();

                if ((double)numNotRouted / numRouted >= fraction)
                {
                    int numberToSelect = (int)(f.nodes.Count * fraction);
                    List<Node> famCandidates = f.nodes.Where(n => n.isRouted).ToList().Take(numberToSelect).ToList();
                    removedSelection.AddRange(famCandidates);
                }
                else
                {
                    List<Node> famCandidates = f.nodes.Where(n => n.isRouted).ToList().Take(numNotRouted).ToList();
                    removedSelection.AddRange(famCandidates);
                }
                
            }

            return removedSelection;

            //Random random = new Random();
            //List<Node> shuffledItems = this.customers.OrderBy(x => this.random.Next()).ToList();

            //return shuffledItems.Take(numberToSelect).ToList();
        }



        private List<Node> SelectInsertedCustomers(List<Node> removedCustomers)
        {
            List<int> familyReqs = new List<int>(new int[this.m.numFam]);
            foreach (Node rn in removedCustomers)
            {
                familyReqs[rn.family.id]++;
            }

            List<Node> candidates = this.m.customers.OrderByDescending(x => x.timesInBestSol).Where(x => x.isRouted == false).ToList();

            List<Node> newSelections = new List<Node>();

            for (int i = 0; i < familyReqs.Count(); i++)
            {
                if (familyReqs[i] > 0)
                {
                    List<Node> famCandidates = candidates.Where(c => c.family.id == i).ToList().Take(familyReqs[i]).ToList();
                    newSelections.AddRange(famCandidates);
                }
            }

            //foreach (Node n in candidates)
            //{
            //    if (familyReqs[n.family.id] > 0)
            //    {
            //        newSelections.Add(n);
            //        familyReqs[n.family.id]--;
            //    }
            //}

            //for (int i = 0; i < familyReqs.Count(); i++)
            //{
            //    if (familyReqs[i] > 0)
            //    {
            //        candidates = removedCustomers.OrderByDescending(x => x.timesInBestSol).Where(c => !newSelections.Contains(c) && c.family.id == i).ToList().Take(familyReqs[i]).ToList();
            //        newSelections.AddRange(candidates);
            //        familyReqs[i]--;
            //    }
            //}

            //foreach (Node n in removedCustomers)
            //{
            //    Console.Write(n.family.id + " ");
            //}
            //Console.WriteLine();
            //foreach (Node n in newSelections)
            //{
            //    Console.Write(n.family.id + " ");
            //}
            return newSelections;

        }


        private Solution InsertNewSelections (Solution sol, List<Node> removed, List<Node> inserted)
        {
            
            Solution newSol = sol.DeepCopy();

            foreach (Route rt in newSol.routes) { 
                for (int i = 1; i < rt.sequenceOfNodes.Count - 1; i++)
                {
                    Node r = rt.sequenceOfNodes[i];
                    if (removed.Contains(r))
                    {
                        

                        Node replacement = FindBestReplacement(inserted, r, rt, i);


                        rt.sequenceOfNodes.RemoveAt(i);
                        rt.load -= r.demand;
                        r.isRouted = false;

                        if (replacement != r)
                        {

                            inserted.Remove(replacement);
                            rt.sequenceOfNodes.Insert(i, replacement);
                            rt.load += replacement.demand;
                            replacement.isRouted = true;
                        }

                        //replacement.isRouted = true;
                    }
                }
            }

            if (inserted.Count() > 0)
            {
                foreach (Node node in inserted) 
                { 
                    (Route rt, int pos, float insertCost) = FindBestInsertionPoint(node, newSol);
                    if (pos != -1) 
                    {
                        rt.sequenceOfNodes.Insert(pos, node);
                        rt.load += node.demand;
                        newSol.cost += insertCost;
                    }
                }

            }


            if (!CheckSolutionFeasibility(newSol))
            {
                ReportSolution(newSol);
            }
            return newSol;

        }


        private Solution InsertWithPerturbation(Solution sol, List<Node> removed, List<Node> inserted)
        {

            Solution newSol = sol.DeepCopy();
            
            foreach (Node candidate in inserted)
            {
                Route minRoute = null;
                int minPert = int.MaxValue;

                foreach(Route rt in newSol.routes)
                {
                    int pert = 0;
                    foreach(Node n in rt.sequenceOfNodes)
                    {
                        pert += this.perturbationMatrix[candidate.id][n.id];
                    }
                    if (pert < minPert)
                    {
                        pert = minPert;
                        minRoute = rt;
                    }
                }

                int insertPos = 0;
                float minInsertCost = float.MaxValue;

                for (int i = 1; i < minRoute.sequenceOfNodes.Count() - 1; i++)
                {
                    Node n = minRoute.sequenceOfNodes[i];
                    if (removed.Contains(n))
                    {
                        if (minRoute.load + candidate.demand - n.demand <= this.m.capacity)
                        {
                            float insertCost = candidate.costs[minRoute.sequenceOfNodes[i - 1].id] + candidate.costs[minRoute.sequenceOfNodes[i + 1].id];
                            if (insertCost < minInsertCost)
                            {
                                minInsertCost = insertCost;
                                insertPos = i;
                            }
                        }
                    }
                }

                minRoute.sequenceOfNodes.RemoveAt(insertPos);
                minRoute.sequenceOfNodes.Insert(insertPos, candidate);


            }


            if (!CheckSolutionFeasibility(newSol))
            {
                ReportSolution(newSol);
            }
            return newSol;

        }

        private (Route, int, float) FindBestInsertionPoint(Node node, Solution sol) 
        {
            float minCost = 1000000;
            int minPos = -1;
            Route minRoute = null;
            foreach (Route rt in sol.routes) 
            {
                if (rt.load + node.demand > rt.capacity)
                {
                    continue;
                }

                for (int i = 1; i < rt.sequenceOfNodes.Count; i++)
                {
                    float costAdded = this.m.costMatrix[rt.sequenceOfNodes[i - 1].id][node.id] + this.m.costMatrix[node.id][rt.sequenceOfNodes[i].id];
                    float costRemoved = 0;
                    if (rt.sequenceOfNodes[i - 1] != rt.sequenceOfNodes[i])
                    {
                        costRemoved = this.m.costMatrix[rt.sequenceOfNodes[i - 1].id][rt.sequenceOfNodes[i].id];
                    }

                    float insertionCost = costAdded - costRemoved;

                    if (insertionCost < minCost)
                    {
                        minCost = insertionCost;
                        minPos = i;
                        minRoute = rt;
                    }
                }

            }

            return (minRoute, minPos, minCost);

        }



        private Node FindBestReplacement(List<Node> candidates, Node removedNode, Route rt, int i)
        {

            Node minNode = removedNode;
            float minCost = 1000000000;

            foreach (Node candidate in candidates)
            {
                if (candidate.demand - removedNode.demand + rt.load <= rt.capacity)
                {
                    float replaceCost = rt.sequenceOfNodes[i - 1].costs[candidate.id]*(1+(float)0.05*this.adjMatrix[rt.sequenceOfNodes[i - 1].id, candidate.id]) + rt.sequenceOfNodes[i + 1].costs[candidate.id] * (1 + (float)0.05 * this.adjMatrix[rt.sequenceOfNodes[i + 1].id, candidate.id]);
                    if (replaceCost < minCost)
                    {
                        minCost = replaceCost;
                        minNode = candidate;
                    }
                }
               
            }
            return minNode;
        }

        private void UpdatePerturbationMatrix(Solution s)
        {

            foreach (Route rt in s.routes)
            {
                List<Node> customers = new List<Node>();

                for (int i = 1; i < rt.sequenceOfNodes.Count - 1; i++)
                {
                    customers.Add(rt.sequenceOfNodes[i]);
                }

                List<Tuple<int, int>> pairs = new List<Tuple<int, int>>();

                for (int i = 0; i < customers.Count; i++)
                {
                    for (int j = i + 1; j < customers.Count; j++)
                    {
                        pairs.Add(Tuple.Create(customers[i].id, customers[j].id));
                    }
                }

                foreach (Tuple<int, int> pair in pairs)
                {
                    this.perturbationMatrix[pair.Item1][pair.Item2] += 1;
                }

            }

           
        }

        public bool CheckSolutionFeasibility(Solution sol)
        {
            return CheckFamilyDemands(sol) && CheckRouteCapacity(sol) && CheckCustomerUniqueness(sol);
        }

        private bool CheckFamilyDemands(Solution sol)
        {
            List<int> visitedPerFamily = new List<int>(new int[this.m.numFam]);

            foreach (Route rt in sol.routes)
            {
                if (rt.load > this.m.capacity)
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
                if (visitedPerFamily[i] != this.m.famReq[i])
                {
                    Console.WriteLine("Family demand violation for family: " + i);
                    return false;
                }
            }
            return true;
        }

        private bool CheckRouteCapacity(Solution sol)
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


        private bool CheckCustomerUniqueness(Solution sol)
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

        private List<Node> GetRandomSubsetWithFamilyDistribution(List<Node> removedCustomers)
        {
            List<Node> randomSubset = new List<Node>();
            foreach (Node removed in removedCustomers)
            {
                List<Node> candidates = removed.family.nodes.Where(x => x.isRouted == false).ToList();
                if (candidates != null && candidates.Count > 0)
                {
                    Node candidate = candidates.OrderByDescending(x => x.timesInBestSol).First();
                    candidate.isRouted = true;
                    randomSubset.Add(candidate);

                }
                else
                {
                    randomSubset.Add(removed);
                }

            }
            return randomSubset;
        }

    }
}
