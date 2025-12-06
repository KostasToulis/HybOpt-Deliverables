using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using static F_CVRP.AdaptiveMemory;

namespace F_CVRP
{
    public class AdaptiveMemory 
    {

        public Model m { get; set; }
        public int poolSize { get; set; }
        public List<RouteMember> members { get; set; }
        public List<Chain> chains { get; set; }
        public HashSet<int> chainIds { get; set; }
        public Random random { get; set; }
        public int iteration { get; set; }
        public Dictionary<int, Chain> chainDictionary { get; set; }
        public Dictionary<int, List<Chain>> memberChains { get; set; }

        public AdaptiveMemory(Solver s)
        {

            m = s.m;
            poolSize = s.m.runData.poolSize;//s.numReq * 5;
            members = new List<RouteMember>();
            chainIds = new HashSet<int>();
            chains = new List<Chain>();
            random = s.m.runData.random;
            iteration = 0;        
            chainDictionary = new Dictionary<int, Chain>();
            memberChains = new Dictionary<int, List<Chain>>();
        }


        [Serializable]
        public class RouteMember
        {
            public int id { get; set; }
            public List<Node> route { get; set; }
            public float solCost { get; set; }

            //public RouteMember Copy()
            //{
            //    RouteMember rtm = new RouteMember(new Route(), this.solCost);
            //    rtm.id = this.id;
            //    rtm.solCost = this.solCost;
            //    rtm.route = new List<Node>();
            //    foreach (Node n in this.route)
            //    {
            //        rtm.route.Add(n);
            //    }
            //    return rtm;
            //}

            public RouteMember(Route rt, float cost)
            {
                this.route = rt.sequenceOfNodes;
                this.solCost = cost;
                this.id = ComputeListHash(rt.sequenceOfNodes);
            }
        }

        public class Chain
        {
            public int id { get; set; }
            public List<Node> nodes { get; set; }
            public float solCost { get; set; }
            public float demand { get; set; }
            public float adjCost { get; set; }
            public float cost { get; set; }
            public int freq { get; set; }
            public int timesExtracted { get; set; }
            public int memberId  { get; set; }
            public float routeLoad { get; set; }

            public Chain (List<Node> n, float s, int f)
            {
                id = ComputeListHash(n); //n.Select(node => node.id).GetHashCode();
                nodes = n;
                solCost = s;
                freq = f;
                demand = nodes.Sum(c => c.demand);
                adjCost = CalculateAdjCost(n);
                cost = CalculateCost(n);
                //this.memberId = memberId;

            }

            private float CalculateAdjCost(List<Node> nodes)
            {
                float adjCost = 0;

                if (nodes.Count() >= 2)
                {
                    for (int i = 0; i < nodes.Count() - 1; i++)
                    {
                        adjCost += nodes[i].costs[nodes[i + 1].id];
                        //adjCost = adjCost / (float)(1 + (0.05 * i));
                    }

                    //adjCost = (float)adjCost / (nodes.Count() + nodes.Count() - 2);
                    adjCost = (float)adjCost / nodes.Count;
                }
                
                return adjCost;
            }

            private float CalculateCost(List<Node> nodes)
            {
                float cost = 0;               
                for (int i = 0; i < nodes.Count() - 1; i++)
                {
                    cost += nodes[i].costs[nodes[i + 1].id];
                    
                }
                return cost;
            }
        }

        public class ChainBin
        {
            public List<Chain> chains { get; set; }
            public float load { get; set; }
        }


        public class ListComparer : IEqualityComparer<List<int>>
        {
            public bool Equals(List<int> x, List<int> y)
            {
                if (x == null || y == null)
                    return false;
                if (x.Count != y.Count)
                    return false;
                for (int i = 0; i < x.Count; i++)
                {
                    if (x[i] != y[i])
                        return false;
                }
                return true;
            }

            public int GetHashCode(List<int> obj)
            {
                if (obj == null)
                    return 0;
                int hash = 17;
                foreach (int i in obj)
                {
                    hash = hash * 31 + i;
                }
                return hash;
            }
        }


        private static int ComputeListHash(List<Node> nodes)
        {
            return nodes.Select(n => n.id)
                        .Aggregate(17, (hash, id) => hash * 31 + id.GetHashCode());
        }


        public (Solution, List<Node>) ConstructNewSolWithTrigrams(Solution sol, List<Node> unserved)
        {
            Solution oldSol = sol.DeepCopy();
            List<Chain> newTrigrams = UpdateTrigrams(sol);

            //UpdateMembers(sol);

            PruneExcessTrigrams();

            SelectTrigramExtraction(newTrigrams);

            List<Node> candidateNodes = SelectCandidateNodes();
            List<Node> unservedCustomers = new List<Node>();

            Solution newSol = ExtractAndConstructTrigrams(sol, candidateNodes);

            if (!m.CheckCustomerUniqueness(newSol) || !m.CheckRouteCapacity(newSol))
            {
                return (oldSol, unserved);
            }

            if (!m.CheckFamilyDemands(newSol))
            {
                unservedCustomers = CreateUnservedSet(newSol);
            }

            //Console.WriteLine("New solution cost: " + newSol.cost);

            return (newSol, unservedCustomers);
        }


        private void PruneExcessTrigrams()
        {
            chains = chains.OrderBy(c => c.solCost).ToList();
            if (chains.Count() > poolSize)
            {
                List<Chain> removedChains = chains.TakeLast(chains.Count() - poolSize).ToList();
                foreach (Chain c in removedChains)
                {
                    chains.Remove(c);
                }
            }
        }


        private void RemoveExcessMembers()
        {
            members = members.OrderBy(m => m.solCost).ToList();

            if (members.Count() > poolSize)
            {
                List<RouteMember> removedMembers = members.TakeLast(members.Count() - poolSize).ToList();
                foreach (RouteMember rm in removedMembers)
                {
                    members.Remove(rm);

                    if (chainDictionary.ContainsKey(rm.id))
                    {
                        chainDictionary.Remove(rm.id);
                    }

                    this.chains.Where(c => c.memberId == rm.id).ToList().ForEach(c => c.freq--);

                    this.chains.RemoveAll(c => c.memberId == rm.id);

                }
            }
        }

        private List<Chain> UpdateTrigrams(Solution sol)
        {
            List<Chain> solutionChains = new List<Chain>();
            foreach (Route rt in sol.routes)
            {
                solutionChains.AddRange(GetRouteTrigrams(rt, sol.cost));
            }

            foreach (Chain chain in solutionChains)
            {
                Chain existingChain = this.chains.Find(c => c.id == chain.id);
                if (existingChain != null)
                {
                    existingChain.freq++;
                    if (existingChain.solCost > sol.cost)
                    {
                        existingChain.solCost = sol.cost;
                    }
                }
                else
                {
                    this.chains.Add(chain);
                }
            }

            return solutionChains;
        }


        private List<Chain> GetRouteTrigrams(Route rt, float cost)
        {
            List<Chain> routeChains = new List<Chain>();

            //int id = rt.sequenceOfNodes.GetHashCode();

            for (int i = 1; i < rt.sequenceOfNodes.Count() - 1; i++)
            {
                Node A = rt.sequenceOfNodes[i - 1];
                Node B = rt.sequenceOfNodes[i];
                Node C = rt.sequenceOfNodes[i + 1];

                List<Node> nodeList = new List<Node> { A, B, C };

                Chain c = new Chain(nodeList, cost, 1);

                c.routeLoad = rt.load;

                routeChains.Add(c);
            }

            return routeChains;
        }

        private void SelectTrigramExtraction(List<Chain> solChains)
        {
            solChains = solChains.OrderBy(c => random.Next()).OrderBy(c => c.timesExtracted + c.freq).ToList();
            List<Chain> chainsForRemoval = solChains.Take((int)(solChains.Count() * 0.35)).ToList();
            chainsForRemoval.ForEach(c =>
            {
                c.nodes[1].selectedForRemoval = true;
                c.timesExtracted++;
            });

        }


        private List<Node> SelectCandidateNodes()
        {
            return this.m.nodes.Where(n => n.isRouted == false || n.selectedForRemoval).ToList();
        }


        private Solution ExtractAndConstructTrigrams(Solution sol, List<Node> candidateNodes)
        {

            foreach (Route rt in sol.routes)
            {
                for (int i = 1; i < rt.sequenceOfNodes.Count() - 1; i++)
                {

                    Node A = rt.sequenceOfNodes[i - 1];
                    Node B = rt.sequenceOfNodes[i];
                    Node C = rt.sequenceOfNodes[i + 1];

                    if (B.selectedForRemoval)
                    {
                        Node N = SelectBestReplacement(A, B, C, candidateNodes, rt.load);
                        candidateNodes.Remove(N);

                        B.isRouted = false;
                        B.selectedForRemoval = false;

                        rt.sequenceOfNodes.RemoveAt(i);
                        rt.sequenceOfNodes.Insert(i, N);
                        N.isRouted = true;

                        float costRemoved = A.costs[B.id] + B.costs[C.id];
                        float costAdded = A.costs[N.id] + N.costs[C.id];

                        rt.cost = rt.cost - costRemoved + costAdded;
                    }
                }
            }

            sol.cost = sol.routes.Sum(rt => rt.cost);
            return sol;
        }

        private Node SelectBestReplacement(Node A, Node B, Node C, List<Node> candidateNodes, float load)
        {
            List<Chain> candidateChains = new List<Chain>();
            List<Node> familyNodes = candidateNodes.Where(c => c.family == B.family).ToList();
            Node replacement = B;

            foreach (Node R in familyNodes)
            {
                if (R == B)
                {
                    continue;
                }

                List<Node> nodeList = new List<Node> { A, R, C };
                Chain candidate = new Chain(nodeList, 0, 0);
                Chain existing = this.chains.Find(c => c.id == candidate.id);

                if (existing != null)
                {
                    candidate.freq = existing.freq;
                }

                candidateChains.Add(candidate);
            }

            if (candidateChains.Count() > 0)
            {
                candidateChains = candidateChains.OrderBy(c => c.cost).OrderByDescending(c => c.freq).ToList();
                replacement = candidateChains[0].nodes[1];
            }

            return replacement;
        }

        private List<Node> CreateUnservedSet(Solution sol)
        {
            int[] famEx = new int[m.numFam];
            List<Node> unserved = new List<Node>();

            foreach (Route rt in sol.routes)
            {
                for (int i = 1; i < rt.sequenceOfNodes.Count() - 1; i++)
                {
                    famEx[rt.sequenceOfNodes[i].family.id]++;
                }
            }

            for (int i = 0; i < m.numFam; i++)
            {
                if (famEx[i] < m.famReq[i])
                {
                    unserved.AddRange(m.nodes.Where(n => n.isRouted == false && n.family.id == i).OrderBy(n => random.Next()).Take(m.famReq[i] - famEx[i]));
                }
            }

            return unserved;
        }

        public Solution ConstructPartialSol(Solution sol)
        {
            //if (CheckSolutionFeasibility(sol))
            //{
            //    UpdateMembers(sol);
            //}

            //Console.WriteLine("Old sol: \n");
            //sol.routes.ForEach(r => {
            //    r.sequenceOfNodes.ForEach(n => Console.Write(n.id + "-"));
            //    Console.WriteLine();
            //});

            Solution newSol = ConstructSolFromChains(sol);

            if (newSol.routes.Count != m.vehicles)
            {
                int emptyRoutes = m.vehicles - newSol.routes.Count;
                for (int i = 0; i < emptyRoutes; i++)
                {
                    Route newRt = new Route(newSol.routes.Count, new List<Node> { m.depot, m.depot }, m.capacity, 0, 0);
                    newSol.routes.Add(newRt);
                }
            }
            //Console.WriteLine();
            //Console.WriteLine("New sol: \n");
            //newSol.routes.ForEach(r => {
            //    r.sequenceOfNodes.ForEach(n => Console.Write(n.id + "-"));
            //    Console.WriteLine();
            //});

            return newSol;

        }

        public void UpdateMembers(Solution sol)
        {
            if (m.CheckSolutionFeasibility(sol))
            {
                this.members.OrderByDescending(m => m.solCost);

                if (this.members.Count < this.poolSize || sol.cost < this.members[0].solCost)
                {
                    foreach (Route rt in sol.routes)
                    {

                        if (this.members.Count >= poolSize)
                        {
                            RouteMember rem = this.members[0];
                            this.members.Remove(rem);
                            RemoveChains(rem);
                        }

                        RouteMember newMem = new RouteMember(rt, sol.cost);

                        if (!this.memberChains.ContainsKey(newMem.id))
                        {
                            this.members.Add(newMem);
                            AddChains(newMem);
                        }


                    }
                }
            }       

        }


        private void RemoveChains(RouteMember rem)
        {
            List<Chain> chainsToRemove = this.memberChains[rem.id];
            foreach (Chain c in chainsToRemove)
            {
                c.freq--;
                if (c.freq == 0)
                {
                    this.chains.Remove(c);
                    this.chainDictionary.Remove(c.id);
                }
            }
            this.memberChains.Remove(rem.id);
        }
        

        private void AddChains(RouteMember newMem)
        {
            List<Chain> memChains = new List<Chain>();

            int len = newMem.route.Count();

            int i = 0;

            while (i < newMem.route.Count - 2)
            {
                for (int j = 3; j <= newMem.route.Count - i; j++)
                {
                    List<Node> nodesToAdd = new List<Node>();

                    Chain newChain = new Chain(newMem.route.GetRange(i, j), newMem.solCost, 1);

                    if (chainDictionary.ContainsKey(newChain.id))
                    {
                        Chain existingChain = this.chainDictionary[newChain.id];
                        existingChain.freq++;
                        if (newChain.solCost < existingChain.solCost)
                        {
                            existingChain.solCost = newChain.solCost;
                        }
                    }
                    else
                    {
                        this.chains.Add(newChain);
                        this.chainDictionary.Add(newChain.id, newChain);
                    }

                    memChains.Add(newChain);
                }

                i++;
            }

            this.memberChains.Add(newMem.id, memChains);
        }


        private Solution ConstructSolFromChains(Solution sol)
        {
            this.m.nodes.ForEach(n => n.isRouted = false);
            List<Chain> includedChains = new List<Chain>();
            List<Node> includedCustomers = new List<Node>();
            int[] includedFams = new int[m.numFam];

            foreach (Route rt in sol.routes)
            {
                SelectChainForRoute(includedChains, includedCustomers, includedFams, rt);
            }

            return CreateSolFromSelectedChains(includedChains);
        }


        private void SelectChainForRoute(List<Chain> includedChains, List<Node> includedCustomers, int[] includedFams, Route rt)
        {
            int maxLen = (int)Math.Ceiling(m.runData.maxChainLen * rt.sequenceOfNodes.Count);
            int minLen = (int)Math.Ceiling(m.runData.minChainLen * rt.sequenceOfNodes.Count);

            //double lowerBound = 0.35;
            //double upperBound = 0.65;

            //double low = lowerBound + (random.NextDouble() * (upperBound - lowerBound));
            //double high = lowerBound + 0.2;

            //int maxLen = (int)Math.Ceiling(high * rt.sequenceOfNodes.Count);
            //int minLen = (int)Math.Ceiling(low * rt.sequenceOfNodes.Count);

            int chainLen = m.runData.random.Next(minLen, maxLen);

            if (chainLen < 3)
            {
                chainLen = 3;
            }

            List<Chain> candidateChains = this.chains.Where(c => c.nodes.Count <= chainLen).OrderBy(c => c.adjCost).OrderByDescending(c => c.freq - c.timesExtracted).OrderByDescending(c => c.nodes.Count).ToList();

            foreach (Chain c in candidateChains)
            {
                if (IsChainValid(c, includedCustomers, includedFams))
                {

                    includedChains.Add(c);
                    
                    includedCustomers.AddRange(c.nodes.Where(n => n != m.depot));
                    
                    foreach (Node n in c.nodes)
                    {
                        if (n == m.depot)
                        {
                            continue;
                        }
                        includedFams[n.family.id]++;
                    }
                    
                    c.timesExtracted++;
                    
                    break;
                }
            }
        }


        private Solution CreateSolFromSelectedChains(List<Chain> includedChains)
        {
            List<Route> routes = new List<Route>();

            int i = 0;

            foreach (Chain c in includedChains)
            {

                if (c.nodes[0] != m.depot)
                {
                    c.nodes.Insert(0, m.depot);
                }
                if (c.nodes.Last() != m.depot)
                {
                    c.nodes.Add(m.depot);
                }

                c.nodes.ForEach(n => n.isRouted = true);

                Route rt = new Route(i, c.nodes, this.m.capacity, 0, 0);
                rt.CalculateRouteCostDemand();
                routes.Add(rt);

                i++;
            }

            Solution partialSol = new Solution();
            partialSol.routes = routes;
            partialSol.UpdateCost();
            partialSol.feasible = false;

            return partialSol;
        }


        private bool IsChainValid(Chain c, List<Node> includedCustomers, int[] includedFams)
        {
            int[] newFams = new int[m.numFam];

            foreach (Node n in c.nodes)
            {
                if (n != m.depot)
                {
                    if (includedCustomers.Contains(n))
                    {
                        return false;
                    }
                    newFams[n.family.id]++;
                }
            }

            for (int i = 0; i < m.numFam; i++)
            {
                if (includedFams[i] + newFams[i] > this.m.families[i].required)
                {
                    return false;
                }
            }

            return true;
        }


        //private void UpdateChains()
        //{
        //    List<Chain> chains = new List<Chain>();

        //    foreach (RouteMember member in members)
        //    {

        //        RouteMember rtm = member.Copy();


        //        int len = member.route.Count();

        //        // While route contains more than 2 nodes
        //        while (rtm.route.Count >= 2)
        //        {
        //            int i = 2;

        //            // Iterate from the start of the route and take the first i nodes to create a chain (min 2, max route.length)
        //            while (i <= len)
        //            {
        //                List<int> nodeIds = rtm.route.Take(i).ToList().Select(node => node.id).ToList();
        //                if (chainDictionary.ContainsKey(rtm.id) && chainDictionary[rtm.id].ContainsKey(nodeIds))
        //                {
        //                    // Update the existing chain if necessary
        //                    Chain existingChain = this.chainDictionary[rtm.id][nodeIds];
        //                    existingChain.freq++;
        //                    if (rtm.solCost < existingChain.solCost)
        //                    {
        //                        existingChain.solCost = rtm.solCost;
        //                    }
        //                }
        //                else
        //                {
        //                    Chain chain = new Chain(rtm.route.Take(i).ToList(), rtm.solCost, 1, rtm.id);
        //                    // Add a new entry to the dictionary
        //                    if (!chainDictionary.ContainsKey(rtm.id))
        //                    {
        //                        chainDictionary[rtm.id] = new Dictionary<List<int>, Chain>(new ListComparer());
        //                    }
        //                    chainDictionary[rtm.id][nodeIds] = chain;
        //                }
        //                i++;
        //            }
        //            rtm.route = rtm.route.TakeLast(len - 1).ToList();
        //            len = rtm.route.Count();
        //        }
        //    }
        //}


        //private void UpdateChains2 (Solution sol)
        //{
        //    Solution tempSol = sol.DeepCopy();

        //    List<Chain> solChains = new List<Chain>();

        //    foreach(Route rt in tempSol.routes)
        //    {
        //        List<Chain> newChains = GetRouteChains(rt, tempSol.cost);

        //        foreach (Chain chain in newChains)
        //        {
        //            Chain existing = this.chains.First(c => c.id == chain.id);
        //            if (existing != null)
        //            {
        //                existing.freq++;
        //                if (chain.solCost < existing.solCost)
        //                {
        //                    existing.solCost = chain.solCost;
        //                }
        //            }
        //            else
        //            {
        //                this.chains.Add(chain);
        //            }
        //        }
        //        solChains.AddRange(newChains);
        //    }
        //}


        //private List<Chain> GetRouteChains(Route rt, float cost)
        //{
        //    int id = rt.sequenceOfNodes.GetHashCode();
        //    List<Chain> newChains = new List<Chain>();

        //    rt.sequenceOfNodes.Remove(depot);
        //    rt.sequenceOfNodes.Remove(depot);

        //    int len = rt.sequenceOfNodes.Count();

        //    this.memberChains[id] = new List<List<int>>();

        //    while (len >= 2)
        //    {
        //        int i = 2;

        //        // Iterate from the start of the route and take the first i nodes to create a chain (min 2, max route.length)
        //        while (i <= len)
        //        {
        //            Chain chain = new Chain(rt.sequenceOfNodes.Take(i).ToList(), cost, 1, id);
        //            newChains.Add(chain);
        //            i++;
        //            memberChains[id].Add(chain.id);
        //        }
        //        rt.sequenceOfNodes = rt.sequenceOfNodes.TakeLast(len - 1).ToList();
        //        len = rt.sequenceOfNodes.Count();
        //    }

        //    return newChains;
        //}



        //private (Solution, List<Node>) ExtractAndConstruct()
        //{
        //    Solution sol = new Solution();

        //    // Get max length of chains in chainDictionary


        //    int maxChainLen = chainDictionary.Values.SelectMany(innerDict => innerDict.Values).Max(chain => chain.nodes.Count);


        //    int maxCandidateLen = random.Next((int)Math.Round(maxChainLen / 2.0), maxChainLen);
        //    //int minCandidateLen = random.Next((int)Math.Round(maxChainLen / 6.0), (int)Math.Round(maxChainLen / 4.0));
        //    //List<Chain> candidateChains = chainDictionary.Values//.Where(c => c.nodes.Count() <= maxCandidateLen)//.Where(c => c.nodes.Count() >= minCandidateLen)
        //    //    .OrderBy(c => (float)c.adjCost )// (1.0 + c.timesExtracted))                              
        //    //    //.OrderByDescending(c => (float)c.freq )// (1.0 + c.timesExtracted))
        //    //    //.OrderByDescending(c => c.nodes.Count() )// (1.0 + c.timesExtracted))
        //    //    .ToList();

        //    List<Chain> candidateChains = chainDictionary.Values.SelectMany(innerDict => innerDict.Values).Where(c => c.nodes.Count() <= maxCandidateLen)
        //        .OrderBy(c => (float)c.adjCost / (1.0 + c.timesExtracted))                              
        //        .OrderByDescending(c => (float)c.freq / (1.0 + c.timesExtracted))
        //        //.OrderByDescending(c => c.nodes.Count() / (1.0 + c.timesExtracted))
        //        .ToList();

        //    List<Chain> insertedChains = candidateChains.Where(c => c.timesExtracted > 0).ToList();

        //    List<Node> extractedNodes = new List<Node>();
        //    int[] extractedFamilies = new int[numFam];

        //    List<ChainBin> chainBins = InitializeChainBins();

        //    List<Node> remainingCustomers = new List<Node>();

        //    while (extractedNodes.Count() < numReq)
        //    {
        //        Chain candidateChain = candidateChains.First();
        //        candidateChains.Remove(candidateChain);

        //        bool isValid = CheckChainValidity(extractedNodes, candidateChain, extractedFamilies);


        //        if (isValid)
        //        {
        //            List<ChainBin> candidateBins = chainBins.Where(b => b.load + candidateChain.demand <= capacity).ToList();

        //            if (candidateBins.Count() > 0)
        //            {
        //                int minInsertPos;
        //                ChainBin minBin;
        //                FindMinBin(candidateChain, candidateBins, out minInsertPos, out minBin);

        //                minBin.chains.Insert(minInsertPos, candidateChain);
        //                minBin.load += candidateChain.demand;

        //                extractedNodes.AddRange(candidateChain.nodes);

        //                foreach (Node n in candidateChain.nodes)
        //                {
        //                    extractedFamilies[n.family.id]++;
        //                }
        //            }
        //            candidateChain.timesExtracted++;

        //        }



        //        if (candidateChains.Count() == 0 && extractedNodes.Count() < numReq)
        //        {
        //            remainingCustomers = SelectRemainingCustomers(extractedNodes, extractedFamilies);
        //            extractedNodes.AddRange(remainingCustomers);
        //        }
        //    }

        //    List<Node> unservedCustomers = new List<Node>();

        //    (chainBins, unservedCustomers) = InsertRemainingCustomers(chainBins, remainingCustomers);

        //    List <Route> routes = InitializeRoutesFromBins(chainBins);

        //    Solution newSol = new Solution
        //    {
        //        routes = routes,
        //        cost = routes.Sum(r => r.cost)
        //    };

        //    Console.WriteLine("New solution cost: " + newSol.cost);
        //    if (!CheckSolutionFeasibility(newSol))
        //    {
        //        Console.WriteLine("Adaptive Memory Violation!");
        //    }

        //    return (newSol, unservedCustomers);
        //}

        //private (List<ChainBin>, List<Node>) InsertRemainingCustomers(List<ChainBin> chainBins, List<Node> remainingCustomers)
        //{
        //    List<Chain> remainingChains = new List<Chain>();
        //    foreach (Node customer in remainingCustomers)
        //    {
        //        Chain chain = new Chain(new List<Node> { customer }, 0, 1, 0);
        //        remainingChains.Add(chain);
        //    }

        //    foreach(Chain candidateChain in remainingChains)
        //    {
        //        List<ChainBin> candidateBins = chainBins.Where(b => b.load + candidateChain.demand <= capacity).ToList();

        //        int minInsertPos;
        //        ChainBin minBin;
        //        FindMinBin(candidateChain, candidateBins, out minInsertPos, out minBin);

        //        if (minInsertPos >= 0)
        //        {
        //            minBin.chains.Insert(minInsertPos, candidateChain);
        //            minBin.load += candidateChain.demand;

        //            remainingCustomers.Remove(candidateChain.nodes[0]);
        //        }        
        //    }


        //    return (chainBins, remainingCustomers);
        //}

        //private void FindMinBin(Chain candidateChain, List<ChainBin> candidateBins, out int minInsertPos, out ChainBin minBin)
        //{
        //    float minInsertCost = float.MaxValue;
        //    minInsertPos = -1;
        //    minBin = new ChainBin();
        //    foreach (ChainBin candidateBin in candidateBins)
        //    {
        //        (float insertCost, int insertPos) = CheckChainInsertion(candidateChain, candidateBin);

        //        if (insertCost < minInsertCost)
        //        {
        //            minInsertCost = insertCost;
        //            minInsertPos = insertPos;
        //            minBin = candidateBin;
        //        }
        //    }
        //}

        //private List<ChainBin> InitializeChainBins()
        //{
        //    List<ChainBin> chainBins = new List<ChainBin>();
        //    for (int i = 0; i < vehicles; i++)
        //    {
        //        chainBins.Add(new ChainBin { chains = new List<Chain>(), load = 0 });
        //    }

        //    return chainBins;
        //}

        //private List<Route> InitializeRoutesFromBins(List<ChainBin> chainBins)
        //{
        //    List<Route> routes = new List<Route>();


        //    int i = 0;
        //    foreach (ChainBin bin in chainBins)
        //    {
        //        List<Node> nodeSeq = new List<Node>();
        //        nodeSeq.Add(depot);
        //        float load = 0;
        //        foreach(Chain c in bin.chains)
        //        {
        //            nodeSeq.AddRange(c.nodes);
        //            load += c.demand;
        //        }
        //        nodeSeq.Add(depot);
        //        Route rt = new Route(i, nodeSeq, capacity, 0, load);
        //        routes.Add(rt);
        //        i++;

        //    }

        //    foreach (Route rt in routes)
        //    {
        //        UpdateRouteCostAndLoad(rt);
        //    }

        //    return routes;
        //}

        //public void UpdateRouteCostAndLoad(Route rt)
        //{
        //    float tc = 0;
        //    float tl = 0;
        //    for (int i = 0; i < rt.sequenceOfNodes.Count - 1; i++)
        //    {
        //        Node A = rt.sequenceOfNodes[i];
        //        Node B = rt.sequenceOfNodes[i + 1];
        //        tc += this.costMatrix[A.id][B.id];
        //        tl += A.demand;
        //    }
        //    rt.load = tl;
        //    rt.cost = tc;
        //}

        //private List<Node> SelectRemainingCustomers(List<Node> extractedNodes, int[] extractedFamilies)
        //{

        //    List<Node> unservedCustomers = new List<Node>();

        //    for (int i = 0; i < numFam; i++)
        //    {
        //        if (extractedFamilies[i] < famReq[i])
        //        {
        //            List<Node> familyCustomers = families[i].nodes.Where(n => !extractedNodes.Contains(n)).ToList().OrderBy(n => random.Next()).Take(famReq[i] - extractedFamilies[i]).ToList();
        //            unservedCustomers.AddRange(familyCustomers);
        //        }
        //    }

        //    return unservedCustomers;
        //}

        //private bool CheckChainValidity(List<Node> extractedNodes, Chain candidateChain, int[] famEx)
        //{

        //    bool isValid = true;

        //    int[] newEx = new int[numFam];

        //    if (extractedNodes.Count() + candidateChain.nodes.Count() > numReq)
        //    {
        //        return false;
        //    }

        //    foreach (Node n in candidateChain.nodes)
        //    {
        //        // Customer uniqueness
        //        if (extractedNodes.Contains(n))
        //        {
        //            return false;
        //        }

        //        newEx[n.family.id]++;


        //    }

        //    // Family requirements
        //    for (int i = 0; i < numFam; i++)
        //    {
        //        if (famEx[i] + newEx[i] > famReq[i])
        //        {
        //            return false;
        //        }
        //    }


        //    return isValid;
        //}


        //private (float, int) CheckChainInsertion(Chain candidateChain, ChainBin candidateBin)
        //{
        //    float minInsertCost = float.MaxValue;
        //    int minInsertPos = -1;

        //    if (candidateBin.chains.Count() > 0)
        //    {
        //        float insertCost;
        //        //int insertPos;

        //        for (int i = 0; i <= candidateBin.chains.Count(); i++)
        //        {
        //            if (i == 0)
        //            {
        //                insertCost = costMatrix[depot.id][candidateChain.nodes[0].id] + costMatrix[candidateChain.nodes.Last().id][candidateBin.chains[i].nodes[0].id] - costMatrix[depot.id][candidateBin.chains[i].nodes[0].id];
        //            }
        //            else if (i == candidateBin.chains.Count())
        //            {
        //                insertCost = costMatrix[candidateChain.nodes.Last().id][depot.id] + costMatrix[candidateBin.chains[i-1].nodes.Last().id][candidateChain.nodes[0].id] - costMatrix[candidateBin.chains[i-1].nodes.Last().id][depot.id];
        //            }
        //            else
        //            {
        //                insertCost = costMatrix[candidateBin.chains[i - 1].nodes.Last().id][candidateChain.nodes[0].id] + costMatrix[candidateChain.nodes.Last().id][candidateBin.chains[i].nodes[0].id] - costMatrix[candidateBin.chains[i - 1].nodes.Last().id][candidateBin.chains[i].nodes[0].id];
        //            }

        //            if (insertCost < minInsertCost)
        //            {
        //                minInsertCost = insertCost;
        //                minInsertPos = i;
        //            }
        //        }
        //    }
        //    else
        //    {
        //        minInsertPos = 0;
        //        minInsertCost = costMatrix[depot.id][candidateChain.nodes[0].id] + costMatrix[candidateChain.nodes.Last().id][depot.id];
        //    }


        //    return (minInsertCost, minInsertPos);
        //} 


    }
}
