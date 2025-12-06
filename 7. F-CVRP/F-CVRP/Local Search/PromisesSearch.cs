using F_CVRP.Operators;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using static F_CVRP.Setup;

namespace F_CVRP.Local_Search
{
    public class PromisesSearch
    {

        public int maxNonImprovIterations { get; set; }
        public Solution sol { get; set; }
        public Solution bestSol { get; set; }
        float[,] promisesMatrix = null;
        public int promiseIter { get; set; }
        public List<Node> unservedCustomers { get; set; }
        public double replaceChance { get; set; }
        public double relocateChance { get; set; }
        public double swapChance { get; set; }
        public Model model { get; set; }

        public PromisesSearch(Solver s) 
        {
            Reset(s);
        }

        public void Reset(Solver s)
        {
            model = s.m;
            unservedCustomers = s.unservedCustomers;
            sol = s.sol.DeepCopy();
            promisesMatrix = InitializePromisesMatrix();
            replaceChance = s.wideBudget*s.m.runData.replaceWeight;
            relocateChance = replaceChance + 0.33 * (1 - replaceChance);
            swapChance = relocateChance + 0.33 * (1 - replaceChance);
            //maxNonImprovIterations = CalculateMaxNonImproveIterations();
            maxNonImprovIterations = s.m.runData.maxIterations;
            promiseIter = s.m.runData.random.Next(75,150);
        }
        //public int numIterations { get; set; }

        private int CalculateMaxNonImproveIterations()
        {
            int iter = (int)Math.Round(model.numNodes * Math.Log(model.numNodes) * 20 * 2, 0);
            if (iter < 15000)
            {
                return iter;
            }
            return 15000;
        }

        private float[,] InitializePromisesMatrix()
        {
            int length = model.nodes.Count;
            float[,] promisesMatrix = new float[length, length];

            for (int y = 0; y < length; y++)
            {
                for (int x = 0; x < length; x++)
                {
                    promisesMatrix[y, x] = 100000000000;
                }
            }
            return promisesMatrix;
        }

        public void ApplyLocalSearch(Solution s)
        {
            this.sol = s.DeepCopy();
            this.bestSol = s.DeepCopy();

            if (this.unservedCustomers != null && this.unservedCustomers.Count > 0)
            {
                bestSol.cost = float.MaxValue;
            }

            bool terminationCondition = false;
            int currentNonImprovIterations = 0;
            int localSearchIterator = 0;

            RelocationMove rm = new RelocationMove();
            SwapMove sm = new SwapMove();
            TwoOptMove top = new TwoOptMove();
            ReplaceMove rpm = new ReplaceMove();
       

            while (!terminationCondition)
            {
                ApplyRandomMove(localSearchIterator, rm, sm, top, rpm);

                localSearchIterator++;

                if (localSearchIterator % this.promiseIter == 0)
                {
                    this.promisesMatrix = InitializePromisesMatrix();
                }

                if (this.sol.cost < this.bestSol.cost && (this.unservedCustomers == null || this.unservedCustomers.Count == 0))
                {
                    this.promisesMatrix = InitializePromisesMatrix();
                    currentNonImprovIterations = 0;
                    bestSol = this.sol.DeepCopy();

                }
                else
                {
                    currentNonImprovIterations++;
                }

                if (currentNonImprovIterations > this.maxNonImprovIterations && (this.unservedCustomers == null || this.unservedCustomers.Count == 0))
                {
                    terminationCondition = true;
                }

                //Console.WriteLine(localSearchIterator + " " + sol.cost + " " + bestSol.cost);

            }
            sol = bestSol;

        }

        private void ApplyRandomMove(int localSearchIterator, RelocationMove rm, SwapMove sm, TwoOptMove top, ReplaceMove rpm)
        {
            double oper = model.runData.random.NextDouble();
            //oper = 0;
            if (sol.feasible)
            {
                InsertUnservedCustomer();
            }

            if (oper <= this.replaceChance)
            {
                FindBestReplaceMove(rpm, localSearchIterator);
                if (rpm.routeIndex != -1)
                {
                    ApplyReplaceMove(rpm, localSearchIterator);
                }

            }
            else if (oper <= this.relocateChance)
            {
                FindBestRelocationMove(rm, localSearchIterator);
                if (rm.originRoutePosition != -1)
                {
                    ApplyRelocationMove(rm, localSearchIterator);
                }
            }
            else if (oper <= this.swapChance)
            {
                FindBestSwapMove(sm, localSearchIterator);
                if (sm.firstRouteIndex != -1)
                {
                    ApplySwapMove(sm, localSearchIterator);
                }
            }
            else if (oper <= 1)
            {
                FindBestTwoOptMove(top, localSearchIterator);
                if (top.firstRoutePosition != -1)
                {
                    ApplyTwoOptMove(top, localSearchIterator);
                }
            }


            
        }

        private void InsertUnservedCustomer()
        {
            if (this.unservedCustomers != null && this.unservedCustomers.Count > 0)
            {

                float minLoad = sol.routes.Min(rt => rt.load);
                float minDem = this.unservedCustomers.Min(c => c.demand);
                if (minDem + minLoad <= model.capacity)
                {
                    List<Route> candidateRoutes = sol.routes.Where(rt => rt.load == minLoad).ToList();
                    List<Node> candidateNodes = this.unservedCustomers.Where(n => n.demand == minDem).ToList();


                    foreach (Route rt in candidateRoutes)
                    {

                        foreach (Node n in candidateNodes)
                        {
                            int pos = -1;
                            float cost = 10000;
                            
                            (pos, cost) = FindBestInsertion(rt, n);
                           
                            if (pos > -1)
                            {
                                rt.sequenceOfNodes.Insert(pos, n);
                                rt.cost += cost;
                                this.sol.cost += cost;
                                rt.load += n.demand;
                                this.unservedCustomers.Remove(n);
                                n.isRouted = true;
                                break;
                            }
                        }

                    }
                }
            }
        }


        private void FindBestRelocationMove(RelocationMove rm, int iterator)
        {
            for (int originRouteIndex = 0; originRouteIndex < this.sol.routes.Count; originRouteIndex++)
            {
                Route rt1 = this.sol.routes[originRouteIndex];
                for (int targetRouteIndex = 0; targetRouteIndex < this.sol.routes.Count; targetRouteIndex++)
                {
                    Route rt2 = this.sol.routes[targetRouteIndex];
                    for (int originNodeIndex = 1; originNodeIndex < rt1.sequenceOfNodes.Count - 1; originNodeIndex++)
                    {
                        for (int targetNodeIndex = 0; targetNodeIndex < rt2.sequenceOfNodes.Count - 1; targetNodeIndex++)
                        {
                            if (originRouteIndex == targetRouteIndex && (targetNodeIndex == originNodeIndex || targetNodeIndex == originNodeIndex - 1))
                            {
                                continue;
                            }

                            Node A = rt1.sequenceOfNodes[originNodeIndex - 1];
                            Node B = rt1.sequenceOfNodes[originNodeIndex];
                            Node C = rt1.sequenceOfNodes[originNodeIndex + 1];

                            Node F = rt2.sequenceOfNodes[targetNodeIndex];
                            Node G = rt2.sequenceOfNodes[targetNodeIndex + 1];

                            if (rt1 != rt2)
                            {
                                if (rt2.load + B.demand > rt2.capacity)
                                {
                                    continue;
                                }
                            }

                            float costAdded = model.costMatrix[A.id][C.id] + model.costMatrix[F.id][B.id] + model.costMatrix[B.id][G.id];
                            float costRemoved = model.costMatrix[A.id][B.id] + model.costMatrix[B.id][C.id] + model.costMatrix[F.id][G.id];

                            float originRtCostChange = model.costMatrix[A.id][C.id] - model.costMatrix[A.id][B.id] - model.costMatrix[B.id][C.id];
                            float targetRtCostChange = model.costMatrix[F.id][B.id] + model.costMatrix[B.id][G.id] - model.costMatrix[F.id][G.id];

                            float moveCost = costAdded - costRemoved;

                            if (CheckPromise(A, C, this.sol.cost + moveCost) || CheckPromise(F, B, this.sol.cost + moveCost) || CheckPromise(B, G, this.sol.cost + moveCost))
                            {
                                continue;
                            }

                            if (moveCost < rm.moveCost && moveCost != 0)
                            {
                                rm.StoreBestRelocationMove(originRouteIndex, targetRouteIndex, originNodeIndex, targetNodeIndex, moveCost, originRtCostChange, targetRtCostChange, null);
                            }

                        }
                    }
                }
            }
        }

        private void FindBestSwapMove(SwapMove sm, int iterator)
        {
            for (int firstRouteIndex = 0; firstRouteIndex < this.sol.routes.Count; firstRouteIndex++)
            {
                Route rt1 = this.sol.routes[firstRouteIndex];
                for (int secondRouteIndex = firstRouteIndex; secondRouteIndex < this.sol.routes.Count; secondRouteIndex++)
                {
                    Route rt2 = this.sol.routes[secondRouteIndex];
                    for (int firstNodeIndex = 1; firstNodeIndex < rt1.sequenceOfNodes.Count - 1; firstNodeIndex++)
                    {
                        int startOfSecondNodeIndex = 1;
                        if (rt1 == rt2)
                        {
                            startOfSecondNodeIndex = firstNodeIndex + 1;
                        }
                        for (int secondNodeIndex = startOfSecondNodeIndex; secondNodeIndex < rt2.sequenceOfNodes.Count - 1; secondNodeIndex++)
                        {


                            Node A1 = rt1.sequenceOfNodes[firstNodeIndex - 1];
                            Node B1 = rt1.sequenceOfNodes[firstNodeIndex];
                            Node C1 = rt1.sequenceOfNodes[firstNodeIndex + 1];

                            Node A2 = rt2.sequenceOfNodes[secondNodeIndex - 1];
                            Node B2 = rt2.sequenceOfNodes[secondNodeIndex];
                            Node C2 = rt2.sequenceOfNodes[secondNodeIndex + 1];

                            float costChangeFirstRoute = 0;
                            float costChangeSecondRoute = 0;
                            float moveCost;

                            if (rt1 == rt2)
                            {
                                if (firstNodeIndex == secondNodeIndex - 1)
                                {
                                    float costRemoved = model.costMatrix[A1.id][B1.id] + model.costMatrix[B1.id][B2.id] + model.costMatrix[B2.id][C2.id];
                                    float costAdded = model.costMatrix[A1.id][B2.id] + model.costMatrix[B2.id][B1.id] + model.costMatrix[B1.id][C2.id];
                                    moveCost = costAdded - costRemoved;

                                }
                                else
                                {
                                    float costRemoved1 = model.costMatrix[A1.id][B1.id] + model.costMatrix[B1.id][C1.id];
                                    float costRemoved2 = model.costMatrix[A2.id][B2.id] + model.costMatrix[B2.id][C2.id];

                                    float costAdded1 = model.costMatrix[A1.id][B2.id] + model.costMatrix[B2.id][C1.id];
                                    float costAdded2 = model.costMatrix[A2.id][B1.id] + model.costMatrix[B1.id][C2.id];

                                    moveCost = costAdded1 + costAdded2 - costRemoved1 - costRemoved2;
                                }

                            }
                            else
                            {
                                if ((rt1.load - B1.demand + B2.demand > model.capacity) || (rt2.load - B2.demand + B1.demand > model.capacity))
                                {
                                    continue;
                                }

                                float costRemoved1 = model.costMatrix[A1.id][B1.id] + model.costMatrix[B1.id][C1.id];
                                float costRemoved2 = model.costMatrix[A2.id][B2.id] + model.costMatrix[B2.id][C2.id];

                                float costAdded1 = model.costMatrix[A1.id][B2.id] + model.costMatrix[B2.id][C1.id];
                                float costAdded2 = model.costMatrix[A2.id][B1.id] + model.costMatrix[B1.id][C2.id];

                                costChangeFirstRoute = costAdded1 - costRemoved1;
                                costChangeSecondRoute = costAdded2 - costRemoved2;

                                moveCost = costAdded1 + costAdded2 - costRemoved1 - costRemoved2;



                            }

                            if (CheckPromise(A1, B2, this.sol.cost + moveCost) || CheckPromise(B2, C1, this.sol.cost + moveCost) || CheckPromise(A2, B1, this.sol.cost + moveCost) || CheckPromise(B1, C2, this.sol.cost + moveCost))
                            {
                                continue;
                            }

                            if (moveCost < sm.moveCost && Math.Abs(moveCost) > 0)
                            {
                                //Console.WriteLine(moveCost);
                                sm.StoreBestSwapMove(firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex, moveCost, costChangeFirstRoute, costChangeSecondRoute, null);
                            }
                        }
                    }
                }
            }
        }


        private void FindBestTwoOptMove(TwoOptMove top, int iterator)
        {
            for (int firstRouteIndex = 0; firstRouteIndex < this.sol.routes.Count; firstRouteIndex++)
            {
                Route rt1 = this.sol.routes[firstRouteIndex];
                for (int secondtRouteIndex = 0; secondtRouteIndex < this.sol.routes.Count; secondtRouteIndex++)
                {
                    Route rt2 = this.sol.routes[secondtRouteIndex];
                    for (int firstNodeIndex = 0; firstNodeIndex < rt1.sequenceOfNodes.Count - 1; firstNodeIndex++)
                    {
                        int start2 = 0;
                        if (rt1 == rt2)
                        {
                            start2 = firstNodeIndex + 2;
                        }
                        for (int secondNodeIndex = start2; secondNodeIndex < rt2.sequenceOfNodes.Count - 1; secondNodeIndex++)
                        {
                            float moveCost = 1000000000000000000;

                            Node A = rt1.sequenceOfNodes[firstNodeIndex];
                            Node B = rt1.sequenceOfNodes[firstNodeIndex + 1];

                            Node K = rt2.sequenceOfNodes[secondNodeIndex];
                            Node L = rt2.sequenceOfNodes[secondNodeIndex + 1];

                            if (rt1 == rt2)
                            {
                                if (firstNodeIndex == 0 || secondNodeIndex == rt1.sequenceOfNodes.Count - 2)
                                {
                                    continue;
                                }

                                float costAdded = 0;
                                float costRemoved = 0;


                                for (int i = firstNodeIndex; i <= secondNodeIndex; i++)
                                {
                                    costRemoved += model.costMatrix[rt1.sequenceOfNodes[i].id][rt1.sequenceOfNodes[i + 1].id];
                                }

                                for (int i = secondNodeIndex; i > firstNodeIndex + 1; i--)
                                {
                                    costAdded += model.costMatrix[rt1.sequenceOfNodes[i].id][rt1.sequenceOfNodes[i - 1].id];
                                }
                                costAdded += model.costMatrix[A.id][K.id] + model.costMatrix[B.id][L.id];

                                moveCost = costAdded - costRemoved;

                            }
                            else
                            {
                                if (firstNodeIndex == 0 && secondNodeIndex == 0)
                                {
                                    continue;
                                }

                                if (firstNodeIndex == rt1.sequenceOfNodes.Count - 2 && secondNodeIndex == rt2.sequenceOfNodes.Count - 2)
                                {
                                    continue;
                                }

                                if (CapacityIsViolated(rt1, firstNodeIndex, rt2, secondNodeIndex))
                                {
                                    continue;
                                }

                                float costAdded = model.costMatrix[A.id][L.id] + model.costMatrix[K.id][B.id];
                                float costRemoved = model.costMatrix[A.id][B.id] + model.costMatrix[K.id][L.id];
                                moveCost = costAdded - costRemoved;
                            }

                            if (CheckPromise(A, K, this.sol.cost + moveCost) || CheckPromise(B, L, this.sol.cost + moveCost))
                            {
                                continue;
                            }

                            if (moveCost < top.moveCost && moveCost != 0)
                            {
                                top.StoreBestTwoOptMove(firstRouteIndex, secondtRouteIndex, firstNodeIndex, secondNodeIndex, moveCost, null);
                            }
                        }
                    }
                }
            }
        }

        private void FindBestReplaceMove(ReplaceMove rpm, int iterator)
        {
            for (int routeIndex = 0; routeIndex < this.sol.routes.Count; routeIndex++)
            {
                Route rt = this.sol.routes[routeIndex];
                for (int nodeIndex = 1; nodeIndex < rt.sequenceOfNodes.Count - 1; nodeIndex++)
                {

                    Node A = rt.sequenceOfNodes[nodeIndex - 1];
                    Node B = rt.sequenceOfNodes[nodeIndex];
                    Node C = rt.sequenceOfNodes[nodeIndex + 1];

                    float costAdded = 0;
                    float costRemoved = 0;
                    float moveCost;

                    float costAddedPenalized = 0;
                    float costRemovedPenalized = 0;

                    List<Node> candidates = B.family.nodes.Where(n => n.isRouted == false).ToList();
                    foreach (Node node in candidates)
                    {
                        //if (!node.isRouted)
                        //{
                        costAdded = model.costMatrix[A.id][node.id] + model.costMatrix[node.id][C.id];
                        costRemoved = model.costMatrix[A.id][B.id] + model.costMatrix[B.id][C.id];
                        moveCost = costAdded - costRemoved;

                        if (CheckPromise(B, B, this.sol.cost + moveCost)) //CheckPromise(A, B, this.sol.cost + moveCost) || CheckPromise(B, C, this.sol.cost + moveCost) ||
                        {
                            continue;
                        }

                        if (moveCost < rpm.moveCost && Math.Abs(moveCost) > 0)
                        {
                            rpm.StoreBestReplaceMove(routeIndex, nodeIndex, node, moveCost, null);
                        }
                        //}
                    }

                }
            }
        }


        private void ApplyRelocationMove(RelocationMove rm, int iterator)
        {
            float oldCost = this.sol.cost; // CalculateTotalCost
            Route originRt = this.sol.routes[rm.originRoutePosition];
            Route targetRt = this.sol.routes[rm.targetRoutePosition];

            Node A = originRt.sequenceOfNodes[rm.originNodePosition - 1];
            Node B = originRt.sequenceOfNodes[rm.originNodePosition];
            Node C = originRt.sequenceOfNodes[rm.originNodePosition + 1];
            Node F = targetRt.sequenceOfNodes[rm.targetNodePosition];
            Node G = targetRt.sequenceOfNodes[rm.targetNodePosition + 1];

            if (originRt == targetRt)
            {
                originRt.sequenceOfNodes.RemoveAt(rm.originNodePosition);
                if (rm.originNodePosition < rm.targetNodePosition)
                {
                    targetRt.sequenceOfNodes.Insert(rm.targetNodePosition, B);
                }
                else
                {
                    targetRt.sequenceOfNodes.Insert(rm.targetNodePosition + 1, B);
                }

                originRt.cost += rm.moveCost;
            }
            else
            {
                originRt.sequenceOfNodes.RemoveAt(rm.originNodePosition);
                targetRt.sequenceOfNodes.Insert(rm.targetNodePosition + 1, B);
                originRt.cost += rm.costChangeOriginRt;
                targetRt.cost += rm.costChangeTargetRt;
                originRt.load -= B.demand;
                targetRt.load += B.demand;
            }

            this.sol.cost += rm.moveCost;

            SetPromise(A, B, iterator, this.sol.cost);
            SetPromise(B, C, iterator, this.sol.cost);
            SetPromise(F, G, iterator, this.sol.cost);

            rm.Reset();

        }


        private void ApplySwapMove(SwapMove sm, int iterator)
        {
            float oldCost = this.sol.cost;

            Route rt1 = this.sol.routes[sm.firstRouteIndex];
            Route rt2 = this.sol.routes[sm.secondRouteIndex];

            Node A1 = rt1.sequenceOfNodes[sm.firstNodeIndex - 1];
            Node A2 = rt2.sequenceOfNodes[sm.secondNodeIndex - 1];
            Node B1 = rt1.sequenceOfNodes[sm.firstNodeIndex];
            Node B2 = rt2.sequenceOfNodes[sm.secondNodeIndex];
            Node C1 = rt1.sequenceOfNodes[sm.firstNodeIndex + 1];
            Node C2 = rt2.sequenceOfNodes[sm.secondNodeIndex + 1];

            rt1.sequenceOfNodes[sm.firstNodeIndex] = B2;
            rt2.sequenceOfNodes[sm.secondNodeIndex] = B1;

            if (rt1 == rt2)
            {
                rt1.cost += sm.moveCost;
            }
            else
            {
                rt1.cost += sm.costChangeFirstRt;
                rt2.cost += sm.costChangeSecondRt;
                rt1.load = rt1.load - B1.demand + B2.demand;
                rt2.load = rt2.load - B2.demand + B1.demand;
            }

            this.sol.cost += sm.moveCost;

            SetPromise(A1, B1, iterator, this.sol.cost);
            SetPromise(B1, C1, iterator, this.sol.cost);
            SetPromise(A2, B2, iterator, this.sol.cost);
            SetPromise(B2, C2, iterator, this.sol.cost);

            sm.Reset();

        }


        private void ApplyTwoOptMove(TwoOptMove top, int iterator)
        {
            Route rt1 = this.sol.routes[top.firstRoutePosition];
            Route rt2 = this.sol.routes[top.secondRoutePosition];

            Node A = rt1.sequenceOfNodes[top.firstNodePosition];
            Node B = rt1.sequenceOfNodes[top.firstNodePosition + 1];
            Node K = rt2.sequenceOfNodes[top.secondNodePosition];
            Node L = rt2.sequenceOfNodes[top.secondNodePosition + 1];

            if (rt1 == rt2)
            {

                rt1.sequenceOfNodes.Reverse(top.firstNodePosition + 1, top.secondNodePosition - top.firstNodePosition);

                rt1.cost += top.moveCost;
            }
            else
            {

                List<Node> relocatedSegmentOfRt1 = rt1.sequenceOfNodes.Skip(top.firstNodePosition + 1).ToList();
                List<Node> relocatedSegmentOfRt2 = rt2.sequenceOfNodes.Skip(top.secondNodePosition + 1).ToList();

                rt1.sequenceOfNodes.RemoveRange(top.firstNodePosition + 1, rt1.sequenceOfNodes.Count - (top.firstNodePosition + 1));
                rt2.sequenceOfNodes.RemoveRange(top.secondNodePosition + 1, rt2.sequenceOfNodes.Count - (top.secondNodePosition + 1));

                rt1.sequenceOfNodes.AddRange(relocatedSegmentOfRt2);
                rt2.sequenceOfNodes.AddRange(relocatedSegmentOfRt1);

                rt1.CalculateRouteCostDemand();
                rt2.CalculateRouteCostDemand();              
            }

            this.sol.cost += top.moveCost;

            SetPromise(A, B, iterator, this.sol.cost);
            SetPromise(K, L, iterator, this.sol.cost);

            top.Reset();
        }

        private void ApplyReplaceMove(ReplaceMove rpm, int iterator)
        {
            Route rt = this.sol.routes[rpm.routeIndex];
            Node A = rt.sequenceOfNodes[rpm.nodeIndex - 1];
            Node B = rt.sequenceOfNodes[rpm.nodeIndex];
            Node C = rt.sequenceOfNodes[rpm.nodeIndex + 1];

            Node N = rpm.selectedNode;


            foreach (Route route in sol.routes)
            {
                if (route.sequenceOfNodes.Contains(N))
                {
                    rpm.Reset();
                    return;
                }
            }
            

            rt.sequenceOfNodes.RemoveAt(rpm.nodeIndex);
            rt.sequenceOfNodes.Insert(rpm.nodeIndex, N);

            B.isRouted = false;
            N.isRouted = true;

            rt.cost += rpm.moveCost;
            this.sol.cost += rpm.moveCost;

            //SetPromise(A, B, iterator, this.sol.cost);
            //SetPromise(B, C, iterator, this.sol.cost);

            SetPromise(B, B, iterator, this.sol.cost);



            rpm.Reset();
        }


        private bool CapacityIsViolated(Route rt1, int firstNodeIndex, Route rt2, int secondNodeIndex)
        {
            float rt1FirstSegmentLoad = 0;
            for (int i = 0; i < firstNodeIndex + 1; i++)
            {
                Node n = rt1.sequenceOfNodes[i];
                rt1FirstSegmentLoad += n.demand;
            }
            float rt1SecondSegmentLoad = rt1.load - rt1FirstSegmentLoad;

            float rt2FirstSegmentLoad = 0;
            for (int i = 0; i < secondNodeIndex + 1; i++)
            {
                Node n = rt2.sequenceOfNodes[i];
                rt2FirstSegmentLoad += n.demand;
            }
            float rt2SecondSegmentLoad = rt2.load - rt2FirstSegmentLoad;

            if (rt1FirstSegmentLoad + rt2SecondSegmentLoad > rt1.capacity)
            {
                return true;
            }

            if (rt2FirstSegmentLoad + rt1SecondSegmentLoad > rt2.capacity)
            {
                return true;
            }

            return false;
        }

        private (int, float) FindBestInsertion(Route rt, Node n)
        {
            int minPos = -1;
            float minCost = 10000;
            for (int i = 1; i < rt.sequenceOfNodes.Count; i++)
            {
                float costAdded = model.costMatrix[rt.sequenceOfNodes[i - 1].id][n.id] + model.costMatrix[n.id][rt.sequenceOfNodes[i].id];
                float costRemoved = model.costMatrix[rt.sequenceOfNodes[i - 1].id][rt.sequenceOfNodes[i].id];

                float insertionCost = costAdded - costRemoved;

                if (insertionCost < minCost)
                {
                    minCost = insertionCost;
                    minPos = i;
                }
            }

            return (minPos, minCost);
        }


        public void SetPromise(Node A, Node B, int i, float cost)
        {   
            this.promisesMatrix[A.id, B.id] = cost;
        }


        public bool CheckPromise(Node A, Node B, float newCost)
        {   

            if (newCost < this.promisesMatrix[A.id, B.id])
            {
                return false;
            }
            return true;
        }   
    }
}
