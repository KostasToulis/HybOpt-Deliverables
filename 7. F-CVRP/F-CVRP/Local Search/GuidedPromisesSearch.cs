using F_CVRP.Operators;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection.Metadata.Ecma335;
using System.Runtime.Intrinsics.Arm;
using System.Text;
using System.Xml.Linq;
using static F_CVRP.Setup;

namespace F_CVRP.Local_Search
{
    public class GuidedPromisesSearch
    {
        public List<Node> unservedCustomers { get; set; }
        public int maxNonImprovIterations { get; set; }
        public Solution sol { get; set; }
        public Solution bestSol { get; set; }
        float[,] promisesMatrix = null;
        public int promiseIter { get; set; }
        public double replaceChance { get; set; }
        public double relocateChance { get; set; }
        public double swapChance { get; set; }
        public Model m { get; set; }
        public double lambda { get; set; }

        public int[,] timesPenalized = null;
        List<List<float>> costMatrixPenalized { get; set; }
        public int penalizedId1 { get; set; }
        public int penalizedId2 { get; set; }

        public GuidedPromisesSearch(Solver s) 
        {
            Reset(s);
        }

        public void Reset(Solver s)
        {
            m = s.m;          
            unservedCustomers = s.unservedCustomers;
            sol = s.sol.DeepCopy();
            promisesMatrix = InitializePromisesMatrix();
            //promisesList = InitializePromisesList();
            //temperature = s.temperature;
            replaceChance = s.wideBudget * s.m.runData.replaceWeight;
            relocateChance = replaceChance + 0.33 * (1 - replaceChance);
            swapChance = relocateChance + 0.33 * (1 - replaceChance);
            //maxNonImprovIterations = (int)Math.Round(s.totalIterations * s.narrowBudget * 2, 0);

            maxNonImprovIterations = m.runData.maxIterations;//m.numNodes * m.numNodes * 10;//CalculateMaxNonImproveIterations(); //(int)Math.Round(s.numNodes * Math.Log(s.numNodes) * 20 * 2 + 1000, 0);

            promiseIter = s.m.runData.random.Next(75, 150); //m.runData.promiseIter;

            lambda = s.m.runData.arcPenalty; 
            costMatrixPenalized = s.m.costMatrix.Select(costs => new List<float>(costs)).ToList();
            penalizedId1 = -1;
            penalizedId2 = -1;
            timesPenalized = InitializeTabuArcMatrix();

        }

        private int[,] InitializeTabuArcMatrix()
        {
            int length = this.m.nodes.Count;
            int[,] tabuArcMatrix = new int[length, length];

            for (int y = 0; y < length; y++)
            {
                for (int x = 0; x < length; x++)
                {
                    tabuArcMatrix[y, x] = 0;
                }
            }
            return tabuArcMatrix;
        }
        //public int numIterations { get; set; }

        private float[,] InitializePromisesMatrix()
        {
            int length = this.m.nodes.Count;
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

                //VerifySolutionCost();

                localSearchIterator++;

                if (localSearchIterator % this.promiseIter == 0)
                {
                    this.promisesMatrix = InitializePromisesMatrix();
                }

                if (this.sol.cost < this.bestSol.cost)
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
            double oper = this.m.runData.random.NextDouble();
            bool guided = this.m.runData.random.NextDouble() < this.m.runData.guidedChance;

            if (sol.feasible)
            {
                InsertUnservedCustomer();
            }

            if (oper <= this.replaceChance)
            {
                FindBestReplaceMove(rpm, localSearchIterator, guided);
                if (!guided)
                {
                    if (rpm.routeIndex != -1)
                    {
                        ApplyReplaceMove(rpm, localSearchIterator);
                    }
                }
                else
                {
                    if (rpm.moveCostPenalized < 0)
                    {
                        ApplyReplaceMove(rpm, localSearchIterator);
                    }
                    else
                    {
                        rpm.Reset();
                        PenalizeArcs();
                    }
                }


            }

            else if (oper <= this.relocateChance)
            {

                FindBestRelocationMove(rm, localSearchIterator, guided);
                if (!guided)
                {
                    if (rm.originRoutePosition != -1)
                    {
                        ApplyRelocationMove(rm, localSearchIterator);
                    }
                }
                else
                {
                    if (rm.moveCostPenalized < 0)
                    {
                        ApplyRelocationMove(rm, localSearchIterator);
                    }
                    else
                    {
                        rm.Reset();
                        PenalizeArcs();
                    }
                }
            }

            else if (oper <= this.swapChance)
            {
                FindBestSwapMove(sm, localSearchIterator, guided);
                if (!guided)
                {
                    if (sm.firstRouteIndex != -1)
                    {
                        ApplySwapMove(sm, localSearchIterator);
                    }
                }
                else
                {
                    if (sm.moveCostPenalized < 0)
                    {
                        ApplySwapMove(sm, localSearchIterator);
                    }
                    else
                    {
                        sm.Reset();
                        PenalizeArcs();
                    }
                }

            }

            else if (oper <= 1)
            {
                FindBestTwoOptMove(top, localSearchIterator, guided);
                if (!guided)
                {
                    if (top.firstRoutePosition != -1)
                    {
                        ApplyTwoOptMove(top, localSearchIterator);
                    }
                }
                else
                {
                    if (top.moveCostPenalized < 0)
                    {
                        ApplyTwoOptMove(top, localSearchIterator);
                    }
                    else
                    {
                        top.Reset();
                        PenalizeArcs();
                    }
                }
            }

        }

        private void VerifySolutionCost()
        {
            float cost = 0;
            foreach (Route rt in this.sol.routes)
            {
                for (int i = 0; i < rt.sequenceOfNodes.Count - 1; i++)
                {
                    cost += this.m.costMatrix[rt.sequenceOfNodes[i].id][rt.sequenceOfNodes[i + 1].id];
                }
            }
            if (cost != this.sol.cost)
            {
                Console.WriteLine("Cost mismatch: " + cost + " " + this.sol.cost);
            }
        }

        private void InsertUnservedCustomer()
        {
            if (this.unservedCustomers != null && this.unservedCustomers.Count > 0)
            {

                float minLoad = sol.routes.Min(rt => rt.load);
                float minDem = this.unservedCustomers.Min(c => c.demand);
                if (minDem + minLoad <= this.m.capacity)
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


        private void FindBestRelocationMove(RelocationMove rm, int iterator, bool guided)
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

                            float costAdded = this.m.costMatrix[A.id][C.id] + this.m.costMatrix[F.id][B.id] + this.m.costMatrix[B.id][G.id];
                            float costRemoved = this.m.costMatrix[A.id][B.id] + this.m.costMatrix[B.id][C.id] + this.m.costMatrix[F.id][G.id];

                            float originRtCostChange = this.m.costMatrix[A.id][C.id] - this.m.costMatrix[A.id][B.id] - this.m.costMatrix[B.id][C.id];
                            float targetRtCostChange = this.m.costMatrix[F.id][B.id] + this.m.costMatrix[B.id][G.id] - this.m.costMatrix[F.id][G.id];

                            float moveCost = costAdded - costRemoved;


                            float costAddedPenalized = this.costMatrixPenalized[A.id][C.id] + this.costMatrixPenalized[F.id][B.id] + this.costMatrixPenalized[B.id][G.id];
                            float costRemovedPenalized = this.costMatrixPenalized[A.id][B.id] + this.costMatrixPenalized[B.id][C.id] + this.costMatrixPenalized[F.id][G.id];
                            float moveCostPenalized = costAddedPenalized - costRemovedPenalized;

                            if (CheckPromise(A, C, this.sol.cost + moveCost) || CheckPromise(F, B, this.sol.cost + moveCost) || CheckPromise(B, G, this.sol.cost + moveCost))
                            {
                                continue;
                            }

                            if (guided)
                            {

                                if (moveCost < 0)
                                {
                                    if (moveCost < rm.moveCost && moveCost != 0)
                                    {
                                        rm.StoreBestRelocationMove(originRouteIndex, targetRouteIndex, originNodeIndex, targetNodeIndex, moveCost, originRtCostChange, targetRtCostChange, moveCostPenalized);
                                    }
                                }
                                else
                                {
                                    if (moveCostPenalized < rm.moveCostPenalized)
                                    {
                                        rm.StoreBestRelocationMove(originRouteIndex, targetRouteIndex, originNodeIndex, targetNodeIndex, moveCost, originRtCostChange, targetRtCostChange, moveCostPenalized);
                                    }
                                }

                            }
                            else
                            {
                                if (moveCost < rm.moveCost && moveCost != 0)
                                {
                                    rm.StoreBestRelocationMove(originRouteIndex, targetRouteIndex, originNodeIndex, targetNodeIndex, moveCost, originRtCostChange, targetRtCostChange, moveCostPenalized);
                                }
                            }
                        }
                    }
                }
            }
        }

        private void FindBestSwapMove(SwapMove sm, int iterator, bool guided)
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
                            float moveCostPenalized;

                            if (rt1 == rt2)
                            {
                                if (firstNodeIndex == secondNodeIndex - 1)
                                {
                                    float costRemoved = this.m.costMatrix[A1.id][B1.id] + this.m.costMatrix[B1.id][B2.id] + this.m.costMatrix[B2.id][C2.id];
                                    float costAdded = this.m.costMatrix[A1.id][B2.id] + this.m.costMatrix[B2.id][B1.id] + this.m.costMatrix[B1.id][C2.id];
                                    moveCost = costAdded - costRemoved;

                                    float costRemovedPenalized = this.costMatrixPenalized[A1.id][B1.id] + this.costMatrixPenalized[B1.id][B2.id] + this.costMatrixPenalized[B2.id][C2.id];
                                    float costAddedPenalized = this.costMatrixPenalized[A1.id][B2.id] + this.costMatrixPenalized[B2.id][B1.id] + this.costMatrixPenalized[B1.id][C2.id];
                                    moveCostPenalized = costAddedPenalized - costRemovedPenalized;

                                }
                                else
                                {
                                    float costRemoved1 = this.m.costMatrix[A1.id][B1.id] + this.m.costMatrix[B1.id][C1.id];
                                    float costRemoved2 = this.m.costMatrix[A2.id][B2.id] + this.m.costMatrix[B2.id][C2.id];

                                    float costAdded1 = this.m.costMatrix[A1.id][B2.id] + this.m.costMatrix[B2.id][C1.id];
                                    float costAdded2 = this.m.costMatrix[A2.id][B1.id] + this.m.costMatrix[B1.id][C2.id];

                                    moveCost = costAdded1 + costAdded2 - costRemoved1 - costRemoved2;


                                    float costRemoved1Penalized = this.costMatrixPenalized[A1.id][B1.id] + this.costMatrixPenalized[B1.id][C1.id];
                                    float costRemoved2Penalized = this.costMatrixPenalized[A2.id][B2.id] + this.costMatrixPenalized[B2.id][C2.id];

                                    float costAdded1Penalized = this.costMatrixPenalized[A1.id][B2.id] + this.costMatrixPenalized[B2.id][C1.id];
                                    float costAdded2Penalized = this.costMatrixPenalized[A2.id][B1.id] + this.costMatrixPenalized[B1.id][C2.id];

                                    moveCostPenalized = costAdded1Penalized + costAdded2Penalized - costRemoved1Penalized - costRemoved2Penalized;
                                }

                            }
                            else
                            {
                                if ((rt1.load - B1.demand + B2.demand > this.m.capacity) || (rt2.load - B2.demand + B1.demand > this.m.capacity))
                                {
                                    continue;
                                }

                                float costRemoved1 = this.m.costMatrix[A1.id][B1.id] + this.m.costMatrix[B1.id][C1.id];
                                float costRemoved2 = this.m.costMatrix[A2.id][B2.id] + this.m.costMatrix[B2.id][C2.id];

                                float costAdded1 = this.m.costMatrix[A1.id][B2.id] + this.m.costMatrix[B2.id][C1.id];
                                float costAdded2 = this.m.costMatrix[A2.id][B1.id] + this.m.costMatrix[B1.id][C2.id];

                                costChangeFirstRoute = costAdded1 - costRemoved1;
                                costChangeSecondRoute = costAdded2 - costRemoved2;

                                moveCost = costAdded1 + costAdded2 - costRemoved1 - costRemoved2;


                                float costRemoved1Penalized = this.costMatrixPenalized[A1.id][B1.id] + this.costMatrixPenalized[B1.id][C1.id];
                                float costRemoved2Penalized = this.costMatrixPenalized[A2.id][B2.id] + this.costMatrixPenalized[B2.id][C2.id];

                                float costAdded1Penalized = this.costMatrixPenalized[A1.id][B2.id] + this.costMatrixPenalized[B2.id][C1.id];
                                float costAdded2Penalized = this.costMatrixPenalized[A2.id][B1.id] + this.costMatrixPenalized[B1.id][C2.id];

                                moveCostPenalized = costAdded1Penalized + costAdded2Penalized - costRemoved1Penalized - costRemoved2Penalized;



                            }

                            if (CheckPromise(A1, B2, this.sol.cost + moveCost) || CheckPromise(B2, C1, this.sol.cost + moveCost) || CheckPromise(A2, B1, this.sol.cost + moveCost) || CheckPromise(B1, C2, this.sol.cost + moveCost))
                            {
                                continue;
                            }

                            if (guided)
                            {
                                if (moveCost < 0)
                                {
                                    if (moveCost < sm.moveCost && Math.Abs(moveCost) > 0)
                                    {
                                        //Console.WriteLine(moveCost);
                                        sm.StoreBestSwapMove(firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex, moveCost, costChangeFirstRoute, costChangeSecondRoute, moveCostPenalized);
                                    }
                                }
                                else
                                {
                                    if (moveCostPenalized < sm.moveCostPenalized)
                                    {
                                        //Console.WriteLine(moveCost);
                                        sm.StoreBestSwapMove(firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex, moveCost, costChangeFirstRoute, costChangeSecondRoute, moveCostPenalized);
                                    }
                                }

                            }
                            else
                            {
                                if (moveCost < sm.moveCost && Math.Abs(moveCost) > 0)
                                {
                                    //Console.WriteLine(moveCost);
                                    sm.StoreBestSwapMove(firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex, moveCost, costChangeFirstRoute, costChangeSecondRoute, moveCostPenalized);
                                }
                            }
                        }
                    }
                }
            }
        }


        private void FindBestTwoOptMove(TwoOptMove top, int iterator, bool guided)
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

                            float moveCostPenalized = 1000000000000000000;

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

                                float costAddedPenalized = 0;
                                float costRemovedPenalized = 0;

                                for (int i = firstNodeIndex; i <= secondNodeIndex; i++)
                                {
                                    costRemoved += this.m.costMatrix[rt1.sequenceOfNodes[i].id][rt1.sequenceOfNodes[i + 1].id];

                                    costRemovedPenalized += this.costMatrixPenalized[rt1.sequenceOfNodes[i].id][rt1.sequenceOfNodes[i + 1].id];
                                }

                                for (int i = secondNodeIndex; i > firstNodeIndex + 1; i--)
                                {
                                    costAdded += this.m.costMatrix[rt1.sequenceOfNodes[i].id][rt1.sequenceOfNodes[i - 1].id];

                                    costAddedPenalized += this.costMatrixPenalized[rt1.sequenceOfNodes[i].id][rt1.sequenceOfNodes[i - 1].id];
                                }

                                costAdded += this.m.costMatrix[A.id][K.id] + this.m.costMatrix[B.id][L.id];

                                costAddedPenalized += this.costMatrixPenalized[A.id][K.id] + this.costMatrixPenalized[B.id][L.id];


                                moveCost = costAdded - costRemoved;
                                moveCostPenalized = costAddedPenalized - costRemovedPenalized;
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

                                float costAdded = this.m.costMatrix[A.id][L.id] + this.m.costMatrix[K.id][B.id];
                                float costRemoved = this.m.costMatrix[A.id][B.id] + this.m.costMatrix[K.id][L.id];
                                moveCost = costAdded - costRemoved;

                                float costAddedPenalized = this.costMatrixPenalized[A.id][L.id] + this.costMatrixPenalized[K.id][B.id];
                                float costRemovedPenalized = this.costMatrixPenalized[A.id][B.id] + this.costMatrixPenalized[K.id][L.id];
                                moveCostPenalized = costAddedPenalized - costRemovedPenalized;
                            }

                            if (CheckPromise(A, K, this.sol.cost + moveCost) || CheckPromise(B, L, this.sol.cost + moveCost))
                            {
                                continue;
                            }

                            if (guided)
                            {
                                if (moveCost < 0)
                                {
                                    if (moveCost < top.moveCost && moveCost != 0)
                                    {
                                        top.StoreBestTwoOptMove(firstRouteIndex, secondtRouteIndex, firstNodeIndex, secondNodeIndex, moveCost, moveCostPenalized);
                                    }
                                }
                                else
                                {
                                    if (moveCostPenalized < top.moveCostPenalized)
                                    {
                                        top.StoreBestTwoOptMove(firstRouteIndex, secondtRouteIndex, firstNodeIndex, secondNodeIndex, moveCost, moveCostPenalized);
                                    }
                                }

                            }
                            else
                            {
                                if (moveCost < top.moveCost && moveCost != 0)
                                {
                                    top.StoreBestTwoOptMove(firstRouteIndex, secondtRouteIndex, firstNodeIndex, secondNodeIndex, moveCost, moveCostPenalized);
                                }
                            }
                        }
                    }
                }
            }
        }

        private void FindBestReplaceMove(ReplaceMove rpm, int iterator, bool guided)
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
                    float moveCostPenalized;

                    List<Node> candidates = B.family.nodes.Where(n => n.isRouted == false).ToList();
                    foreach (Node node in candidates)
                    {
                        //if (!node.isRouted)
                        //{
                        costAdded = this.m.costMatrix[A.id][node.id] + this.m.costMatrix[node.id][C.id];
                        costRemoved = this.m.costMatrix[A.id][B.id] + this.m.costMatrix[B.id][C.id];
                        moveCost = costAdded - costRemoved;

                        costAddedPenalized = this.costMatrixPenalized[A.id][node.id] + this.costMatrixPenalized[node.id][C.id];
                        costRemovedPenalized = this.costMatrixPenalized[A.id][B.id] + this.costMatrixPenalized[B.id][C.id];
                        moveCostPenalized = costAddedPenalized - costRemovedPenalized;

                        if (CheckPromise(B, B, this.sol.cost + moveCost)) //CheckPromise(A, B, this.sol.cost + moveCost) || CheckPromise(B, C, this.sol.cost + moveCost) ||
                        {
                            continue;
                        }

                        if (guided)
                        {
                            if (moveCost < 0)
                            {
                                if (moveCost < rpm.moveCost && Math.Abs(moveCost) > 0)
                                {
                                    rpm.StoreBestReplaceMove(routeIndex, nodeIndex, node, moveCost, moveCostPenalized);
                                }
                            }
                            else
                            {
                                if (moveCostPenalized < rpm.moveCostPenalized)
                                {
                                    rpm.StoreBestReplaceMove(routeIndex, nodeIndex, node, moveCost, moveCostPenalized);
                                }
                            }
                        }
                        else
                        {
                            if (moveCost < rpm.moveCost && Math.Abs(moveCost) > 0)
                            {
                                rpm.StoreBestReplaceMove(routeIndex, nodeIndex, node, moveCost, moveCostPenalized);
                            }
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
                float costAdded = this.m.costMatrix[rt.sequenceOfNodes[i - 1].id][n.id] + this.m.costMatrix[n.id][rt.sequenceOfNodes[i].id];
                float costRemoved = this.m.costMatrix[rt.sequenceOfNodes[i - 1].id][rt.sequenceOfNodes[i].id];

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
            //if (B != null)
            //{
            this.promisesMatrix[A.id, B.id] = cost;
            //}
            //else
            //{
            //    this.promisesList[A.id] = cost;
            //}
        }


        public bool CheckPromise(Node A, Node B, float newCost)
        {
            //if (B != null)
            //{
            if (newCost < this.promisesMatrix[A.id, B.id])
            {
                return false;
            }
            return true;
            //}
            //else
            //{
            //    if (newCost < this.promisesList[A.id])
            //    {
            //        return false;
            //    }
            //    return true;
            //}
        }

        private void PenalizeArcs()
        {
            // Collect all arcs and their criteria
            var arcCriteria = new List<(int id1, int id2, float criterion)>();
            foreach (Route rt in this.sol.routes)
            {
                for (int i = 0; i < rt.sequenceOfNodes.Count - 1; i++)
                {
                    int id1 = rt.sequenceOfNodes[i].id;
                    int id2 = rt.sequenceOfNodes[i + 1].id;
                    float criterion = this.m.costMatrix[id1][id2] / (1 + this.timesPenalized[id1, id2]);
                    arcCriteria.Add((id1, id2, criterion));
                }
            }

            // Sort arcs by criterion descending
            arcCriteria.Sort((a, b) => b.criterion.CompareTo(a.criterion));

            // Determine how many arcs to penalize (top 3%)
            int numToPenalize = Math.Max(1, (int)Math.Ceiling(arcCriteria.Count * 0.03));

            // Penalize the top arcs
            for (int i = 0; i < numToPenalize; i++)
            {
                var (pen1, pen2, _) = arcCriteria[i];
                this.timesPenalized[pen1, pen2]++;
                this.costMatrixPenalized[pen1][pen2] = (float)(1 + lambda * this.timesPenalized[pen1, pen2]) * this.m.costMatrix[pen1][pen2];
                // Optionally, update penalizedId1/2 to the last penalized arc
                this.penalizedId1 = pen1;
                this.penalizedId2 = pen2;
            }
        }
    }
}
