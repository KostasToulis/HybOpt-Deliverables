using System.Diagnostics;

namespace CHRVRP
{
    public class LocalSearch
    {
        public Solution solution;
        public ObjectiveCost oc;
        public List<double> objectiveValues;
        public List<bool> mathModelCalled;
        public Move[] moves;
        public PromisesObject promisesObject;
        public Configurations config;
        public Random rand;
        public Stopwatch elapsed;
        public long mathModelTimes;

        public LocalSearch(Solution solution, Move[] moves, Configurations config)
        {
            this.solution = solution;
            
            this.objectiveValues = new List<double>();
            objectiveValues.Add(solution.GetMainKPIObjective());
            this.mathModelCalled = new List<bool>();
            this.mathModelTimes = 0;
            
            this.oc = new ObjectiveCost(solution, config);
            this.moves = moves;
            this.promisesObject = new PromisesObject(solution.model.nodes.Count);
            this.config = config;
            rand = new Random();
            elapsed = new Stopwatch();

        }

        public LocalSearch(Solution solution)
        {
            this.solution = solution;
        }

        public void Run()
        {
            var ticks = new List<long>();
            int nonImprovingCounter = 0;
            int infeasibleCounter = 0;
            int counter = 0;
            double tolerance = 0.0000001;
            Solution bestestSolution = new Solution(this.solution);
            var relocationMove = new Relocation();
            var swapMove = new Swap();
            var topMove = new TwoOpt();
            var mathCounter = 0;
            while (nonImprovingCounter < config.MaxNonImproving)
            {
                
                mathModelCalled.Add(false);
                if (config.UseMath)
                {
                    // This acts as a pertubation mechanism. If the solution is not improve we suffle a bit
                    if (rand.NextDouble() > config.MathModelThresh &&
                        (solution.GetMainKPIObjective() - bestestSolution.GetMainKPIObjective()) / bestestSolution.GetMainKPIObjective() < 0.1)
                    {
                        elapsed.Start();
                        mathCounter++;
                        Console.WriteLine(solution.GetMainKPIObjective());
                        Solver.OptimizeRelaxedSupplyAndCustomerAssignmentModel(solution, config.MaxAdditionsRemovalsPerRoute, config.MinSPChange);
                        Console.WriteLine(solution.GetMainKPIObjective());
                        mathModelCalled[^1] = true;  // ^1 is like -1 in python
                        elapsed.Stop();
                        ticks.Add(elapsed.ElapsedTicks);
                        elapsed.Reset();
                    }   
                }
                
                Move move = FindBestMove(relocationMove, swapMove, topMove);

                /*Console.WriteLine($"Iteration {counter} , {nonImprovingCounter}");
                Console.WriteLine($"Cost {solution.GetMainKPIObjective()}, feasible? {solution.feasible}");
                Console.WriteLine($"Move: {move}");*/
                /*Console.WriteLine();*/
                /*solution.Print();
                Console.WriteLine($"Applying {move}");*/
                
                //Console.WriteLine("MAX = {0}, MIN = {1}", solution.max, solution.min);


                bool prevSolutionFeasible = solution.feasible;
                //Console.WriteLine(counter);

                try
                {
                    ApplyBestMove(move);

                    if (!solution.feasible)
                    {
                        infeasibleCounter++;
                    }

                    solution.CalculateObjectives();
                    
                }
                catch (ArgumentOutOfRangeException e)
                {
                    promisesObject.Reset();
                }
                
                if (Debugger.IsAttached && !solution.CheckEverything())
                {
                    Console.WriteLine(move.ToString());
                    Console.WriteLine("MAX = {0}, MIN = {1}", solution.max, solution.min);
                    Console.WriteLine("oopsie");
                    solution.Print();
                    Console.WriteLine($"Applying {move}");
                    Environment.Exit(1);
                }
                
                objectiveValues.Add(solution.GetMainKPIObjective());
                
                ClearMoves(relocationMove, swapMove, topMove);
                counter++;

                if (bestestSolution.GetMainKPIObjective() - this.solution.GetMainKPIObjective() > tolerance && solution.feasible)
                {
                    //Console.WriteLine($"New Best {bestestSolution.GetMainKPIObjective()}, {this.solution.GetMainKPIObjective()}");
                    bestestSolution = new Solution(this.solution);
                    promisesObject.Reset();
                    nonImprovingCounter = 0;
                }
                else
                {
                    nonImprovingCounter++;
                }
                CheckForReset(solution, prevSolutionFeasible, counter);
                
            }
            mathModelTimes = ticks.Sum(); // create sum of ticks
            
            //Console.WriteLine($"Before Updated {bestestSolution.GetMainKPIObjective()}, {this.solution.GetMainKPIObjective()}");
            this.solution = new Solution(bestestSolution);
            //Console.WriteLine($"Updated {this.solution.GetMainKPIObjective()}");
            //Console.WriteLine(mathCounter);
            /*Console.WriteLine("Infeasible solutions: "+ infeasibleCounter);*/
        }
        
        private void CheckForReset(Solution newSolution, bool prevSolutionFeasible, int counter)
        {
            int threshold = (int)(this.solution.model.customers.Count * config.ResetThresholdMultiplier);
            if ((!prevSolutionFeasible && newSolution.feasible) || counter % threshold == 0)
            {
                promisesObject.Reset();
            }
        }

        private void ClearMoves(Relocation relocation, Swap swap, TwoOpt twoOpt)
        {
            relocation.Clear();
            swap.Clear();
            twoOpt.Clear();
            foreach (var move in this.moves)
            {
                move.Clear();
            }
        }

        // Evgenia simplirose to kritirio me vasi to opoio sta kpis 3 kai 4 mia kinisi einai kaliteri
        // Entaksei Kwsta
        private bool BestMoveIsBetter(double cost)
        {
            switch (solution.model.mainKPI)
            {
                case Model.KPI.kpi1:
                    return cost < solution.objective1;
                case Model.KPI.kpi2:
                    return cost < solution.objective2;
                case Model.KPI.kpi3:
                    return cost < solution.objective3;
                case Model.KPI.kpi4:
                    return cost < solution.objective4;
                default:
                    return false;
            }
        }

        // kane apply tin kaliteri kinisi, des an xriazetai na allakseis parametrous opos min kai max
        private void ApplyBestMove(Move move)
        {
            this.promisesObject.Update(solution, move);

            switch (solution.model.mainKPI)
            {
                case Model.KPI.kpi1:
                    solution.objective1 = move.cost;
                    break;
                case Model.KPI.kpi2:
                    solution.objective2 = move.cost;
                    break;
                case Model.KPI.kpi3:
                    solution.objective3 = move.cost;
                    break;
                case Model.KPI.kpi4:
                    solution.objective4 = move.cost;
                    break;
            }
            // Console.WriteLine(solution.objective1);
            move.Apply(solution);
            solution.FeasibilityCheck();

            var route1 = solution.routes[move.originRouteIndex];
            var route2 = solution.routes[move.targetRouteIndex];

            UpdateInfo(move, route1, route2);
        }

        public void UpdateInfo(Move move, Route route1, Route route2)
        {
            UpdateNodesInfo(move.originNodeIndex, route1);
            UpdateRouteInfo(route1);
            UpdateNodesInfo(move.targetNodeIndex, route2);
            UpdateRouteInfo(route2);
            
            UpdateSolutionMinMax();
        }

        private void UpdateSolutionMinMax()
        {
            double max = solution.routes[0].max;
            int maxInd = 0;
            double min = solution.routes[0].min;
            int minInd = 0;
            for (int i = 1; i < solution.routes.Count; i++)
            {
                if (solution.routes[i].max > max)
                { 
                    max = solution.routes[i].max;
                    maxInd = i;
                }
                if (solution.routes[i].min < min)
                {
                    min = solution.routes[i].min;
                    minInd = i;
                }
            }
            solution.max = max;
            solution.min = min;
        }

        public void UpdateRouteInfo(Route route)
        {
            if (route.load > 0)
            {
                route.cumDistance = 0;
                route.min = route.sequence[2].arrivalTime;
                route.max = route.sequence.Last().arrivalTime;

                for (int index = 2; index < route.sequence.Count; index++)
                {
                    route.cumDistance += route.sequence[index].arrivalTime;
                }
            }
        }

        public void UpdateNodesInfo(int index, Route route)
        {
            var matrix = solution.model.distances;
            
            if (index == 2 && route.load > 0)
            {
                Node vehicle = route.sequence[0];
                Node sp = route.sequence[1];
                Node n = route.sequence[2];
                route.sequence[2].arrivalTime = matrix[sp.serialNumber, n.serialNumber] + matrix[vehicle.serialNumber, sp.serialNumber];
                index++;
            }
            for (int ind = index; ind < route.sequence.Count; ind++)
            {
                var prev = route.sequence[ind - 1];
                var current = route.sequence[ind];
                current.routeIndex = route.id;

                current.arrivalTime = prev.arrivalTime + matrix[prev.serialNumber, current.serialNumber];
            }
        }

        // nomizo afti de xriazetai kapoia allagi sta kpis 3 kai 4
        // kala ta les
        private Move FindBestMove(Relocation relocationMove, Swap swapMove, TwoOpt topMove)
        {
            FindBestSwap(swapMove);
            FindBestRelocation(relocationMove);
            FindBestTwoOpt(topMove);

            var minCost = swapMove.cost;
            Move bestMove = swapMove;
            if (minCost > relocationMove.cost)
            {
                minCost = relocationMove.cost;
                bestMove = relocationMove;
            }

            if (minCost > topMove.cost)
            {
                minCost = topMove.cost;
                bestMove = topMove;
            }

            return bestMove;
        }

        private void FindBestSwap(Swap move)
        {
            // TODO: allakse tis loupes gia na koitazei mono tous kontinous geitones
            move.Clear();

            foreach (var customer in solution.model.customers)
            {
                var rt1 = customer.routeIndex;
                var nd1 = customer.indexInRoute;

                foreach (var neighbor in customer.nearestNodes)
                {
                    var rt2 = neighbor.routeIndex;
                    var nd2 = neighbor.indexInRoute;
                    
                    var objectiveCost = CalculateCost(rt1, rt2, nd1, nd2, move);

                    var potentialMove = CreatePotentialMove(move, rt1, rt2, nd1, nd2, objectiveCost);
                                
                    // CheckIfBroken returns false if promises are not broken, so negate to get true
                    var promisesNotBroken = !promisesObject.CheckIfBroken(solution, potentialMove);

                    if (promisesNotBroken && potentialMove.cost < move.cost)
                    {
                        move.StoreMove(potentialMove);
                    }
                }
            }
        }
        
        private void FindBestRelocation(Relocation move)
        {
            // TODO: allakse tis loupes gia na koitazei mono tous kontinous geitones
            move.Clear();
            foreach (var customer in solution.model.customers)
            {
                var rt1 = customer.routeIndex;
                var nd1 = customer.indexInRoute;

                foreach (var neighbor in customer.nearestNodes)
                {
                    var rt2 = neighbor.routeIndex;
                    var nd2 = neighbor.indexInRoute;
                    
                    var objectiveCost = CalculateCost(rt1, rt2, nd1, nd2, move);

                    var potentialMove = CreatePotentialMove(move, rt1, rt2, nd1, nd2, objectiveCost);
                                
                    // CheckIfBroken returns false if promises are not broken, so negate to get true
                    var promisesNotBroken = !promisesObject.CheckIfBroken(solution, potentialMove);

                    if (promisesNotBroken && potentialMove.cost < move.cost)
                    {
                        move.StoreMove(potentialMove);
                    }
                }
            }
        }
        
        private void FindBestTwoOpt(TwoOpt move)
        {
            // TODO: allakse tis loupes gia na koitazei mono tous kontinous geitones
            move.Clear();

            for (int i = 0; i < solution.model.customers.Count(); i++)
            {
                var customer = solution.model.customers[i];
                var rt1 = customer.routeIndex;
                var nd1 = customer.indexInRoute;

                for (int j = i+1; j < customer.nearestNodes.Count; j++)
                {
                    var neighbor = customer.nearestNodes[j];
                    var rt2 = neighbor.routeIndex;
                    var nd2 = neighbor.indexInRoute;

                    var objectiveCost = CalculateCost(rt1, rt2, nd1, nd2, move);

                    var potentialMove = CreatePotentialMove(move, rt1, rt2, nd1, nd2, objectiveCost);
                    // CheckIfBroken returns false if promises are not broken, so negate to get true
                    var promisesNotBroken = !promisesObject.CheckIfBroken(solution, potentialMove);

                    if (promisesNotBroken && potentialMove.cost < move.cost)
                    {
                        move.StoreMove(potentialMove);
                    }
                }
            }
        }

        private Move CreatePotentialMove(Move move, int rt1, int rt2, int nd1, int nd2, double objectiveCost)
        {
            if (move is Relocation)
            {
                moves[1].StoreMove(rt1, rt2, nd1, nd2, objectiveCost);
                return moves[1];
            }
            if (move is Swap)
            {
                moves[0].StoreMove(rt1, rt2, nd1, nd2, objectiveCost);
                return moves[0];
            }
            if (move is TwoOpt)
            {
                moves[2].StoreMove(rt1, rt2, nd1, nd2, objectiveCost);
                return moves[2];
            }

            return null;
        }

        // edo profanos ftiaxe gia ta kpis 3 kai 4 tis antistoixes methodous
        private double CalculateCost(int route1, int route2, int nd1, int nd2, Move move)
        {
            var matrix = solution.model.distances;

            int A = nd1 - 1;
            int B = nd1;
            int C = nd1 + 1;
            int D = nd2 - 1;
            int E = nd2;
            int F = nd2 + 1;

            double penalty = CalculatePenalty(route1, route2, nd1, nd2, move);

            switch (solution.model.mainKPI)
            {
                case Model.KPI.kpi1:
                    return oc.Objective1Cost(A, B, C, D, E, F, route1, route2, matrix, move) + penalty;
                case Model.KPI.kpi2:
                    return oc.Objective2Cost(A, B, C, D, E, F, route1, route2, matrix, move) + penalty;
                case Model.KPI.kpi3:
                    return oc.Objective3Cost(A, B, C, D, E, F, route1, route2, matrix, move) + penalty;
                case Model.KPI.kpi4:
                    return oc.Objective4Cost(A, B, C, D, E, F, route1, route2, matrix, move) + penalty;
                default:
                    return -1;
            }

        }

        private double CalculatePenalty(int route1, int route2, int nd1, int nd2, Move move)
        {
            int timesViolated = 0;
            for (int i = 0; i < solution.routes.Count; i++) 
            {
                int adjustedload = solution.routes[i].load;
                if (move.GetType() == typeof(Relocation))
                {
                    if (i == route1)
                    {
                        adjustedload -= 1;
                    }
                    if (i == route2)
                    {
                        adjustedload += 1;
                    }
                }
                else if (move.GetType() == typeof(TwoOpt) && route1 != route2)
                {
                    if (i == route1)
                    {
                        adjustedload += ChangeInLoad(route1, route2, nd1, nd2);
                    }
                    if (i == route2)
                    {
                        adjustedload += ChangeInLoad(route2, route1, nd2, nd1);
                    }
                }
                if (adjustedload - solution.model.capacity > 0) 
                {
                    timesViolated += adjustedload - solution.model.capacity;
                }
            }
            return config.Penalty * timesViolated;
        }

        private int ChangeInLoad(int rt1, int rt2, int nd1, int nd2)
        {
            var route1 = solution.routes[rt1];
            var route2 = solution.routes[rt2];

            return (nd1 - route1.sequence.Count()) + (route2.sequence.Count() - nd2);
        }
    }
}
