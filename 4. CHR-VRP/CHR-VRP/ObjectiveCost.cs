using static CHRVRP.LocalSearch;

namespace CHRVRP
{
    public class ObjectiveCost
    {
        Solution solution;
        private Configurations config;
        public ObjectiveCost(Solution solution, Configurations config)
        {
            this.solution = solution;
            this.config = config;
        }

        public double Objective1Cost(int a, int b, int c, int d, int e, int f, int rt1, int rt2, double[,] matrix, Move move)
        {
            const double tolerance = 0.000001;
            var route1 = solution.routes[rt1];
            var route2 = solution.routes[rt2];
            if (Math.Abs(route1.totalDistance - solution.objective1 % config.Penalty) < tolerance || Math.Abs(route2.totalDistance - solution.objective1 % config.Penalty) < tolerance)
            {
                double maxTotalDistance = 0;

                if (move.GetType() == typeof(Swap))
                {
                    maxTotalDistance = Objective1Swap(a, b, c, d, e, f, rt1, rt2, matrix);
                }
                else if (move.GetType() == typeof(Relocation))
                {
                    maxTotalDistance = Objective1Relocation(a, b, c, d, e, f, rt1, rt2, matrix);
                }
                else if (move.GetType() == typeof(TwoOpt))
                {
                    maxTotalDistance = Objective1TwoOpt(a, b, c, d, e, f, rt1, rt2, matrix);
                }

                for (int routeInd = 0; routeInd < this.solution.routes.Count; routeInd++)
                {
                    if (routeInd == rt1 || routeInd == rt2)
                    {
                        continue;
                    }

                    if (solution.routes[routeInd].totalDistance > maxTotalDistance)
                    {
                        maxTotalDistance = solution.routes[routeInd].totalDistance;
                    }
                }

                return maxTotalDistance;

            }
            return Math.Pow(10, 9);
        }

        private double Objective1TwoOpt(int a, int b, int c, int d, int e, int f, int rt1, int rt2, double[,] matrix)
        {
            var route1 = solution.routes[rt1];
            var route2 = solution.routes[rt2];

            if (rt1 == rt2)
            {
                var costChange = CalculateTwoOptSameRouteCostChange(route1, matrix, b, c, e, f);
                return route1.totalDistance + costChange;
            }

            var (costChange1, costChange2) = CalculateTwoOptDifferentRouteCostChange(route1, route2, matrix, b, c, e, f);
            
            if (route1.totalDistance + costChange1 > route2.totalDistance + costChange2)
            {
                return route1.totalDistance + costChange1;
            }

            return route2.totalDistance + costChange2;
        }

        private double Objective1Relocation(int a, int b, int c, int d, int e, int f, int rt1, int rt2, double[,] matrix)
        {
            var route1 = solution.routes[rt1];
            var route2 = solution.routes[rt2];

            var (costChange1, costChange2) = CalculateCostChanges(rt1, rt2, matrix, a, b, c, e, f);
            
            if (rt1 == rt2)
            {
                return route1.totalDistance + costChange1 + costChange2;
            }

            if (route1.totalDistance + costChange1 > route2.totalDistance + costChange2)
            {
                return route1.totalDistance + costChange1;
            }

            return route2.totalDistance + costChange2;

        }

        private double Objective1Swap(int a, int b, int c, int d, int e, int f, int rt1, int rt2, double[,] matrix)
        {
            var route1 = solution.routes[rt1];
            var route2 = solution.routes[rt2];

            double totalDistance;

            if (rt1 == rt2 && b == d && c == e)
            {
                totalDistance = solution.objective1 + CalculateConsecutiveNodeChangeOriginBefore(rt1, a, b, e, f, matrix);
            }
            else if(rt1 == rt2 && a == e && b == f)
            {
                totalDistance = solution.objective1 + CalculateConsecutiveNodeChangeOriginAfter(rt1, b, c, d, e, matrix);
            }
            else
            {
                var (costChange1, costChange2) = CalculateCostChanges(rt1, rt2, matrix, a, b, c, d, e, f);

                if (rt1 == rt2)
                {
                    totalDistance = route1.totalDistance + costChange1 + costChange2;
                }
                else
                {
                    var totalDistance1 = route1.totalDistance + costChange1;
                    var totalDistance2 = route2.totalDistance + costChange2;

                    if (totalDistance1 > totalDistance2)
                    {
                        totalDistance = totalDistance1;
                    }
                    else
                    {
                        totalDistance = totalDistance2;
                    }
                }
            }
            return totalDistance;
        }

        public double Objective2Cost(int a, int b, int c, int d, int e, int f, int rt1, int rt2, double[,] matrix, Move move)
        {
            double costChange = 0;

            if (move.GetType() == typeof(Swap))
            {
                costChange = Objective2Swap(a, b, c, d, e, f, rt1, rt2, matrix);
            }
            else if (move.GetType() == typeof(Relocation))
            {
                costChange = Objective2Relocation(a, b, c, d, e, f, rt1, rt2, matrix);
            }
            else if (move.GetType() == typeof(TwoOpt))
            {
                costChange = Objective2TwoOpt(a, b, c, d, e, f, rt1, rt2, matrix);
            }

            /*  % config.Penalty is used to remove the penalty already applied in previous iterations
             *  Maybe we need a new attribute storing objective without penalties                    */
            return solution.objective2  % config.Penalty + costChange;
        }

        private double Objective2TwoOpt(int a, int b, int c, int d, int e, int f, int rt1, int rt2, double[,] matrix)
        {
            var route1 = solution.routes[rt1];
            var route2 = solution.routes[rt2];

            if (rt1 == rt2)
            {
                return CalculateTwoOptSameRouteCostChange(route1, matrix, b, c, e, f);
            }
            
            var (costChange1, costChange2) = CalculateTwoOptDifferentRouteCostChange(route1, route2, matrix, b, c, e, f);
            return costChange1 + costChange2;
        }

        private double Objective2Relocation(int a, int b, int c, int d, int e, int f, int rt1, int rt2, double[,] matrix)
        {
            var (costChange1, costChange2) = CalculateCostChanges(rt1, rt2, matrix, a, b, c, e, f);

            return costChange1 + costChange2;
        }

        private double Objective2Swap(int a, int b, int c, int d, int e, int f, int rt1, int rt2, double[,] matrix)
        {
            if (rt1 == rt2 && b == d && c == e)  // an einai sto idio route kai oi 2 pelates einai diadoxikoi
            {
                return CalculateConsecutiveNodeChangeOriginBefore(rt1, a, b, e, f, matrix);
            }
            if(rt1 == rt2 && a == e && b == f)
            {
                return CalculateConsecutiveNodeChangeOriginAfter(rt1, b, c, d, e, matrix);
            }

            var (costChange1, costChange2) = CalculateCostChanges(rt1, rt2, matrix, a, b, c, d, e, f);

            return costChange1 + costChange2;
        }

        public double Objective3Cost(int a, int b, int c, int d, int e, int f, int rt1, int rt2, double[,] matrix, Move move)
        {
            double costChange = 0;

            if (move.GetType() == typeof(Swap))
            {
                costChange = Objective3Swap(a, b, c, d, e, f, rt1, rt2, matrix);
            }
            else if (move.GetType() == typeof(Relocation))
            {
                costChange = Objective3Relocation(a, b, c, d, e, rt1, rt2, matrix);
            }
            else if (move.GetType() == typeof(TwoOpt))
            {
                costChange = Objective3TwoOpt(b, c, e, f, rt1, rt2, matrix);
            }

            return solution.objective3 + costChange;
        }

        private double Objective3TwoOpt(int b, int c, int e, int f, int rt1, int rt2, double[,] matrix)
        {
            var (costChange1, costChange2) = CalculateCumulativeCostChanges(rt1, rt2, matrix, b, c, e, f);
            return (costChange1 + costChange2) / this.solution.model.customers.Count;
        }

        private double Objective3Relocation(int a, int b, int c, int d, int e, int rt1, int rt2, double[,] matrix)
        {
            var (costChange1, costChange2) = CalculateCumulativeCostChanges(rt1, rt2, matrix, a, b, c, d, e);
            return (costChange1 + costChange2) / this.solution.model.customers.Count;
        }

        private double Objective3Swap(int a, int b, int c, int d, int e, int f, int rt1, int rt2, double[,] matrix)
        {
            var (costChange1, costChange2) = CalculateCumulativeCostChanges(rt1, rt2, matrix, a, b, c, d, e, f);
            return (costChange1 + costChange2) / this.solution.model.customers.Count;
        }

        public double Objective4Cost(int a, int b, int c, int d, int e, int f, int rt1, int rt2, double[,] matrix, Move move)
        {
            double costChange = 0;

            if (move.GetType() == typeof(Swap))
            {
                costChange = Objective4Swap(a, b, c, d, e, f, rt1, rt2, matrix);
            }
            else if (move.GetType() == typeof(Relocation))
            {
                costChange = Objective4Relocation(a, b, c, e, f, rt1, rt2, matrix);
            }
            else if (move.GetType() == typeof(TwoOpt))
            {
                costChange = Objective4TwoOpt(a, b, c, e, f, rt1, rt2, matrix);
            }

            return costChange;
        }

        private double Objective4TwoOpt(int a, int b, int c, int e, int f, int rt1, int rt2, double[,] matrix)
        {
            double newMin = CalculateTwoOptMin(b, e, rt1, rt2, matrix);
            double newMax = CalculateTwoOptMax(b, c, e, f, rt1, rt2, matrix);
            return newMax - newMin;
        }

        private double Objective4Relocation(int a, int b, int c, int e, int f, int rt1, int rt2, double[,] matrix)
        {
            double newMin = CalculateRelocationMin(b, c, e, rt1, rt2, matrix);
            double newMax = CalculateRelocationMax(a, b, c, e, f, rt1, rt2, matrix);
            return newMax - newMin;
        }

        private double Objective4Swap(int a, int b, int c, int d, int e, int f, int rt1, int rt2, double[,] matrix)
        {
            double newMin = CalculateSwapMin(b, e, rt1, rt2, matrix);
            double newMax = CalculateSwapMax(a, b, c, d, e, f, rt1, rt2, matrix);
            return newMax - newMin;
        }

        private double CalculateSwapMax(int a, int b, int c, int d, int e, int f, int rt1, int rt2, double[,] matrix)
        {
            var route1 = solution.routes[rt1];
            var route2 = solution.routes[rt2];
            double rt1Max;
            double rt2Max;
            if (rt1 == rt2 && b == d && c == e)
            {
                double costChange = CalculateConsecutiveNodeChangeOriginBefore(rt1, a, b, e, f, matrix);
                rt1Max = costChange + route1.max;
                rt2Max = costChange + route2.max;
            }
            else if(rt1 == rt2 && a == e && b == f)
            {
                double costChange = CalculateConsecutiveNodeChangeOriginAfter(rt1, b, c, d, e, matrix);
                rt1Max = costChange + route1.max;
                rt2Max = costChange + route2.max;
            }
            else
            {
                var (costChange1, costChange2) = CalculateCostChanges(rt1, rt2, matrix, a, b, c, d, e, f);
                if (rt1 == rt2)
                {
                    rt1Max = costChange1 + costChange2 + route1.max;
                    rt2Max = rt1Max;
                }
                else
                {
                    rt1Max = costChange1 + route1.max;
                    rt2Max = costChange2 + route2.max;
                }
            }
            double max = 0;
            for (int i = 0; i < this.solution.routes.Count; i++)
            {
                if (i == rt1)
                {
                    max = Math.Max(rt1Max, max);
                }
                else if (i == rt2)
                {
                    max = Math.Max(rt2Max, max);
                }
                else
                {
                    max = Math.Max(this.solution.routes[i].max, max);
                }
            }
            return max;
        }

        private double CalculateRelocationMax(int a, int b, int c, int e, int f, int rt1, int rt2, double[,] matrix)
        {
            var route1 = solution.routes[rt1];
            var route2 = solution.routes[rt2];
            double rt1Max;
            double rt2Max;
            if (rt1 == rt2 && c == e)
            {
                double costChange = CalculateConsecutiveNodeChangeOriginBefore(rt1, a, b, e, f, matrix);
                rt1Max = costChange + route1.max;
                rt2Max = costChange + route2.max;
            }
            else
            {
                var (costChange1, costChange2) = CalculateCostChanges(rt1, rt2, matrix, a, b, c, e, f);
                if (rt1 == rt2)
                {
                    rt1Max = costChange1 + costChange2 + route1.max;
                    rt2Max = rt1Max;
                }
                else
                {
                    rt1Max = costChange1 + route1.max;
                    rt2Max = costChange2 + route2.max;
                }
            }
            double max = 0;
            for (int i = 0; i < this.solution.routes.Count; i++)
            {
                if (i == rt1)
                {
                    max = Math.Max(rt1Max, max);
                }
                else if (i == rt2)
                {
                    max = Math.Max(rt2Max, max);
                }
                else
                {
                    max = Math.Max(this.solution.routes[i].max, max);
                }
            }
            return max;
        }
        
        private double CalculateTwoOptMax(int b, int c, int e, int f, int rt1, int rt2, double[,] matrix)
        {
            var route1 = solution.routes[rt1];
            var route2 = solution.routes[rt2];
            double rt1Max;
            double rt2Max;
            if (rt1 == rt2)
            {
                var costChange = CalculateTwoOptSameRouteCostChange(route1, matrix, b, c, e, f);
                rt1Max = costChange + route1.max;
                rt2Max = costChange + route2.max;
            }
            else
            {
                var (costChange1, costChange2) = CalculateTwoOptDifferentRouteCostChange(route1, route2, matrix, b, c, e, f);
                rt1Max = costChange1 + route1.max;
                rt2Max = costChange2 + route2.max;
                
            }
            double max = 0;
            for (int i = 0; i < this.solution.routes.Count; i++)
            {
                if (i == rt1)
                {
                    max = Math.Max(rt1Max, max);
                }
                else if (i == rt2)
                {
                    max = Math.Max(rt2Max, max);
                }
                else
                {
                    max = Math.Max(this.solution.routes[i].max, max);
                }
            }
            return max;
        }
        
        private double CalculateTwoOptMin(int b, int e, int rt1, int rt2, double[,] matrix)
        {
            var route1 = solution.routes[rt1];
            var route2 = solution.routes[rt2];

            var c = b + 1;
            var f = e + 1;

            if (((b > 1) && (e > 1)))
            {
                return this.solution.min;
            }
            double min = 10_000_000;
            for (int i = 0; i < this.solution.routes.Count; i++)
            {
                if (i == rt1 && b == 1 && f < route2.sequence.Count)
                {
                    Node vehicle = route1.sequence[0];
                    Node sp = route1.sequence[b];
                    Node nodeF = route2.sequence[f];
                    min = Math.Min(min,
                        matrix[sp.serialNumber, nodeF.serialNumber] + matrix[vehicle.serialNumber, sp.serialNumber]);
                }
                else if (i == rt2 && e == 1 && c < route1.sequence.Count)
                {
                    Node vehicle = route2.sequence[0];
                    Node sp = route2.sequence[e];
                    Node nodeC = route1.sequence[c];
                    min = Math.Min(min, matrix[sp.serialNumber, nodeC.serialNumber] + matrix[vehicle.serialNumber, sp.serialNumber]);
                }
                else
                {
                    min = Math.Min(this.solution.routes[i].min, min);
                }
            }
            return min;
        }

        private double CalculateSwapMin(int b, int e, int rt1, int rt2, double[,] matrix)
        {
            var route1 = solution.routes[rt1];
            var route2 = solution.routes[rt2];

            if ((b > 2) && (e > 2))
            {
                return this.solution.min;
            }
            double min = 10_000_000;
            for (int i = 0; i < this.solution.routes.Count; i++)
            {
                if (i == rt1 && b == 2)
                {
                    Node vehicle = route1.sequence[0];
                    Node sp = route1.sequence[1];
                    Node nodeE = route2.sequence[e];
                    min = Math.Min(min,
                        matrix[sp.serialNumber, nodeE.serialNumber] + matrix[vehicle.serialNumber, sp.serialNumber]);
                }
                else if (i == rt2 && e == 2)
                {
                    Node vehicle = route2.sequence[0];
                    Node sp = route2.sequence[1];
                    Node nodeB = route1.sequence[b];
                    min = Math.Min(min, matrix[sp.serialNumber, nodeB.serialNumber] + matrix[vehicle.serialNumber, sp.serialNumber]);
                }
                else
                {
                    min = Math.Min(this.solution.routes[i].min, min);
                }
            }
            return min;
        }

        private double CalculateRelocationMin(int b, int c, int e, int rt1, int rt2, double[,] matrix)
        {
            var route1 = solution.routes[rt1];
            var route2 = solution.routes[rt2];

            if ((b > 2) && (e > 2))
            {
                return this.solution.min;
            }
            double min = 10_000_000;
            for (int i = 0; i < this.solution.routes.Count; i++)
            {
                if (i == rt1 && b == 2 && c < route1.sequence.Count)
                {
                    Node vehicle = route1.sequence[0];
                    Node sp = route1.sequence[1];
                    Node nodeC = route1.sequence[c];
                    min = Math.Min(min, matrix[sp.serialNumber, nodeC.serialNumber] + matrix[vehicle.serialNumber, sp.serialNumber]);
                }
                else if (i == rt2 && e == 2 && b < route1.sequence.Count)
                {
                    Node vehicle = route2.sequence[0];
                    Node sp = route2.sequence[1];
                    Node nodeB = route1.sequence[b];
                    min = Math.Min(min, matrix[sp.serialNumber, nodeB.serialNumber] + matrix[vehicle.serialNumber, sp.serialNumber]);
                }
                else
                {
                    min = Math.Min(this.solution.routes[i].min, min);
                }
            }
            return min;
        }

        // ftiaxe ki alles an deis oti xreiazontai, aftes me volepsan gia ta kpis 1 kai 2
        private (double, double) CalculateCostChanges(int rt1, int rt2, double[,] matrix, int a, int b, int c, int d, int e, int f)
        {
            var route1 = solution.routes[rt1];
            var route2 = solution.routes[rt2];

            double costAdded1 = matrix[route1.sequence[a].serialNumber, route2.sequence[e].serialNumber];
            double costAdded2 = matrix[route2.sequence[d].serialNumber, route1.sequence[b].serialNumber];
            double costRemoved1 = matrix[route1.sequence[a].serialNumber, route1.sequence[b].serialNumber];
            double costRemoved2 = matrix[route2.sequence[d].serialNumber, route2.sequence[e].serialNumber];

            if (c < route1.sequence.Count)
            {
                costRemoved1 += matrix[route1.sequence[b].serialNumber, route1.sequence[c].serialNumber];
                costAdded1 += matrix[route2.sequence[e].serialNumber, route1.sequence[c].serialNumber];
            }

            if (f < route2.sequence.Count)
            {
                costRemoved2 += matrix[route2.sequence[e].serialNumber, route2.sequence[f].serialNumber];
                costAdded2 += matrix[route1.sequence[b].serialNumber, route2.sequence[f].serialNumber];
            }

            return (costAdded1 - costRemoved1, costAdded2 - costRemoved2);
        }
        
        private double CalculateTwoOptSameRouteCostChange(Route route1, double[,] matrix, int b, int c, int e, int f)
        {
            var costAdded = 0.0;
            var costRemoved = 0.0;
            
            if (c < route1.sequence.Count)
            {
                costRemoved += matrix[route1.sequence[b].serialNumber, route1.sequence[c].serialNumber];
            }
            
            costAdded += matrix[route1.sequence[b].serialNumber, route1.sequence[e].serialNumber];
            
            if (f < route1.sequence.Count)
            {
                costRemoved += matrix[route1.sequence[e].serialNumber, route1.sequence[f].serialNumber];
            }

            if (c < route1.sequence.Count && f < route1.sequence.Count)
            {
                costAdded += matrix[route1.sequence[c].serialNumber, route1.sequence[f].serialNumber];
            }
            
            return costAdded - costRemoved;
        }
        
        private (double costChange1, double costChange2) CalculateTwoOptDifferentRouteCostChange(Route route1, Route route2, double[,] matrix, int b, int c, int e, int f)
        {
            var costAdded1 = 0.0;
            var costAdded2 = 0.0;
            var costRemoved1 = 0.0;
            var costRemoved2 = 0.0;

            var node1 = route1.sequence[b];
            Node? node1Next;
            var route1LastNode = route1.sequence.Last();
            
            var node2 = route2.sequence[e];
            Node? node2Next;
            var route2LastNode = route2.sequence.Last();

            if (c < route1.sequence.Count)
            {
                node1Next = route1.sequence[c];
                costRemoved1 += route1LastNode.arrivalTime - node1.arrivalTime;
                costAdded2 += route1LastNode.arrivalTime - node1Next.arrivalTime + matrix[node2.serialNumber, node1Next.serialNumber];
            }
            
            if (f < route2.sequence.Count)
            {
                node2Next = route2.sequence[f];
                costRemoved2 += route2LastNode.arrivalTime - node2.arrivalTime;
                costAdded1 += route2LastNode.arrivalTime - node2Next.arrivalTime + matrix[node1.serialNumber, node2Next.serialNumber];
            }

            return (costAdded1 - costRemoved1, costAdded2 - costRemoved2);
        }

        private (double, double) CalculateCostChanges(int rt1, int rt2, double[,] matrix, int a, int b, int c, int e, int f)
        {
            var route1 = solution.routes[rt1];
            var route2 = solution.routes[rt2];

            // Tuple<double, double> costChanges = CalculateCostChanges(rt1, rt2, matrix, a, b, c, e, f);
            var costRemoved1 = 0.0;
            var costRemoved2 = 0.0;
            var costAdded1 = 0.0;
            var costAdded2 = 0.0;

            int d = e - 1;
            /**************************************************************
             * idio route kai h relocation ginetai sto amesos epomeno node
             */
            if (rt1 == rt2 && b == d)
            {
                costRemoved1 += matrix[route1.sequence[a].serialNumber, route1.sequence[b].serialNumber];
                costAdded1 += matrix[route1.sequence[a].serialNumber, route1.sequence[c].serialNumber];

                if (f < route1.sequence.Count)
                {
                    costRemoved1 += matrix[route1.sequence[e].serialNumber, route1.sequence[f].serialNumber];
                    costAdded1 += matrix[route1.sequence[b].serialNumber, route1.sequence[f].serialNumber];
                }

                return (costAdded1 - costRemoved1, 0);
            }
            // ************************************************************

            /**************************************************************
             * idio route kai h relocation ginetai se epomeno node
             */
            if (rt1 == rt2 && b < d)
            {
                costRemoved1 += matrix[route1.sequence[a].serialNumber, route1.sequence[b].serialNumber];
                costRemoved1 += matrix[route1.sequence[b].serialNumber, route1.sequence[c].serialNumber];
                costAdded1 += matrix[route1.sequence[a].serialNumber, route1.sequence[c].serialNumber];
                costAdded1 += matrix[route1.sequence[e].serialNumber, route1.sequence[b].serialNumber];

                if (f < route1.sequence.Count)
                {
                    costRemoved1 += matrix[route1.sequence[e].serialNumber, route1.sequence[f].serialNumber];
                    costAdded1 += matrix[route1.sequence[b].serialNumber, route1.sequence[f].serialNumber];
                }

                return (costAdded1 - costRemoved1, 0);
            }
            // ************************************************************

            costRemoved1 += matrix[route1.sequence[a].serialNumber, route1.sequence[b].serialNumber];
            costAdded2 += matrix[route2.sequence[d].serialNumber, route1.sequence[b].serialNumber];

            if (c < route1.sequence.Count)
            {
                costRemoved1 += matrix[route1.sequence[b].serialNumber, route1.sequence[c].serialNumber];
                costAdded1 += matrix[route1.sequence[a].serialNumber, route1.sequence[c].serialNumber];
            }

            if (e < route2.sequence.Count)
            {
                costRemoved2 += matrix[route2.sequence[d].serialNumber, route2.sequence[e].serialNumber];
                costAdded2 += matrix[route1.sequence[b].serialNumber, route2.sequence[e].serialNumber];
            }

            return (costAdded1 - costRemoved1, costAdded2 - costRemoved2);
        }

        private double CalculateConsecutiveNodeChangeOriginBefore(int rt1, int a, int b, int e, int f, double[,] matrix)
        {
            var route = solution.routes[rt1];
            double costChange1 = matrix[route.sequence[a].serialNumber, route.sequence[e].serialNumber] - matrix[route.sequence[a].serialNumber, route.sequence[b].serialNumber];
            double costChange2 = 0;

            if (f < route.sequence.Count)
            {
                costChange2 = matrix[route.sequence[b].serialNumber, route.sequence[f].serialNumber] - matrix[route.sequence[e].serialNumber, route.sequence[f].serialNumber];
            }

            return costChange1 + costChange2;
        }
        
        private double CalculateConsecutiveNodeChangeOriginAfter(int rt1, int b, int c, int d, int e, double[,] matrix)
        {
            var route = solution.routes[rt1];
            double costChange1 = 0;
            double costChange2 = matrix[route.sequence[d].serialNumber, route.sequence[b].serialNumber] - matrix[route.sequence[d].serialNumber, route.sequence[e].serialNumber];

            if (c < route.sequence.Count)
            {
                costChange1 = matrix[route.sequence[e].serialNumber, route.sequence[c].serialNumber] - matrix[route.sequence[b].serialNumber, route.sequence[c].serialNumber];
            }

            return costChange1 + costChange2;
        }
        private (double, double) CalculateCumulativeCostChanges(int rt1, int rt2, double[,] matrix, int b, int c, int e, int f)
        {
            var route1 = solution.routes[rt1];
            var route2 = solution.routes[rt2];
            Node nodeB = route1.sequence[b];
            Node nodeE = route2.sequence[e];

            if (rt1 == rt2)
            {
                double costChanged = 0;
                
                if (c < route1.sequence.Count)
                {
                    Node nodeC = route1.sequence[c];
                    costChanged -= matrix[nodeB.serialNumber, nodeC.serialNumber] * (route1.sequence.Count - c);
                }
                
                costChanged += matrix[nodeB.serialNumber, nodeE.serialNumber] * (route1.sequence.Count - c);
                
                if (f < route1.sequence.Count)
                {
                    Node nodeF = route2.sequence[f];
                    Node nodeC = route1.sequence[c];
                    costChanged += matrix[nodeB.serialNumber, nodeF.serialNumber] * (route1.sequence.Count - f) -
                                    matrix[nodeC.serialNumber, nodeF.serialNumber] * (route1.sequence.Count - f);
                }
                return (costChanged, 0);
            }

            double costChanged1 = 0;
            
            if (c < route1.sequence.Count)
            {
                Node nodeC = route1.sequence[c];
                costChanged1 += (nodeE.arrivalTime + matrix[nodeE.serialNumber, nodeC.serialNumber]) * (route1.sequence.Count - c) -
                               nodeC.arrivalTime * (route1.sequence.Count - c);
            }
            
            double costChanged2 = 0;
            
            if (f < route2.sequence.Count)
            {
                Node nodeF = route2.sequence[f];
                costChanged2 += (nodeB.arrivalTime + matrix[nodeB.serialNumber, nodeF.serialNumber]) * (route2.sequence.Count - f) -
                                nodeF.arrivalTime * (route2.sequence.Count - f);
            }

            return (costChanged1, costChanged2);
        }
        
        private (double, double) CalculateCumulativeCostChanges(int rt1, int rt2, double[,] matrix, int a, int b, int c, int d, int e, int f)
        {
            var route1 = solution.routes[rt1];
            var route2 = solution.routes[rt2];
            Node nodeA = route1.sequence[a];
            Node nodeB = route1.sequence[b];
            Node nodeD = route2.sequence[d];
            Node nodeE = route2.sequence[e];

            if (rt1 == rt2 && b == d && c == e)
            {
                double costChanged = matrix[nodeA.serialNumber, nodeE.serialNumber] * (route1.sequence.Count - b) -
                                    matrix[nodeA.serialNumber, nodeB.serialNumber] * (route1.sequence.Count - b);
                if (f < route1.sequence.Count)
                {
                    Node nodeC = route1.sequence[c];
                    Node nodeF = route2.sequence[f];
                    costChanged += matrix[nodeB.serialNumber, nodeF.serialNumber] * (route1.sequence.Count - f) -
                                    matrix[nodeC.serialNumber, nodeF.serialNumber] * (route1.sequence.Count - f);
                }
                return (costChanged, 0);
            } 
            else if (rt1 == rt2 && a == e && b == f)
            {
                double costChanged = matrix[nodeD.serialNumber, nodeB.serialNumber] * (route1.sequence.Count - e) - 
                                     matrix[nodeD.serialNumber, nodeE.serialNumber] * (route1.sequence.Count - e);

                if (c < route1.sequence.Count)
                {
                    Node nodeC = route1.sequence[c];
                    costChanged += matrix[nodeE.serialNumber, nodeC.serialNumber] * (route1.sequence.Count - c)  - 
                                   matrix[nodeB.serialNumber, nodeC.serialNumber] * (route1.sequence.Count - c);
                }

                return (costChanged, 0);
            }

            double costChanged1 = matrix[nodeA.serialNumber, nodeE.serialNumber] * (route1.sequence.Count - b) -
                                  matrix[nodeA.serialNumber, nodeB.serialNumber] * (route1.sequence.Count - b);
            if (c < route1.sequence.Count)
            {
                Node nodeC = route1.sequence[c];
                costChanged1 += matrix[nodeE.serialNumber, nodeC.serialNumber] * (route1.sequence.Count - c) -
                                matrix[nodeB.serialNumber, nodeC.serialNumber] * (route1.sequence.Count - c);
            }
            double costChanged2 = matrix[nodeD.serialNumber, nodeB.serialNumber] * (route2.sequence.Count - e) -
                                  matrix[nodeD.serialNumber, nodeE.serialNumber] * (route2.sequence.Count - e);
            if (f < route2.sequence.Count)
            {
                Node nodeF = route2.sequence[f];
                costChanged2 += matrix[nodeB.serialNumber, nodeF.serialNumber] * (route2.sequence.Count - f) -
                                matrix[nodeE.serialNumber, nodeF.serialNumber] * (route2.sequence.Count - f);
            }

            return (costChanged1, costChanged2);
        }

        private (double, double) CalculateCumulativeCostChanges(int rt1, int rt2, double[,] matrix, int a, int b, int c, int e, int f)
        {
            var route1 = solution.routes[rt1];
            var route2 = solution.routes[rt2];
            Node nodeA = route1.sequence[a];
            Node nodeB = route1.sequence[b];
            Node nodeE = route2.sequence[e];
            
            double arrivalTimeA;
            if (b == 2)
            {
                Node vehicle = route1.sequence[0];
                Node sp = route1.sequence[1];
                arrivalTimeA = matrix[vehicle.serialNumber, sp.serialNumber];
            }
            else
            {
                arrivalTimeA = nodeA.arrivalTime;
            }

            double arrivalTimeE;
            if (e == 1)
            {
                Node vehicle = route2.sequence[0];
                Node sp = route2.sequence[1];
                arrivalTimeE = matrix[vehicle.serialNumber, sp.serialNumber];
            }
            else
            {
                arrivalTimeE = nodeE.arrivalTime;
            }

            if (rt1 == rt2)
            {

                Node nodeF = route1.sequence[f];
                double arrivalTimeF;

                if (f == 2)
                {
                    Node vehicle = route1.sequence[0];
                    Node sp = route1.sequence[1];
                    arrivalTimeF = matrix[vehicle.serialNumber, sp.serialNumber];
                }
                else
                {
                    arrivalTimeF = nodeF.arrivalTime;
                }

                if (f == a)
                {
                    return (0, 0);
                }
                else if (b == f)
                {
                    Node nodeD = route1.sequence[e - 1];
                    
                    double costChanged = matrix[nodeD.serialNumber, nodeB.serialNumber] * (route1.sequence.Count - e) - 
                                         matrix[nodeD.serialNumber, nodeE.serialNumber] * (route1.sequence.Count - e);

                    if (c < route1.sequence.Count)
                    {
                        Node nodeC = route1.sequence[c];
                        costChanged += matrix[nodeE.serialNumber, nodeC.serialNumber] * (route1.sequence.Count - c)  - 
                                       matrix[nodeB.serialNumber, nodeC.serialNumber] * (route1.sequence.Count - c);
                    }

                    return (costChanged, 0);
                }
                else if (b < f)
                {
                    double costChanged = -matrix[nodeA.serialNumber, nodeB.serialNumber] * (route1.sequence.Count - b + 1) -
                                        arrivalTimeA +
                                        arrivalTimeF +
                                        matrix[nodeF.serialNumber, nodeB.serialNumber];

                    if (c < route1.sequence.Count)
                    {
                        Node nodeC = route1.sequence[c];
                        costChanged += matrix[nodeA.serialNumber, nodeC.serialNumber] * (route1.sequence.Count - b) -
                                       matrix[nodeB.serialNumber, nodeC.serialNumber] * (route1.sequence.Count - c + 1);
                    }

                    if (f < route1.sequence.Count - 1)
                    {
                        Node nodeG = route1.sequence[f + 1];
                        costChanged += -matrix[nodeF.serialNumber, nodeG.serialNumber] * (route1.sequence.Count - f - 1) +
                                       matrix[nodeF.serialNumber, nodeB.serialNumber] * (route1.sequence.Count - f - 1) +
                                       matrix[nodeB.serialNumber, nodeG.serialNumber] * (route1.sequence.Count - f - 1);
                    }

                    return (costChanged, 0);
                }
                else
                {
                    double costChanged = -matrix[nodeA.serialNumber, nodeB.serialNumber] * (route1.sequence.Count - b) -
                                          arrivalTimeA +
                                          arrivalTimeE -
                                          matrix[nodeE.serialNumber, nodeF.serialNumber] * (route1.sequence.Count - f - 1) +
                                          matrix[nodeE.serialNumber, nodeB.serialNumber] * (route1.sequence.Count - f) +
                                          matrix[nodeB.serialNumber, nodeF.serialNumber] * (route1.sequence.Count - f - 1); ;

                    if (c >= route1.sequence.Count) return (costChanged, 0);
                    Node nodeC = route1.sequence[c];
                    costChanged += matrix[nodeA.serialNumber, nodeC.serialNumber] * (route1.sequence.Count - c) -
                                   matrix[nodeB.serialNumber, nodeC.serialNumber] * (route1.sequence.Count - c);


                    return (costChanged, 0);
                }
            }

            var costChanged1 = -matrix[nodeA.serialNumber, nodeB.serialNumber] -
                               arrivalTimeA;
            if (c < route1.sequence.Count)
            {
                Node nodeC = route1.sequence[c];
                costChanged1 += -matrix[nodeA.serialNumber, nodeB.serialNumber] * (route1.sequence.Count - b - 1) +
                                matrix[nodeA.serialNumber, nodeC.serialNumber] * (route1.sequence.Count - c) -
                                matrix[nodeB.serialNumber, nodeC.serialNumber] * (route1.sequence.Count - c);
            }

            var costChanged2 = matrix[nodeE.serialNumber, nodeB.serialNumber] +
                               arrivalTimeE;

            if (f < route2.sequence.Count)
            {
                Node nodeF = route2.sequence[f];

                costChanged2 += matrix[nodeE.serialNumber, nodeB.serialNumber] * (route2.sequence.Count - e - 1) +
                                matrix[nodeB.serialNumber, nodeF.serialNumber] * (route2.sequence.Count - f) -
                                matrix[nodeE.serialNumber, nodeF.serialNumber] * (route2.sequence.Count - f);
            }
            return (costChanged1, costChanged2);
        }
    }
}