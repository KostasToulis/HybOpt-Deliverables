using CHRVRP;

namespace CHRVRP
{
    public abstract class Move
    {
        public int originRouteIndex;
        public int targetRouteIndex;
        public int originNodeIndex;
        public int targetNodeIndex;
        public double cost;

        public Move()
        {
            originNodeIndex = -1;
            originRouteIndex = -1;
            targetRouteIndex = -1;
            targetNodeIndex = -1;
            cost = Math.Pow(10, 9);
        }

        public Move(int originNodeIndex, int targetNodeIndex, int originRouteIndex, int targetRouteIndex, double cost)
        {
            this.originNodeIndex = originNodeIndex;
            this.originRouteIndex = originRouteIndex;
            this.targetRouteIndex = targetRouteIndex;
            this.targetNodeIndex = targetNodeIndex;
            this.cost = cost;
        }

        public virtual void StoreMove(int rtInd1, int rtInd2, int nodeInd1, int nodeInd2, double cst) { }

        public void StoreMove(Move potentialMove)
        {
            originRouteIndex = potentialMove.originRouteIndex;
            targetRouteIndex = potentialMove.targetRouteIndex;
            originNodeIndex = potentialMove.originNodeIndex;
            targetNodeIndex = potentialMove.targetNodeIndex;
            cost = potentialMove.cost;
        }

        public virtual void Apply(Solution solution) { }

        public void Clear()
        {
            originNodeIndex = -1;
            originRouteIndex = -1;
            targetRouteIndex = -1;
            targetNodeIndex = -1;
            cost = Math.Pow(10, 9);
        }

        public override String ToString()
        {
            return $"Move(Origin Node: {originNodeIndex}, Target Node: {targetNodeIndex}, Origin Route: {originRouteIndex}, Target Route: {targetRouteIndex}, Cost: {cost})";
        }
    }

        public class Swap : Move
        {
            public Swap() : base() { }
            
            public Swap(int originNodeIndex, int targetNodeIndex, int originRouteIndex, int targetRouteIndex, double cost) 
                : base(originNodeIndex, targetNodeIndex, originRouteIndex, targetRouteIndex, cost) {}

            public override void StoreMove(int rtInd1, int rtInd2, int nodeInd1, int nodeInd2, double cst)  // double frcd, double srcd
            {
                originRouteIndex = rtInd1;
                targetRouteIndex = rtInd2;
                originNodeIndex = nodeInd1;
                targetNodeIndex = nodeInd2;
                cost = cst;
                /*          originRouteCostDiff = frcd;
                            targetRouteCostDiff = srcd;*/
            }

            public override void Apply(Solution solution)
            {
                var matrix = solution.model.distances;
                var route1 = solution.routes[this.originRouteIndex];
                var route2 = solution.routes[this.targetRouteIndex];

                var node1 = route1.sequence[this.originNodeIndex];
                var node1Prev = route1.sequence[this.originNodeIndex - 1];
                Node? node1Next = null;
                if (this.originNodeIndex + 1 < route1.sequence.Count)
                {
                    node1Next = route1.sequence[this.originNodeIndex + 1];
                }

                var node2 = route2.sequence[this.targetNodeIndex];
                var node2Prev = route2.sequence[this.targetNodeIndex - 1];
                Node? node2Next = null;
                if (this.targetNodeIndex + 1 < route2.sequence.Count)
                {
                    node2Next = route2.sequence[this.targetNodeIndex + 1];
                }

                if (originRouteIndex == targetRouteIndex && originNodeIndex == targetNodeIndex + 1)
                {
                    route2.totalDistance += matrix[node2Prev.serialNumber, node1.serialNumber] - matrix[node2Prev.serialNumber, node2.serialNumber];
                
                    if (node1Next != null)
                    {
                        route1.totalDistance += matrix[node2.serialNumber, node1Next.serialNumber] - matrix[node1.serialNumber, node1Next.serialNumber];
                    }
                }
                else if (originRouteIndex == targetRouteIndex && originNodeIndex == targetNodeIndex - 1)
                {
                    route1.totalDistance += matrix[node1Prev.serialNumber, node2.serialNumber] - matrix[node1Prev.serialNumber, node1.serialNumber];
                    
                    if (node2Next != null)
                    {
                        route2.totalDistance += matrix[node1.serialNumber, node2Next.serialNumber] - matrix[node2.serialNumber, node2Next.serialNumber];
                    }
                }
                else
                {
                    route1.totalDistance += matrix[node1Prev.serialNumber, node2.serialNumber] - matrix[node1Prev.serialNumber, node1.serialNumber];
                    route2.totalDistance += matrix[node2Prev.serialNumber, node1.serialNumber] - matrix[node2Prev.serialNumber, node2.serialNumber];
                
                    if (node1Next != null)
                    {
                        route1.totalDistance += matrix[node2.serialNumber, node1Next.serialNumber] - matrix[node1.serialNumber, node1Next.serialNumber];
                    }
                    if (node2Next != null)
                    {
                        route2.totalDistance += matrix[node1.serialNumber, node2Next.serialNumber] - matrix[node2.serialNumber, node2Next.serialNumber];
                    }
                }

                route1.sequence[this.originNodeIndex] = node2;
                node2.routeIndex = this.originRouteIndex;
                node2.indexInRoute = this.originNodeIndex;

                route2.sequence[this.targetNodeIndex] = node1;
                node1.routeIndex = this.targetRouteIndex;
                node1.indexInRoute = this.targetNodeIndex;
            }

            public override string ToString()
            {
                return "Swap" + base.ToString();
            }
        }
    }

    public class Relocation : Move
    {
        public Relocation() : base() { }
        
        public Relocation(int originNodeIndex, int targetNodeIndex, int originRouteIndex, int targetRouteIndex, double cost)
            : base(originNodeIndex, targetNodeIndex, originRouteIndex, targetRouteIndex, cost) {}

        public override void StoreMove(int rtInd1, int rtInd2, int nodeInd1, int nodeInd2, double cst)
        {
            originRouteIndex = rtInd1;
            targetRouteIndex = rtInd2;
            originNodeIndex = nodeInd1;
            targetNodeIndex = nodeInd2;
            cost = cst;
            /*        originRouteCostDiff = frcd;
                    targetRouteCostDiff = srcd;
                    penalty = pnlty;*/
        }

        public override void Apply(Solution solution)
        {
            var matrix = solution.model.distances;
            var route1 = solution.routes[this.originRouteIndex];
            var route2 = solution.routes[this.targetRouteIndex];

            var node1 = route1.sequence[this.originNodeIndex];
            var node1prev = route1.sequence[this.originNodeIndex - 1];
            Node? node1next = null;
            if (this.originNodeIndex + 1 < route1.sequence.Count)
            {
                node1next = route1.sequence[this.originNodeIndex + 1];
            }

            var node2prev = route2.sequence[this.targetNodeIndex - 1];
            Node? node2 = null;
            if (this.targetNodeIndex < route2.sequence.Count)
            {
                node2 = route2.sequence[this.targetNodeIndex];
            }

            if (route1 == route2 && this.targetNodeIndex == this.originNodeIndex + 1)
            {
                route1.totalDistance -= matrix[node1prev.serialNumber, node1.serialNumber];
                route1.totalDistance += matrix[node1prev.serialNumber, node1next.serialNumber];
                if (this.targetNodeIndex + 1 < route1.sequence.Count)
                {
                    var node2next = route1.sequence[this.targetNodeIndex + 1];
                    route1.totalDistance -= matrix[node2.serialNumber, node2next.serialNumber];
                    route1.totalDistance += matrix[node1.serialNumber, node2next.serialNumber];
                }
            }
            else if (route1 == route2 && this.targetNodeIndex > this.originNodeIndex + 1)
            {
                route1.totalDistance -= matrix[node1prev.serialNumber, node1.serialNumber];
                route1.totalDistance -= matrix[node1.serialNumber, node1next.serialNumber];
                route1.totalDistance += matrix[node1prev.serialNumber, node1next.serialNumber];
                route1.totalDistance += matrix[node2.serialNumber, node1.serialNumber];
                if (this.targetNodeIndex + 1 < route1.sequence.Count)
                {
                    var node2next = route1.sequence[this.targetNodeIndex + 1];
                    route1.totalDistance -= matrix[node2.serialNumber, node2next.serialNumber];
                    route1.totalDistance += matrix[node1.serialNumber, node2next.serialNumber];
                }
            }
            else
            {
                route1.totalDistance -= matrix[node1prev.serialNumber, node1.serialNumber];
                if (node1next != null)
                {
                    route1.totalDistance += matrix[node1prev.serialNumber, node1next.serialNumber] - matrix[node1.serialNumber, node1next.serialNumber];
                }

                route2.totalDistance += matrix[node2prev.serialNumber, node1.serialNumber];
                if (node2 != null)
                {
                    route2.totalDistance += matrix[node1.serialNumber, node2.serialNumber] - matrix[node2prev.serialNumber, node2.serialNumber];
                }
            }

            route1.sequence.Remove(node1);

            if (route1 == route2)
            {
                route1.sequence.Insert(this.targetNodeIndex, node1);
            }
            else
            {
                route1.load--;
                route2.sequence.Insert(this.targetNodeIndex, node1);
                route2.load++;
                node1.routeIndex = this.targetRouteIndex;
                route2.IndexInRoute();
            }

            route1.IndexInRoute();
        }

        public override string ToString()
        {
            return "Relocation" + base.ToString();
        }
    }
    
    public class TwoOpt : Move
    {
        public TwoOpt() : base() { }
        
        public TwoOpt(int originNodeIndex, int targetNodeIndex, int originRouteIndex, int targetRouteIndex, double cost)
            : base(originNodeIndex, targetNodeIndex, originRouteIndex, targetRouteIndex, cost) {}

        public override void StoreMove(int rtInd1, int rtInd2, int nodeInd1, int nodeInd2, double cst)
        {
            originRouteIndex = rtInd1;
            targetRouteIndex = rtInd2;
            originNodeIndex = nodeInd1;
            targetNodeIndex = nodeInd2;
            cost = cst;
            /*        originRouteCostDiff = frcd;
                    targetRouteCostDiff = srcd;
                    penalty = pnlty;*/
        }

        public override void Apply(Solution solution)
        {
            var matrix = solution.model.distances;
            var route1 = solution.routes[this.originRouteIndex];
            var route2 = solution.routes[this.targetRouteIndex];

            var node1 = route1.sequence[this.originNodeIndex];
            Node? node1Next = null;
            if (this.originNodeIndex + 1 < route1.sequence.Count)
            {
                node1Next = route1.sequence[this.originNodeIndex + 1];
            }

            Node? node2 = route2.sequence[this.targetNodeIndex];

            Node? node2Next = null;
            if (this.targetNodeIndex + 1 < route2.sequence.Count)
            {
                node2Next = route2.sequence[this.targetNodeIndex + 1];
            }

            if (route1 == route2)
            {
                if (this.targetNodeIndex == this.originNodeIndex + 1 || this.targetNodeIndex == this.originNodeIndex - 1)
                {
                    return;
                }
                
                route1.totalDistance += matrix[node1.serialNumber, node2.serialNumber];
                
                if (node1Next != null)
                {
                    route1.totalDistance -= matrix[node1.serialNumber, node1Next.serialNumber];
                }
                
                if (node2Next != null)
                {
                    route1.totalDistance -= matrix[node2.serialNumber, node2Next.serialNumber];
                }
                
                if (node1Next != null && node2Next != null)
                {
                    route1.totalDistance += matrix[node1Next.serialNumber, node2Next.serialNumber];
                }
                
            }
            else
            {
                var costAdded1 = 0.0;
                var costAdded2 = 0.0;
                var costRemoved1 = 0.0;
                var costRemoved2 = 0.0;
                
                if (node1Next != null)
                {
                    costRemoved1 += route1.sequence.Last().arrivalTime - node1.arrivalTime;
                    costAdded2 += route1.sequence.Last().arrivalTime - node1Next.arrivalTime + matrix[node2.serialNumber, node1Next.serialNumber];
                }
            
                if (node2Next != null)
                {
                    costRemoved2 += route2.sequence.Last().arrivalTime - node2.arrivalTime;
                    costAdded1 += route2.sequence.Last().arrivalTime - node2Next.arrivalTime + matrix[node1.serialNumber, node2Next.serialNumber];
                }

                route1.totalDistance += (costAdded1 - costRemoved1);
                route2.totalDistance += (costAdded2 - costRemoved2);
            }
            
            if (route1.id == route2.id)
            {
                /*
                 * In case the target index is lower than the origin index, make a temporary swap to correctly change
                 * the route without raising an exception.
                 */
                var b = originNodeIndex;
                var e = targetNodeIndex;
                if (e < b)
                {
                    b = targetNodeIndex;
                    e = originNodeIndex;
                }
                
                var tempSeq1 = route1.sequence.GetRange(0, b + 1);
                var tempSeq2 = route1.sequence.GetRange(b + 1, e - b);
                tempSeq2.Reverse();
                tempSeq1.AddRange(tempSeq2);
                var tempSeq3 = route1.sequence.GetRange(e + 1, route1.sequence.Count - e - 1);
                tempSeq1.AddRange(tempSeq3);
                route1.sequence = tempSeq1;
                
                route1.IndexInRoute();
            }
            else
            {
                var tempSeq1 = route1.sequence.GetRange(0, originNodeIndex + 1);
                var tempSeq3 = route2.sequence.GetRange(0, targetNodeIndex + 1);
                
                if (route1.sequence.Count > originNodeIndex + 1)
                {
                    var tempSeq2 = route1.sequence.GetRange(originNodeIndex + 1, route1.sequence.Count - originNodeIndex - 1);
                    tempSeq3.AddRange(tempSeq2);
                }
                if (route2.sequence.Count > targetNodeIndex + 1)
                {
                    var tempSeq4 = route2.sequence.GetRange(targetNodeIndex + 1, route2.sequence.Count - targetNodeIndex - 1);
                    tempSeq1.AddRange(tempSeq4);
                }
                
                route1.sequence = tempSeq1;
                route2.sequence = tempSeq3;
                route1.load = route1.sequence.Count - 2;
                route2.load = route2.sequence.Count - 2;
                route1.IndexInRoute();
                route2.IndexInRoute();
            }
        }

        public override string ToString()
        {
            return "TwoOpt" + base.ToString();
        }
    }